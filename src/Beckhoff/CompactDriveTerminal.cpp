/**
 * @file CompactDriveTerminal.cpp
 * @brief Implementation of the DRV-interface compact-drive driver.
 *
 * See CompactDriveTerminal.hpp for the API documentation.  All field
 * offsets are resolved from the SII PDO entries by (object index,
 * subindex); the axis objects are 0x7010+0x70*axis (DRV outputs) and
 * 0x6010+0x70*axis (DRV inputs), feedback position lives on
 * 0x6000+0x70*axis:17.
 */

#include "tether/Beckhoff/CompactDriveTerminal.hpp"

#include <algorithm>
#include <cstring>

#include "tether/Beckhoff/PdoChannelLayout.hpp"
#include "tether/ethercat/LogicalAddressManager.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/TetherConfig.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/platform/Platform.hpp"

#if TETHER_ENABLE_SII
#include "tether/sii/SIIParser.hpp"
#endif

namespace EtherCAT {
namespace Beckhoff {

static const char* TAG = "CompactDriveTerminal";

namespace {

/// DRV object candidates for axis `a` — the channel-2 object index is
/// 0x7080 on EL7062/ELM72x2 but 0x7020 on other dual-channel DRV devices.
constexpr uint16_t drvOutObject(size_t a) { return 0x7010 + 0x70 * a; }
constexpr uint16_t drvInObject(size_t a)  { return 0x6010 + 0x70 * a; }
constexpr uint16_t drvOutObjectAlt(size_t a) { return 0x7010 + 0x10 * a; }
constexpr uint16_t drvInObjectAlt(size_t a)  { return 0x6010 + 0x10 * a; }

} // anonymous namespace

// ---------------------------------------------------------------------------
// Construction / factories
// ---------------------------------------------------------------------------

CompactDriveTerminal::CompactDriveTerminal(Master& master,
                                           uint16_t slave_index,
                                           const DeviceIdentity& identity)
    : TerminalBase(master, slave_index, identity) {
}

CompactDriveTerminal::CompactDriveTerminal(Master& master,
                                           const DiscoveredSlave& slave,
                                           const DeviceIdentity& identity)
    : TerminalBase(master, slave, identity) {
}

CompactDriveTerminal::~CompactDriveTerminal() = default;

Result<CompactDriveTerminal> CompactDriveTerminal::findFirst(
    Master& master, const DeviceIdentity& identity) {
    auto scan = master.discovery().discover(
        {DiscoveryOption::VendorId, DiscoveryOption::ProductCode});
    return findFirst(master, identity, scan);
}

Result<CompactDriveTerminal> CompactDriveTerminal::findFirst(
    Master& master, const DeviceIdentity& identity,
    std::span<const DiscoveredSlave> scan) {
    for (const auto& s : scan) {
        if (matches(s, identity)) {
            return CompactDriveTerminal(master, s, identity);
        }
    }
    return std::unexpected(Error::NoDeviceFound);
}

// ---------------------------------------------------------------------------
// Layout resolution
// ---------------------------------------------------------------------------

bool CompactDriveTerminal::resolveLayout() {
    resolveSyncManagers();

#if TETHER_ENABLE_SII
    if (info_ && info_->rx_pdos && info_->tx_pdos &&
        sm_out_.enabled && sm_in_.enabled) {
        const auto& rx = *info_->rx_pdos;
        const auto& tx = *info_->tx_pdos;
        const uint8_t oc = sm_out_.channel;
        const uint8_t ic = sm_in_.channel;

        auto field = [](std::span<const SII::SIIPDO> pdos, uint8_t ch,
                        uint16_t index, int sub) {
            Field f;
            if (!detail::findValueEntry(pdos, ch, index, sub, 0, 8,
                                        f.bit_off, f.bit_len)) {
                f.bit_off = kInvalid;
                f.bit_len = 0;
            }
            return f;
        };

        // FB position objects: 0x60xx objects with a >=32-bit entry at
        // :17, excluding the DRV-input objects already assigned to axes.
        std::vector<uint16_t> claimed_fb;
        auto fbPosition = [&](uint16_t drv_in) {
            const uint16_t first = drv_in - 0x10;   // 0x6000, 0x6070, ...
            for (uint16_t x : {first, (uint16_t)0x6000, (uint16_t)0x6070,
                               (uint16_t)0x6020, (uint16_t)0x6030}) {
                if (x == drv_in) continue;
                if (std::ranges::find(claimed_fb, x) != claimed_fb.end()) {
                    continue;
                }
                Field f = field(tx, ic, x, 0x11);
                if (f.present() && f.bit_len >= 16) {
                    claimed_fb.push_back(x);
                    return f;
                }
            }
            return Field{};
        };

        // Axes are counted by DRV controlwords found in the output image.
        for (size_t a = 0; a < kMaxAxes; ++a) {
            uint16_t out_obj = drvOutObject(a), in_obj = drvInObject(a);
            Field cw = field(rx, oc, out_obj, 0x01);
            Field sw = field(tx, ic, in_obj, 0x01);
            if (!cw.present() && !sw.present() && a > 0) {
                // Alternate ch-n spacing (0x7020/0x6020 for axis 1 ...).
                out_obj = drvOutObjectAlt(a);
                in_obj  = drvInObjectAlt(a);
                cw = field(rx, oc, out_obj, 0x01);
                sw = field(tx, ic, in_obj, 0x01);
            }
            if (!cw.present() && !sw.present()) break;
            Axis ax{};
            ax.controlword  = cw;
            ax.modes        = field(rx, oc, out_obj, 0x03);
            ax.target_pos   = field(rx, oc, out_obj, 0x05);
            ax.target_vel   = field(rx, oc, out_obj, 0x06);
            ax.target_torque= field(rx, oc, out_obj, 0x09);
            ax.statusword   = sw;
            ax.modes_disp   = field(tx, ic, in_obj, 0x03);
            ax.following_err= field(tx, ic, in_obj, 0x06);
            ax.vel_act      = field(tx, ic, in_obj, 0x07);
            ax.torque_act   = field(tx, ic, in_obj, 0x08);
            ax.fb_position  = fbPosition(in_obj);
            axes_.push_back(ax);
        }
        if (axes_.empty()) {
            TETHER_LOGE(TAG, "{}: no DRV controlword/statusword in the "
                        "process image — not a DRV-interface drive",
                        logPrefix().c_str());
            return false;
        }
    }
#endif

    // Fallback without SII: one axis, the EL7201 default 6B/6B shape.
    //   out: [ctrl(2) tgt_vel(4)]      in: [pos(4) status(2)]
    if (!sm_out_.enabled) {
        sm_out_ = {2, 0x1100, 6, 0x24, true, 0x1600};
    }
    if (!sm_in_.enabled) {
        sm_in_ = {3, 0x1180, 6, 0x20, true, 0x1A00};
    }
    if (axes_.empty()) {
        Axis ax{};
        ax.controlword = {0, 16};        // out byte 0
        ax.target_vel  = {16, 32};       // out byte 2
        ax.fb_position = {0, 32};        // in byte 0
        ax.statusword  = {32, 16};       // in byte 4
        axes_.push_back(ax);
    }
    return sm_in_.enabled && sm_out_.enabled &&
           sm_in_.length <= 512 && sm_out_.length <= 512 &&
           !axes_.empty();
}

// ---------------------------------------------------------------------------
// Bring-up
// ---------------------------------------------------------------------------

Result<> CompactDriveTerminal::prepare(PDO::PDOAddressMode mode) {
    if (prepared_) return {};
    if (slave_index_ >= PDO::kMaxPDOSlaves) {
        return std::unexpected(Error::SlaveIndexOutOfRange);
    }

    fetchDiscovery();
    if (!verifyIdentity()) return std::unexpected(Error::WrongDevice);

    if (!resolveLayout()) {
        return std::unexpected(Error::SmConfigFailed);
    }

    TETHER_LOGI(TAG, "{}: DRV interface out SM{} {}B / in SM{} {}B"
                " ({} axis)",
                logPrefix().c_str(), sm_out_.channel, sm_out_.length,
                sm_in_.channel, sm_in_.length, axes_.size());

    stageSyncManagers();
    configureMailboxOrDeclare();
    flushSyncManagers();

    if (auto r = enterPreOp(); !r) return r;
    return registerProcessData(mode);
}

Result<> CompactDriveTerminal::configure() {
    if (auto r = prepare(PDO::PDOAddressMode::Position); !r) return r;
    return TerminalBase::enterSafeOp();
}

Result<> CompactDriveTerminal::prepareForLogicalExchange() {
    return prepare(PDO::PDOAddressMode::Logical);
}

Result<> CompactDriveTerminal::mapLogicalAndEnterSafeOp() {
    if (configured_) return {};
    if (!prepared_) {
        if (auto r = prepare(PDO::PDOAddressMode::Logical); !r) return r;
    }
    return programFmmuAndEnterSafeOp();
}

Result<> CompactDriveTerminal::requestOp(int timeout_ms) {
    return TerminalBase::requestOp(timeout_ms);
}

Result<> CompactDriveTerminal::start(const StartOptions& opts) {
    if (auto r = configure(); !r) return r;
    if (auto r = startLoopIfManaged(opts); !r) return r;
    if (auto r = requestOp(opts.op_timeout_ms); !r) return r;
    TETHER_LOGI(TAG, "{}: in OP", logPrefix().c_str());
    return {};
}

// ---------------------------------------------------------------------------
// Field access helpers
// ---------------------------------------------------------------------------

int64_t CompactDriveTerminal::inSigned(const Field& f) const {
    if (!f.present()) return 0;
    const size_t byte = f.bit_off / 8;
    if (byte + (f.bit_len + 7) / 8 > in_buf_.size()) return 0;
    return detail::signExtendLE(in_buf_.data() + byte, f.bit_len);
}

void CompactDriveTerminal::outSigned(const Field& f, int32_t v) {
    if (!f.present()) return;
    const size_t byte = f.bit_off / 8;
    if (byte + (f.bit_len + 7) / 8 > out_buf_.size()) return;
    if (f.bit_len > 16) {
        std::memcpy(out_buf_.data() + byte, &v, 4);
    } else if (f.bit_len > 8) {
        const int16_t s = static_cast<int16_t>(v);
        std::memcpy(out_buf_.data() + byte, &s, 2);
    } else {
        out_buf_[byte] = static_cast<uint8_t>(v);
    }
}

uint16_t CompactDriveTerminal::inWord(const Field& f) const {
    return static_cast<uint16_t>(inSigned(f) & 0xFFFF);
}

// ---------------------------------------------------------------------------
// Control
// ---------------------------------------------------------------------------

void CompactDriveTerminal::setControlword(size_t axis, uint16_t cw) {
    auto* a = axisPtr(axis);
    if (!a || !a->controlword.present()) return;
    const size_t byte = a->controlword.bit_off / 8;
    if (byte + 2 <= out_buf_.size()) {
        std::memcpy(out_buf_.data() + byte, &cw, 2);
    }
}

uint16_t CompactDriveTerminal::controlword(size_t axis) const {
    const auto* a = axisPtr(axis);
    if (!a || !a->controlword.present()) return 0;
    const size_t byte = a->controlword.bit_off / 8;
    uint16_t w = 0;
    if (byte + 2 <= out_buf_.size()) {
        std::memcpy(&w, out_buf_.data() + byte, 2);
    }
    return w;
}

bool CompactDriveTerminal::hasControlword(size_t axis) const {
    const auto* a = axisPtr(axis);
    return a && a->controlword.present();
}

void CompactDriveTerminal::setModesOfOperation(size_t axis, int8_t mode) {
    auto* a = axisPtr(axis);
    if (!a || !a->modes.present()) return;
    const size_t byte = a->modes.bit_off / 8;
    if (byte < out_buf_.size()) out_buf_[byte] = static_cast<uint8_t>(mode);
}

bool CompactDriveTerminal::hasModesOfOperation(size_t axis) const {
    const auto* a = axisPtr(axis);
    return a && a->modes.present();
}

void CompactDriveTerminal::setTargetPosition(size_t axis, int32_t pos) {
    auto* a = axisPtr(axis);
    if (a) outSigned(a->target_pos, pos);
}
bool CompactDriveTerminal::hasTargetPosition(size_t axis) const {
    const auto* a = axisPtr(axis);
    return a && a->target_pos.present();
}

void CompactDriveTerminal::setTargetVelocity(size_t axis, int32_t vel) {
    auto* a = axisPtr(axis);
    if (a) outSigned(a->target_vel, vel);
}
bool CompactDriveTerminal::hasTargetVelocity(size_t axis) const {
    const auto* a = axisPtr(axis);
    return a && a->target_vel.present();
}

void CompactDriveTerminal::setTargetTorque(size_t axis, int16_t tq) {
    auto* a = axisPtr(axis);
    if (a) outSigned(a->target_torque, tq);
}
bool CompactDriveTerminal::hasTargetTorque(size_t axis) const {
    const auto* a = axisPtr(axis);
    return a && a->target_torque.present();
}

// ---------------------------------------------------------------------------
// Status
// ---------------------------------------------------------------------------

uint16_t CompactDriveTerminal::statusword(size_t axis) const {
    const auto* a = axisPtr(axis);
    return a ? inWord(a->statusword) : 0;
}
bool CompactDriveTerminal::hasStatusword(size_t axis) const {
    const auto* a = axisPtr(axis);
    return a && a->statusword.present();
}

int8_t CompactDriveTerminal::modesOfOperationDisplay(size_t axis) const {
    const auto* a = axisPtr(axis);
    return a ? static_cast<int8_t>(inSigned(a->modes_disp)) : 0;
}
bool CompactDriveTerminal::hasModesOfOperationDisplay(size_t axis) const {
    const auto* a = axisPtr(axis);
    return a && a->modes_disp.present();
}

int32_t CompactDriveTerminal::actualPosition(size_t axis) const {
    const auto* a = axisPtr(axis);
    return a ? static_cast<int32_t>(inSigned(a->fb_position)) : 0;
}
bool CompactDriveTerminal::hasActualPosition(size_t axis) const {
    const auto* a = axisPtr(axis);
    return a && a->fb_position.present();
}

int32_t CompactDriveTerminal::actualVelocity(size_t axis) const {
    const auto* a = axisPtr(axis);
    return a ? static_cast<int32_t>(inSigned(a->vel_act)) : 0;
}
bool CompactDriveTerminal::hasActualVelocity(size_t axis) const {
    const auto* a = axisPtr(axis);
    return a && a->vel_act.present();
}

int16_t CompactDriveTerminal::actualTorque(size_t axis) const {
    const auto* a = axisPtr(axis);
    return a ? static_cast<int16_t>(inSigned(a->torque_act)) : 0;
}
bool CompactDriveTerminal::hasActualTorque(size_t axis) const {
    const auto* a = axisPtr(axis);
    return a && a->torque_act.present();
}

int32_t CompactDriveTerminal::followingError(size_t axis) const {
    const auto* a = axisPtr(axis);
    return a ? static_cast<int32_t>(inSigned(a->following_err)) : 0;
}
bool CompactDriveTerminal::hasFollowingError(size_t axis) const {
    const auto* a = axisPtr(axis);
    return a && a->following_err.present();
}

// ---------------------------------------------------------------------------
// CiA402 helpers
// ---------------------------------------------------------------------------

DriveState CompactDriveTerminal::decodeState(
    uint16_t sw) {
    // Canonical CiA402 statusword decode (bits 0-6 determine the state).
    if ((sw & 0x004F) == 0x0000) return DriveState::NotReadyToSwitchOn;
    if ((sw & 0x004F) == 0x0040) return DriveState::SwitchOnDisabled;
    if ((sw & 0x006F) == 0x0021) return DriveState::ReadyToSwitchOn;
    if ((sw & 0x006F) == 0x0023) return DriveState::SwitchedOn;
    if ((sw & 0x006F) == 0x0027) return DriveState::OperationEnabled;
    if ((sw & 0x006F) == 0x0007) return DriveState::QuickStopActive;
    if ((sw & 0x004F) == 0x000F) return DriveState::FaultReactionActive;
    if ((sw & 0x004F) == 0x0008) return DriveState::Fault;
    return DriveState::Unknown;
}

DriveState CompactDriveTerminal::driveState(
    size_t axis) const {
    return decodeState(statusword(axis));
}

bool CompactDriveTerminal::fault(size_t axis) const {
    return (statusword(axis) & 0x0008) != 0;
}
bool CompactDriveTerminal::operationEnabled(size_t axis) const {
    return driveState(axis) == DriveState::OperationEnabled;
}
bool CompactDriveTerminal::warningPresent(size_t axis) const {
    return (statusword(axis) & 0x0080) != 0;
}
bool CompactDriveTerminal::targetReached(size_t axis) const {
    return (statusword(axis) & 0x0400) != 0;
}

void CompactDriveTerminal::requestShutdown(size_t axis) {
    setControlword(axis, 0x0006);
}
void CompactDriveTerminal::requestSwitchOn(size_t axis) {
    setControlword(axis, 0x0007);
}
void CompactDriveTerminal::requestEnableOperation(size_t axis) {
    setControlword(axis, 0x000F);
}
void CompactDriveTerminal::requestDisableVoltage(size_t axis) {
    setControlword(axis, 0x0000);
}
void CompactDriveTerminal::requestQuickStop(size_t axis) {
    setControlword(axis, 0x0002);
}
void CompactDriveTerminal::requestFaultReset(size_t axis) {
    setControlword(axis, controlword(axis) | 0x0080);
}
void CompactDriveTerminal::clearFaultReset(size_t axis) {
    setControlword(axis, controlword(axis) & ~0x0080u);
}

} // namespace Beckhoff

} // namespace EtherCAT
