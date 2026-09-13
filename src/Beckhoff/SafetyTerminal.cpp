/**
 * @file SafetyTerminal.cpp
 * @brief Implementation of the TwinSAFE terminal driver.
 *
 * See SafetyTerminal.hpp for the API documentation.  The FSoE PDU rides
 * directly inside the SM2/SM3 process images; exchange() forwards both
 * buffers to FSoEMasterConnection::exchangeViaPDO() once per cycle.
 */

#include "tether/Beckhoff/SafetyTerminal.hpp"

#include "tether/Beckhoff/PdoChannelLayout.hpp"
#include "tether/ethercat/LogicalAddressManager.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/TetherConfig.hpp"
#include "tether/fsoe/FSoEMasterConnection.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/platform/Platform.hpp"

#if TETHER_ENABLE_SII
#include "tether/sii/SIIParser.hpp"
#endif

namespace EtherCAT {
namespace Beckhoff {

static const char* TAG = "SafetyTerminal";

// ---------------------------------------------------------------------------
// Construction / factories
// ---------------------------------------------------------------------------

SafetyTerminal::SafetyTerminal(Master& master, uint16_t slave_index,
                               const DeviceIdentity& identity)
    : TerminalBase(master, slave_index, identity) {
}

SafetyTerminal::SafetyTerminal(Master& master, const DiscoveredSlave& slave,
                               const DeviceIdentity& identity)
    : TerminalBase(master, slave, identity) {
}

SafetyTerminal::~SafetyTerminal() = default;

Result<SafetyTerminal> SafetyTerminal::findFirst(
    Master& master, const DeviceIdentity& identity) {
    auto scan = master.discovery().discover(
        {DiscoveryOption::VendorId, DiscoveryOption::ProductCode});
    return findFirst(master, identity, scan);
}

Result<SafetyTerminal> SafetyTerminal::findFirst(
    Master& master, const DeviceIdentity& identity,
    std::span<const DiscoveredSlave> scan) {
    for (const auto& s : scan) {
        if (matches(s, identity)) return SafetyTerminal(master, s, identity);
    }
    return std::unexpected(Error::NoDeviceFound);
}

// ---------------------------------------------------------------------------
// Layout — the whole SM2/SM3 images are the FSoE black channel
// ---------------------------------------------------------------------------

bool SafetyTerminal::resolveLayout() {
    resolveSyncManagers();

#if TETHER_ENABLE_SII
    if (info_ && info_->rx_pdos && sm_out_.enabled) {
        sm_out_.first_pdo =
            detail::firstPdoIndex(*info_->rx_pdos, sm_out_.channel);
    }
    if (info_ && info_->tx_pdos && sm_in_.enabled) {
        sm_in_.first_pdo =
            detail::firstPdoIndex(*info_->tx_pdos, sm_in_.channel);
    }
#endif

    // Fallback without SII: image sizes are not derivable from the
    // identity — a 6B/6B FSoE frame is the minimum useful shape.
    if (!sm_out_.enabled) {
        sm_out_ = {2, 0x1200, 6, 0x24, true, 0x1600};
    }
    if (!sm_in_.enabled) {
        sm_in_ = {3, 0x1D00, 6, 0x20, true, 0x1A00};
    }
    return sm_in_.enabled && sm_out_.enabled &&
           sm_in_.length >= 5 && sm_out_.length >= 5 &&
           sm_in_.length <= 512 && sm_out_.length <= 512;
}

// ---------------------------------------------------------------------------
// Bring-up
// ---------------------------------------------------------------------------

Result<> SafetyTerminal::prepare(PDO::PDOAddressMode mode) {
    if (prepared_) return {};
    if (slave_index_ >= PDO::kMaxPDOSlaves) {
        return std::unexpected(Error::SlaveIndexOutOfRange);
    }

    fetchDiscovery();
    if (!verifyIdentity()) return std::unexpected(Error::WrongDevice);

    if (!resolveLayout()) {
        return std::unexpected(Error::SmConfigFailed);
    }

    TETHER_LOGI(TAG, "{}: FSoE out SM{} {}B / in SM{} {}B",
                logPrefix().c_str(), sm_out_.channel, sm_out_.length,
                sm_in_.channel, sm_in_.length);

    stageSyncManagers();
    configureMailboxOrDeclare();
    flushSyncManagers();

    if (auto r = enterPreOp(); !r) return r;
    return registerProcessData(mode);

    // FSoE connection creation happens in configure()/start() — it needs
    // the resolved image sizes.
}

Result<> SafetyTerminal::configure(const Config& cfg) {
    if (auto r = prepare(PDO::PDOAddressMode::Position); !r) return r;

    FSoE::MasterConnectionConfig fc{};
    fc.slave_addr = slave_index_ + 0x1000;  // configured station address
    fc.slave_safety_addr = cfg.safety_addr;
    fc.connection_id = cfg.connection_id ? cfg.connection_id
                                         : cfg.safety_addr;
    fc.master_addr = 0;
    fc.safety_level = cfg.safety_level;
    // 0 = derive: image minus cmd(1) + CRC(2) + connID(2).
    fc.input_size = cfg.safe_input_bytes
        ? cfg.safe_input_bytes
        : static_cast<uint8_t>(sm_in_.length > 5 ? sm_in_.length - 5 : 1);
    fc.output_size = cfg.safe_output_bytes
        ? cfg.safe_output_bytes
        : static_cast<uint8_t>(sm_out_.length > 5 ? sm_out_.length - 5 : 1);
    fc.fail_safe_values = cfg.fail_safe_values;
    fc.app_parameters = cfg.app_parameters;

    fsoe_ = std::make_unique<FSoE::FSoEMasterConnection>(fc);
    if (!fsoe_->initialize()) {
        fsoe_.reset();
        TETHER_LOGE(TAG, "{}: FSoE connection init failed "
                    "(in={}B out={}B)", logPrefix().c_str(),
                    fc.input_size, fc.output_size);
        return std::unexpected(Error::FsoeInitFailed);
    }

    return TerminalBase::enterSafeOp();
}

Result<> SafetyTerminal::prepareForLogicalExchange() {
    return prepare(PDO::PDOAddressMode::Logical);
}

Result<> SafetyTerminal::mapLogicalAndEnterSafeOp() {
    if (configured_) return {};
    if (!prepared_) {
        if (auto r = prepare(PDO::PDOAddressMode::Logical); !r) return r;
    }
    return programFmmuAndEnterSafeOp();
}

Result<> SafetyTerminal::requestOp(int timeout_ms) {
    return TerminalBase::requestOp(timeout_ms);
}

Result<> SafetyTerminal::start(const Config& cfg, const StartOptions& opts) {
    if (auto r = configure(cfg); !r) return r;
    if (auto r = startLoopIfManaged(opts); !r) return r;
    if (auto r = requestOp(opts.op_timeout_ms); !r) return r;
    TETHER_LOGI(TAG, "{}: in OP — pumping FSoE (safety addr 0x{:04X})",
                logPrefix().c_str(), cfg.safety_addr);
    return {};
}

// ---------------------------------------------------------------------------
// FSoE cyclic exchange
// ---------------------------------------------------------------------------

bool SafetyTerminal::exchange(uint64_t current_time_ms) {
    if (!fsoe_) return false;
    return fsoe_->exchangeViaPDO(out_buf_.data(), out_buf_.size(),
                                 in_buf_.data(), in_buf_.size(),
                                 current_time_ms);
}

void SafetyTerminal::resetConnection() {
    if (fsoe_) fsoe_->resetConnection();
}

// ---------------------------------------------------------------------------
// Safe I/O
// ---------------------------------------------------------------------------

bool SafetyTerminal::safeInputBit(uint8_t bit) const {
    return fsoe_ && fsoe_->getSafeInputBit(bit);
}
bool SafetyTerminal::setSafeOutputBit(uint8_t bit, bool value) {
    return fsoe_ && fsoe_->setSafeOutputBit(bit, value);
}
uint8_t SafetyTerminal::safeInputByte(uint8_t index) const {
    return fsoe_ ? fsoe_->getSafeInputByte(index) : 0;
}
bool SafetyTerminal::setSafeOutputByte(uint8_t index, uint8_t value) {
    return fsoe_ && fsoe_->setSafeOutputByte(index, value);
}

// ---------------------------------------------------------------------------
// Connection status
// ---------------------------------------------------------------------------

bool SafetyTerminal::isOperational() const {
    return fsoe_ && fsoe_->isOperational();
}
bool SafetyTerminal::isFailSafe() const {
    return fsoe_ && fsoe_->isFailSafe();
}
uint8_t SafetyTerminal::fsoeState() const {
    return fsoe_ ? fsoe_->getState() : 0;
}
uint16_t SafetyTerminal::fsoeErrorCode() const {
    return fsoe_ ? fsoe_->getErrorCode() : 0;
}

} // namespace Beckhoff

} // namespace EtherCAT
