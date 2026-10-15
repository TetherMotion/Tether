#include "tether/fsoe/Synapticon/FSoEDriveEmulator.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "logging/Logger.hpp"
#include "tether/fsoe/EmulatedChannel.hpp"
#include "tether/fsoe/FSoECRC.hpp"

namespace EtherCAT::Drives::Synapticon::SafeMotion {

namespace {
constexpr const char* kTag = "FSoEDriveEmulator";
namespace SynPDO = EtherCAT::Drives::SynapticonPDO;
}

FSoEDriveEmulator::FSoEDriveEmulator(const FSoEDriveEmulatorConfig& cfg)
    : cfg_(cfg) {}

bool FSoEDriveEmulator::initialize() {
    ServoEmulatorConfig ec{};
    ec.slave_address = cfg_.slave_addr;
    ec.connection_id = cfg_.conn_id;
    ec.safety_address = cfg_.safety_addr;
    ec.watchdog_time_ms = cfg_.watchdog_ms;
    // Some FSoE masters broadcast identical Session/Reset frames
    // (conn_id=0) into every channel slot before assigning connection
    // IDs — the slave must adopt whatever ID the master's Connection
    // frame carries rather than rejecting it against a fixed value.
    ec.accept_any_connection_id = true;
    ec.reset_crc_on_state_transition = true;
    // Emulators join a running FSoE handshake mid-stream — recover the
    // master's CRC chain seed from the first frame instead of being stuck.
    ec.crc_resync = true;
    // The configured response length determines the safe-data payload:
    // 31 B wire = LW1 (14 B data), 35 B wire = LW2 (16 B data).
    if (const size_t dl = FSoE::CRC::fsoeDataLen(cfg_.resp_frame_len);
        dl > 0) {
        ec.safe_input_size_override = static_cast<uint8_t>(dl);
        if (dl > Codec::kSlaveToMainSizeLW1) {
            ec.frame_variant = SynPDO::FSoEFrameVariant::LW2;
        }
    }
    ec.invert_sto = cfg_.invert_sto;
    ec.invert_sos = cfg_.invert_sos;
    ec.command_active_high = cfg_.command_active_high;
    ec.suppress_motion_demands = cfg_.suppress_demands;

    emu_ = std::make_unique<SafeMotionServoEmulator>(ec);
    if (!emu_->initialize()) {
        TETHER_LOGE(kTag, "FSoE{} slave emulator initialization failed",
                    cfg_.index);
        return false;
    }
    // Seed an initial slave→master response (Reset state).
    response_len_ =
        emu_->rawSlave().prepareTxFrame(response_.data(), response_.size());
    return true;
}

void FSoEDriveEmulator::reset() {
    if (!emu_) return;
    emu_->rawSlave().reset();
    emu_->resetToSafeState();
    have_master_ = false;
    last_master_len_ = 0;
    last_master_.fill(0);
    new_frame_ = false;
    response_len_ =
        emu_->rawSlave().prepareTxFrame(response_.data(), response_.size());
}

void FSoEDriveEmulator::process(const uint8_t* master_frame, size_t len,
                                uint64_t time_ms) {
    new_frame_ = false;
    if (!emu_) return;

    if (master_frame && len >= 3 && len <= last_master_.size()) {
        const bool changed =
            !have_master_ || len != last_master_len_ ||
            std::memcmp(last_master_.data(), master_frame, len) != 0;
        if (changed) {
            have_master_ = true;
            last_master_len_ = len;
            std::memcpy(last_master_.data(), master_frame, len);
            new_frame_ = true;
        }
        if (::FSoE::isValidCommandByte(master_frame[0])) {
            // Feed EVERY frame — even byte-identical repeats.  The master
            // resends the same frame as a keep-alive while in the same
            // state; FSoESlave::processRxFrame detects the duplicate,
            // refreshes lastValidFrameMs_ (watchdog) and returns without
            // re-running the state machine.  Deduping here would starve
            // the slave watchdog and drive it to fail-safe.
            const uint8_t before = emu_->rawSlave().getState();
            const bool ok =
                emu_->rawSlave().processRxFrame(master_frame, len);
            const uint8_t after = emu_->rawSlave().getState();
            if (changed && (before != after || !ok)) {
                TETHER_LOGI_FMT(kTag,
                                "ch{}: RX cmd=0x{:02X} len={} ok={} "
                                "state {}->{}",
                                cfg_.index, master_frame[0], len, ok, before,
                                after);
            }
        }
    }

    emu_->rawSlave().update(time_ms);
    // Decodes the latest master command, republishes the status via
    // Codec::encodeSlaveToMain() and mirrors each command bit into the
    // corresponding status bit (Codec::kStatusBitMirrors) in Data state.
    emu_->synchronizeCommandAndStatus();
    response_len_ =
        emu_->rawSlave().prepareTxFrame(response_.data(), response_.size());
}

uint8_t FSoEDriveEmulator::state() const {
    return emu_ ? emu_->rawSlave().getState() : 0;
}

void FSoEDriveEmulator::processMotion(
    const CiA402::MotionCommand& cmd, double dt_seconds) {
    if (!emu_) return;
    if (dt_seconds <= 0) dt_seconds = 1e-3;

    // The FSoE safety layer gates the power stage: while the master
    // demands a function (STO/SS*/SOS/SBC) the drive drops to Switch on
    // disabled — same as a physical drive behaves in passthrough.
    const bool motion_ok = emu_->controlMotionAllowed();
    const uint16_t cw = cmd.controlword;
    uint16_t sw;
    switch (cw) {
        case 0x0006: sw = 0x0231; break;   // Shutdown      → Ready to switch on
        case 0x0007: sw = 0x0233; break;   // Switch on     → Switched on
        case 0x000F:                       // Enable operation (legal from
            sw = ((motion_fb_.statusword & 0x006F) == 0x0023)
                     ? 0x0237              //   Switched on → Operation enabled
                     : 0x0233;
            break;
        default:     sw = 0x0250; break;   // Disable voltage / fault reset
                                           // / anything else → Sw on disabled
    }
    if (!motion_ok) sw = 0x0250;
    motion_fb_.statusword = sw;

    // CSP tracking: while operation-enabled the drive follows the target
    // position.  step() integrates the safe position published in the
    // FSoE frame, so the master observes the same motion as a real
    // drive.  The tracking velocity is clamped: while a demanded stop
    // function held the axis, the target kept advancing — releasing the
    // stop must produce a bounded catch-up, not a one-cycle position
    // jump.
    const bool enabled = (sw & 0x006F) == 0x0027;
    double vel =
        enabled ? (cmd.target_position - emulated_position_) / dt_seconds
                : 0.0;
    vel = std::clamp(vel, -cfg_.tracking_velocity_limit,
                     cfg_.tracking_velocity_limit);
    emulated_position_ += vel * dt_seconds;
    emu_->step(vel, dt_seconds);

    motion_fb_.modes_of_operation_display = cmd.modes_of_operation;
    motion_fb_.position_actual =
        static_cast<int32_t>(std::lround(emulated_position_));
    motion_fb_.velocity_actual = static_cast<int32_t>(std::lround(vel));
    motion_fb_.torque_actual = 0;
}

} // namespace EtherCAT::Drives::Synapticon::SafeMotion
