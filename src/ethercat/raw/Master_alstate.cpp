/**
 * @file Master_alstate.cpp
 * @brief Master — AL state request/read and PRE_OP transition.
 *
 * TU split out of Master_slave.cpp.
 */

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/LogicalAddressManager.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/DebugFlags.hpp"
#include "tether/sii/SIIParser.hpp"
#include "tether/fmmu/FMMUConfiguration.hpp"
#include "raw/internal.hpp"
#include "raw/SlaveRegistry.hpp"
#include "tether/platform/Platform.hpp"
#include <cstring>
#include <format>
#include "tether/ethercat/TransactionRouter.hpp"
#include "tether/ethercat/CyclicChannel.hpp"

namespace EtherCAT {

static const char* TAG = "ethercat";

// ============================================================================
// AL state management
// ============================================================================

bool Master::requestSlaveApplicationLayerState(SlaveAddress slave_address, uint8_t state_code)
{
    uint16_t slave_index = 0;
    if (slave_address.isPhysical()) slave_index = slave_address.slavePosition();

    if (debug_flags_.stateMachine && debug_flags_.stateMachineFilt.allows(slave_index)) {
        uint8_t current_state_code = 0;
        readSlaveApplicationLayerState(slave_address, current_state_code);
        const char* current_state_name = getECStateName(current_state_code);
        const char* target_state_name = getECStateName(state_code);
        
        TETHER_LOGI(TAG, "╔══════════════════════════════════════════════════════════════╗");
        TETHER_LOGI(TAG, "║  AL State Request: {}                                  ║", slaveLogPrefix(slave_address.slavePosition()).c_str());
        TETHER_LOGI(TAG, "╠══════════════════════════════════════════════════════════════╣");
        TETHER_LOGI(TAG, "║  Current State: {} (0x{:02X})", current_state_name, current_state_code);
        TETHER_LOGI(TAG, "║  Target State:  {} (0x{:02X})", target_state_name, state_code);
        TETHER_LOGI(TAG, "║  Action:        Writing AL_CONTROL register");
        TETHER_LOGI(TAG, "╚══════════════════════════════════════════════════════════════╝");
    }
    
    bool result = false;
    if (al_control_status_kick_.load(std::memory_order_relaxed)) {
        // Combined frame: APWR(AL_CONTROL) + APRD(AL_STATUS) in one
        // packet.  The read "kicks" firmware-driven ESCs that only
        // process pending register work while servicing traffic, and
        // returns the post-write state in the same round-trip.
        const uint16_t adp = slave_address.raw();
        const bool logical = slave_address.isLogical();
        const uint16_t le_state =
            Raw::host_to_le16(static_cast<uint16_t>(state_code));
        RxDatagram resp_w{}, resp_r{};
        const uint8_t idx_w = allocIdx();
        const uint8_t idx_r = allocIdx();
        const size_t slot_w = preRegisterResponseWaiter(
            idx_w, resp_w.data, sizeof(resp_w.data));
        const size_t slot_r = preRegisterResponseWaiter(
            idx_r, resp_r.data, sizeof(resp_r.data));
        if (slot_w >= TransactionRouter::kNumSlots ||
            slot_r >= TransactionRouter::kNumSlots) {
            if (slot_w < TransactionRouter::kNumSlots)
                packet_router_.cancelPreRegistered(slot_w);
            if (slot_r < TransactionRouter::kNumSlots)
                packet_router_.cancelPreRegistered(slot_r);
        } else {
            const MultiDatagramSpec specs[2] = {
                {logical ? Command::FPWR : Command::APWR, idx_w, adp,
                 Raw::EC_REG_AL_CONTROL, &le_state, 2, true},
                {logical ? Command::FPRD : Command::APRD, idx_r, adp,
                 Raw::EC_REG_AL_STATUS, nullptr, 2, true},
            };
            if (sendMultiDatagram(specs, 2) == 0) {
                packet_router_.cancelPreRegistered(slot_w);
                packet_router_.cancelPreRegistered(slot_r);
            } else {
                const WaitResult wr = waitForPreRegistered(slot_w, 200);
                const WaitResult rd = waitForPreRegistered(slot_r, 200);
                last_wkc_.store(rd.wkc, std::memory_order_relaxed);
                const uint16_t al = rd.data_length >= 2
                    ? static_cast<uint16_t>(resp_r.data[0] |
                        (resp_r.data[1] << 8))
                    : 0;
                TETHER_LOGI(TAG, "{}: AL_CONTROL=0x{:02X} + AL_STATUS kick: "
                                 "write[ok={} wkc={}] "
                                 "status[ok={} wkc={}] AL_STATUS=0x{:04X}",
                            slaveLogPrefix(
                                slave_address.slavePosition()).c_str(),
                            state_code, wr.success, wr.wkc,
                            rd.success, rd.wkc, al);
                result = rd.success;
            }
        }
    } else {
        result = writeRegister(slave_address,
                               RegisterAddress(Raw::EC_REG_AL_CONTROL),
                               static_cast<uint16_t>(state_code));
    }

    // Some firmware-driven ESCs (e.g. AS715N) execute the AL_CONTROL write
    // but report WKC=0 (or drop the write reply), making writeRegister()
    // return false even though the request landed.  When the write reports
    // failure, verify the slave is still reachable via AL_STATUS — if the
    // read succeeds, treat the request as issued and let the caller's
    // state-confirmation poll be the authoritative check.
    if (!result && !al_control_status_kick_.load(std::memory_order_relaxed)) {
        uint16_t al_status = 0;
        if (readRegister(slave_address, RegisterAddress(Raw::EC_REG_AL_STATUS),
                         al_status, 200)) {
            TETHER_LOGW(TAG, "{}: AL_CONTROL=0x{:02X} write unacknowledged "
                             "(wkc=0) but slave reachable (AL_STATUS=0x{:04X}) "
                             "— treating request as issued",
                        slaveLogPrefix(slave_address.slavePosition()).c_str(),
                        state_code, al_status);
            result = true;
        }
    }

    if (debug_flags_.stateMachine && debug_flags_.stateMachineFilt.allows(slave_index)) {
        TETHER_LOGI(TAG, "╔══════════════════════════════════════════════════════════════╗");
        TETHER_LOGI(TAG, "║  AL State Request Result: {}                                 ║", result ? "SUCCESS" : "FAILED");
        TETHER_LOGI(TAG, "╚══════════════════════════════════════════════════════════════╝");
    }
    
    return result;
}

bool Master::readSlaveApplicationLayerState(SlaveAddress slave_address, uint8_t& state_code)
{
    uint16_t application_layer_status = 0;
    if (!readRegister(slave_address, RegisterAddress(Raw::EC_REG_AL_STATUS), application_layer_status, 200)) {
        return false;
    }

    state_code = static_cast<uint8_t>(Raw::le16_to_host(application_layer_status) & 0x0F);
    return true;
}

bool Master::transitionSlaveToPreOperational(SlaveAddress slave_address)
{
    return setPreopAndConfirm(slave_address.slavePosition());
}

} // namespace EtherCAT

