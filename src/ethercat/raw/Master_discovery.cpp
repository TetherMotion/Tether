/**
 * @file Master_discovery.cpp
 * @brief Master — Slave discovery, PRE_OP transition and master task
 */

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/DC.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/SDOManager.hpp"
#include "tether/ethercat/FoE.hpp"
#include "tether/ethercat/VoE.hpp"
#include "tether/ethercat/EoE.hpp"
#include "tether/ethercat/FaultDetection.hpp"
#include "tether/ethercat/RealtimeLoop.hpp"
#include "tether/ethercat/SyncManagerValidation.hpp"
#include "tether/ethercat/DebugFlags.hpp"
#include "tether/sii/SIIParser.hpp"
#include "tether/fmmu/FMMUConfiguration.hpp"
#include "raw/internal.hpp"
#include "tether/platform/Platform.hpp"

#include <thread>
#include <chrono>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include "sii/SIIReader.hpp"
#include <inttypes.h>

namespace EtherCAT {

static const char* TAG = "ethercat";

// ============================================================================
// Internal helpers: scan frame builder
// ============================================================================

static Raw::EtherCATScanFrame buildScanFrame(const uint8_t src_mac[6])
{
    using namespace Raw;
    EtherCATScanFrame frame{};
    const uint8_t dst[6] = {0x01,0x01,0x05,0x00,0x00,0x00};
    std::memcpy(frame.eth.dst, dst, 6);
    std::memcpy(frame.eth.src, src_mac, 6);
    frame.eth.etherType_be = host_to_be16(EtherCAT::kEtherTypeEtherCAT);
    constexpr uint16_t payload_len = 10+2+2, type = 0x1;
    frame.ec.raw_le = host_to_le16(
        static_cast<uint16_t>((payload_len & 0x07FFu)|((type & 0x0Fu)<<12)));
    frame.dg.cmd = Command::BRD;
    frame.dg.adp_le = host_to_le16(0);
    frame.dg.ado_le = host_to_le16(EC_REG_AL_STATUS);
    constexpr uint16_t datalen = 2, rt = (1u<<14);
    frame.dg.lenFlags.raw_le = host_to_le16(static_cast<uint16_t>((datalen&0x07FFu)|rt));
    return frame;
}

// ============================================================================
// Internal: discover slaves
// ============================================================================

bool Master::discoverSlaves()
{
    for (int attempt = 0; attempt < 200; attempt++) {
        if (!running_.load(std::memory_order_acquire)) return false;

        const uint8_t idx = allocIdx();
        auto frame = buildScanFrame(src_mac_);
        frame.dg.idx = idx;

        // Pre-register waiter BEFORE sending to avoid race condition
        RxDatagram resp{};
        size_t slot = preRegisterResponseWaiter(idx, resp.data, sizeof(resp.data));
        if (slot >= TransactionRouter::kNumSlots) {
            TETHER_LOGW(TAG, "Failed to pre-register waiter for discovery");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        uint8_t txbuf[60] = {0};
        std::memcpy(txbuf, &frame, sizeof(frame));
        if (!sendRawFrame(txbuf, sizeof(txbuf))) {
            packet_router_.cancelPreRegistered(slot);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        WaitResult result = waitForPreRegistered(slot, 300);
        if (result.success && result.wkc > 0) {
            // Copy received data to resp datagram structure
            resp.idx = result.idx;
            resp.cmd = result.cmd;
            resp.adp = result.adp;
            resp.ado = result.ado;
            resp.datalen = result.data_length;
            resp.wkc = result.wkc;

            discovered_slave_count_.store(resp.wkc, std::memory_order_release);
            initSlaves(resp.wkc);
            if (faults_) {
                faults_->init(resp.wkc);
            }
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
}

// ============================================================================
// Internal: set PRE_OP and confirm
// ============================================================================

bool Master::setPreopAndConfirm(uint16_t slave_index)
{
    using namespace Raw;

    if (debug_flags_.stateMachine && debug_flags_.stateMachineFilt.allows(slave_index)) {
        TETHER_LOGI(TAG, "╔══════════════════════════════════════════════════════════════╗");
        TETHER_LOGI(TAG, "║  State Machine Transition: Slave %u (INIT => PRE_OP)          ║", slave_index);
        TETHER_LOGI(TAG, "╠══════════════════════════════════════════════════════════════╣");
        TETHER_LOGI(TAG, "║  Transition: INIT => PRE_OP");
        TETHER_LOGI(TAG, "║  Reason:    Automatism - enabling mailbox operations");
        TETHER_LOGI(TAG, "║  Requirements:");
        TETHER_LOGI(TAG, "║    - Slave must respond to AL_STATUS register reads");
        TETHER_LOGI(TAG, "║    - Slave must accept AL_CONTROL register writes");
        TETHER_LOGI(TAG, "║  Status:     Starting transition process with retry logic");
        TETHER_LOGI(TAG, "╚══════════════════════════════════════════════════════════════╝");
    }

    // We'll attempt to set PRE_OP multiple times, with an extended wait per attempt.
    // This helps recover from transient conditions where the slave's mailbox or
    // internal state is not yet ready to accept AL state changes.
    const int max_attempts = 3;          // Number of attempts
    const int inner_tries = 200;         // Wait checks per attempt (inner loop)
    const int inner_sleep_ms = 20;       // Delay between checks

    for (int attempt = 1; attempt <= max_attempts; ++attempt) {
        if (debug_flags_.stateMachine && debug_flags_.stateMachineFilt.allows(slave_index)) {
            TETHER_LOGI(TAG, "╔══════════════════════════════════════════════════════════════╗");
            TETHER_LOGI(TAG, "║  Attempt %d/%d for Slave %u                                    ║", attempt, max_attempts, slave_index);
            TETHER_LOGI(TAG, "╚══════════════════════════════════════════════════════════════╝");
        }
        
        // Read current AL status and decide whether to request PRE_OP
        uint16_t al_le = 0;
        (void)readRegister(SlaveAddress(slave_index), EC_REG_AL_STATUS, al_le, 200);
        const uint16_t al0 = le16_to_host(al_le);
        const bool has_error = (al0 & 0x0010u) != 0;

        if (debug_flags_.stateMachine && debug_flags_.stateMachineFilt.allows(slave_index)) {
            const char* state_name = al_status_get_state_name(al0);
            TETHER_LOGI(TAG, "  Current AL_STATUS: 0x%04X (State=%s, Error=%s)", 
                       al0, state_name, has_error ? "true" : "false");
            TETHER_LOGI(TAG, "  Requesting PRE_OP with error bit: %s", has_error ? "SET" : "CLEAR");
        }

        const uint16_t req = static_cast<uint16_t>(0x0002u | (has_error ? 0x0010u : 0));
        (void)writeRegister(SlaveAddress(slave_index), EC_REG_AL_CONTROL, req);

        if (debug_flags_.stateMachine && debug_flags_.stateMachineFilt.allows(slave_index)) {
            TETHER_LOGI(TAG, "  Wrote AL_CONTROL: 0x%04X", req);
            TETHER_LOGI(TAG, "  Waiting for PRE_OP to become active (max %d checks, %dms each)...", 
                       inner_tries, inner_sleep_ms);
        }

        // Wait for PRE_OP to become active
        for (int i = 0; i < inner_tries; i++) {
            uint16_t s_le = 0;
            if (readRegister(SlaveAddress(slave_index), EC_REG_AL_STATUS, s_le, 200)) {
                if ((le16_to_host(s_le) & 0x000Fu) == 0x0002u) {
                    if (attempt > 1) {
                        TETHER_LOGI(TAG, "setPreop: succeeded on attempt %d", attempt);
                    }
                    if (debug_flags_.stateMachine && debug_flags_.stateMachineFilt.allows(slave_index)) {
                        TETHER_LOGI(TAG, "╔══════════════════════════════════════════════════════════════╗");
                        TETHER_LOGI(TAG, "║  Transition Result: Slave %u => PRE_OP SUCCESS                ║", slave_index);
                        TETHER_LOGI(TAG, "║  Confirmed after %d checks on attempt %d/%d                   ║", i+1, attempt, max_attempts);
                        TETHER_LOGI(TAG, "╚══════════════════════════════════════════════════════════════╝");
                    }

                    // Post-PRE_OP SM validation safety net
                    uint8_t sm0_ctrl = 0, sm1_ctrl = 0;
                    (void)readRegister(SlaveAddress(slave_index), static_cast<uint16_t>(EC_REG_SM0 + 0x04), sm0_ctrl, 200);
                    (void)readRegister(SlaveAddress(slave_index), static_cast<uint16_t>(EC_REG_SM1 + 0x04), sm1_ctrl, 200);
                    if (sm0_ctrl != 0x26 || sm1_ctrl != 0x22) {
                        TETHER_LOGW(TAG, "setPreop: SM0=0x%02X SM1=0x%02X (expected 0x26/0x22) — slave may have rejected mailbox config", sm0_ctrl, sm1_ctrl);
                    }

                    return true;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(inner_sleep_ms));
        }

        // If we reached here, the attempt failed — gather diagnostics and retry
        uint16_t s_le_fail = 0;
        if (readRegister(SlaveAddress(slave_index), EC_REG_AL_STATUS, s_le_fail, 200)) {
            const uint16_t al_raw = le16_to_host(s_le_fail);
            const char* state_name = EtherCAT::al_status_get_state_name(al_raw);
            const bool is_error = EtherCAT::al_status_has_error(al_raw);

            // Try to read AL_STATUS_CODE for more detail
            uint16_t al_code_le = 0;
            const bool have_code = readRegister(SlaveAddress(slave_index), EC_REG_AL_STATUS_CODE, al_code_le, 200);
            uint16_t al_code = have_code ? le16_to_host(al_code_le) : 0;

            if (is_error) {
                TETHER_LOGW(TAG, "setPreop: attempt %d failed, AL_STATUS=0x%04X (State=%s, ERROR=true)",
                         attempt, al_raw, state_name);
            } else {
                TETHER_LOGW(TAG, "setPreop: attempt %d failed, AL_STATUS=0x%04X (State=%s)",
                         attempt, al_raw, state_name);
            }

            if (have_code) {
                TETHER_LOGW(TAG, "setPreop: AL status code: %s (0x%04X)", EtherCAT::getALStatusCodeName(static_cast<EtherCAT::ALStatusCode>(al_code)), al_code);

                // Fallback: If the slave reports Invalid Mailbox Configuration on the
                // first attempt to enter PRE_OP, optionally try forcing conservative mailbox
                // settings (ignore SII) and reconfigure SM0/SM1. This helps with
                // devices whose SM0/SM1 registers were cleared (e.g. after a software reset).
                if ((al_code == static_cast<uint16_t>(ALStatusCode::InvalidMailboxConfig) ||
                     al_code == static_cast<uint16_t>(ALStatusCode::InvalidMailboxConfigPreOp)) &&
                    attempt == 1) {
                    if (config_.enable_mailbox_fallback) {
                        TETHER_LOGW(TAG, "setPreop: AL status code indicates invalid mailbox for slave %u — applying mailbox defaults (enable_mailbox_fallback=true)", slave_index);
                        if (forceMailboxDefaults(slave_index)) {
                            TETHER_LOGI(TAG, "setPreop: mailbox defaults applied for slave %u; retrying PRE_OP", slave_index);
                        } else {
                            TETHER_LOGW(TAG, "setPreop: forceMailboxDefaults failed for slave %u", slave_index);
                        }
                        // Wait for slave to process new SM config
                        std::this_thread::sleep_for(std::chrono::milliseconds(200));
                    } else {
                        TETHER_LOGW(TAG, "setPreop: AL status code indicates invalid mailbox for slave %u — set enable_mailbox_fallback=true to auto-fix", slave_index);
                    }
                }
            }

            // If AL_STATUS had an error bit set, issue a one-time fault diagnostic
            // dump for this slave so users get actionable guidance.
            if (is_error) {
                std::lock_guard<std::mutex> _lg(m_diag_mutex_);
                if (m_diagnosed_slaves_.find(slave_index) == m_diagnosed_slaves_.end()) {
                    TETHER_LOGI(TAG, "setPreop: issuing one-time fault_diagnose() for slave %u", slave_index);
                    faults_->diagnose(slave_index);
                    m_diagnosed_slaves_.insert(slave_index);
                }
            }
        } else {
            TETHER_LOGW(TAG, "setPreop: attempt %d failed, AL_STATUS read failed", attempt);
        }

        // Read SM0 (mailbox status) for additional context
        uint8_t sm0 = 0;
        const uint16_t sm0_ado = 0x0805; // SM0 status register
        if (readRegister(SlaveAddress(slave_index), sm0_ado, sm0, 200)) {
            TETHER_LOGW(TAG, "setPreop: SM0=0x%02X (mailbox status)", sm0);
        } else {
            TETHER_LOGW(TAG, "setPreop: SM0 read failed");
        }

        // Backoff before retrying
        const int backoff_ms = 200 * attempt;
        std::this_thread::sleep_for(std::chrono::milliseconds(backoff_ms));
    }

    TETHER_LOGE(TAG, "setPreopAndConfirm: all %d attempts failed", 3);
    return false;
}

bool Master::forceMailboxDefaults(SlaveAddress slave_address)
{
    uint16_t slave_index = 0;
    if (!resolvePhysicalSlaveIndex(slave_address, slave_index)) {
        return false;
    }

    // Conservative hardcoded fallback values with smaller sizes for better compatibility
    // Use 128-byte mailboxes which are more commonly supported by simple devices
    // Standard EtherCAT convention: SM0 (Receive/MbxIn, M→S),
    //                               SM1 (Send/MbxOut, S→M)
    constexpr uint16_t kHardcodedWrAddr = 0x1000;  // Receive/MbxIn (M→S, SM0)
    constexpr uint16_t kHardcodedWrLen = 128;
    constexpr uint16_t kHardcodedRdAddr = 0x1400;  // Send/MbxOut (S→M, SM1)
    constexpr uint16_t kHardcodedRdLen = 128;

    if (slave_index >= PDO::kMaxPDOSlaves) return false;

    // Compute ADP for this slave (0x0000 for slave 0, 0xFFFF for slave 1, etc.)
    uint16_t adp = adpForSlaveIndex(slave_index);

    // When called as a fallback for Invalid Mailbox Configuration errors,
    // skip SII completely and use truly conservative hardcoded defaults.
    // The SII values are likely incorrect if we're in this fallback path.
    TETHER_LOGW(TAG, "forceMailboxDefaults: Using conservative hardcoded mailbox defaults (bypassing potentially incorrect SII)");
    uint16_t wr_addr = kHardcodedWrAddr;
    uint16_t wr_len = kHardcodedWrLen;
    uint16_t rd_addr = kHardcodedRdAddr;
    uint16_t rd_len = kHardcodedRdLen;

    auto* slave_configs = pdo_->slaveConfigs();
    // Standard EtherCAT mailbox SM convention:
    // SM0 = Receive/MbxIn (MASTER→SLAVE, control=0x26)
    // SM1 = Send/MbxOut    (SLAVE→MASTER, control=0x22)
    slave_configs[slave_index].sm[0] = PDO::SyncManagerConfig::mailbox_write(wr_addr, wr_len);  // SM0 = Receive/MbxIn (M→S)
    slave_configs[slave_index].sm[1] = PDO::SyncManagerConfig::mailbox_read(rd_addr, rd_len);   // SM1 = Send/MbxOut (S→M)

    std::vector<PDO::SyncManagerConfig> sm_vec;
    for (int i=0; i<4; ++i) sm_vec.push_back(slave_configs[slave_index].sm[i]);
    
    auto val_res = SyncManagerValidation::validate(sm_vec);
    if (!val_res.valid) {
        TETHER_LOGE(TAG, "SyncManager Validation Failed for slave %u: %s", slave_index, val_res.error_message.c_str());
        return false;
    }

    bool applied = pdo_->configureSlavesSMs(slave_index);
    if (mailbox_fallback_cb_) mailbox_fallback_cb_(slave_index);
    return applied;
}

} // namespace EtherCAT
