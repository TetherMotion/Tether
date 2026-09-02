/**
 * @file CiA402DriveStateMachine.cpp
 * @brief CiA 402 Drive State Machine and EtherCAT state transitions
 * 
 * This file contains the CiA 402 state machine implementation,
 * EtherCAT state transitions, and drive enable/disable logic.
 */

#include "tether/profiles/cia402/CiA402Drive.hpp"
#include "tether/profiles/cia402/CiA402StateUtils.hpp"
#include "tether/profiles/cia402/DynaDriveController.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/DC.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/FaultDetection.hpp"
#include "tether/platform/Platform.hpp"

#include <cinttypes>
#include <cstring>
#include <bit>

static const char* TAG = "CiA402StateMachine";

namespace EtherCAT {

// ============================================================================
// EtherCAT State Machine
// ============================================================================

ECState CiA402Drive::getECState() {
    uint8_t state_code = 0;
    if (m_master && m_master->readSlaveApplicationLayerState(m_slave_index, state_code)) {
        // Mask out error bit (bit 4) and other flags to get just the state
        return static_cast<ECState>(state_code & 0x0F);
    }
    return ECState::Unknown;
}

bool CiA402Drive::gotoInit() {
    TETHER_LOGI(TAG, "{}: Requesting INIT state", logPrefix().c_str());
    if (!m_master) return false;
    return m_master->requestSlaveApplicationLayerState(m_slave_index, static_cast<uint8_t>(ECState::Init));
}

bool CiA402Drive::gotoPreOp() {
    TETHER_LOGI(TAG, "{}: Requesting PRE_OP state (with mailbox config)", logPrefix().c_str());
    if (!m_master) return false;
    return m_master->transitionSlaveToPreOperational(m_slave_index);
}

bool CiA402Drive::gotoSafeOp() {
    TETHER_LOGI(TAG, "{}: Requesting SAFE_OP state", logPrefix().c_str());
    if (!m_master) return false;
    if (!m_master->requestSlaveApplicationLayerState(m_slave_index, static_cast<uint8_t>(ECState::SafeOp)))
        return false;
    
    // Wait for slave to reach SAFE_OP (up to 2 seconds)
    for (int attempt = 0; attempt < 20; attempt++) {
        Tether::Platform::Clock::instance().delayMilliseconds(100);
        uint8_t state = 0;
        if (m_master->readSlaveApplicationLayerState(m_slave_index, state)) {
            if (state == static_cast<uint8_t>(ECState::SafeOp)) {
                TETHER_LOGI(TAG, "{}: SAFE_OP confirmed after {} ms", logPrefix().c_str(), (attempt+1)*100);
                return true;
            }
        }
    }
    
    // Read final state and error for diagnostics
    uint8_t final_state = 0;
    m_master->readSlaveApplicationLayerState(m_slave_index, final_state);
    uint8_t asc[2] = {0};
    m_master->readRegister(SlaveAddress(m_slave_index), 0x0134, asc, 2, 200);
    uint16_t al_code = asc[0] | (asc[1] << 8);
    TETHER_LOGE(TAG, "{}: SAFE_OP not confirmed after 2s, state=0x{:02X} (AL status code: {} (0x{:04X}))",
             logPrefix().c_str(), final_state, getALStatusCodeName(al_code), al_code);
    return false;
}

bool CiA402Drive::gotoOp() {
    TETHER_LOGI(TAG, "{}: Requesting OP state", logPrefix().c_str());
    if (!m_master) return false;
    // Request OP with Error Acknowledge bit (0x08 | 0x10 = 0x18)
    // Some slaves require the ACK bit to clear internal error latches.
    if (!m_master->requestSlaveApplicationLayerState(m_slave_index, static_cast<uint8_t>(ECState::Op) | 0x10))
        return false;
    
    // Wait for slave to actually reach OP (up to 5 seconds).
    // The DC realtime task drives PDO exchange when pdo_enabled_ is set,
    // which was done by transitionToOp() before calling this method.
    // The slave needs continuous PDO data on SM2 to accept the OP transition.
    const uint8_t* src_mac = m_master->getSrcMac();
    for (int attempt = 0; attempt < 50; attempt++) {
        Tether::Platform::Clock::instance().delayMilliseconds(100);
        
        uint8_t state = 0;
        if (m_master->readSlaveApplicationLayerState(m_slave_index, state)) {
            if (state == static_cast<uint8_t>(ECState::Op)) {
                TETHER_LOGI(TAG, "{}: OP confirmed after {} ms", logPrefix().c_str(), (attempt+1)*100);
                return true;
            }
            // Check for unexpected state (not SAFE_OP or OP)
            if (state != static_cast<uint8_t>(ECState::SafeOp) &&
                state != static_cast<uint8_t>(ECState::Op)) {
                uint8_t asc[2] = {0};
                m_master->readRegister(SlaveAddress(m_slave_index), 0x0134, asc, 2, 200);
                uint16_t al_code = asc[0] | (asc[1] << 8);
                TETHER_LOGE(TAG, "{}: Unexpected state 0x{:02X} during OP transition (AL status code: {} (0x{:04X}))",
                         logPrefix().c_str(), state, getALStatusCodeName(al_code), al_code);
                return false;
            }
            // Re-request OP every second - some slaves need repeated requests
            if ((attempt % 10) == 9) {
                m_master->requestSlaveApplicationLayerState(m_slave_index, static_cast<uint8_t>(ECState::Op) | 0x10);
                // Read raw AL_STATUS (16-bit, including error bit)
                uint16_t al_raw = 0;
                m_master->readRegister(SlaveAddress(m_slave_index), 0x0130, al_raw, 200);
                // Read AL_STATUS_CODE
                uint16_t al_code = 0;
                m_master->readRegister(SlaveAddress(m_slave_index), 0x0134, al_code, 200);
                // Read DC SYNC active register
                uint8_t dc_sync_act = 0;
                m_master->dc().get()->readRegister(m_slave_index, DCRegisters::DCSyncAct, &dc_sync_act, 1, 200);

                // Read DC System Time
                uint8_t dc_time[8] = {0};
                m_master->dc().get()->readRegister(m_slave_index, DCRegisters::DCSysTime, dc_time, 8, 200);
                uint64_t sys_time_lo = dc_time[0] | (dc_time[1]<<8) | (dc_time[2]<<16) | (dc_time[3]<<24);

                // Read SYNC Latch status (0x098E) — bit 0 toggles with each SYNC0 event
                uint8_t sync_latch = 0;
                m_master->dc().get()->readRegister(m_slave_index, DCRegisters::DCSyncLatch, &sync_latch, 1, 200);
                // Read SM2 SM Event Request (0x0820) to see if outputs are being consumed
                uint8_t sm2_event = 0;
                m_master->readRegister(SlaveAddress(m_slave_index), 0x0820, sm2_event, 200);
                // PDO exchange stats
                auto pstats = m_master->pdoForSlave(m_slave_index).getPhysicalStats();
                TETHER_LOGI(TAG, "{}: Still waiting for OP, AL_STATUS=0x{:04X} AL status code: {} (0x{:04X}) DC_SYNC_ACT=0x{:02X} DC_SysTime_lo=0x{:08X} ({} ms)\n  PDO: fpwr_ok={} fpwr_err={} fprd_ok={} fprd_err={}  SYNC_LATCH=0x{:02X} SM2_EVT=0x{:02X}",
                         logPrefix().c_str(), al_raw, getALStatusCodeName(al_code), al_code, dc_sync_act, (unsigned long)sys_time_lo, (attempt+1)*100,
                         pstats.fpwr_success, pstats.fpwr_wkc_errors, pstats.fprd_success, pstats.fprd_wkc_errors, sync_latch, sm2_event);
            }
        }
    }
    
    // Read final state and error
    uint8_t final_state = 0;
    m_master->readSlaveApplicationLayerState(m_slave_index, final_state);
    uint8_t asc[2] = {0};
    m_master->readRegister(SlaveAddress(m_slave_index), 0x0134, asc, 2, 200);
    uint16_t al_code = asc[0] | (asc[1] << 8);

    // Also read full AL_STATUS for error flag
    uint16_t al_status = 0;
    m_master->readRegister(SlaveAddress(m_slave_index), 0x0130, al_status, 200);
    
    // Read PDO exchange stats to diagnose if PDO was ever active
    auto pstats = m_master->pdoForSlave(m_slave_index).getPhysicalStats();
    
    TETHER_LOGW(TAG, "{}: OP not confirmed after 5s, current state=0x{:02X} (AL status code: {} (0x{:04X}))\n"
             "  AL_STATUS=0x{:04X}{}\n"
             "  PDO stats: fpwr_ok={} fpwr_err={} fprd_ok={} fprd_err={}",
             logPrefix().c_str(), final_state, getALStatusCodeName(al_code), al_code,
             al_status, (al_status & 0x10) ? " (ERROR)" : "",
             pstats.fpwr_success, pstats.fpwr_wkc_errors,
             pstats.fprd_success, pstats.fprd_wkc_errors);
    
    if (pstats.fpwr_success == 0 && pstats.fprd_success == 0) {
        TETHER_LOGE(TAG, "{}: PDO exchange never occurred! Check that DC realtime loop has PDO exchange wired in.", logPrefix().c_str());
    }
    
    return (final_state == static_cast<uint8_t>(ECState::Op));
}

bool CiA402Drive::transitionToOp(bool apply_pdo_mapping) {
    TETHER_LOGI(TAG, "{}: Beginning transition to OP", logPrefix().c_str());
    
    // Check if slave is already in PRE_OP. If so, skip gotoPreOp() to avoid
    // re-initializing the slave which would reset all CoE objects (PDO mapping,
    // operating mode, etc.) that were configured by the application.
    uint8_t current_state = 0;
    bool need_preop = true;
    if (m_master && m_master->readSlaveApplicationLayerState(m_slave_index, current_state)) {
        if (current_state == static_cast<uint8_t>(ECState::PreOp)) {
            TETHER_LOGI(TAG, "{}: Already in PRE_OP (0x{:02X}), skipping gotoPreOp()", 
                     logPrefix().c_str(), current_state);
            need_preop = false;
        } else {
            TETHER_LOGI(TAG, "{}: Current state=0x{:02X}, requesting PRE_OP", 
                     logPrefix().c_str(), current_state);
        }
    }
    
    if (need_preop) {
        if (!gotoPreOp()) {
            TETHER_LOGE(TAG, "{}: Failed to reach PRE_OP", logPrefix().c_str());
            return false;
        }
        Tether::Platform::Clock::instance().delayMilliseconds(100);
    }
    
    // Apply fixed PDO mapping in PRE_OP (via assignFixedPDOs before this call)
    if (apply_pdo_mapping && !m_pdo_configured && m_rxpdo_size > 0) {
        if (!assignFixedPDOs(m_rxpdo_index, m_txpdo_index, m_rxpdo_size, m_txpdo_size)) {
            TETHER_LOGW(TAG, "{}: PDO mapping failed, continuing anyway", logPrefix().c_str());
        }
    }
    
    // CRITICAL: Configure SM2/SM3 (process data sync managers) from SII before SAFE_OP.
    // The slave validates SM2/SM3 during the PRE_OP → SAFE_OP transition and will
    // reject the transition with AL_STATUS_CODE 0x001E "Invalid input configuration"
    // if SM2/SM3 are not properly configured.
    TETHER_LOGI(TAG, "{}: Configuring PDO sync managers (SM2/SM3) from SII", logPrefix().c_str());
    if (!m_master->configureProcessDataSyncManagersFromSii(m_slave_index)) {
        TETHER_LOGW(TAG, "{}: SM2/SM3 configuration from SII failed, SAFE_OP may fail", logPrefix().c_str());
    }
    
    // CRITICAL: Update SM2/SM3 lengths to match the actual PDO mapping sizes.
    // The SII stores the drive's *default* PDO sizes, but assignFixedPDOs() may have
    // reconfigured the PDOs via SDO to use different sizes. The slave validates that
    // SM2/SM3 lengths match the actual PDO mapping during SAFE_OP transition.
    if (m_pdo_configured) {
        auto* slave_configs = m_master->pdoForSlave(m_slave_index).slaveConfigs();
        const uint16_t rxpdo_size = m_rxpdo_size;
        const uint16_t txpdo_size = m_txpdo_size;
        
        if (rxpdo_size > 0 && slave_configs[m_slave_index].sm[2].type != PDO::SyncManagerType::Unused) {
            TETHER_LOGI(TAG, "{}: Updating SM2 length: {} -> {} (from PDO config)",
                     logPrefix().c_str(), slave_configs[m_slave_index].sm[2].length, rxpdo_size);
            slave_configs[m_slave_index].sm[2].length = rxpdo_size;
            slave_configs[m_slave_index].rxpdo_size = rxpdo_size;
        }
        if (txpdo_size > 0 && slave_configs[m_slave_index].sm[3].type != PDO::SyncManagerType::Unused) {
            TETHER_LOGI(TAG, "{}: Updating SM3 length: {} -> {} (from PDO config)",
                     logPrefix().c_str(), slave_configs[m_slave_index].sm[3].length, txpdo_size);
            slave_configs[m_slave_index].sm[3].length = txpdo_size;
            slave_configs[m_slave_index].txpdo_size = txpdo_size;
        }
        
        // Re-write SM2/SM3 to slave with corrected lengths
        // const uint8_t* src_mac = m_master->getSrcMac(); // Not used
        for (uint8_t sm = 2; sm < 4; sm++) {
            const auto& cfg = slave_configs[m_slave_index].sm[sm];
            if (cfg.type != PDO::SyncManagerType::Unused && cfg.phys_start_addr != 0) {
                uint16_t base = static_cast<uint16_t>(0x0800 + sm * 8);

                // Disable SM first
                uint8_t disable = 0x00;
                m_master->writeRegister(SlaveAddress(m_slave_index), static_cast<uint16_t>(base + 6), &disable, 1, 200);

                // Write physical address
                uint16_t addr_le = cfg.phys_start_addr;
                // Write the sync-manager registers with explicit little-endian fields.
                uint8_t addr_buf[2] = {static_cast<uint8_t>(cfg.phys_start_addr & 0xFF),
                                       static_cast<uint8_t>((cfg.phys_start_addr >> 8) & 0xFF)};
                m_master->writeRegister(SlaveAddress(m_slave_index), base, addr_buf, 2, 200);

                // Write length
                uint8_t len_buf[2] = {static_cast<uint8_t>(cfg.length & 0xFF),
                                      static_cast<uint8_t>((cfg.length >> 8) & 0xFF)};
                m_master->writeRegister(SlaveAddress(m_slave_index), static_cast<uint16_t>(base + 2), len_buf, 2, 200);

                // Write control
                uint8_t ctrl_byte = std::bit_cast<uint8_t>(cfg.control);
                m_master->writeRegister(SlaveAddress(m_slave_index), static_cast<uint16_t>(base + 4), &ctrl_byte, 1, 200);

                // Enable SM
                uint8_t activate = cfg.enable ? 0x01 : 0x00;
                m_master->writeRegister(SlaveAddress(m_slave_index), static_cast<uint16_t>(base + 6), &activate, 1, 200);

                TETHER_LOGI(TAG, "{}: Re-wrote SM{}: Addr=0x{:04X} Len={} Ctrl=0x{:02X}",
                         logPrefix().c_str(), sm, cfg.phys_start_addr, cfg.length, ctrl_byte);
            }
        }
    }
    
    // PRE_OP -> SAFE_OP -> OP (common tail)
    return transitionSafeOpToOp();
}

// ============================================================================
// transitionSafeOpToOp — common SAFE_OP → OP tail
// ============================================================================

bool CiA402Drive::transitionSafeOpToOp() {
    // PRE_OP -> SAFE_OP
    if (!gotoSafeOp()) {
        TETHER_LOGE(TAG, "{}: Failed to reach SAFE_OP", logPrefix().c_str());
        return false;
    }

    // CRITICAL: Reconfigure DC SYNC signals AFTER SM configuration
    // The initial DC config during slave discovery may have failed because
    // the slave wasn't ready. Now that we're in SAFE_OP with SM configured,
    // retry the DC SYNC configuration.
    TETHER_LOGI(TAG, "{}: Reconfiguring DC SYNC in SAFE_OP", logPrefix().c_str());
    if (m_master) {
        if (!m_master->dc().reconfigureSync(m_slave_index))
            TETHER_LOGW(TAG, "{}: DC reconfiguration failed, continuing anyway", logPrefix().c_str());
    }
    Tether::Platform::Clock::instance().delayMilliseconds(50);

    // CRITICAL: Enable PDO exchange BEFORE requesting OP!
    // The slave's PDI watchdog starts in SAFE_OP and expects process data.
    // We must start sending PDO data now to prevent watchdog timeout.
    TETHER_LOGI(TAG, "{}: Enabling PDO exchange in SAFE_OP (pre-OP)", logPrefix().c_str());
    if (m_master) {
        m_master->pdoForSlave(m_slave_index).resetStats();
        m_master->dc().setPDOEnabled(true);
    }

    // Give the PDO exchange a moment to send some frames before OP request
    Tether::Platform::Clock::instance().delayMilliseconds(200);

    // DIAGNOSTIC: Read back SM2/SM3 and verify they're correct before OP request
    if (m_master) {
        for (uint8_t sm = 2; sm <= 3; sm++) {
            uint16_t base = static_cast<uint16_t>(0x0800 + sm * 8);
            uint8_t sm_regs[8] = {0};
            m_master->readRegister(SlaveAddress(m_slave_index), base, sm_regs, 8, 200);
            uint16_t addr = sm_regs[0] | (sm_regs[1] << 8);
            uint16_t len  = sm_regs[2] | (sm_regs[3] << 8);
            uint8_t  ctrl = sm_regs[4];
            uint8_t  stat = sm_regs[5];
            uint8_t  act  = sm_regs[6];
            uint8_t  pdi  = sm_regs[7];
            TETHER_LOGI(TAG, "{}: SM{} readback: Addr=0x{:04X} Len={} Ctrl=0x{:02X} Status=0x{:02X} Act=0x{:02X} PDI=0x{:02X}",
                     logPrefix().c_str(), sm, addr, len, ctrl, stat, act, pdi);
        }
        // Read watchdog divider/counter
        uint16_t wdt_status = 0;
        m_master->readRegister(SlaveAddress(m_slave_index), 0x0440, wdt_status, 200);
        TETHER_LOGI(TAG, "{}: Watchdog Status=0x{:04X}", logPrefix().c_str(), wdt_status);
    }

    // DIAGNOSTIC: Read PDO assignment and mode SDOs before OP request
    if (m_master) {
        m_master->sdoManager(m_slave_index).setDiagEnabled(true);
        auto sm2_count_res = m_master->sdoManager(m_slave_index).readU8( 0x1C12, 0, {.timeout_ms = m_sdo_timeout_ms});
        if (sm2_count_res.has_value())
            TETHER_LOGI(TAG, "{}: 0x1C12:0 (SM2 PDO count) = {}", logPrefix().c_str(), sm2_count_res.value());
        else
            TETHER_LOGW(TAG, "{}: Failed to read 0x1C12:0", logPrefix().c_str());

        auto sm2_pdo_res = m_master->sdoManager(m_slave_index).readU16( 0x1C12, 1, {.timeout_ms = m_sdo_timeout_ms});
        if (sm2_pdo_res.has_value())
            TETHER_LOGI(TAG, "{}: 0x1C12:1 (SM2 RxPDO) = 0x{:04X}", logPrefix().c_str(), sm2_pdo_res.value());
        else
            TETHER_LOGW(TAG, "{}: Failed to read 0x1C12:1", logPrefix().c_str());

        if (sm2_count_res.has_value() && sm2_count_res.value() >= 2) {
            auto sm2_pdo2_res = m_master->sdoManager(m_slave_index).readU16( 0x1C12, 2, {.timeout_ms = m_sdo_timeout_ms});
            if (sm2_pdo2_res.has_value())
                TETHER_LOGI(TAG, "{}: 0x1C12:2 (SM2 RxPDO #2) = 0x{:04X}", logPrefix().c_str(), sm2_pdo2_res.value());
            else
                TETHER_LOGW(TAG, "{}: Failed to read 0x1C12:2", logPrefix().c_str());
        }

        auto sm3_count_res = m_master->sdoManager(m_slave_index).readU8( 0x1C13, 0, {.timeout_ms = m_sdo_timeout_ms});
        if (sm3_count_res.has_value())
            TETHER_LOGI(TAG, "{}: 0x1C13:0 (SM3 PDO count) = {}", logPrefix().c_str(), sm3_count_res.value());
        else
            TETHER_LOGW(TAG, "{}: Failed to read 0x1C13:0", logPrefix().c_str());

        auto sm3_pdo_res = m_master->sdoManager(m_slave_index).readU16( 0x1C13, 1, {.timeout_ms = m_sdo_timeout_ms});
        if (sm3_pdo_res.has_value())
            TETHER_LOGI(TAG, "{}: 0x1C13:1 (SM3 TxPDO) = 0x{:04X}", logPrefix().c_str(), sm3_pdo_res.value());
        else
            TETHER_LOGW(TAG, "{}: Failed to read 0x1C13:1", logPrefix().c_str());

        if (sm3_count_res.has_value() && sm3_count_res.value() >= 2) {
            auto sm3_pdo2_res = m_master->sdoManager(m_slave_index).readU16( 0x1C13, 2, {.timeout_ms = m_sdo_timeout_ms});
            if (sm3_pdo2_res.has_value())
                TETHER_LOGI(TAG, "{}: 0x1C13:2 (SM3 TxPDO #2) = 0x{:04X}", logPrefix().c_str(), sm3_pdo2_res.value());
            else
                TETHER_LOGW(TAG, "{}: Failed to read 0x1C13:2", logPrefix().c_str());
        }

        auto mode_op_res = m_master->sdoManager(m_slave_index).readU8( 0x6060, 0, {.timeout_ms = m_sdo_timeout_ms});
        if (mode_op_res.has_value())
            TETHER_LOGI(TAG, "{}: 0x6060 (Modes of Operation) = {}", logPrefix().c_str(), (int8_t)mode_op_res.value());

        auto mode_disp_res = m_master->sdoManager(m_slave_index).readU8( 0x6061, 0, {.timeout_ms = m_sdo_timeout_ms});
        if (mode_disp_res.has_value())
            TETHER_LOGI(TAG, "{}: 0x6061 (Mode Display) = {}", logPrefix().c_str(), (int8_t)mode_disp_res.value());

        auto supported_modes_res = m_master->sdoManager(m_slave_index).readU32( 0x6502, 0, {.timeout_ms = m_sdo_timeout_ms});
        if (supported_modes_res.has_value())
            TETHER_LOGI(TAG, "{}: 0x6502 (Supported Modes) = 0x{:08X}", logPrefix().c_str(), supported_modes_res.value());

        auto rxpdo_count_res = m_master->sdoManager(m_slave_index).readU8( 0x1600, 0, {.timeout_ms = m_sdo_timeout_ms});
        if (rxpdo_count_res.has_value())
            TETHER_LOGI(TAG, "{}: 0x1600:0 (RxPDO entry count) = {}", logPrefix().c_str(), rxpdo_count_res.value());

        auto txpdo_count_res = m_master->sdoManager(m_slave_index).readU8( 0x1A00, 0, {.timeout_ms = m_sdo_timeout_ms});
        if (txpdo_count_res.has_value())
            TETHER_LOGI(TAG, "{}: 0x1A00:0 (TxPDO entry count) = {}", logPrefix().c_str(), txpdo_count_res.value());
        m_master->sdoManager(m_slave_index).setDiagEnabled(false);
    }

    // Clear any pending error by writing Error Acknowledge + OP (0x18)
    // Some slaves need the ACK bit before accepting OP.
    if (m_master) {
        // First ack any error in current state
        uint16_t ack = static_cast<uint16_t>(ECState::SafeOp) | 0x10;  // SAFE_OP + ACK
        m_master->writeRegister(SlaveAddress(m_slave_index), 0x0120, ack);  // 0x0120 = AL_CONTROL
        Tether::Platform::Clock::instance().delayMilliseconds(50);
    }

    // SAFE_OP -> OP
    if (!gotoOp()) {
        TETHER_LOGE(TAG, "{}: Failed to reach OP", logPrefix().c_str());
        // Disable PDO on failure
        if (m_master) {
            m_master->dc().setPDOEnabled(false);
        }
        return false;
    }
    Tether::Platform::Clock::instance().delayMilliseconds(100);

    TETHER_LOGI(TAG, "{}: Successfully transitioned to OP", logPrefix().c_str());
    return true;
}

// ============================================================================
// transitionToOp — multi-PDO-per-SM variant
// ============================================================================

bool CiA402Drive::transitionToOp(const Slave::MultiPDOAssignment& assignment) {
    TETHER_LOGI(TAG, "{}: Beginning transition to OP (multi-PDO, {} SM configs)",
                logPrefix().c_str(), assignment.sm_configs.size());

    // Check if slave is already in PRE_OP. If so, skip gotoPreOp() to avoid
    // re-initializing the slave which would reset all CoE objects.
    uint8_t current_state = 0;
    bool need_preop = true;
    if (m_master && m_master->readSlaveApplicationLayerState(m_slave_index, current_state)) {
        if (current_state == static_cast<uint8_t>(ECState::PreOp)) {
            TETHER_LOGI(TAG, "{}: Already in PRE_OP (0x{:02X}), skipping gotoPreOp()",
                     logPrefix().c_str(), current_state);
            need_preop = false;
        } else {
            TETHER_LOGI(TAG, "{}: Current state=0x{:02X}, requesting PRE_OP",
                     logPrefix().c_str(), current_state);
        }
    }

    if (need_preop) {
        if (!gotoPreOp()) {
            TETHER_LOGE(TAG, "{}: Failed to reach PRE_OP", logPrefix().c_str());
            return false;
        }
        Tether::Platform::Clock::instance().delayMilliseconds(100);
    }

    // Configure multi-PDO sync managers and FMMUs via Slave::configureMultiPDOs.
    // This writes SM registers, PDO assignments (0x1C12/0x1C13), and FMMU
    // configuration — all the work that the SII-based path in the single-PDO
    // transitionToOp(bool) would otherwise do.
    auto& slave = m_master->slave(m_slave_index);
    const auto pdo_err = slave.configureMultiPDOs(assignment);
    if (pdo_err != SlaveError::Ok) {
        TETHER_LOGE(TAG, "{}: configureMultiPDOs failed: {}",
                    logPrefix().c_str(), slaveErrorToString(pdo_err));
        return false;
    }
    TETHER_LOGI(TAG, "{}: Multi-PDO SM/FMMU configuration complete", logPrefix().c_str());

    // Compute total Rx/Tx sizes from the assignment and set internal buffer
    // sizes.  We use setPDOBufferSizes() (not assignFixedPDOs()) because
    // configureMultiPDOs() already wrote the PDO assignments (0x1C12/0x1C13)
    // via the multi-PDO path.  Calling assignFixedPDOs() here would
    // overwrite the multi-PDO assignment with a single-PDO assignment,
    // destroying the FSoE PDO mapping (e.g. removing 0x1700 from SM2 and
    // 0x1B00 from SM3, leaving only the motion PDOs 0x1600/0x1A00).
    uint16_t total_rx = 0, total_tx = 0;
    uint16_t rx_index = 0, tx_index = 0;
    for (const auto& sm_cfg : assignment.sm_configs) {
        const bool is_write = (sm_cfg.control_byte & 0x04) != 0;
        for (const auto& pdo_region : sm_cfg.pdo_mappings) {
            if (is_write) {
                total_rx += pdo_region.size_bytes;
                if (rx_index == 0) rx_index = pdo_region.pdo_index;
            } else {
                total_tx += pdo_region.size_bytes;
                if (tx_index == 0) tx_index = pdo_region.pdo_index;
            }
        }
    }

    if (total_rx > 0 || total_tx > 0) {
        setPDOBufferSizes(rx_index, tx_index, total_rx, total_tx);
        if (!registerPDOBuffers()) {
            TETHER_LOGE(TAG, "{}: Failed to register PDO buffers", logPrefix().c_str());
            return false;
        }
        TETHER_LOGI(TAG, "{}: PDO buffers registered: Rx={} bytes, Tx={} bytes",
                    logPrefix().c_str(), total_rx, total_tx);
    }

    // Do NOT call configureProcessDataSyncManagersFromSii() or re-write SM
    // registers here — configureMultiPDOs() already configured SMs and FMMUs
    // correctly.  Proceed directly to SAFE_OP → OP.
    return transitionSafeOpToOp();
}

// ============================================================================
// CiA 402 State Machine
// ============================================================================

DriveState CiA402Drive::getDriveState() {
    uint16_t sw;
    if (!readStatusword(sw)) {
        return DriveState::Unknown;
    }
    return decodeDriveState(sw);
}

uint16_t CiA402Drive::getStatusword() {
    uint16_t sw;
    if (readStatusword(sw)) {
        return sw;
    }
    return m_statusword;
}

bool CiA402Drive::enable(uint32_t timeout_ms) {
    // Reset any fault first
    DriveState state = getDriveState();
    if (state == DriveState::Fault) {
        TETHER_LOGI(TAG, "{}: Resetting fault", logPrefix().c_str());
        if (!resetFault()) {
            return false;
        }
        Tether::Platform::Clock::instance().delayMilliseconds(100);
        state = getDriveState();
    }
    
    // Shutdown (Ready to Switch On)
    if (state == DriveState::SwitchOnDisabled) {
        m_controlword = 0x0006;  // Shutdown
        writeControlword(m_controlword);
        if (!waitForDriveState(DriveState::ReadyToSwitchOn, timeout_ms)) {
            return false;
        }
        state = getDriveState();
    }
    
    // Switch On
    if (state == DriveState::ReadyToSwitchOn) {
        m_controlword = 0x0007;  // Switch On
        writeControlword(m_controlword);
        if (!waitForDriveState(DriveState::SwitchedOn, timeout_ms)) {
            return false;
        }
        state = getDriveState();
    }
    
    // Enable Operation
    if (state == DriveState::SwitchedOn) {
        m_controlword = 0x000F;  // Enable Operation
        writeControlword(m_controlword);
        if (!waitForDriveState(DriveState::OperationEnabled, timeout_ms)) {
            return false;
        }
    }
    
    TETHER_LOGI(TAG, "{}: Drive enabled successfully", logPrefix().c_str());
    return true;
}

bool CiA402Drive::disable() {
    m_controlword = 0x0000;  // Disable voltage
    return writeControlword(m_controlword);
}

bool CiA402Drive::quickStop() {
    m_controlword = 0x0002;  // Quick stop
    return writeControlword(m_controlword);
}

bool CiA402Drive::resetFault() {
    // Rising edge on fault reset bit
    m_controlword &= ~0x0080;
    writeControlword(m_controlword);
    Tether::Platform::Clock::instance().delayMilliseconds(10);
    m_controlword |= 0x0080;
    writeControlword(m_controlword);
    Tether::Platform::Clock::instance().delayMilliseconds(10);
    m_controlword &= ~0x0080;
    return writeControlword(m_controlword);
}

bool CiA402Drive::isEnabled() {
    return getDriveState() == DriveState::OperationEnabled;
}

bool CiA402Drive::isFaulted() {
    DriveState state = getDriveState();
    return state == DriveState::Fault || state == DriveState::FaultReactionActive;
}

bool CiA402Drive::isTargetReached() {
    uint16_t sw = getStatusword();
    return (sw & CiA402::StatuswordBits::TargetReached) != 0;
}

// ============================================================================
// DynaDrive Custom FSM (rsl_drive_sdk / ANYdrive)
// ============================================================================

const char* CiA402Drive::getDynaDriveStateName(DynaDriveState state) {
    return DynaDriveController::getStateName(state);
}

CiA402Drive::DynaDriveState CiA402Drive::decodeDynaDriveState(uint32_t statusword) {
    return DynaDriveController::decodeState(statusword);
}

bool CiA402Drive::readDynaDriveStatusword(uint32_t& statusword) {
    DynaDriveController ctrl(*m_master, m_slave_index, m_sdo_timeout_ms);
    return ctrl.readStatusword(statusword);
}

bool CiA402Drive::sendDynaDriveControlword(EtherCAT::Drives::Registers::DynaDrive::Controlword::Options controlword) {
    DynaDriveController ctrl(*m_master, m_slave_index, m_sdo_timeout_ms);
    return ctrl.sendControlword(controlword);
}

bool CiA402Drive::enableDynaDrive(uint32_t timeout_ms) {
    DynaDriveController ctrl(*m_master, m_slave_index, m_sdo_timeout_ms);
    return ctrl.enable(timeout_ms);
}

bool CiA402Drive::disableDynaDrive() {
    DynaDriveController ctrl(*m_master, m_slave_index, m_sdo_timeout_ms);
    return ctrl.disable();
}

bool CiA402Drive::isDynaDriveControlOp() {
    DynaDriveController ctrl(*m_master, m_slave_index, m_sdo_timeout_ms);
    return ctrl.isControlOp();
}

bool CiA402Drive::waitForDriveState(DriveState target, uint32_t timeout_ms) {
    uint32_t elapsed = 0;
    const uint32_t poll_interval = 10;
    
    while (elapsed < timeout_ms) {
        DriveState current = getDriveState();
        if (current == target) {
            return true;
        }
        if (current == DriveState::Fault) {
            TETHER_LOGE(TAG, "{}: Fault during state transition", logPrefix().c_str());
            return false;
        }
        Tether::Platform::Clock::instance().delayMilliseconds(poll_interval);
        elapsed += poll_interval;
    }
    
    TETHER_LOGE(TAG, "{}: Timeout waiting for state {}", 
             logPrefix().c_str(), getDriveStateName(target));
    return false;
}

bool CiA402Drive::writeControlword(uint16_t controlword) {
    m_controlword = controlword;
    // Fast path: when the drive is in OP with RxPDOs registered, write the
    // controlword directly into the PDO buffer instead of issuing a slow SDO
    // transaction.  The cyclic PDO exchange picks up the buffer contents.
    if (m_pdo_registered && getECState() == ECState::Op &&
        m_rxpdo_size >= sizeof(controlword)) {
        std::memcpy(m_rxpdo_buffer, &controlword, sizeof(controlword));
        return true;
    }
    auto result = m_master->sdoManager(m_slave_index).writeU16(
        static_cast<uint16_t>(CiA402::Register::Controlword), 0, controlword,
        {.timeout_ms = m_sdo_timeout_ms});
    return result.has_value();
}

// Public wrapper to allow immediate SDO write from other modules
bool CiA402Drive::sendControlwordSDO(uint16_t controlword) {
    m_controlword = controlword;
    auto result = m_master->sdoManager(m_slave_index).writeU16(
        static_cast<uint16_t>(CiA402::Register::Controlword), 0, controlword,
        {.timeout_ms = m_sdo_timeout_ms});
    return result.has_value();
}

bool CiA402Drive::readStatusword(uint16_t& statusword) {
    // Fast path: when the drive is in OP with TxPDOs registered, read the
    // statusword directly from the PDO buffer (populated by the cyclic PDO
    // exchange) instead of issuing a slow SDO transaction.
    if (m_pdo_registered && getECState() == ECState::Op &&
        m_txpdo_size >= sizeof(statusword)) {
        std::memcpy(&statusword, m_txpdo_buffer, sizeof(statusword));
        m_statusword = statusword;
        return true;
    }
    auto result = m_master->sdoManager(m_slave_index).readU16(
        static_cast<uint16_t>(CiA402::Register::Statusword), 0,
        {.timeout_ms = m_sdo_timeout_ms});
    if (!result.has_value()) return false;
    statusword = result.value();
    m_statusword = statusword;
    return true;
}

// ============================================================================
// Operating Mode
// ============================================================================

bool CiA402Drive::setOperatingMode(int8_t mode) {
    // Use PDO by default if the offset has been configured; otherwise SDO.
    if (m_opmode_pdo_offset >= 0) {
        return setOperatingModePDO(mode);
    }
    return setOperatingModeSDO(mode);
}

bool CiA402Drive::setOperatingModePDO(int8_t mode) {
    if (m_opmode_pdo_offset < 0) {
        TETHER_LOGE(TAG, "{}: setOperatingModePDO called but PDO offset not configured",
                    logPrefix().c_str());
        return false;
    }
    const size_t offset = static_cast<size_t>(m_opmode_pdo_offset);
    if (offset + 1 > kMaxPDOBufferSize) {
        TETHER_LOGE(TAG, "{}: setOperatingModePDO offset {} out of bounds",
                    logPrefix().c_str(), offset);
        return false;
    }
    m_rxpdo_buffer[offset] = static_cast<uint8_t>(mode);
    TETHER_LOGI(TAG, "{}: Operating mode set to {} ({}) via PDO (offset={})",
                logPrefix().c_str(),
                CiA402::getOperatingModeName(mode),
                static_cast<int>(mode), offset);
    return true;
}

bool CiA402Drive::setOperatingModeSDO(int8_t mode) {
    uint8_t umode = static_cast<uint8_t>(mode);
    auto write_res = m_master->sdoManager(m_slave_index).writeU8(
        static_cast<uint16_t>(CiA402::Register::ModesOfOperation), 0, umode,
        {.timeout_ms = m_sdo_timeout_ms});
    if (write_res.has_value()) {
        // Read back mode display to verify
        uint8_t mode_display = 0;
        Tether::Platform::Clock::instance().delayMilliseconds(50); // Give drive time to process
        auto read_res = m_master->sdoManager(m_slave_index).readU8(
            static_cast<uint16_t>(CiA402::Register::ModesOfOperationDisplay), 0,
            {.timeout_ms = m_sdo_timeout_ms});
        if (read_res.has_value()) {
            mode_display = read_res.value();
            if (static_cast<int8_t>(mode_display) == mode) {
                TETHER_LOGI(TAG, "{}: Operating mode set to {} ({}), readback successful",
                         logPrefix().c_str(), CiA402::getOperatingModeName(mode), mode);
            } else {
                TETHER_LOGW(TAG, "{}: Operating mode set to {} ({}), readback mismatch: {} ({})",
                         logPrefix().c_str(), CiA402::getOperatingModeName(mode), mode,
                         CiA402::getOperatingModeName(static_cast<int8_t>(mode_display)),
                         (int8_t)mode_display);
            }
        } else {
            TETHER_LOGW(TAG, "{}: Operating mode set to {} ({}), readback failed",
                     logPrefix().c_str(), CiA402::getOperatingModeName(mode), mode);
        }
        return true;
    }

    // Diagnostic: SDO write failed. Try to read error registers (0x1001, 0x1003) to get more info
    TETHER_LOGE(TAG, "{}: Failed to set operating mode via SDO (index 0x6060)", logPrefix().c_str());

    auto err_reg_res = m_master->sdoManager(m_slave_index).readU8(
        static_cast<uint16_t>(0x1001), 0,
        {.timeout_ms = m_sdo_timeout_ms});
    if (err_reg_res.has_value()) {
        TETHER_LOGW(TAG, "{}: Error register (0x1001) = 0x{:02X}", logPrefix().c_str(), err_reg_res.value());
    } else {
        TETHER_LOGW(TAG, "{}: Unable to read Error Register (0x1001)", logPrefix().c_str());
    }

    // Manufacturer error log (0x1003) - subindex 0 = number of errors
    auto mfr_count_res = m_master->sdoManager(m_slave_index).readU8(
        static_cast<uint16_t>(0x1003), 0,
        {.timeout_ms = m_sdo_timeout_ms});
    if (mfr_count_res.has_value()) {
        uint8_t mfr_err_count = mfr_count_res.value();
        TETHER_LOGW(TAG, "{}: Manufacturer Error count (0x1003) = {}", logPrefix().c_str(), (unsigned)mfr_err_count);
        for (uint8_t i = 1; i <= mfr_err_count && i < 16; ++i) {
            auto err_res = m_master->sdoManager(m_slave_index).readU32(
                static_cast<uint16_t>(0x1003), i,
                {.timeout_ms = m_sdo_timeout_ms});
            if (err_res.has_value()) {
                TETHER_LOGW(TAG, "{}: Manufacturer Error[{}] = 0x{:08X}", logPrefix().c_str(), (unsigned)i, err_res.value());
            }
        }
    } else {
        TETHER_LOGW(TAG, "{}: Unable to read Manufacturer Error Log (0x1003)", logPrefix().c_str());
    }

    // Additional diagnostic: submit an asynchronous SDO request for the same
    // ModesOfOperation object to capture full SDOResponse (abort code and any
    // returned/echoed data). This helps determine why the sync write failed.
    {
        SDO::SDORequest dreq = {};
        dreq.slave_index = m_slave_index;
        dreq.index = static_cast<uint16_t>(CiA402::Register::ModesOfOperation);
        dreq.subindex = 0;
        dreq.operation = SDO::SDOOperation::Download;
        dreq.data_size = 1;
        dreq.timeout_ms = m_sdo_timeout_ms;
        dreq.data[0] = umode;

#ifdef TETHER_DIAG_SDO_IO
        m_master->sdoManager(m_slave_index).setDiagEnabled(true);
#endif
        uint32_t req_id = m_master->sdoManager(m_slave_index).queueRequest(dreq);
        if (req_id == 0) {
            TETHER_LOGW(TAG, "{}: Diagnostic SDO queue failed", logPrefix().c_str());
#ifdef TETHER_DIAG_SDO_IO
            m_master->sdoManager(m_slave_index).setDiagEnabled(false);
#endif
        } else {
            SDO::SDOResponse resp = {};
            const uint32_t poll_ms = 100;
            uint32_t waited = 0;
            const uint32_t max_wait = (dreq.timeout_ms > 0) ? dreq.timeout_ms : SDO::kDefaultSDOTimeoutMs;
            while (waited < max_wait) {
                Tether::Platform::Clock::instance().delayMilliseconds(poll_ms);
                waited += poll_ms;
                if (m_master->sdoManager(m_slave_index).getResponse(req_id, resp)) {
                    TETHER_LOGI(TAG, "{}: Diagnostic SDO response status={} abort=0x{:08X} size={}",
                             logPrefix().c_str(), (unsigned)resp.status, (uint32_t)resp.abort_code, (unsigned)resp.data_size);
                    if (resp.data_size > 0) {
                        char hex[128] = {0};
                        const size_t n = std::min(resp.data_size, (size_t)40);
                        size_t hex_len = 0;
                        for (size_t i = 0; i < n && hex_len < sizeof(hex) - 1; ++i) {
                            int written = snprintf(hex + hex_len, sizeof(hex) - hex_len, "%02X", resp.data[i]);
                            if (written < 0 || written >= (int)(sizeof(hex) - hex_len)) break;
                            hex_len += written;
                            if (i + 1 < n && hex_len < sizeof(hex) - 1) {
                                written = snprintf(hex + hex_len, sizeof(hex) - hex_len, " ");
                                if (written < 0 || written >= (int)(sizeof(hex) - hex_len)) break;
                                hex_len += written;
                            }
                        }
                        TETHER_LOGI(TAG, "{}: Diagnostic SDO data: {}", logPrefix().c_str(), hex);
                    }
#ifdef TETHER_DIAG_SDO_IO
                    m_master->sdoManager(m_slave_index).setDiagEnabled(false);
#endif
                    break;
                }
            }
            if (waited >= max_wait) {
                TETHER_LOGW(TAG, "{}: Diagnostic SDO response timed out", logPrefix().c_str());
#ifdef TETHER_DIAG_SDO_IO
                m_master->sdoManager(m_slave_index).setDiagEnabled(false);
#endif
            }
        }
    }

    return false;
}

int8_t CiA402Drive::getOperatingMode() {
    auto result = m_master->sdoManager(m_slave_index).readU8(
        static_cast<uint16_t>(CiA402::Register::ModesOfOperationDisplay), 0,
        {.timeout_ms = m_sdo_timeout_ms});
    if (result.has_value()) {
        return static_cast<int8_t>(result.value());
    }
    return 0;  // Unknown
}

// ============================================================================
// Homing
// ============================================================================

bool CiA402Drive::setHomingMethod(int8_t method) {
    uint8_t umethod = static_cast<uint8_t>(method);
    auto result = m_master->sdoManager(m_slave_index).writeU8(
        static_cast<uint16_t>(CiA402::Register::HomingMethod), 0, umethod,
        {.timeout_ms = m_sdo_timeout_ms});
    return result.has_value();
}

bool CiA402Drive::homeToCurrentPosition(int32_t home_offset) {
    TETHER_LOGI(TAG, "{}: Homing to current position (offset={})", 
             logPrefix().c_str(), (long)home_offset);
    
    // Set home offset
    if (home_offset != 0) {
        m_master->sdoManager(m_slave_index).writeI32(
            static_cast<uint16_t>(CiA402::Register::HomeOffset), 0, home_offset,
            {.timeout_ms = m_sdo_timeout_ms});
    }
    
    // Set homing method 35 (current position = home)
    if (!setHomingMethod(CiA402::HomingMethodValue::CurrentPosition)) {
        TETHER_LOGE(TAG, "{}: Failed to set homing method", logPrefix().c_str());
        return false;
    }
    
    // Switch to homing mode
    if (!setModeHM()) {
        TETHER_LOGE(TAG, "{}: Failed to set homing mode", logPrefix().c_str());
        return false;
    }
    
    // Execute homing
    return executeHoming(5000);  // 5 second timeout for current position homing
}

bool CiA402Drive::executeHoming(uint32_t timeout_ms) {
    // Start homing (set bit 4)
    m_controlword |= CiA402::ControlwordBits::HomingOperationStart;
    writeControlword(m_controlword);
    
    // Wait for homing complete
    uint32_t elapsed = 0;
    const uint32_t poll_interval = 50;
    
    while (elapsed < timeout_ms) {
        if (isHomingComplete()) {
            TETHER_LOGI(TAG, "{}: Homing complete", logPrefix().c_str());
            m_controlword &= ~CiA402::ControlwordBits::HomingOperationStart;
            writeControlword(m_controlword);
            return true;
        }
        if (hasHomingError()) {
            TETHER_LOGE(TAG, "{}: Homing error", logPrefix().c_str());
            m_controlword &= ~CiA402::ControlwordBits::HomingOperationStart;
            writeControlword(m_controlword);
            return false;
        }
        Tether::Platform::Clock::instance().delayMilliseconds(poll_interval);
        elapsed += poll_interval;
    }
    
    TETHER_LOGE(TAG, "{}: Homing timeout", logPrefix().c_str());
    return false;
}

bool CiA402Drive::isHomingComplete() {
    uint16_t sw = getStatusword();
    return (sw & CiA402::StatuswordBits::HomingAttained) != 0;
}

bool CiA402Drive::hasHomingError() {
    uint16_t sw = getStatusword();
    return (sw & CiA402::StatuswordBits::HomingError) != 0;
}

} // namespace EtherCAT
