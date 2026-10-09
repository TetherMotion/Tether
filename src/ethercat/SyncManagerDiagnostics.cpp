/**
 * @file SyncManagerDiagnostics.cpp
 * @brief SyncManager — mailbox/PDO hardware debug dumps.
 *
 * TU split out of SyncManager.cpp.
 */

#include "tether/ethercat/SyncManager.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/platform/Platform.hpp"

#include <cstdio>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <bit>

namespace EtherCAT {

// ============================================================================
// Debug function: mailbox hardware configuration
// ============================================================================

void debugMailboxConfiguration(Master& master, uint16_t slave_index, const char* tag) {
    TETHER_LOGI(tag, "\n╔══════════════════════════════════════════════════════════════╗\n║  Mailbox Hardware Configuration Debug ({})            ║\n╚══════════════════════════════════════════════════════════════╝\n", master.slaveLogPrefix(slave_index).c_str());

    auto& slave = master.slave(slave_index);

    // Helper to decode control register bits
    auto decode_control = [](const SyncManager::SMControlReg& ctrl) -> std::string {
        std::ostringstream oss;
        const uint8_t mode = ctrl.mode;
        if (mode == static_cast<uint8_t>(SyncManager::SMMode::Buffered)) oss << "BUFFERED";
        else if (mode == static_cast<uint8_t>(SyncManager::SMMode::Mailbox)) oss << "MAILBOX";
        else oss << "UNKNOWN(0x" << std::hex << (int)mode << ")";

        oss << " ";
        if (ctrl.direction) oss << "DIR_WRITE ";
        else oss << "DIR_READ ";

        if (ctrl.ecat_irq)   oss << "IRQ_ECAT ";
        if (ctrl.pdi_irq)    oss << "IRQ_PDI ";
        if (ctrl.watchdog)   oss << "WATCHDOG ";
        if (ctrl.repeat_req) oss << "REPEAT_REQ ";

        return oss.str();
    };

    // Read SM0 (Mailbox Receive / Master→Slave)
    TETHER_LOGI(tag, "\n📋 SM0 (Mailbox Receive / Master→Slave)");
    auto sm0 = slave.sm(0);
    auto hw0 = sm0.readHardwareConfig();
    if (hw0.read_ok) {
        TETHER_LOGI(tag, "  Physical Register Base: 0x{:04X}", (unsigned)sm0.physRegisterBase());
        TETHER_LOGI(tag, "  Start Address:          0x{:04X}", (unsigned)hw0.start_addr);
        TETHER_LOGI(tag, "  Length:                 {} bytes", (unsigned)hw0.length);
        TETHER_LOGI(tag, "  Control Register (0x{:02X}): {}", (unsigned)std::bit_cast<uint8_t>(hw0.control), decode_control(hw0.control).c_str());
        TETHER_LOGI(tag, "  Status Register:        0x{:02X}", (unsigned)std::bit_cast<uint8_t>(hw0.status));
        TETHER_LOGI(tag, "  Activate Register:      0x{:02X} ({})", (unsigned)std::bit_cast<uint8_t>(hw0.activate), hw0.isEnabled() ? "ENABLED" : "disabled");
        TETHER_LOGI(tag, "  PDI Control:            0x{:02X}", (unsigned)std::bit_cast<uint8_t>(hw0.pdi_ctrl));
    } else {
        TETHER_LOGE(tag, "  ❌ Failed to read SM0 hardware registers");
    }

    // Read SM1 (Mailbox Send / Slave→Master)
    TETHER_LOGI(tag, "\n📋 SM1 (Mailbox Send / Slave→Master)");
    auto sm1 = slave.sm(1);
    auto hw1 = sm1.readHardwareConfig();
    if (hw1.read_ok) {
        TETHER_LOGI(tag, "  Physical Register Base: 0x{:04X}", (unsigned)sm1.physRegisterBase());
        TETHER_LOGI(tag, "  Start Address:          0x{:04X}", (unsigned)hw1.start_addr);
        TETHER_LOGI(tag, "  Length:                 {} bytes", (unsigned)hw1.length);
        TETHER_LOGI(tag, "  Control Register (0x{:02X}): {}", (unsigned)std::bit_cast<uint8_t>(hw1.control), decode_control(hw1.control).c_str());
        TETHER_LOGI(tag, "  Status Register:        0x{:02X}", (unsigned)std::bit_cast<uint8_t>(hw1.status));
        TETHER_LOGI(tag, "  Activate Register:      0x{:02X} ({})", (unsigned)std::bit_cast<uint8_t>(hw1.activate), hw1.isEnabled() ? "ENABLED" : "disabled");
        TETHER_LOGI(tag, "  PDI Control:            0x{:02X}", (unsigned)std::bit_cast<uint8_t>(hw1.pdi_ctrl));
    } else {
        TETHER_LOGE(tag, "  ❌ Failed to read SM1 hardware registers");
    }

    // Read SM Watchdog status
    TETHER_LOGI(tag, "\n📋 SM Watchdog Status (Register 0x0440)");
    uint8_t wd[2] = {0};
    if (master.readRegister(EtherCAT::SlaveAddress(slave_index), EtherCAT::SyncManager::kWatchdogStatusReg, wd, sizeof(wd), 200)) {
        const uint16_t wdStatus = static_cast<uint16_t>(wd[0] | (static_cast<uint16_t>(wd[1]) << 8));
        TETHER_LOGI(tag, "  Watchdog Status: 0x{:04X} {}", (unsigned)wdStatus, (wdStatus == 0) ? "(OK)" : "(EXPIRED!)");
    } else {
        TETHER_LOGW(tag, "  ⚠ Failed to read SM watchdog status register");
    }

    // Read CommType via SDO (if available)
    TETHER_LOGI(tag, "\n📋 SM Communication Types (via SDO 0x1C00)");
    uint8_t commType0 = 0xFF, commType1 = 0xFF;
    SlaveError err0 = sm0.readCommType(commType0);
    SlaveError err1 = sm1.readCommType(commType1);

    if (err0 == SlaveError::Ok) {
        const char* type0 = (commType0 == 0x01) ? "MailboxReceive" :
                           (commType0 == 0x02) ? "MailboxSend" :
                           (commType0 == 0x03) ? "ProcessOutput" :
                           (commType0 == 0x04) ? "ProcessInput" :
                           (commType0 == 0x00) ? "NotUsed" : "Unknown";
        TETHER_LOGI(tag, "  SM0 CommType: 0x{:02X} ({})", (unsigned)commType0, type0);
    } else {
        TETHER_LOGW(tag, "  ⚠ SM0 CommType read failed (SDO error)");
    }

    if (err1 == SlaveError::Ok) {
        const char* type1 = (commType1 == 0x01) ? "MailboxReceive" :
                           (commType1 == 0x02) ? "MailboxSend" :
                           (commType1 == 0x03) ? "ProcessOutput" :
                           (commType1 == 0x04) ? "ProcessInput" :
                           (commType1 == 0x00) ? "NotUsed" : "Unknown";
        TETHER_LOGI(tag, "  SM1 CommType: 0x{:02X} ({})", (unsigned)commType1, type1);
    } else {
        TETHER_LOGW(tag, "  ⚠ SM1 CommType read failed (SDO error)");
    }
}

// ============================================================================
// Debug function: PDO (non-mailbox) sync manager hardware configuration
// ============================================================================

void debugPDOSyncManagerConfiguration(Master& master, uint16_t slave_index, const char* tag) {
    TETHER_LOGI(tag, "\n╔══════════════════════════════════════════════════════════════╗\n║  PDO Sync Manager Configuration Debug ({})             ║\n╚══════════════════════════════════════════════════════════════╝\n", master.slaveLogPrefix(slave_index).c_str());

    auto& slave = master.slave(slave_index);

    // Helper to decode control register bits
    auto decode_control = [](const SyncManager::SMControlReg& ctrl) -> std::string {
        std::ostringstream oss;
        const uint8_t mode = ctrl.mode;
        if (mode == static_cast<uint8_t>(SyncManager::SMMode::Buffered)) oss << "BUFFERED";
        else if (mode == static_cast<uint8_t>(SyncManager::SMMode::Mailbox)) oss << "MAILBOX";
        else oss << "UNKNOWN(0x" << std::hex << (int)mode << ")";

        oss << " ";
        if (ctrl.direction) oss << "DIR_WRITE ";
        else oss << "DIR_READ ";

        if (ctrl.ecat_irq)   oss << "IRQ_ECAT ";
        if (ctrl.pdi_irq)    oss << "IRQ_PDI ";
        if (ctrl.watchdog)   oss << "WATCHDOG ";
        if (ctrl.repeat_req) oss << "REPEAT_REQ ";

        return oss.str();
    };

    // Read SM2 (Process Data Output / RxPDO / Master→Slave)
    TETHER_LOGI(tag, "\n📋 SM2 (Process Data Output / RxPDO / Master→Slave)");
    auto sm2 = slave.sm(2);
    auto hw2 = sm2.readHardwareConfig();
    if (hw2.read_ok) {
        TETHER_LOGI(tag, "  Physical Register Base: 0x{:04X}", (unsigned)sm2.physRegisterBase());
        TETHER_LOGI(tag, "  Start Address:          0x{:04X}", (unsigned)hw2.start_addr);
        TETHER_LOGI(tag, "  Length:                 {} bytes", (unsigned)hw2.length);
        TETHER_LOGI(tag, "  Control Register (0x{:02X}): {}", (unsigned)std::bit_cast<uint8_t>(hw2.control), decode_control(hw2.control).c_str());
        TETHER_LOGI(tag, "  Status Register:        0x{:02X}", (unsigned)std::bit_cast<uint8_t>(hw2.status));
        TETHER_LOGI(tag, "  Activate Register:      0x{:02X} ({})", (unsigned)std::bit_cast<uint8_t>(hw2.activate), hw2.isEnabled() ? "ENABLED" : "disabled");
        TETHER_LOGI(tag, "  PDI Control:            0x{:02X}", (unsigned)std::bit_cast<uint8_t>(hw2.pdi_ctrl));
    } else {
        TETHER_LOGE(tag, "  ❌ Failed to read SM2 hardware registers");
    }

    // Read SM3 (Process Data Input / TxPDO / Slave→Master)
    TETHER_LOGI(tag, "\n📋 SM3 (Process Data Input / TxPDO / Slave→Master)");
    auto sm3 = slave.sm(3);
    auto hw3 = sm3.readHardwareConfig();
    if (hw3.read_ok) {
        TETHER_LOGI(tag, "  Physical Register Base: 0x{:04X}", (unsigned)sm3.physRegisterBase());
        TETHER_LOGI(tag, "  Start Address:          0x{:04X}", (unsigned)hw3.start_addr);
        TETHER_LOGI(tag, "  Length:                 {} bytes", (unsigned)hw3.length);
        TETHER_LOGI(tag, "  Control Register (0x{:02X}): {}", (unsigned)std::bit_cast<uint8_t>(hw3.control), decode_control(hw3.control).c_str());
        TETHER_LOGI(tag, "  Status Register:        0x{:02X}", (unsigned)std::bit_cast<uint8_t>(hw3.status));
        TETHER_LOGI(tag, "  Activate Register:      0x{:02X} ({})", (unsigned)std::bit_cast<uint8_t>(hw3.activate), hw3.isEnabled() ? "ENABLED" : "disabled");
        TETHER_LOGI(tag, "  PDI Control:            0x{:02X}", (unsigned)std::bit_cast<uint8_t>(hw3.pdi_ctrl));
    } else {
        TETHER_LOGE(tag, "  ❌ Failed to read SM3 hardware registers");
    }

    // Read SM Watchdog status
    TETHER_LOGI(tag, "\n📋 SM Watchdog Status (Register 0x0440)");
    uint8_t wd[2] = {0};
    if (master.readRegister(EtherCAT::SlaveAddress(slave_index), EtherCAT::SyncManager::kWatchdogStatusReg, wd, sizeof(wd), 200)) {
        const uint16_t wdStatus = static_cast<uint16_t>(wd[0] | (static_cast<uint16_t>(wd[1]) << 8));
        TETHER_LOGI(tag, "  Watchdog Status: 0x{:04X} {}", (unsigned)wdStatus, (wdStatus == 0) ? "(OK)" : "(EXPIRED!)");
    } else {
        TETHER_LOGW(tag, "  ⚠ Failed to read SM watchdog status register");
    }

    // Read CommType via SDO (if available)
    TETHER_LOGI(tag, "\n📋 SM Communication Types (via SDO 0x1C00)");
    uint8_t commType2 = 0xFF, commType3 = 0xFF;
    SlaveError err2 = sm2.readCommType(commType2);
    SlaveError err3 = sm3.readCommType(commType3);

    if (err2 == SlaveError::Ok) {
        const char* type2 = (commType2 == 0x01) ? "MailboxReceive" :
                           (commType2 == 0x02) ? "MailboxSend" :
                           (commType2 == 0x03) ? "ProcessOutput" :
                           (commType2 == 0x04) ? "ProcessInput" :
                           (commType2 == 0x00) ? "NotUsed" : "Unknown";
        TETHER_LOGI(tag, "  SM2 CommType: 0x{:02X} ({})", (unsigned)commType2, type2);
    } else {
        TETHER_LOGW(tag, "  ⚠ SM2 CommType read failed (SDO error)");
    }

    if (err3 == SlaveError::Ok) {
        const char* type3 = (commType3 == 0x01) ? "MailboxReceive" :
                           (commType3 == 0x02) ? "MailboxSend" :
                           (commType3 == 0x03) ? "ProcessOutput" :
                           (commType3 == 0x04) ? "ProcessInput" :
                           (commType3 == 0x00) ? "NotUsed" : "Unknown";
        TETHER_LOGI(tag, "  SM3 CommType: 0x{:02X} ({})", (unsigned)commType3, type3);
    } else {
        TETHER_LOGW(tag, "  ⚠ SM3 CommType read failed (SDO error)");
    }
}

} // namespace EtherCAT
