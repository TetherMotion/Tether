// SPDX-License-Identifier: MIT
/**
 * @file SyncManager.cpp
 * @brief Implementation of SyncManagerAccessor — per-SM configuration,
 *        validation, and diagnostic methods.
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
// Internal helpers
// ============================================================================

/// Return a human-readable name for a CommType byte.
static const char* commTypeName(uint8_t type) {
    using namespace EtherCAT::SyncManager::CommType;
    switch (type) {
        case NotUsed:        return "NotUsed";
        case MailboxReceive: return "MailboxReceive";
        case MailboxSend:    return "MailboxSend";
        case ProcessOutput:  return "ProcessOutput(RxPDO)";
        case ProcessInput:   return "ProcessInput(TxPDO)";
        default:             return "Unknown";
    }
}

// ============================================================================
// SyncManagerAccessor
// ============================================================================

SyncManagerAccessor::SyncManagerAccessor(Slave& slave, uint8_t smIndex)
    : slave_(slave), index_(smIndex)
{
}

// -----------------------------------------------------------------------
// Hardware register access
// -----------------------------------------------------------------------

SyncManagerAccessor::RawHWConfig
SyncManagerAccessor::readHardwareConfig(unsigned int timeout_ms) const {
    RawHWConfig cfg{};
    uint8_t buf[8] = {0};

    const uint16_t base = physRegisterBase();

    cfg.read_ok = slave_.master().readRegister(EtherCAT::SlaveAddress(slave_.index()), base, buf, sizeof(buf), timeout_ms);
    if (!cfg.read_ok) {
        return cfg;
    }

    cfg.start_addr = static_cast<uint16_t>(buf[0] | (static_cast<uint16_t>(buf[1]) << 8));
    cfg.length     = static_cast<uint16_t>(buf[2] | (static_cast<uint16_t>(buf[3]) << 8));
    cfg.control    = std::bit_cast<SyncManager::SMControlReg>(buf[4]);
    cfg.status     = std::bit_cast<SyncManager::SMStatusReg>(buf[5]);
    cfg.activate   = std::bit_cast<SyncManager::SMActivateReg>(buf[6]);
    cfg.pdi_ctrl   = std::bit_cast<SyncManager::SMPDICtrlReg>(buf[7]);
    return cfg;
}

// -----------------------------------------------------------------------
// SDO access
// -----------------------------------------------------------------------

SlaveError SyncManagerAccessor::readCommType(uint8_t& type) const {
    // CiA 301 object 0x1C00 : subindex = smIndex + 1
    const uint8_t sub = static_cast<uint8_t>(index_ + 1U);
    return slave_.sdoReadU8(EtherCAT::SyncManager::kCommTypeIndex, sub, type);
}

SlaveError SyncManagerAccessor::readPDOAssignCount(uint8_t& count) const {
    const uint16_t odIdx = EtherCAT::SyncManager::pdoAssignIndex(index_);
    return slave_.sdoReadU8(odIdx, 0x00, count);
}

SlaveError SyncManagerAccessor::readPDOAssignment(uint8_t subIndex, uint16_t& pdoIndex) const {
    const uint16_t odIdx = EtherCAT::SyncManager::pdoAssignIndex(index_);
    return slave_.sdoReadU16(odIdx, subIndex, pdoIndex);
}

// -----------------------------------------------------------------------
// Validation
// -----------------------------------------------------------------------

SyncManagerAccessor::ValidationResult
SyncManagerAccessor::validate(const PDO::SyncManagerConfig& expected) const {
    ValidationResult result;

    if (!expected.enable) {
        // If the expected config says the SM should be disabled, skip validation.
        result.valid   = true;
        result.message = "SM" + std::to_string(index_) + " expected disabled; not validated";
        return result;
    }

    const auto hw = readHardwareConfig();

    if (!hw.read_ok) {
        result.valid   = false;
        result.message = "SM" + std::to_string(index_) + ": hardware register read failed (APRD timeout?)";
        return result;
    }

    std::ostringstream oss;

    if (hw.start_addr != expected.phys_start_addr) {
        oss << "SM" << static_cast<int>(index_)
            << ": start_addr mismatch: hw=0x" << std::hex << hw.start_addr
            << " expected=0x" << expected.phys_start_addr;
        result.valid   = false;
        result.message = oss.str();
        return result;
    }

    if (hw.length != expected.length) {
        oss << "SM" << static_cast<int>(index_)
            << ": length mismatch: hw=" << std::dec << hw.length
            << " expected=" << expected.length;
        result.valid   = false;
        result.message = oss.str();
        return result;
    }

    if (hw.control != expected.control) {
        oss << "SM" << static_cast<int>(index_)
            << ": control mismatch: hw=0x" << std::hex << static_cast<int>(std::bit_cast<uint8_t>(hw.control))
            << " expected=0x" << static_cast<int>(std::bit_cast<uint8_t>(expected.control));
        result.valid   = false;
        result.message = oss.str();
        return result;
    }

    if (!hw.isEnabled()) {
        oss << "SM" << static_cast<int>(index_) << ": expected ENABLED but activate=0x"
            << std::hex << static_cast<int>(std::bit_cast<uint8_t>(hw.activate)) << " (bit 0 not set)";
        result.valid   = false;
        result.message = oss.str();
        return result;
    }

    result.valid = true;
    return result;
}

SyncManagerAccessor::ValidationResult
SyncManagerAccessor::validateCommType(uint8_t expectedType) const {
    ValidationResult result;

    uint8_t actual = 0xFF;
    const SlaveError err = readCommType(actual);
    if (err != SlaveError::Ok) {
        result.valid   = false;
        result.message = "SM" + std::to_string(index_) +
                         ": SDO read of CommType (0x1C00) failed";
        return result;
    }

    if (actual != expectedType) {
        std::ostringstream oss;
        oss << "SM" << static_cast<int>(index_)
            << ": CommType mismatch: actual=" << commTypeName(actual)
            << " (0x" << std::hex << static_cast<int>(actual) << ")"
            << " expected=" << commTypeName(expectedType)
            << " (0x" << static_cast<int>(expectedType) << ")";
        result.valid   = false;
        result.message = oss.str();
        return result;
    }

    result.valid = true;
    return result;
}

// -----------------------------------------------------------------------
// Diagnostics
// -----------------------------------------------------------------------

std::string SyncManagerAccessor::formatConfig(const RawHWConfig& cfg) const {
    using namespace EtherCAT::PDO;
    if (!cfg.read_ok) {
        char tmp[64];
        snprintf(tmp, sizeof(tmp), "SM%u: <read failed>", static_cast<unsigned>(index_));
        return std::string(tmp);
    }

    const char* modeStr = "UNKNOWN";
    const uint8_t modeBits = cfg.control.mode;
    if (modeBits == static_cast<uint8_t>(SyncManager::SMMode::Buffered)) modeStr = "BUFFERED";
    else if (modeBits == static_cast<uint8_t>(SyncManager::SMMode::Mailbox))  modeStr = "MAILBOX";

    const char* dirStr = cfg.control.direction ? "MASTER->SLAVE" : "SLAVE->MASTER";

    std::string flags;
    if (cfg.control.ecat_irq)   flags += "IRQ_ECAT ";
    if (cfg.control.pdi_irq)    flags += "IRQ_PDI ";
    if (cfg.control.watchdog)   flags += "WATCHDOG ";
    if (cfg.control.repeat_req && (modeBits == static_cast<uint8_t>(SyncManager::SMMode::Mailbox)))
                                flags += "REPEAT_REQ ";

    const bool enabled = cfg.isEnabled();

    // Detect conservative default mailbox configurations
    bool fallback = false;
    const uint8_t ctrl_raw = std::bit_cast<uint8_t>(cfg.control);
    if (index_ == 0) {
        // Standard ETG: SM0=MbxIn(M→S) ctrl=0x26.
        fallback = (cfg.start_addr == 0x1000 && cfg.length == 256 && ctrl_raw == 0x26);
    } else if (index_ == 1) {
        // Standard ETG: SM1=MbxOut(S→M) ctrl=0x22.
        fallback = (cfg.start_addr == 0x1400 && cfg.length == 256 && ctrl_raw == 0x22);
    }

    char tmp[320];
    snprintf(tmp, sizeof(tmp),
             "SM%u: start=0x%04X len=%u mode=%s dir=%s flags=[%s] "
             "ctrl=0x%02X stat=0x%02X act=0x%02X (%s) pdi=0x%02X%s",
             static_cast<unsigned>(index_),
             cfg.start_addr, cfg.length,
             modeStr, dirStr,
             flags.empty() ? "-" : flags.c_str(),
             ctrl_raw, std::bit_cast<uint8_t>(cfg.status), std::bit_cast<uint8_t>(cfg.activate),
             enabled ? "ENABLED" : "disabled",
             std::bit_cast<uint8_t>(cfg.pdi_ctrl),
             fallback ? " [fallback/mailbox default]" : "");
    return std::string(tmp);
}

void SyncManagerAccessor::dump(const char* tag) const {
    const auto hw = readHardwareConfig();
    const std::string fmt = formatConfig(hw);
    if (hw.read_ok) {
        TETHER_LOGI(tag, "%s", fmt.c_str());
    } else {
        TETHER_LOGW(tag, "%s", fmt.c_str());
    }
}

void SyncManagerAccessor::dumpMailboxStatus(const char* tag) const {
    // Read 2 bytes at register offset 5 (status + activate)
    const uint16_t statusOff = static_cast<uint16_t>(physRegisterBase() + EtherCAT::SyncManager::kRegOffsetStatus);

    uint8_t st[2] = {0};
    const bool ok = slave_.master().readRegister(EtherCAT::SlaveAddress(slave_.index()), statusOff, st, sizeof(st), 200);
    if (ok) {
        TETHER_LOGI(tag, "SM%u: status=0x%02X activate=0x%02X (%s)",
                    static_cast<unsigned>(index_),
                    st[0], st[1],
                    (st[1] & 0x01U) ? "ENABLED" : "disabled");
    } else {
        TETHER_LOGW(tag, "SM%u: status register read failed", static_cast<unsigned>(index_));
    }

    // Also read the SM watchdog status register for mailbox SMs (SM0/SM1)
    if (index_ <= 1) {
        uint8_t wd[2] = {0};
        if (slave_.master().readRegister(EtherCAT::SlaveAddress(slave_.index()), EtherCAT::SyncManager::kWatchdogStatusReg, wd, sizeof(wd), 200)) {
            const uint16_t wdStatus = static_cast<uint16_t>(wd[0] | (static_cast<uint16_t>(wd[1]) << 8));
            TETHER_LOGI(tag, "SM watchdog status=0x%04X %s",
                        static_cast<unsigned>(wdStatus),
                        (wdStatus == 0) ? "(OK)" : "(EXPIRED!)");
        }
    }
}

void SyncManagerAccessor::dumpPDOAssignments(const char* tag) const {
    uint8_t count = 0;
    SlaveError err = readPDOAssignCount(count);
    if (err != SlaveError::Ok) {
        TETHER_LOGW(tag, "SM%u: PDO assign count read failed (SDO error)", static_cast<unsigned>(index_));
        return;
    }
    TETHER_LOGI(tag, "SM%u: %u PDO(s) assigned (OD 0x%04X)",
                static_cast<unsigned>(index_), static_cast<unsigned>(count),
                static_cast<unsigned>(EtherCAT::SyncManager::pdoAssignIndex(index_)));
    for (uint8_t sub = 1; sub <= count; ++sub) {
        uint16_t pdoIdx = 0;
        err = readPDOAssignment(sub, pdoIdx);
        if (err == SlaveError::Ok) {
            TETHER_LOGI(tag, "  SM%u PDO[%u] = 0x%04X",
                        static_cast<unsigned>(index_), static_cast<unsigned>(sub),
                        static_cast<unsigned>(pdoIdx));
        } else {
            TETHER_LOGW(tag, "  SM%u PDO[%u]: read failed", static_cast<unsigned>(index_), static_cast<unsigned>(sub));
        }
    }
}

// ============================================================================
// Debug function: mailbox hardware configuration
// ============================================================================

void debugMailboxConfiguration(Master& master, uint16_t slave_index, const char* tag) {
    TETHER_LOGI(tag, "\n╔══════════════════════════════════════════════════════════════╗\n║  Mailbox Hardware Configuration Debug (Slave %u)            ║\n╚══════════════════════════════════════════════════════════════╝\n", (unsigned)slave_index);

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
        TETHER_LOGI(tag, "  Physical Register Base: 0x%04X", (unsigned)sm0.physRegisterBase());
        TETHER_LOGI(tag, "  Start Address:          0x%04X", (unsigned)hw0.start_addr);
        TETHER_LOGI(tag, "  Length:                 %u bytes", (unsigned)hw0.length);
        TETHER_LOGI(tag, "  Control Register (0x%02X): %s", (unsigned)std::bit_cast<uint8_t>(hw0.control), decode_control(hw0.control).c_str());
        TETHER_LOGI(tag, "  Status Register:        0x%02X", (unsigned)std::bit_cast<uint8_t>(hw0.status));
        TETHER_LOGI(tag, "  Activate Register:      0x%02X (%s)", (unsigned)std::bit_cast<uint8_t>(hw0.activate), hw0.isEnabled() ? "ENABLED" : "disabled");
        TETHER_LOGI(tag, "  PDI Control:            0x%02X", (unsigned)std::bit_cast<uint8_t>(hw0.pdi_ctrl));
    } else {
        TETHER_LOGE(tag, "  ❌ Failed to read SM0 hardware registers");
    }

    // Read SM1 (Mailbox Send / Slave→Master)
    TETHER_LOGI(tag, "\n📋 SM1 (Mailbox Send / Slave→Master)");
    auto sm1 = slave.sm(1);
    auto hw1 = sm1.readHardwareConfig();
    if (hw1.read_ok) {
        TETHER_LOGI(tag, "  Physical Register Base: 0x%04X", (unsigned)sm1.physRegisterBase());
        TETHER_LOGI(tag, "  Start Address:          0x%04X", (unsigned)hw1.start_addr);
        TETHER_LOGI(tag, "  Length:                 %u bytes", (unsigned)hw1.length);
        TETHER_LOGI(tag, "  Control Register (0x%02X): %s", (unsigned)std::bit_cast<uint8_t>(hw1.control), decode_control(hw1.control).c_str());
        TETHER_LOGI(tag, "  Status Register:        0x%02X", (unsigned)std::bit_cast<uint8_t>(hw1.status));
        TETHER_LOGI(tag, "  Activate Register:      0x%02X (%s)", (unsigned)std::bit_cast<uint8_t>(hw1.activate), hw1.isEnabled() ? "ENABLED" : "disabled");
        TETHER_LOGI(tag, "  PDI Control:            0x%02X", (unsigned)std::bit_cast<uint8_t>(hw1.pdi_ctrl));
    } else {
        TETHER_LOGE(tag, "  ❌ Failed to read SM1 hardware registers");
    }

    // Read SM Watchdog status
    TETHER_LOGI(tag, "\n📋 SM Watchdog Status (Register 0x0440)");
    uint8_t wd[2] = {0};
    if (master.readRegister(EtherCAT::SlaveAddress(slave_index), EtherCAT::SyncManager::kWatchdogStatusReg, wd, sizeof(wd), 200)) {
        const uint16_t wdStatus = static_cast<uint16_t>(wd[0] | (static_cast<uint16_t>(wd[1]) << 8));
        TETHER_LOGI(tag, "  Watchdog Status: 0x%04X %s", (unsigned)wdStatus, (wdStatus == 0) ? "(OK)" : "(EXPIRED!)");
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
        TETHER_LOGI(tag, "  SM0 CommType: 0x%02X (%s)", (unsigned)commType0, type0);
    } else {
        TETHER_LOGW(tag, "  ⚠ SM0 CommType read failed (SDO error)");
    }

    if (err1 == SlaveError::Ok) {
        const char* type1 = (commType1 == 0x01) ? "MailboxReceive" :
                           (commType1 == 0x02) ? "MailboxSend" :
                           (commType1 == 0x03) ? "ProcessOutput" :
                           (commType1 == 0x04) ? "ProcessInput" :
                           (commType1 == 0x00) ? "NotUsed" : "Unknown";
        TETHER_LOGI(tag, "  SM1 CommType: 0x%02X (%s)", (unsigned)commType1, type1);
    } else {
        TETHER_LOGW(tag, "  ⚠ SM1 CommType read failed (SDO error)");
    }
}

} // namespace EtherCAT
