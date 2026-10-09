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

SlaveError SyncManagerAccessor::readAllPDOAssignments(std::vector<uint16_t>& out_indices) const {
    out_indices.clear();

    uint8_t count = 0;
    SlaveError err = readPDOAssignCount(count);
    if (err != SlaveError::Ok) {
        return err;
    }

    out_indices.reserve(count);
    for (uint8_t sub = 1; sub <= count; ++sub) {
        uint16_t pdo_idx = 0;
        err = readPDOAssignment(sub, pdo_idx);
        if (err != SlaveError::Ok) {
            return err;
        }
        out_indices.push_back(pdo_idx);
    }
    return SlaveError::Ok;
}

SlaveError SyncManagerAccessor::writePDOAssignments(std::initializer_list<uint16_t> pdo_indices) {
    std::vector<uint16_t> vec(pdo_indices);
    return writePDOAssignments(vec);
}

SlaveError SyncManagerAccessor::writePDOAssignments(const std::vector<uint16_t>& pdo_indices) {
    const uint16_t odIdx = EtherCAT::SyncManager::pdoAssignIndex(index_);

    // CiA 301 write ordering: clear count first, write entries, then set count
    uint8_t zero = 0;
    SlaveError err = slave_.sdoWriteU8(odIdx, 0x00, zero);
    if (err != SlaveError::Ok) {
        return err;
    }

    for (size_t i = 0; i < pdo_indices.size(); i++) {
        uint8_t sub = static_cast<uint8_t>(i + 1);
        err = slave_.sdoWriteU16(odIdx, sub, pdo_indices[i]);
        if (err != SlaveError::Ok) {
            return err;
        }
    }

    uint8_t count = static_cast<uint8_t>(pdo_indices.size());
    err = slave_.sdoWriteU8(odIdx, 0x00, count);
    if (err != SlaveError::Ok) {
        return err;
    }

    return SlaveError::Ok;
}

SlaveError SyncManagerAccessor::clearPDOAssignments() {
    const uint16_t odIdx = EtherCAT::SyncManager::pdoAssignIndex(index_);
    uint8_t zero = 0;
    return slave_.sdoWriteU8(odIdx, 0x00, zero);
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
        TETHER_LOGI(tag, "{}", fmt.c_str());
    } else {
        TETHER_LOGW(tag, "{}", fmt.c_str());
    }
}

void SyncManagerAccessor::dumpMailboxStatus(const char* tag) const {
    // Read 2 bytes at register offset 5 (status + activate)
    const uint16_t statusOff = static_cast<uint16_t>(physRegisterBase() + EtherCAT::SyncManager::kRegOffsetStatus);

    uint8_t st[2] = {0};
    const bool ok = slave_.master().readRegister(EtherCAT::SlaveAddress(slave_.index()), statusOff, st, sizeof(st), 200);
    if (ok) {
        TETHER_LOGI(tag, "SM{}: status=0x{:02X} activate=0x{:02X} ({})",
                    static_cast<unsigned>(index_),
                    st[0], st[1],
                    (st[1] & 0x01U) ? "ENABLED" : "disabled");
    } else {
        TETHER_LOGW(tag, "SM{}: status register read failed", static_cast<unsigned>(index_));
    }

    // Also read the SM watchdog status register for mailbox SMs (SM0/SM1)
    if (index_ <= 1) {
        uint8_t wd[2] = {0};
        if (slave_.master().readRegister(EtherCAT::SlaveAddress(slave_.index()), EtherCAT::SyncManager::kWatchdogStatusReg, wd, sizeof(wd), 200)) {
            const uint16_t wdStatus = static_cast<uint16_t>(wd[0] | (static_cast<uint16_t>(wd[1]) << 8));
            TETHER_LOGI(tag, "SM watchdog status=0x{:04X} {}",
                        static_cast<unsigned>(wdStatus),
                        (wdStatus == 0) ? "(OK)" : "(EXPIRED!)");
        }
    }
}

void SyncManagerAccessor::dumpPDOAssignments(const char* tag) const {
    uint8_t count = 0;
    SlaveError err = readPDOAssignCount(count);
    if (err != SlaveError::Ok) {
        TETHER_LOGW(tag, "SM{}: PDO assign count read failed (SDO error)", static_cast<unsigned>(index_));
        return;
    }
    TETHER_LOGI(tag, "SM{}: {} PDO(s) assigned (OD 0x{:04X})",
                static_cast<unsigned>(index_), static_cast<unsigned>(count),
                static_cast<unsigned>(EtherCAT::SyncManager::pdoAssignIndex(index_)));
    for (uint8_t sub = 1; sub <= count; ++sub) {
        uint16_t pdoIdx = 0;
        err = readPDOAssignment(sub, pdoIdx);
        if (err == SlaveError::Ok) {
            TETHER_LOGI(tag, "  SM{} PDO[{}] = 0x{:04X}",
                        static_cast<unsigned>(index_), static_cast<unsigned>(sub),
                        static_cast<unsigned>(pdoIdx));
        } else {
            TETHER_LOGW(tag, "  SM{} PDO[{}]: read failed", static_cast<unsigned>(index_), static_cast<unsigned>(sub));
        }
    }
}

} // namespace EtherCAT
