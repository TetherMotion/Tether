#include "tether/ethercat/SyncManagerValidation.hpp"
#include <sstream>
#include <algorithm>
#include <iomanip>

namespace EtherCAT {

using PDO::SyncManagerConfig;

bool SyncManagerValidation::checkOverlap(const SyncManagerConfig& a, const SyncManagerConfig& b) {
    if (!a.enable || !b.enable) return false;
    
    // Addresses are in slave physical memory
    uint16_t start_a = a.phys_start_addr;
    uint32_t end_a   = start_a + a.length; // Use 32-bit to prevent overflow
    uint16_t start_b = b.phys_start_addr;
    uint32_t end_b   = start_b + b.length;

    // Zero-length usually doesn't overlap, but let's be strict if they occupy same start
    if (a.length == 0 || b.length == 0) return false; 

    // Standard overlap check: max(StartA, StartB) < min(EndA, EndB)
    return std::max((uint32_t)start_a, (uint32_t)start_b) < std::min(end_a, end_b);
}

SyncManagerValidationResult SyncManagerValidation::validate(const std::vector<SyncManagerConfig>& configs) {
    SyncManagerValidationResult result{true, ""};
    std::stringstream ss;

    // 1. Check overlaps (Critical)
    for (size_t i = 0; i < configs.size(); ++i) {
        for (size_t j = i + 1; j < configs.size(); ++j) {
            if (checkOverlap(configs[i], configs[j])) {
                ss << "Overlap detected between SM" << i << " (0x" 
                   << std::hex << configs[i].phys_start_addr << ", len " << std::dec << configs[i].length 
                   << ") and SM" << j << " (0x" 
                   << std::hex << configs[j].phys_start_addr << ", len " << std::dec << configs[j].length << ")";
                return {false, ss.str()};
            }
        }
    }

    // 2. Validate specific SM types if present
    
    // SM0: MbxIn / Receive (MASTER→SLAVE, ECAT writes) — must be 0x26 (SOEM convention)
    if (configs.size() > 0) {
        const auto& sm = configs[0];
        if (sm.enable) {
            if (sm.control != 0x26) {
                 ss << "SM0 invalid control: expected 0x26 (MbxIn/MASTER→SLAVE), got 0x" << std::hex << (int)sm.control;
                 return {false, ss.str()};
            }
        }
    }

    // SM1: MbxOut / Send (SLAVE→MASTER, ECAT reads) — must be 0x22 (SOEM convention)
    if (configs.size() > 1) {
        const auto& sm = configs[1];
        if (sm.enable) {
            if (sm.control != 0x22) {
                 ss << "SM1 invalid control: expected 0x22 (MbxOut/SLAVE→MASTER), got 0x" << std::hex << (int)sm.control;
                 return {false, ss.str()};
            }
        }
    }

    // Check mailbox address ordering: SM1 address should be > SM0 address
    // This is the typical convention but not strictly forbidden by the standard
    if (configs.size() > 1) {
        const auto& sm0 = configs[0];
        const auto& sm1 = configs[1];
        if (sm0.enable && sm1.enable) {
            if (sm1.phys_start_addr <= sm0.phys_start_addr) {
                 ss << "WARNING: SM1 address (0x" << std::hex << sm1.phys_start_addr 
                    << ") <= SM0 address (0x" << sm0.phys_start_addr 
                    << "). This is unusual but not forbidden. Typical convention: SM0 < SM1";
                 // Don't fail, just log the warning
                 result.error_message = ss.str();
            }
        }
    }

    // SM2: Process data — accept any buffered mode control byte (direction varies by vendor)
    if (configs.size() > 2) {
        const auto& sm = configs[2];
        if (sm.enable) {
            if ((sm.control & 0x03) != 0x00) {  // Must be BUFFERED mode (bits[1:0]=00)
                 ss << "SM2 invalid mode: expected BUFFERED (0x00), got mode bits 0x" << std::hex << (int)(sm.control & 0x03);
                 return {false, ss.str()};
            }
        }
    }

    // SM3: Process data — accept any buffered mode control byte (direction varies by vendor)
    if (configs.size() > 3) {
        const auto& sm = configs[3];
        if (sm.enable) {
            if ((sm.control & 0x03) != 0x00) {  // Must be BUFFERED mode (bits[1:0]=00)
                 ss << "SM3 invalid mode: expected BUFFERED (0x00), got mode bits 0x" << std::hex << (int)(sm.control & 0x03);
                 return {false, ss.str()};
            }
        }
    }

    return {true, ""};
}

} // namespace EtherCAT
