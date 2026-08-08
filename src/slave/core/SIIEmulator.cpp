// SPDX-License-Identifier: MIT

#include "tether/slave/core/SIIEmulator.hpp"
#include "tether/slave/core/SlaveTypes.hpp"  // brings ESCReg alias into EtherCAT::slave

#include <cstring>

namespace EtherCAT { namespace slave {

// ============================================================================
// Command processing
// ============================================================================

void SIIEmulator::processCommand() {
    if (siiState_.control.read_op) {
        // Set busy while processing
        siiState_.control.busy = 1;
        uint16_t ctrlBusy = std::bit_cast<uint16_t>(siiState_.control);
        (*registers_)[ESCReg::SIIControl] = ctrlBusy & 0xFF;
        (*registers_)[ESCReg::SIIControl + 1] = (ctrlBusy >> 8) & 0xFF;

        // Read operation
        uint16_t word = readWord(static_cast<uint16_t>(siiState_.address));
        siiState_.data[0] = word & 0xFF;
        siiState_.data[1] = (word >> 8) & 0xFF;

        // Update data register
        (*registers_)[ESCReg::SIIData] = siiState_.data[0];
        (*registers_)[ESCReg::SIIData + 1] = siiState_.data[1];

        // Clear read bit and busy
        siiState_.control.read_op = 0;
        siiState_.control.busy = 0;
        uint16_t ctrlRaw = std::bit_cast<uint16_t>(siiState_.control);
        (*registers_)[ESCReg::SIIControl] = ctrlRaw & 0xFF;
        (*registers_)[ESCReg::SIIControl + 1] = (ctrlRaw >> 8) & 0xFF;
    }

    if (siiState_.control.write_op) {
        // Set busy while processing
        siiState_.control.busy = 1;
        uint16_t ctrlBusy = std::bit_cast<uint16_t>(siiState_.control);
        (*registers_)[ESCReg::SIIControl] = ctrlBusy & 0xFF;
        (*registers_)[ESCReg::SIIControl + 1] = (ctrlBusy >> 8) & 0xFF;

        // Write operation
        uint16_t word = siiState_.data[0] | (siiState_.data[1] << 8);
        writeWord(static_cast<uint16_t>(siiState_.address), word);

        // Clear write bit and busy
        siiState_.control.write_op = 0;
        siiState_.control.busy = 0;
        uint16_t ctrlRaw = std::bit_cast<uint16_t>(siiState_.control);
        (*registers_)[ESCReg::SIIControl] = ctrlRaw & 0xFF;
        (*registers_)[ESCReg::SIIControl + 1] = (ctrlRaw >> 8) & 0xFF;
    }
}

// ============================================================================
// Register write dispatch
// ============================================================================

void SIIEmulator::handleRegisterWrite(uint16_t addr, const uint8_t* data, uint16_t len) {
    // Handle SII access
    if (addr == ESCReg::SIIControl && len >= 6) {
        // Combined write: comm(2) + addr(2) + d2(2) — master writes EepromCmd struct
        siiState_.control = std::bit_cast<EtherCAT::SII::SIIControlReg>(
            static_cast<uint16_t>(data[0] | (data[1] << 8)));
        siiState_.address = data[2] | (data[3] << 8) | (data[4] << 16) | (data[5] << 24);
        processCommand();
    } else if (addr == ESCReg::SIIControl && len >= 2) {
        siiState_.control = std::bit_cast<EtherCAT::SII::SIIControlReg>(
            static_cast<uint16_t>(data[0] | (data[1] << 8)));
        processCommand();
    }
    if (addr == ESCReg::SIIAddress && len >= 4) {
        siiState_.address = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
    }
}

// ============================================================================
// Initialization
// ============================================================================

void SIIEmulator::initializeFromIdentity(const SlaveIdentity& identity) {
    // Create minimal SII data based on identity
    std::vector<uint8_t> sii;
    sii.resize(128, 0xFF);  // Default to 0xFF (empty EEPROM)

    // SII Header (first 16 bytes)
    // Word 0: PDI Control
    sii[0] = 0x00; sii[1] = 0x00;
    // Word 1: PDI Config
    sii[2] = 0x00; sii[3] = 0x00;
    // Word 2: Sync Impulse Length
    sii[4] = 0x00; sii[5] = 0x00;
    // Word 3: PDI Config 2
    sii[6] = 0x00; sii[7] = 0x00;
    // Word 4: Station Alias
    sii[8] = 0x00; sii[9] = 0x00;
    // Word 5-6: Reserved
    // Word 7: Checksum (calculated later)

    // Words 8-11: Vendor ID (4 bytes)
    sii[16] = identity.vendorId & 0xFF;
    sii[17] = (identity.vendorId >> 8) & 0xFF;
    sii[18] = (identity.vendorId >> 16) & 0xFF;
    sii[19] = (identity.vendorId >> 24) & 0xFF;

    // Words 12-13: Product Code
    sii[24] = identity.productCode & 0xFF;
    sii[25] = (identity.productCode >> 8) & 0xFF;
    sii[26] = (identity.productCode >> 16) & 0xFF;
    sii[27] = (identity.productCode >> 24) & 0xFF;

    // Words 14-15: Revision Number
    sii[28] = identity.revisionNumber & 0xFF;
    sii[29] = (identity.revisionNumber >> 8) & 0xFF;
    sii[30] = (identity.revisionNumber >> 16) & 0xFF;
    sii[31] = (identity.revisionNumber >> 24) & 0xFF;

    // Words 16-17: Serial Number
    sii[32] = identity.serialNumber & 0xFF;
    sii[33] = (identity.serialNumber >> 8) & 0xFF;
    sii[34] = (identity.serialNumber >> 16) & 0xFF;
    sii[35] = (identity.serialNumber >> 24) & 0xFF;

    siiState_.eepromData = std::move(sii);
}

}} // namespace EtherCAT::slave
