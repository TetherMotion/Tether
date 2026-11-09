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

        // Read operation: EEPDAT is 32-bit and returns the EEPROM word at
        // the addressed location plus the following word (masters read
        // word-pairs per command).
        uint16_t w0 = readWord(static_cast<uint16_t>(siiState_.address));
        uint16_t w1 = readWord(static_cast<uint16_t>(siiState_.address + 1));
        siiState_.data[0] = w0 & 0xFF;
        siiState_.data[1] = (w0 >> 8) & 0xFF;
        siiState_.data[2] = w1 & 0xFF;
        siiState_.data[3] = (w1 >> 8) & 0xFF;

        // Update data register
        for (int i = 0; i < 4; ++i) {
            (*registers_)[ESCReg::SIIData + i] = siiState_.data[i];
        }

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
    if (addr == ESCReg::SIIAddress && len >= 2) {
        // Masters write the word address as a 16-bit value; accept 2-4
        // bytes and zero-extend to the 32-bit internal address.
        uint32_t a = 0;
        for (uint16_t i = 0; i < len && i < 4; ++i) {
            a |= static_cast<uint32_t>(data[i]) << (8 * i);
        }
        siiState_.address = a;
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

    // Identity block: four consecutive 32-bit values at words 0x08-0x0F
    // (vendor, product, revision, serial — 4 bytes each).
    auto put32 = [&sii](size_t byte, uint32_t v) {
        sii[byte + 0] = v & 0xFF;
        sii[byte + 1] = (v >> 8) & 0xFF;
        sii[byte + 2] = (v >> 16) & 0xFF;
        sii[byte + 3] = (v >> 24) & 0xFF;
    };
    put32(16, identity.vendorId);
    put32(20, identity.productCode);
    put32(24, identity.revisionNumber);
    put32(28, identity.serialNumber);

    siiState_.eepromData = std::move(sii);
}

}} // namespace EtherCAT::slave
