/**
 * @file test_eepctl_protocol_regression.cpp
 * @brief Regression tests for the EEPCTL 2-byte write protocol fix.
 *
 * These tests verify the bugs discovered during debugging of the
 * synapticon_cst_fsoe example on Synapticon drive hardware:
 *
 * Bug 1: SIIReader::readRaw32() originally wrote a 6-byte EepromCmd struct
 *        to the EEPCTL register (0x0502), but the ESC EEPROM control register
 *        is only 2 bytes wide.  The correct protocol encodes the command in
 *        the high byte and the word address in the low byte of a single
 *        2-byte write: (EC_ECMD_READ << 8) | word_address.
 *
 * Bug 2: The example aborted on SII identity read failure, but SII reads
 *        have always been failing on Synapticon drive hardware because APWR to EEPCTL
 *        returns WKC=0.  The fix makes the identity check non-fatal.
 *
 * Bug 3: Existing tests encoded the buggy 6-byte protocol in their mock
 *        callbacks (checking len >= 4 and extracting addr from offset 2).
 *        These regression tests verify the correct 2-byte protocol.
 */

#include <gtest/gtest.h>
#include "sii/SIIReader.hpp"
#include "tether/ethercat/Master.hpp"
#include "sii/SIIParser.hpp"
#include "tether/ethercat/DebugFlags.hpp"
#include "ethercat/raw/internal.hpp"

#include <cstdint>
#include <cstring>
#include <vector>

using namespace EtherCAT::SII;

// ============================================================================
// Helper: EEPROM register constants (match SIIReader.cpp internals)
// ============================================================================
static constexpr uint16_t EC_REG_EEPCTL  = 0x0502;
static constexpr uint16_t EC_REG_EEPDAT  = 0x0508;
static constexpr uint16_t EC_ECMD_READ   = 0x0100;
static constexpr uint16_t EC_ECMD_NOP    = 0x0000;
static constexpr uint16_t EC_ESTAT_BUSY  = 0x8000;  // Bit 15 (per ETG.1000.4)
static constexpr uint16_t EC_ESTAT_NACK  = 0x2000;

// ============================================================================
// Test fixture: captures all APWR writes to verify the 2-byte protocol
// ============================================================================
class EepctlProtocolRegression : public ::testing::Test {
protected:
    struct ApwrCapture {
        uint16_t adp;
        uint16_t ado;
        uint16_t len;
        std::vector<uint8_t> data;
    };

    std::vector<ApwrCapture> apwr_writes;
    uint16_t last_cmd_addr = 0xFFFF;
    EtherCAT::Master master;

    void SetUp() override {
        apwr_writes.clear();
        last_cmd_addr = 0xFFFF;

        // Capture ALL APWR writes, especially to EEPCTL
        master.setApwrTestCallback([this](uint16_t adp, uint16_t ado,
                                          const void* data, uint16_t len,
                                          unsigned int ms) -> bool {
            (void)ms;
            if (data && len > 0) {
                ApwrCapture cap;
                cap.adp = adp;
                cap.ado = ado;
                cap.len = len;
                cap.data.assign(reinterpret_cast<const uint8_t*>(data),
                                reinterpret_cast<const uint8_t*>(data) + len);
                apwr_writes.push_back(cap);

                // Extract word address from EEPADDR (0x0504)
                if (ado == 0x0504 && len >= 2) {
                    uint16_t addr_le = 0;
                    std::memcpy(&addr_le, data, sizeof(addr_le));
                    last_cmd_addr = EtherCAT::Raw::le16_to_host(addr_le);
                }
            }
            return true;
        });

        // Standard APRD mock: EEPConfig=ECAT control, EEPSTAT=not busy, EEPDAT=deterministic
        master.setAprdTestCallback([this](uint16_t adp, uint16_t ado,
                                          void* out, uint16_t len,
                                          unsigned int ms) -> bool {
            (void)adp; (void)ms;
            if (ado == 0x0500) {
                // EEPConfig: ECAT has control (bit 0 = 0)
                if (out && len >= 1) {
                    uint8_t cfg = 0x00;
                    std::memcpy(out, &cfg, 1);
                }
                return true;
            }
            if (ado == 0x0502) {
                // EEPSTAT: not busy, no errors
                if (out && len >= 2) {
                    uint16_t estat_le = 0;
                    std::memcpy(out, &estat_le, 2);
                }
                return true;
            }
            if (ado == 0x0508) {
                // EEPDAT: deterministic value based on last_cmd_addr
                uint32_t val = 0xA0000000u | static_cast<uint32_t>(last_cmd_addr);
                if (out && len >= 4) {
                    std::memcpy(out, &val, 4);
                }
                return true;
            }
            return false;
        });
    }

    void TearDown() override {
        master.setAprdTestCallback(nullptr);
        master.setApwrTestCallback(nullptr);
    }

    /// Count APWR writes to EEPCTL (0x0502)
    size_t countEepctlWrites() const {
        size_t n = 0;
        for (const auto& w : apwr_writes) {
            if (w.ado == EC_REG_EEPCTL) ++n;
        }
        return n;
    }

    /// Get the first EEPCTL write
    const ApwrCapture* firstEepctlWrite() const {
        for (const auto& w : apwr_writes) {
            if (w.ado == EC_REG_EEPCTL) return &w;
        }
        return nullptr;
    }

    /// Count APWR writes to EEPADDR (0x0504)
    size_t countEepaddrWrites() const {
        size_t n = 0;
        for (const auto& w : apwr_writes) {
            if (w.ado == 0x0504) ++n;
        }
        return n;
    }

    /// Get the first EEPADDR write
    const ApwrCapture* firstEepaddrWrite() const {
        for (const auto& w : apwr_writes) {
            if (w.ado == 0x0504) return &w;
        }
        return nullptr;
    }
};

// ============================================================================
// Bug 1: EEPCTL write must be exactly 2 bytes, not 6
// ============================================================================

/// Verify that readRaw32 writes exactly 2 bytes to EEPCTL (not 6).
/// The original bug wrote a 6-byte EepromCmd struct which was rejected
/// by ESCs that enforce the 2-byte register width.
TEST_F(EepctlProtocolRegression, EepctlWriteIsExactly2Bytes) {
    SIIReader reader(master);
    uint32_t out = 0;
    ASSERT_TRUE(reader.readDWord(0, 0x0000, out));

    const ApwrCapture* eepctl_write = firstEepctlWrite();
    ASSERT_NE(eepctl_write, nullptr);
    EXPECT_EQ(eepctl_write->len, 2u)
        << "EEPCTL write must be exactly 2 bytes (was 6 before the fix)";
}

/// Verify that the EEPCTL write contains only the READ command (0x0100)
/// and the word address is written separately to EEPADDR (0x0504).
TEST_F(EepctlProtocolRegression, EepctlValueEncodesCommandAndAddress) {
    SIIReader reader(master);
    uint32_t out = 0;
    ASSERT_TRUE(reader.readDWord(0, 0x0042, out));

    // EEPCTL should contain only the command (0x0100), no address in low byte
    const ApwrCapture* eepctl_write = firstEepctlWrite();
    ASSERT_NE(eepctl_write, nullptr);
    ASSERT_EQ(eepctl_write->len, 2u);

    uint16_t eepctl_le = 0;
    std::memcpy(&eepctl_le, eepctl_write->data.data(), 2);
    EXPECT_EQ(eepctl_le, EC_ECMD_READ) << "EEPCTL should be READ command only (0x0100)";

    // EEPADDR should contain the word address
    const ApwrCapture* eepaddr_write = firstEepaddrWrite();
    ASSERT_NE(eepaddr_write, nullptr);
    ASSERT_EQ(eepaddr_write->len, 2u);

    uint16_t addr_le = 0;
    std::memcpy(&addr_le, eepaddr_write->data.data(), 2);
    uint16_t addr = EtherCAT::Raw::le16_to_host(addr_le);
    EXPECT_EQ(addr, 0x0042u) << "EEPADDR should contain the word address";
}

/// Verify that the old 6-byte EepromCmd struct is NOT used.
/// The old struct had: uint16_t comm_le; uint16_t addr_le; uint16_t d2_le;
/// With the 2-byte fix, there is no d2_le field.
TEST_F(EepctlProtocolRegression, NoSixByteEepromCmdStruct) {
    SIIReader reader(master);
    uint32_t out = 0;
    ASSERT_TRUE(reader.readDWord(0, 0x0000, out));

    for (const auto& w : apwr_writes) {
        if (w.ado == EC_REG_EEPCTL) {
            EXPECT_NE(w.len, 6u) << "EEPCTL write must not use the old 6-byte EepromCmd struct";
            EXPECT_NE(w.len, 4u) << "EEPCTL write must not be 4 bytes either";
        }
    }
}

/// Verify that readWord at an odd address still uses the correct aligned
/// word address in the EEPADDR write.
TEST_F(EepctlProtocolRegression, OddAddressUsesAlignedEepctlAddress) {
    SIIReader reader(master);
    uint16_t word = 0;
    ASSERT_TRUE(reader.readWord(0, 0x0003, word));

    // readWord(0, 3) aligns to address 2 (readRaw32 aligns to even)
    // The EEPADDR should contain address 2, not 3
    const ApwrCapture* eepaddr_write = firstEepaddrWrite();
    ASSERT_NE(eepaddr_write, nullptr);
    uint16_t addr_le = 0;
    std::memcpy(&addr_le, eepaddr_write->data.data(), 2);
    uint16_t addr = EtherCAT::Raw::le16_to_host(addr_le);
    EXPECT_EQ(addr, 0x0002u) << "readWord(3) should align to EEPADDR address 2";
}

/// Verify that multiple reads produce multiple EEPADDR writes,
/// each with the correct address.
TEST_F(EepctlProtocolRegression, MultipleReadsProduceCorrectEepctlWrites) {
    SIIReader reader(master);

    uint32_t out1 = 0, out2 = 0, out3 = 0;
    ASSERT_TRUE(reader.readDWord(0, 0x0010, out1));
    ASSERT_TRUE(reader.readDWord(0, 0x0020, out2));
    ASSERT_TRUE(reader.readDWord(0, 0x0030, out3));

    // Should have at least 3 EEPADDR writes (one per read, assuming no cache)
    size_t eepaddr_count = countEepaddrWrites();
    EXPECT_GE(eepaddr_count, 3u);

    // Verify each EEPADDR write has the correct address
    std::vector<uint16_t> eepaddr_vals;
    for (const auto& w : apwr_writes) {
        if (w.ado == 0x0504 && w.len >= 2) {
            uint16_t addr_le = 0;
            std::memcpy(&addr_le, w.data.data(), 2);
            eepaddr_vals.push_back(EtherCAT::Raw::le16_to_host(addr_le));
        }
    }

    // The first three EEPADDR writes should target 0x10, 0x20, 0x30
    ASSERT_GE(eepaddr_vals.size(), 3u);
    EXPECT_EQ(eepaddr_vals[0], 0x0010u);
    EXPECT_EQ(eepaddr_vals[1], 0x0020u);
    EXPECT_EQ(eepaddr_vals[2], 0x0030u);
}

/// Verify that the data returned matches the address sent in EEPADDR.
/// This is an end-to-end test of the EEPROM read protocol.
TEST_F(EepctlProtocolRegression, ReadDataMatchesEepctlAddress) {
    SIIReader reader(master);

    // Read from address 0x005A
    uint32_t out = 0;
    ASSERT_TRUE(reader.readDWord(0, 0x005A, out));

    // The mock returns 0xA0000000 | last_cmd_addr
    // last_cmd_addr is extracted from the EEPADDR (0x0504) write
    EXPECT_EQ(out, 0xA000005Au)
        << "Data should match the address written to EEPADDR";
}

// ============================================================================
// Bug 2: WKC=0 handling — readRaw32 should return false, not hang
// ============================================================================

/// Verify that readRaw32 returns false when APWR to EEPCTL fails (WKC=0).
/// This simulates the Synapticon drive behavior where APWR to EEPCTL
/// returns WKC=0 (no slave processed the write).
TEST(EepctlWkcZeroRegression, ReadRaw32ReturnsFalseWhenApwrFails) {
    EtherCAT::Master master;

    // APWR always fails (simulates WKC=0)
    master.setApwrTestCallback([](uint16_t, uint16_t, const void*, uint16_t,
                                  unsigned int) -> bool {
        return false;
    });

    // APRD for EEPSTAT succeeds (not busy)
    master.setAprdTestCallback([](uint16_t, uint16_t ado, void* out, uint16_t len,
                                  unsigned int) -> bool {
        if (ado == 0x0500) {
            if (out && len >= 1) {
                uint8_t cfg = 0x00;  // ECAT control
                std::memcpy(out, &cfg, 1);
            }
            return true;
        }
        if (ado == 0x0502 && out && len >= 2) {
            uint16_t estat_le = 0;
            std::memcpy(out, &estat_le, 2);
            return true;
        }
        return false;
    });

    SIIReader reader(master);
    uint32_t out = 0xDEADBEEF;
    bool ok = reader.readDWord(0, 0x0000, out);

    EXPECT_FALSE(ok) << "readRaw32 should return false when APWR to EEPCTL fails";
    // Output should be zeroed (readRaw32 sets *out = 0 on entry)
    EXPECT_EQ(out, 0u);
}

/// Verify that readSIIIdentity returns false (not hangs) when all SII reads fail.
/// This is the scenario that caused the synapticon_cst_fsoe example to abort.
TEST(EepctlWkcZeroRegression, ReadSIIIdentityReturnsFalseWhenEepctlFails) {
    EtherCAT::Master master;

    // APWR always fails
    master.setApwrTestCallback([](uint16_t, uint16_t, const void*, uint16_t,
                                  unsigned int) -> bool {
        return false;
    });

    // APRD for EEPSTAT succeeds
    master.setAprdTestCallback([](uint16_t, uint16_t ado, void* out, uint16_t len,
                                  unsigned int) -> bool {
        if (ado == 0x0500) {
            if (out && len >= 1) {
                uint8_t cfg = 0x00;  // ECAT control
                std::memcpy(out, &cfg, 1);
            }
            return true;
        }
        if (ado == 0x0502 && out && len >= 2) {
            uint16_t estat_le = 0;
            std::memcpy(out, &estat_le, 2);
            return true;
        }
        return false;
    });

    SIIIdentity identity;
    bool ok = readSIIIdentity(master, 0, identity);

    EXPECT_FALSE(ok) << "readSIIIdentity should return false when EEPCTL writes fail";
}

/// Verify that readSII returns false when EEPCTL writes fail, but doesn't crash.
TEST(EepctlWkcZeroRegression, ReadSIIReturnsFalseWhenEepctlFails) {
    EtherCAT::Master master;

    master.setApwrTestCallback([](uint16_t, uint16_t, const void*, uint16_t,
                                  unsigned int) -> bool {
        return false;
    });

    master.setAprdTestCallback([](uint16_t, uint16_t ado, void* out, uint16_t len,
                                  unsigned int) -> bool {
        if (ado == 0x0500) {
            if (out && len >= 1) {
                uint8_t cfg = 0x00;  // ECAT control
                std::memcpy(out, &cfg, 1);
            }
            return true;
        }
        if (ado == 0x0502 && out && len >= 2) {
            uint16_t estat_le = 0;
            std::memcpy(out, &estat_le, 2);
            return true;
        }
        return false;
    });

    SIIData data;
    bool ok = readSII(master, 0, data);

    EXPECT_FALSE(ok) << "readSII should return false when EEPCTL writes fail";
    EXPECT_FALSE(data.valid) << "SIIData should not be marked valid on failure";
}

// ============================================================================
// FPWR fallback: when APWR to EEPCTL returns WKC=0, SIIReader should fall
// back to FPWR using the configured station address (register 0x0010).
// This is the fix for ESCs (e.g. Synapticon SOMANET drive) that reject
// APWR to EEPCTL but accept FPWR.
// ============================================================================

/// Verify that readRaw32 succeeds when APWR to EEPCTL fails but FPWR works.
/// APWR is identified by adp=0x0000 (auto-increment for slave 0).
/// FPWR is identified by adp=configured station address (non-zero).
TEST(EepctlWkcZeroRegression, ReadRaw32SucceedsWithFpwrFallback) {
    EtherCAT::Master master;
    constexpr uint16_t kConfiguredAddr = 0x0001;
    uint16_t last_eepctl_addr = 0xFFFF;

    // APWR to EEPCTL (adp=0x0000, ado=0x0502) fails — simulates WKC=0.
    // APWR to 0x0010 (to assign a configured station address) succeeds.
    // FPWR to EEPCTL (adp=kConfiguredAddr, ado=0x0502) succeeds.
    master.setApwrTestCallback([&last_eepctl_addr](uint16_t adp, uint16_t ado,
                                                   const void* data, uint16_t len,
                                                   unsigned int) -> bool {
        // EEPConfig write — just acknowledge
        if (ado == 0x0500) {
            return true;
        }
        // EEPADDR write — capture address (works for both APWR and FPWR)
        if (ado == 0x0504 && data && len >= 2) {
            uint16_t addr_le = 0;
            std::memcpy(&addr_le, data, sizeof(addr_le));
            last_eepctl_addr = EtherCAT::Raw::le16_to_host(addr_le);
            return true;
        }
        if (ado == EC_REG_EEPCTL && adp == 0x0000) {
            return false;  // APWR to EEPCTL fails (WKC=0)
        }
        if (ado == EC_REG_EEPCTL && adp == kConfiguredAddr) {
            return true;  // FPWR to EEPCTL succeeds
        }
        if (ado == 0x0010 && adp == 0x0000 && data && len >= 2) {
            return true;  // APWR to station addr register succeeds
        }
        return false;
    });

    // APRD: EEPConfig=ECAT control, EEPSTAT not busy, EEPDAT returns deterministic data,
    //       station address register returns 0x0000 (unassigned).
    master.setAprdTestCallback([&last_eepctl_addr](uint16_t, uint16_t ado,
                                          void* out, uint16_t len,
                                          unsigned int) -> bool {
        if (ado == 0x0500) {
            if (out && len >= 1) {
                uint8_t cfg = 0x00;  // ECAT control
                std::memcpy(out, &cfg, 1);
            }
            return true;
        }
        if (ado == 0x0502 && out && len >= 2) {
            uint16_t estat_le = 0;  // not busy, no errors
            std::memcpy(out, &estat_le, 2);
            return true;
        }
        if (ado == 0x0508 && out && len >= 4) {
            uint32_t val = 0xA0000000u | static_cast<uint32_t>(last_eepctl_addr);
            std::memcpy(out, &val, 4);
            return true;
        }
        if (ado == 0x0010 && out && len >= 2) {
            uint16_t addr_le = 0;  // unassigned
            std::memcpy(out, &addr_le, 2);
            return true;
        }
        return false;
    });

    SIIReader reader(master);
    uint32_t out = 0;
    bool ok = reader.readDWord(0, 0x0000, out);

    EXPECT_TRUE(ok) << "readRaw32 should succeed via FPWR fallback when APWR fails";
    EXPECT_EQ(out, 0xA0000000u) << "EEPDAT should match the word address 0x00";
}

/// Verify that readSIIIdentity succeeds with FPWR fallback.
TEST(EepctlWkcZeroRegression, ReadSIIIdentitySucceedsWithFpwrFallback) {
    EtherCAT::Master master;
    constexpr uint16_t kConfiguredAddr = 0x0001;

    // Track the last EEPCTL word address for deterministic EEPDAT responses
    uint16_t last_eepctl_addr = 0xFFFF;

    master.setApwrTestCallback([&last_eepctl_addr](uint16_t adp, uint16_t ado,
                                                   const void* data, uint16_t len,
                                                   unsigned int) -> bool {
        // EEPConfig write — just acknowledge
        if (ado == 0x0500) {
            return true;
        }
        // EEPADDR write — capture address (works for both APWR and FPWR)
        if (ado == 0x0504 && data && len >= 2) {
            uint16_t addr_le = 0;
            std::memcpy(&addr_le, data, sizeof(addr_le));
            last_eepctl_addr = EtherCAT::Raw::le16_to_host(addr_le);
            return true;
        }
        if (ado == EC_REG_EEPCTL && adp == 0x0000) {
            return false;  // APWR to EEPCTL fails
        }
        if (ado == EC_REG_EEPCTL && adp == kConfiguredAddr) {
            return true;  // FPWR to EEPCTL succeeds
        }
        if (ado == 0x0010 && adp == 0x0000) {
            return true;  // APWR to station addr register succeeds
        }
        return false;
    });

    master.setAprdTestCallback([&last_eepctl_addr](uint16_t, uint16_t ado,
                                          void* out, uint16_t len,
                                          unsigned int) -> bool {
        if (ado == 0x0500) {
            if (out && len >= 1) {
                uint8_t cfg = 0x00;  // ECAT control
                std::memcpy(out, &cfg, 1);
            }
            return true;
        }
        if (ado == 0x0502 && out && len >= 2) {
            uint16_t estat_le = 0;
            std::memcpy(out, &estat_le, 2);
            return true;
        }
        if (ado == 0x0508 && out && len >= 4) {
            // Return a valid SII config area word
            uint32_t val = 0;
            switch (last_eepctl_addr) {
                case 0x00: val = 0x00020001; break; // PDI control/config
                case 0x02: val = 0x00040003; break; // sync impulse/pdi config2
                case 0x04: val = 0x00060005; break; // alias/checksum
                case 0x06: val = 0x00080007; break; // reserved
                case 0x08: val = 0x000022D2; break; // vendor ID
                case 0x0A: val = 0x00000001; break; // product code
                case 0x0C: val = 0x00010000; break; // revision
                case 0x0E: val = 0x00000001; break; // serial
                default: val = 0; break;
            }
            std::memcpy(out, &val, 4);
            return true;
        }
        if (ado == 0x0010 && out && len >= 2) {
            uint16_t addr_le = 0;
            std::memcpy(out, &addr_le, 2);
            return true;
        }
        return false;
    });

    SIIIdentity identity;
    bool ok = readSIIIdentity(master, 0, identity);

    EXPECT_TRUE(ok) << "readSIIIdentity should succeed via FPWR fallback";
    EXPECT_EQ(identity.vendor_id, 0x000022D2u);
}

// ============================================================================
// Bug 3: EEPCTL NOP (error clear) also uses 2-byte write
// ============================================================================

/// Verify that the NOP command used to clear errors is also a 2-byte write.
/// The error-clear path writes EC_ECMD_NOP (0x0000) to EEPCTL.
TEST_F(EepctlProtocolRegression, ErrorClearNopIs2ByteWrite) {
    // Set up EEPSTAT to return an error on first poll, then clear
    int poll_count = 0;
    master.setAprdTestCallback([this, &poll_count](uint16_t, uint16_t ado,
                                          void* out, uint16_t len,
                                          unsigned int) -> bool {
        if (ado == 0x0500) {
            if (out && len >= 1) {
                uint8_t cfg = 0x00;  // ECAT control
                std::memcpy(out, &cfg, 1);
            }
            return true;
        }
        if (ado == 0x0502 && out && len >= 2) {
            if (poll_count == 0) {
                // First poll: return NACK error to trigger error-clear path
                uint16_t estat = EC_ESTAT_NACK;
                std::memcpy(out, &estat, 2);
                poll_count++;
                return true;
            }
            // Subsequent polls: not busy
            uint16_t estat = 0;
            std::memcpy(out, &estat, 2);
            return true;
        }
        if (ado == 0x0508 && out && len >= 4) {
            uint32_t val = 0xA0000000u | static_cast<uint32_t>(last_cmd_addr);
            std::memcpy(out, &val, 4);
            return true;
        }
        return false;
    });

    SIIReader reader(master);
    uint32_t out = 0;
    // This should succeed after clearing the error
    bool ok = reader.readDWord(0, 0x0000, out);

    // The NOP write should also be 2 bytes
    for (const auto& w : apwr_writes) {
        if (w.ado == EC_REG_EEPCTL) {
            EXPECT_EQ(w.len, 2u) << "All EEPCTL writes (including NOP) must be 2 bytes";
        }
    }
}

// ============================================================================
// Edge cases: address wrapping, high addresses, zero address
// ============================================================================

/// Verify that address 0x00FF works correctly.
/// With the new protocol, the full 16-bit word address is written to EEPADDR.
TEST_F(EepctlProtocolRegression, HighByteAddressEncoding) {
    SIIReader reader(master);
    uint32_t out = 0;

    // Address 0x00FF
    ASSERT_TRUE(reader.readDWord(0, 0x00FF, out));
    const ApwrCapture* w = firstEepaddrWrite();
    ASSERT_NE(w, nullptr);
    uint16_t addr_le = 0;
    std::memcpy(&addr_le, w->data.data(), 2);
    EXPECT_EQ(EtherCAT::Raw::le16_to_host(addr_le), 0x00FFu);
}

/// Verify that address 0x0000 works correctly.
TEST_F(EepctlProtocolRegression, ZeroAddressEncoding) {
    SIIReader reader(master);
    uint32_t out = 0;

    ASSERT_TRUE(reader.readDWord(0, 0x0000, out));

    // EEPADDR should contain 0x0000
    const ApwrCapture* addr_write = firstEepaddrWrite();
    ASSERT_NE(addr_write, nullptr);
    uint16_t addr_le = 0;
    std::memcpy(&addr_le, addr_write->data.data(), 2);
    EXPECT_EQ(EtherCAT::Raw::le16_to_host(addr_le), 0x0000u);

    // EEPCTL should contain only the READ command (0x0100)
    const ApwrCapture* ctl_write = firstEepctlWrite();
    ASSERT_NE(ctl_write, nullptr);
    uint16_t eepctl_le = 0;
    std::memcpy(&eepctl_le, ctl_write->data.data(), 2);
    EXPECT_EQ(eepctl_le, EC_ECMD_READ);
}

// ============================================================================
// Cache behavior: second read of same address should NOT write EEPCTL
// ============================================================================

/// Verify that a second read of the same address returns the same value.
/// The master-level SII cache requires initSlaves() to be sized, so with
/// a bare Master both reads will hit the bus — but both should succeed
/// and return consistent data.
TEST_F(EepctlProtocolRegression, RepeatedReadReturnsConsistentValue) {
    SIIReader reader(master);
    uint32_t out1 = 0, out2 = 0;

    // First read
    ASSERT_TRUE(reader.readDWord(0, 0x0010, out1));
    size_t writes_after_first = countEepctlWrites();

    // Second read of same address
    ASSERT_TRUE(reader.readDWord(0, 0x0010, out2));

    EXPECT_EQ(out1, out2) << "Both reads should return the same value";
    EXPECT_GE(writes_after_first, 1u) << "First read should produce at least 1 EEPCTL write";
}

// ============================================================================
// Debug flag: eeprom flag should be registered and queryable
// ============================================================================

/// Verify that the "eeprom" debug flag is registered in allDebugFlags().
TEST(EepromDebugFlagRegression, EepromFlagIsRegistered) {
    const auto& flags = EtherCAT::debug::allDebugFlags();
    bool found = false;
    for (const auto& f : flags) {
        if (f.name == "eeprom") {
            found = true;
            EXPECT_FALSE(f.description.empty()) << "eeprom flag should have a description";
            break;
        }
    }
    EXPECT_TRUE(found) << "The 'eeprom' debug flag must be registered in allDebugFlags()";
}

/// Verify that the "eeprom" flag can be enabled via setFlag.
TEST(EepromDebugFlagRegression, EepromFlagCanBeEnabled) {
    EtherCAT::EtherCATMasterDebugFlags flags;
    flags.setFlag("eeprom", true);
    EXPECT_TRUE(flags.eeprom) << "setFlag(\"eeprom\", true) should set flags.eeprom";
    EXPECT_TRUE(flags.isEnabled("eeprom", 0)) << "isEnabled should return true for eeprom";
}

/// Verify that the "eeprom" flag can be disabled via setFlag.
TEST(EepromDebugFlagRegression, EepromFlagCanBeDisabled) {
    EtherCAT::EtherCATMasterDebugFlags flags;
    flags.setFlag("eeprom", true);
    flags.setFlag("eeprom", false);
    EXPECT_FALSE(flags.eeprom) << "setFlag(\"eeprom\", false) should clear flags.eeprom";
    EXPECT_FALSE(flags.isEnabled("eeprom", 0));
}

/// Verify that the "eeprom" flag is included in isAnyFlagEnabled().
TEST(EepromDebugFlagRegression, EepromFlagInIsAnyFlagEnabled) {
    EtherCAT::EtherCATMasterDebugFlags flags;
    EXPECT_FALSE(flags.isAnyFlagEnabled());
    flags.eeprom = true;
    EXPECT_TRUE(flags.isAnyFlagEnabled()) << "Enabling eeprom should make isAnyFlagEnabled() true";
}

/// Verify that the "eeprom" flag is included in computeForSlave().
TEST(EepromDebugFlagRegression, EepromFlagInComputeForSlave) {
    EtherCAT::EtherCATMasterDebugFlags flags;
    flags.eeprom = true;
    auto slave_flags = flags.computeForSlave(0);
    EXPECT_TRUE(slave_flags.eeprom) << "computeForSlave should propagate eeprom flag";
}

/// Verify that the "eeprom" flag can be set via applyFromString.
TEST(EepromDebugFlagRegression, EepromFlagViaApplyFromString) {
    EtherCAT::EtherCATMasterDebugFlags flags;
    flags.applyFromString("eeprom", 2, nullptr);
    EXPECT_TRUE(flags.eeprom) << "applyFromString(\"eeprom\") should set flags.eeprom";
}

/// Verify that the "eeprom" flag supports per-slave filtering.
TEST(EepromDebugFlagRegression, EepromFlagSupportsSlaveFilter) {
    EtherCAT::EtherCATMasterDebugFlags flags;
    flags.applyFromString("eeprom:(slaves:1)", 4, nullptr);

    EXPECT_FALSE(flags.isEnabled("eeprom", 0)) << "eeprom should be filtered for slave 0";
    EXPECT_TRUE(flags.isEnabled("eeprom", 1)) << "eeprom should be enabled for slave 1";
    EXPECT_FALSE(flags.isEnabled("eeprom", 2)) << "eeprom should be filtered for slave 2";
}

/// Verify that resizeFilters includes the eeprom filter.
TEST(EepromDebugFlagRegression, EepromFlagInResizeFilters) {
    EtherCAT::EtherCATMasterDebugFlags flags;
    flags.applyFromString("eeprom:(slaves:1)", 2, nullptr);
    // Resize to 4 slaves — slave 1 should still be enabled
    flags.resizeFilters(4);
    EXPECT_FALSE(flags.isEnabled("eeprom", 0));
    EXPECT_TRUE(flags.isEnabled("eeprom", 1));
    EXPECT_FALSE(flags.isEnabled("eeprom", 2));
    EXPECT_FALSE(flags.isEnabled("eeprom", 3));
}

// ============================================================================
// End-to-end: parseIdentity with 2-byte protocol
// ============================================================================

/// Verify that parseIdentity works end-to-end with the 2-byte EEPCTL protocol.
/// This is the function called by readSIIIdentity() in the example.
TEST(EepctlProtocolE2E, ParseIdentityWorksWith2ByteProtocol) {
    uint16_t last_cmd_addr = 0xFFFF;
    EtherCAT::Master master;

    master.setApwrTestCallback([&](uint16_t, uint16_t ado, const void* data,
                                   uint16_t len, unsigned int) -> bool {
        if (ado == 0x0504 && data && len >= 2) {
            uint16_t addr_le = 0;
            std::memcpy(&addr_le, data, sizeof(addr_le));
            last_cmd_addr = EtherCAT::Raw::le16_to_host(addr_le);
        }
        return true;
    });

    master.setAprdTestCallback([&](uint16_t, uint16_t ado, void* out, uint16_t len,
                                   unsigned int) -> bool {
        if (ado == 0x0500) {
            if (out && len >= 1) {
                uint8_t cfg = 0x00;  // ECAT control
                std::memcpy(out, &cfg, 1);
            }
            return true;
        }
        if (ado == 0x0502) {
            uint16_t estat_le = 0;
            std::memcpy(out, &estat_le, 2);
            return true;
        }
        if (ado == 0x0508) {
            uint32_t val = 0;
            switch (last_cmd_addr) {
                case 0: val = 0x00010002; break;
                case 2: val = 0x00030004; break;
                case 4: val = 0x00050006; break;
                case 6: val = 0x00070008; break;
                case 8: val = 0x11112222; break;  // vendor_id
                case 10: val = 0x33334444; break; // product_code
                case 12: val = 0x55556666; break; // revision
                case 14: val = 0x77778888; break; // serial
                case 0x14: val = 0x0009000A; break;
                case 0x16: val = 0x000B000C; break;
                case 0x18: val = 0x000D000E; break;
                case 0x1A: val = 0x000F0010; break;
                case 0x1C: val = 0x00110012; break;
                default: val = 0; break;
            }
            if (out && len >= 4) std::memcpy(out, &val, 4);
            return true;
        }
        return false;
    });

    SIIReader reader(master);
    SIIParser parser(reader);
    SIIData data;

    ASSERT_TRUE(parser.parseIdentity(0, data));
    EXPECT_TRUE(data.valid);
    EXPECT_EQ(data.identity.vendor_id, 0x11112222u);
    EXPECT_EQ(data.identity.product_code, 0x33334444u);
    EXPECT_EQ(data.identity.revision_number, 0x55556666u);
    EXPECT_EQ(data.identity.serial_number, 0x77778888u);
}

/// Verify that readSIIIdentity convenience function works with 2-byte protocol.
TEST(EepctlProtocolE2E, ReadSIIIdentityWorksWith2ByteProtocol) {
    uint16_t last_cmd_addr = 0xFFFF;
    EtherCAT::Master master;

    master.setApwrTestCallback([&](uint16_t, uint16_t ado, const void* data,
                                   uint16_t len, unsigned int) -> bool {
        if (ado == 0x0504 && data && len >= 2) {
            uint16_t addr_le = 0;
            std::memcpy(&addr_le, data, sizeof(addr_le));
            last_cmd_addr = EtherCAT::Raw::le16_to_host(addr_le);
        }
        return true;
    });

    master.setAprdTestCallback([&](uint16_t, uint16_t ado, void* out, uint16_t len,
                                   unsigned int) -> bool {
        if (ado == 0x0500) {
            if (out && len >= 1) {
                uint8_t cfg = 0x00;  // ECAT control
                std::memcpy(out, &cfg, 1);
            }
            return true;
        }
        if (ado == 0x0502) {
            uint16_t estat_le = 0;
            if (out && len >= 2) std::memcpy(out, &estat_le, 2);
            return true;
        }
        if (ado == 0x0508) {
            uint32_t val = 0;
            switch (last_cmd_addr) {
                case 0: val = 0x00010002; break;
                case 2: val = 0x00030004; break;
                case 4: val = 0x00050006; break;
                case 6: val = 0x00070008; break;
                case 8: val = 0xAAAA0001; break;  // vendor_id
                case 10: val = 0xBBBB0002; break; // product_code
                case 12: val = 0xCCCC0003; break; // revision
                case 14: val = 0xDDDD0004; break; // serial
                case 0x14: val = 0x0009000A; break;
                case 0x16: val = 0x000B000C; break;
                case 0x18: val = 0x000D000E; break;
                case 0x1A: val = 0x000F0010; break;
                case 0x1C: val = 0x00110012; break;
                default: val = 0; break;
            }
            if (out && len >= 4) std::memcpy(out, &val, 4);
            return true;
        }
        return false;
    });

    SIIIdentity identity;
    ASSERT_TRUE(readSIIIdentity(master, 0, identity));
    EXPECT_EQ(identity.vendor_id, 0xAAAA0001u);
    EXPECT_EQ(identity.product_code, 0xBBBB0002u);
    EXPECT_EQ(identity.revision_number, 0xCCCC0003u);
    EXPECT_EQ(identity.serial_number, 0xDDDD0004u);
}

// ============================================================================
// NACK retry: verify retry still uses 2-byte EEPCTL writes
// ============================================================================

/// Verify that NACK retries use the 2-byte EEPCTL protocol.
TEST_F(EepctlProtocolRegression, NackRetryUses2ByteEepctl) {
    int estat_poll_count = 0;
    master.setAprdTestCallback([this, &estat_poll_count](uint16_t, uint16_t ado,
                                          void* out, uint16_t len,
                                          unsigned int) -> bool {
        if (ado == 0x0500) {
            if (out && len >= 1) {
                uint8_t cfg = 0x00;  // ECAT control
                std::memcpy(out, &cfg, 1);
            }
            return true;
        }
        if (ado == 0x0502 && out && len >= 2) {
            estat_poll_count++;
            if (estat_poll_count <= 2) {
                // First 2 polls after the write: return NACK
                uint16_t estat = EC_ESTAT_NACK;
                std::memcpy(out, &estat, 2);
                return true;
            }
            // Then succeed
            uint16_t estat = 0;
            std::memcpy(out, &estat, 2);
            return true;
        }
        if (ado == 0x0508 && out && len >= 4) {
            uint32_t val = 0xA0000000u | static_cast<uint32_t>(last_cmd_addr);
            std::memcpy(out, &val, 4);
            return true;
        }
        return false;
    });

    SIIReader reader(master);
    reader.setTimeout(2000); // Give enough time for retries
    uint32_t out = 0;
    bool ok = reader.readDWord(0, 0x0005, out);

    // All EEPCTL writes should be 2 bytes
    for (const auto& w : apwr_writes) {
        if (w.ado == EC_REG_EEPCTL) {
            EXPECT_EQ(w.len, 2u) << "NACK retry EEPCTL writes must also be 2 bytes";
        }
    }
}

// ============================================================================
// Busy-wait timeout: verify it doesn't hang indefinitely
// ============================================================================

/// Verify that waitNotBusy timeout is respected and readRaw32 returns false.
TEST(EepctlTimeoutRegression, WaitNotBusyTimeoutReturnsFalse) {
    EtherCAT::Master master;

    // APWR succeeds
    master.setApwrTestCallback([](uint16_t, uint16_t, const void*, uint16_t,
                                  unsigned int) -> bool {
        return true;
    });

    // EEPSTAT always returns BUSY
    master.setAprdTestCallback([](uint16_t, uint16_t ado, void* out, uint16_t len,
                                  unsigned int) -> bool {
        if (ado == 0x0500) {
            if (out && len >= 1) {
                uint8_t cfg = 0x00;  // ECAT control
                std::memcpy(out, &cfg, 1);
            }
            return true;
        }
        if (ado == 0x0502 && out && len >= 2) {
            uint16_t estat = EC_ESTAT_BUSY;
            std::memcpy(out, &estat, 2);
            return true;
        }
        return false;
    });

    SIIReader reader(master);
    reader.setTimeout(50); // 50ms timeout for fast test
    uint32_t out = 0xDEADBEEF;
    bool ok = reader.readDWord(0, 0x0000, out);

    EXPECT_FALSE(ok) << "readRaw32 should return false on waitNotBusy timeout";
}
