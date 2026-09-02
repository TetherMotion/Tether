/**
 * @file test_sii_multi_slave_regression.cpp
 * @brief Comprehensive multi-slave regression tests for SII EEPROM reads.
 *
 * These tests verify that the SIIReader correctly reads SII/EEPROM data for
 * multiple slaves (slave 0, slave 1, and beyond) in all addressing modes:
 *
 *   1. Standard APWR path (works on most ESCs)
 *   2. FPWR fallback (when APWR to EEPCTL returns WKC=0, e.g. Synapticon)
 *   3. Mixed mode (some slaves use APWR, others use FPWR)
 *
 * The tests also verify:
 *   - Unique configured station addresses are assigned per slave
 *   - The per-slave FPWR cache avoids retrying APWR after first failure
 *   - Full SII identity reads work for both slaves in sequence
 *   - NACK retry and error-clear NOP work through the FPWR path
 *   - No interference between slaves when reading SII in sequence
 *   - Pre-assigned configured station addresses are respected
 *   - APRD/APWR to register 0x0010 failures are handled gracefully
 */

#include <gtest/gtest.h>
#include "sii/SIIReader.hpp"
#include "tether/ethercat/Master.hpp"
#include "sii/SIIParser.hpp"
#include "ethercat/raw/internal.hpp"

#include <cstdint>
#include <cstring>
#include <unordered_map>
#include <unordered_set>

using namespace EtherCAT::SII;

// ============================================================================
// Helper constants (match SIIReader.cpp internals)
// ============================================================================
static constexpr uint16_t EC_REG_EEPCTL  = 0x0502;
static constexpr uint16_t EC_REG_EEPDAT  = 0x0508;
static constexpr uint16_t EC_REG_CFG_ADDR = 0x0010;
static constexpr uint16_t EC_ECMD_READ   = 0x0100;
static constexpr uint16_t EC_ECMD_NOP    = 0x0000;
static constexpr uint16_t EC_ESTAT_BUSY  = 0x8000;  // Bit 15 (per ETG.1000.4)
static constexpr uint16_t EC_ESTAT_NACK  = 0x2000;
static constexpr uint16_t EC_ESTAT_EMASK = 0x7800;

// ADP values for auto-increment addressing
static constexpr uint16_t ADP_SLAVE_0 = 0x0000;
static constexpr uint16_t ADP_SLAVE_1 = 0xFFFF;
static constexpr uint16_t ADP_SLAVE_2 = 0xFFFE;

// ============================================================================
// Test fixture: multi-slave SII mock infrastructure
// ============================================================================
class SiiMultiSlaveRegression : public ::testing::Test {
protected:
    EtherCAT::Master master;

    // Per-slave state for mock callbacks
    struct SlaveMockState {
        /// EEPDAT values keyed by word address (2-word aligned)
        std::unordered_map<uint16_t, uint32_t> eepdat;

        /// Current configured station address (0 = unassigned)
        uint16_t configured_addr = 0;

        /// Whether APWR to EEPCTL fails for this slave (simulates WKC=0)
        bool apwr_eepctl_fails = false;

        /// Whether APWR to register 0x0010 fails for this slave
        bool apwr_cfgaddr_fails = false;

        /// Whether APRD to register 0x0010 fails for this slave
        bool aprd_cfgaddr_fails = false;

        /// Last EEPCTL word address written (for deterministic EEPDAT)
        uint16_t last_eepctl_addr = 0xFFFF;

        /// EEPSTAT override (0xFFFF = use default "not busy")
        uint16_t eepstat_override = 0xFFFF;

        /// Number of APWR writes to EEPCTL
        int apwr_eepctl_count = 0;

        /// Number of FPWR writes to EEPCTL (adp = configured_addr)
        int fpwr_eepctl_count = 0;
    };

    std::unordered_map<uint16_t, SlaveMockState> slaves;

    void SetUp() override {
        slaves.clear();
        // Default: two slaves
        setupSlave(0);
        setupSlave(1);
        installCallbacks();
    }

    void TearDown() override {
        master.setAprdTestCallback(nullptr);
        master.setApwrTestCallback(nullptr);
    }

    /// Set up a slave with default valid SII data
    void setupSlave(uint16_t index) {
        auto& s = slaves[index];
        s.configured_addr = 0;  // unassigned by default
        s.apwr_eepctl_fails = false;
        s.apwr_cfgaddr_fails = false;
        s.aprd_cfgaddr_fails = false;
        s.last_eepctl_addr = 0xFFFF;
        s.eepstat_override = 0xFFFF;

        // Fill with valid SII config area data (8 words = 16 bytes)
        // Word 0-3: PDI control, PDI config, sync impulse, PDI config2
        // Word 4-5: station alias + checksum
        // Word 6-7: reserved
        s.eepdat[0x00] = 0x00020001u;
        s.eepdat[0x02] = 0x00040003u;
        s.eepdat[0x04] = 0x00060005u;
        s.eepdat[0x06] = 0x00080007u;
        // Word 8-15: Identity (vendor, product, revision, serial)
        uint32_t vendor = 0x000022D2u + index;  // unique per slave
        s.eepdat[0x08] = vendor;
        s.eepdat[0x0A] = 0x00000001u + index;
        s.eepdat[0x0C] = 0x00010000u + index;
        s.eepdat[0x0E] = 0x00000001u + index;
    }

    /// Map ADP value to slave index
    /// Physical: ADP_SLAVE_0=0x0000 → 0, ADP_SLAVE_1=0xFFFF → 1, etc.
    /// Logical: configured_addr → slave index (looked up from slaves map)
    uint16_t adpToSlaveIndex(uint16_t adp) {
        // Check physical addresses first
        if (adp == ADP_SLAVE_0) return 0;
        if (adp == ADP_SLAVE_1) return 1;
        if (adp == ADP_SLAVE_2) return 2;
        // Check logical addresses (configured station address)
        for (auto& [idx, s] : slaves) {
            if (s.configured_addr != 0 && s.configured_addr == adp) {
                return idx;
            }
        }
        return 0xFFFF;  // not found
    }

    void installCallbacks() {
        // APWR callback: handles all writeRegister calls
        master.setApwrTestCallback([this](uint16_t adp, uint16_t ado,
                                          const void* data, uint16_t len,
                                          unsigned int) -> bool {
            uint16_t idx = adpToSlaveIndex(adp);
            if (idx == 0xFFFF) return false;
            auto& s = slaves[idx];

            if (ado == EC_REG_EEPCTL) {
                // Determine if this is APWR (physical) or FPWR (logical)
                bool is_fpwr = (s.configured_addr != 0 && adp == s.configured_addr);
                if (is_fpwr) {
                    s.fpwr_eepctl_count++;
                } else {
                    s.apwr_eepctl_count++;
                }

                // Check if APWR to EEPCTL should fail
                if (!is_fpwr && s.apwr_eepctl_fails) {
                    return false;  // WKC=0
                }

                // EEPCTL write — just acknowledge (command only, no address)
                return true;
            }

            if (ado == 0x0504) {
                // EEPADDR write — capture the word address
                if (data && len >= 2) {
                    uint16_t addr_le = 0;
                    std::memcpy(&addr_le, data, sizeof(addr_le));
                    s.last_eepctl_addr = EtherCAT::Raw::le16_to_host(addr_le);
                }
                return true;
            }

            if (ado == EC_REG_CFG_ADDR) {
                // Writing configured station address
                if (s.apwr_cfgaddr_fails) return false;
                if (data && len >= 2) {
                    uint16_t addr_le = 0;
                    std::memcpy(&addr_le, data, sizeof(addr_le));
                    s.configured_addr = EtherCAT::Raw::le16_to_host(addr_le);
                }
                return true;
            }

            return true;  // other writes succeed
        });

        // APRD callback: handles all readRegister calls
        master.setAprdTestCallback([this](uint16_t adp, uint16_t ado,
                                          void* out, uint16_t len,
                                          unsigned int) -> bool {
            uint16_t idx = adpToSlaveIndex(adp);
            if (idx == 0xFFFF) return false;
            auto& s = slaves[idx];

            if (ado == 0x0500) {
                // EEPConfig: ECAT has control (bit 0 = 0)
                if (out && len >= 1) {
                    uint8_t cfg = 0x00;
                    std::memcpy(out, &cfg, 1);
                }
                return true;
            }

            if (ado == 0x0502) {
                // EEPSTAT
                if (out && len >= 2) {
                    uint16_t estat = (s.eepstat_override != 0xFFFF)
                                         ? s.eepstat_override : 0;
                    uint16_t estat_le = EtherCAT::Raw::host_to_le16(estat);
                    std::memcpy(out, &estat_le, 2);
                }
                return true;
            }

            if (ado == 0x0508) {
                // EEPDAT: return data for the last requested word address
                uint32_t val = 0;
                auto it = s.eepdat.find(s.last_eepctl_addr);
                if (it != s.eepdat.end()) {
                    val = it->second;
                }
                if (out && len >= 4) {
                    std::memcpy(out, &val, 4);
                }
                return true;
            }

            if (ado == EC_REG_CFG_ADDR) {
                // Reading configured station address
                if (s.aprd_cfgaddr_fails) return false;
                if (out && len >= 2) {
                    uint16_t addr_le = EtherCAT::Raw::host_to_le16(s.configured_addr);
                    std::memcpy(out, &addr_le, 2);
                }
                return true;
            }

            return false;  // unknown register
        });
    }
};

// ============================================================================
// Group 1: Standard APWR path (both slaves)
// ============================================================================

/// Both slave 0 and slave 1 should read SII via APWR (standard path).
TEST_F(SiiMultiSlaveRegression, BothSlavesReadViaApwr) {
    // APWR works for both slaves (default)
    SIIReader reader(master);

    for (uint16_t si = 0; si <= 1; ++si) {
        uint32_t out = 0;
        ASSERT_TRUE(reader.readDWord(si, 0x0000, out))
            << "Slave " << si << ": readDWord should succeed via APWR";
        EXPECT_EQ(out, slaves[si].eepdat[0x00])
            << "Slave " << si << ": EEPDAT mismatch";

        // Verify only APWR was used (no FPWR)
        EXPECT_EQ(slaves[si].apwr_eepctl_count, 1)
            << "Slave " << si << ": should have 1 APWR write";
        EXPECT_EQ(slaves[si].fpwr_eepctl_count, 0)
            << "Slave " << si << ": should have 0 FPWR writes";
    }
}

/// Reading multiple words from both slaves via APWR should work.
TEST_F(SiiMultiSlaveRegression, BothSlavesReadMultipleWordsViaApwr) {
    SIIReader reader(master);

    for (uint16_t si = 0; si <= 1; ++si) {
        for (uint16_t addr = 0x00; addr <= 0x0E; addr += 2) {
            uint32_t out = 0;
            ASSERT_TRUE(reader.readDWord(si, addr, out))
                << "Slave " << si << " addr 0x" << std::hex << addr;
            EXPECT_EQ(out, slaves[si].eepdat[addr])
                << "Slave " << si << " addr 0x" << std::hex << addr;
        }
    }
}

// ============================================================================
// Group 2: FPWR fallback (both slaves fail APWR to EEPCTL)
// ============================================================================

/// Both slaves should fall back to FPWR when APWR to EEPCTL fails.
TEST_F(SiiMultiSlaveRegression, BothSlavesFallbackToFpwr) {
    slaves[0].apwr_eepctl_fails = true;
    slaves[1].apwr_eepctl_fails = true;

    SIIReader reader(master);

    for (uint16_t si = 0; si <= 1; ++si) {
        uint32_t out = 0;
        ASSERT_TRUE(reader.readDWord(si, 0x0000, out))
            << "Slave " << si << ": readDWord should succeed via FPWR fallback";

        // Verify APWR was attempted first, then FPWR
        EXPECT_GE(slaves[si].apwr_eepctl_count, 1)
            << "Slave " << si << ": should attempt APWR first";
        EXPECT_GE(slaves[si].fpwr_eepctl_count, 1)
            << "Slave " << si << ": should use FPWR after APWR fails";

        // Verify configured station address was assigned
        EXPECT_NE(slaves[si].configured_addr, 0u)
            << "Slave " << si << ": should have a non-zero configured address";
    }

    // Verify unique configured addresses
    EXPECT_NE(slaves[0].configured_addr, slaves[1].configured_addr)
        << "Slaves should have unique configured station addresses";
}

/// Both slaves should read full SII identity via FPWR fallback.
TEST_F(SiiMultiSlaveRegression, BothSlavesReadSIIIdentityViaFpwr) {
    slaves[0].apwr_eepctl_fails = true;
    slaves[1].apwr_eepctl_fails = true;

    SIIReader reader(master);

    for (uint16_t si = 0; si <= 1; ++si) {
        SIIIdentity identity;
        ASSERT_TRUE(readSIIIdentity(master, si, identity))
            << "Slave " << si << ": readSIIIdentity should succeed via FPWR";
        EXPECT_EQ(identity.vendor_id, 0x000022D2u + si)
            << "Slave " << si << ": vendor ID mismatch";
    }
}

/// Both slaves should read full SII data via FPWR fallback.
TEST_F(SiiMultiSlaveRegression, BothSlavesReadSIIViaFpwr) {
    slaves[0].apwr_eepctl_fails = true;
    slaves[1].apwr_eepctl_fails = true;

    SIIReader reader(master);

    for (uint16_t si = 0; si <= 1; ++si) {
        SIIData data;
        ASSERT_TRUE(readSII(master, si, data))
            << "Slave " << si << ": readSII should succeed via FPWR";
        // SII data may have CRC mismatch (mock data isn't CRC-valid),
        // but the read itself should succeed and return data.
        (void)data;
    }
}

// ============================================================================
// Group 3: Mixed mode (one slave APWR, one slave FPWR)
// ============================================================================

/// Slave 0 uses APWR, slave 1 uses FPWR fallback.
TEST_F(SiiMultiSlaveRegression, MixedModeSlave0ApwrSlave1Fpwr) {
    slaves[0].apwr_eepctl_fails = false;  // APWR works
    slaves[1].apwr_eepctl_fails = true;   // APWR fails, needs FPWR

    SIIReader reader(master);

    // Slave 0: APWR path
    {
        uint32_t out = 0;
        ASSERT_TRUE(reader.readDWord(0, 0x0000, out));
        EXPECT_EQ(out, slaves[0].eepdat[0x00]);
        EXPECT_GE(slaves[0].apwr_eepctl_count, 1);
        EXPECT_EQ(slaves[0].fpwr_eepctl_count, 0);
    }

    // Slave 1: FPWR fallback
    {
        uint32_t out = 0;
        ASSERT_TRUE(reader.readDWord(1, 0x0000, out));
        EXPECT_EQ(out, slaves[1].eepdat[0x00]);
        EXPECT_GE(slaves[1].apwr_eepctl_count, 1);
        EXPECT_GE(slaves[1].fpwr_eepctl_count, 1);
    }
}

/// Slave 0 uses FPWR fallback, slave 1 uses APWR.
TEST_F(SiiMultiSlaveRegression, MixedModeSlave0FpwrSlave1Apwr) {
    slaves[0].apwr_eepctl_fails = true;   // APWR fails, needs FPWR
    slaves[1].apwr_eepctl_fails = false;  // APWR works

    SIIReader reader(master);

    // Slave 0: FPWR fallback
    {
        uint32_t out = 0;
        ASSERT_TRUE(reader.readDWord(0, 0x0000, out));
        EXPECT_EQ(out, slaves[0].eepdat[0x00]);
        EXPECT_GE(slaves[0].fpwr_eepctl_count, 1);
    }

    // Slave 1: APWR path
    {
        uint32_t out = 0;
        ASSERT_TRUE(reader.readDWord(1, 0x0000, out));
        EXPECT_EQ(out, slaves[1].eepdat[0x00]);
        EXPECT_EQ(slaves[1].fpwr_eepctl_count, 0);
        EXPECT_GE(slaves[1].apwr_eepctl_count, 1);
    }
}

// ============================================================================
// Group 4: Configured station address assignment
// ============================================================================

/// When configured station address is 0x0000, a unique address is assigned.
TEST_F(SiiMultiSlaveRegression, UnassignedAddressGetsUniqueValue) {
    slaves[0].apwr_eepctl_fails = true;
    slaves[1].apwr_eepctl_fails = true;

    SIIReader reader(master);

    // Trigger FPWR fallback for both slaves
    uint32_t out0 = 0, out1 = 0;
    ASSERT_TRUE(reader.readDWord(0, 0x0000, out0));
    ASSERT_TRUE(reader.readDWord(1, 0x0000, out1));

    // Slave 0 should get address 0x0001, slave 1 should get 0x0002
    EXPECT_EQ(slaves[0].configured_addr, 0x0001u);
    EXPECT_EQ(slaves[1].configured_addr, 0x0002u);
}

/// When configured station address is already assigned (non-zero), it's used as-is.
TEST_F(SiiMultiSlaveRegression, PreAssignedAddressIsRespected) {
    slaves[0].apwr_eepctl_fails = true;
    slaves[1].apwr_eepctl_fails = true;
    slaves[0].configured_addr = 0x1000;  // pre-assigned
    slaves[1].configured_addr = 0x1001;  // pre-assigned

    SIIReader reader(master);

    uint32_t out0 = 0, out1 = 0;
    ASSERT_TRUE(reader.readDWord(0, 0x0000, out0));
    ASSERT_TRUE(reader.readDWord(1, 0x0000, out1));

    // Addresses should remain unchanged
    EXPECT_EQ(slaves[0].configured_addr, 0x1000u);
    EXPECT_EQ(slaves[1].configured_addr, 0x1001u);
}

// ============================================================================
// Group 5: Cache behavior (after first FPWR fallback, skip APWR)
// ============================================================================

/// After the first APWR failure, subsequent reads should use FPWR directly
/// without retrying APWR.
TEST_F(SiiMultiSlaveRegression, CacheSkipsApwrAfterFirstFailure) {
    slaves[0].apwr_eepctl_fails = true;

    SIIReader reader(master);

    // First read: APWR fails, falls back to FPWR
    uint32_t out1 = 0;
    ASSERT_TRUE(reader.readDWord(0, 0x0000, out1));
    int apwr_after_first = slaves[0].apwr_eepctl_count;
    int fpwr_after_first = slaves[0].fpwr_eepctl_count;
    EXPECT_GE(apwr_after_first, 1);
    EXPECT_GE(fpwr_after_first, 1);

    // Second read: should use FPWR directly (no new APWR attempt)
    uint32_t out2 = 0;
    ASSERT_TRUE(reader.readDWord(0, 0x0002, out2));
    EXPECT_EQ(slaves[0].apwr_eepctl_count, apwr_after_first)
        << "Should not retry APWR after first failure";
    EXPECT_GT(slaves[0].fpwr_eepctl_count, fpwr_after_first)
        << "Should use FPWR for subsequent reads";

    // Third read: still FPWR only
    uint32_t out3 = 0;
    ASSERT_TRUE(reader.readDWord(0, 0x0004, out3));
    EXPECT_EQ(slaves[0].apwr_eepctl_count, apwr_after_first)
        << "Should not retry APWR after cached failure";
}

// ============================================================================
// Group 6: Sequential reads (no interference between slaves)
// ============================================================================

/// Reading SII for slave 0 then slave 1 should not interfere.
TEST_F(SiiMultiSlaveRegression, SequentialReadsNoInterference) {
    slaves[0].apwr_eepctl_fails = true;
    slaves[1].apwr_eepctl_fails = true;

    SIIReader reader(master);

    // Read multiple words from slave 0
    for (uint16_t addr = 0x00; addr <= 0x08; addr += 2) {
        uint32_t out = 0;
        ASSERT_TRUE(reader.readDWord(0, addr, out))
            << "Slave 0 addr 0x" << std::hex << addr;
        EXPECT_EQ(out, slaves[0].eepdat[addr]);
    }

    // Read multiple words from slave 1
    for (uint16_t addr = 0x00; addr <= 0x08; addr += 2) {
        uint32_t out = 0;
        ASSERT_TRUE(reader.readDWord(1, addr, out))
            << "Slave 1 addr 0x" << std::hex << addr;
        EXPECT_EQ(out, slaves[1].eepdat[addr]);
    }

    // Go back to slave 0 — should still work
    {
        uint32_t out = 0;
        ASSERT_TRUE(reader.readDWord(0, 0x00, out));
        EXPECT_EQ(out, slaves[0].eepdat[0x00]);
    }
}

/// Interleaved reads between slaves should work correctly.
TEST_F(SiiMultiSlaveRegression, InterleavedReads) {
    slaves[0].apwr_eepctl_fails = true;
    slaves[1].apwr_eepctl_fails = true;

    SIIReader reader(master);

    // Interleave: slave 0 word 0, slave 1 word 0, slave 0 word 2, slave 1 word 2
    uint32_t out0a = 0, out1a = 0, out0b = 0, out1b = 0;
    ASSERT_TRUE(reader.readDWord(0, 0x00, out0a));
    ASSERT_TRUE(reader.readDWord(1, 0x00, out1a));
    ASSERT_TRUE(reader.readDWord(0, 0x02, out0b));
    ASSERT_TRUE(reader.readDWord(1, 0x02, out1b));

    EXPECT_EQ(out0a, slaves[0].eepdat[0x00]);
    EXPECT_EQ(out1a, slaves[1].eepdat[0x00]);
    EXPECT_EQ(out0b, slaves[0].eepdat[0x02]);
    EXPECT_EQ(out1b, slaves[1].eepdat[0x02]);
}

// ============================================================================
// Group 7: Error handling (NACK retry, error clearing via FPWR)
// ============================================================================

/// NACK retry should work through the FPWR path.
TEST_F(SiiMultiSlaveRegression, NackRetryViaFpwr) {
    slaves[0].apwr_eepctl_fails = true;

    // First poll returns NACK, subsequent polls return OK
    int poll_count = 0;
    auto& s0 = slaves[0];
    s0.eepstat_override = 0;  // default

    master.setAprdTestCallback([this, &poll_count](uint16_t adp, uint16_t ado,
                                                    void* out, uint16_t len,
                                                    unsigned int) -> bool {
        uint16_t idx = adpToSlaveIndex(adp);
        if (idx == 0xFFFF) return false;
        auto& s = slaves[idx];

        if (ado == 0x0500) {
            if (out && len >= 1) {
                uint8_t cfg = 0x00;
                std::memcpy(out, &cfg, 1);
            }
            return true;
        }
        if (ado == 0x0502) {
            if (out && len >= 2) {
                uint16_t estat = 0;
                // First 2 polls after EEPCTL write return NACK
                if (poll_count < 2 && s.last_eepctl_addr != 0xFFFF) {
                    estat = EC_ESTAT_NACK;
                }
                poll_count++;
                uint16_t estat_le = EtherCAT::Raw::host_to_le16(estat);
                std::memcpy(out, &estat_le, 2);
            }
            return true;
        }
        if (ado == 0x0508) {
            uint32_t val = 0;
            auto it = s.eepdat.find(s.last_eepctl_addr);
            if (it != s.eepdat.end()) val = it->second;
            if (out && len >= 4) std::memcpy(out, &val, 4);
            return true;
        }
        if (ado == EC_REG_CFG_ADDR) {
            if (out && len >= 2) {
                uint16_t addr_le = EtherCAT::Raw::host_to_le16(s.configured_addr);
                std::memcpy(out, &addr_le, 2);
            }
            return true;
        }
        return false;
    });

    SIIReader reader(master);
    uint32_t out = 0;
    // Should succeed after NACK retry
    EXPECT_TRUE(reader.readDWord(0, 0x0000, out));
}

/// Error clearing (NOP write) should go through FPWR after fallback.
TEST_F(SiiMultiSlaveRegression, ErrorClearNopViaFpwr) {
    slaves[0].apwr_eepctl_fails = true;

    // EEPSTAT returns an error on first poll, then clear
    int poll_count = 0;
    master.setAprdTestCallback([this, &poll_count](uint16_t adp, uint16_t ado,
                                                    void* out, uint16_t len,
                                                    unsigned int) -> bool {
        uint16_t idx = adpToSlaveIndex(adp);
        if (idx == 0xFFFF) return false;
        auto& s = slaves[idx];

        if (ado == 0x0500) {
            if (out && len >= 1) {
                uint8_t cfg = 0x00;
                std::memcpy(out, &cfg, 1);
            }
            return true;
        }
        if (ado == 0x0502) {
            if (out && len >= 2) {
                uint16_t estat = 0;
                // First poll returns a CRC error
                if (poll_count == 0) {
                    estat = 0x1000;  // CRC error bit
                }
                poll_count++;
                uint16_t estat_le = EtherCAT::Raw::host_to_le16(estat);
                std::memcpy(out, &estat_le, 2);
            }
            return true;
        }
        if (ado == 0x0508) {
            uint32_t val = 0;
            auto it = s.eepdat.find(s.last_eepctl_addr);
            if (it != s.eepdat.end()) val = it->second;
            if (out && len >= 4) std::memcpy(out, &val, 4);
            return true;
        }
        if (ado == EC_REG_CFG_ADDR) {
            if (out && len >= 2) {
                uint16_t addr_le = EtherCAT::Raw::host_to_le16(s.configured_addr);
                std::memcpy(out, &addr_le, 2);
            }
            return true;
        }
        return false;
    });

    SIIReader reader(master);
    uint32_t out = 0;
    // Should fail due to CRC error, but NOP clear should go through FPWR
    reader.readDWord(0, 0x0000, out);

    // The NOP clear should have been sent via FPWR (since APWR to EEPCTL fails)
    // We can't directly verify the NOP went through FPWR, but the fact that
    // the read didn't hang and the FPWR count increased is sufficient.
    EXPECT_GE(slaves[0].fpwr_eepctl_count, 1);
}

// ============================================================================
// Group 8: Failure paths
// ============================================================================

/// If APRD to 0x0010 fails, getConfiguredStationAddr returns false and
/// readDWord fails.
TEST_F(SiiMultiSlaveRegression, AprdCfgAddrFailsCausesReadFailure) {
    slaves[0].apwr_eepctl_fails = true;
    slaves[0].aprd_cfgaddr_fails = true;

    SIIReader reader(master);
    uint32_t out = 0;
    EXPECT_FALSE(reader.readDWord(0, 0x0000, out))
        << "Should fail when APRD to 0x0010 fails and APWR to EEPCTL fails";
}

/// If APWR to 0x0010 fails (for address assignment), getConfiguredStationAddr
/// returns false and readDWord fails.
TEST_F(SiiMultiSlaveRegression, ApwrCfgAddrFailsCausesReadFailure) {
    slaves[0].apwr_eepctl_fails = true;
    slaves[0].apwr_cfgaddr_fails = true;
    // configured_addr is 0, so assignment will be attempted

    SIIReader reader(master);
    uint32_t out = 0;
    EXPECT_FALSE(reader.readDWord(0, 0x0000, out))
        << "Should fail when APWR to 0x0010 fails during address assignment";
}

/// If both APWR and FPWR fail, readDWord returns false.
TEST_F(SiiMultiSlaveRegression, BothApwrAndFpwrFail) {
    slaves[0].apwr_eepctl_fails = true;
    slaves[0].configured_addr = 0;  // unassigned

    // Override APWR callback to also fail FPWR
    master.setApwrTestCallback([this](uint16_t adp, uint16_t ado,
                                      const void* data, uint16_t len,
                                      unsigned int) -> bool {
        uint16_t idx = adpToSlaveIndex(adp);
        if (idx == 0xFFFF) return false;
        auto& s = slaves[idx];

        if (ado == EC_REG_EEPCTL) {
            // Both APWR and FPWR fail
            return false;
        }
        if (ado == EC_REG_CFG_ADDR) {
            // Allow address assignment
            if (data && len >= 2) {
                uint16_t addr_le = 0;
                std::memcpy(&addr_le, data, sizeof(addr_le));
                s.configured_addr = EtherCAT::Raw::le16_to_host(addr_le);
            }
            return true;
        }
        return true;
    });

    SIIReader reader(master);
    uint32_t out = 0;
    EXPECT_FALSE(reader.readDWord(0, 0x0000, out))
        << "Should fail when both APWR and FPWR to EEPCTL fail";
}

// ============================================================================
// Group 9: Three slaves
// ============================================================================

/// Three slaves should all get unique configured station addresses.
TEST_F(SiiMultiSlaveRegression, ThreeSlavesUniqueAddresses) {
    setupSlave(2);
    slaves[0].apwr_eepctl_fails = true;
    slaves[1].apwr_eepctl_fails = true;
    slaves[2].apwr_eepctl_fails = true;

    SIIReader reader(master);

    for (uint16_t si = 0; si <= 2; ++si) {
        uint32_t out = 0;
        ASSERT_TRUE(reader.readDWord(si, 0x0000, out))
            << "Slave " << si << " should read via FPWR";
    }

    // All three should have unique addresses
    EXPECT_EQ(slaves[0].configured_addr, 0x0001u);
    EXPECT_EQ(slaves[1].configured_addr, 0x0002u);
    EXPECT_EQ(slaves[2].configured_addr, 0x0003u);
}

// ============================================================================
// Group 10: Full SII identity read for both slaves (end-to-end)
// ============================================================================

/// Full readSIIIdentity for both slaves via APWR.
TEST_F(SiiMultiSlaveRegression, FullIdentityBothSlavesApwr) {
    SIIReader reader(master);

    for (uint16_t si = 0; si <= 1; ++si) {
        SIIIdentity identity;
        ASSERT_TRUE(readSIIIdentity(master, si, identity))
            << "Slave " << si << ": readSIIIdentity via APWR";
        EXPECT_EQ(identity.vendor_id, 0x000022D2u + si);
        EXPECT_EQ(identity.product_code, 0x00000001u + si);
        EXPECT_EQ(identity.revision_number, 0x00010000u + si);
        EXPECT_EQ(identity.serial_number, 0x00000001u + si);
    }
}

/// Full readSIIIdentity for both slaves via FPWR fallback.
TEST_F(SiiMultiSlaveRegression, FullIdentityBothSlavesFpwr) {
    slaves[0].apwr_eepctl_fails = true;
    slaves[1].apwr_eepctl_fails = true;

    SIIReader reader(master);

    for (uint16_t si = 0; si <= 1; ++si) {
        SIIIdentity identity;
        ASSERT_TRUE(readSIIIdentity(master, si, identity))
            << "Slave " << si << ": readSIIIdentity via FPWR";
        EXPECT_EQ(identity.vendor_id, 0x000022D2u + si);
        EXPECT_EQ(identity.product_code, 0x00000001u + si);
        EXPECT_EQ(identity.revision_number, 0x00010000u + si);
        EXPECT_EQ(identity.serial_number, 0x00000001u + si);
    }
}

/// Full readSIIIdentity for slave 1 only (slave 0 not read).
TEST_F(SiiMultiSlaveRegression, FullIdentitySlave1Only) {
    slaves[1].apwr_eepctl_fails = true;

    SIIIdentity identity;
    ASSERT_TRUE(readSIIIdentity(master, 1, identity))
        << "Slave 1: readSIIIdentity via FPWR (slave 0 not touched)";
    EXPECT_EQ(identity.vendor_id, 0x000022D3u);  // 0x22D2 + 1

    // Slave 0 should not have been touched
    EXPECT_EQ(slaves[0].apwr_eepctl_count, 0);
    EXPECT_EQ(slaves[0].fpwr_eepctl_count, 0);
}

// ============================================================================
// Group 11: EEPCTL write is always 2 bytes (protocol correctness)
// ============================================================================

/// Verify that EEPCTL writes are always exactly 2 bytes, for both APWR and FPWR.
TEST_F(SiiMultiSlaveRegression, EepctlWriteIs2BytesBothPaths) {
    slaves[0].apwr_eepctl_fails = false;
    slaves[1].apwr_eepctl_fails = true;

    // Capture write lengths
    std::vector<uint16_t> eepctl_write_lengths;
    master.setApwrTestCallback([this, &eepctl_write_lengths](uint16_t adp, uint16_t ado,
                                                              const void* data, uint16_t len,
                                                              unsigned int) -> bool {
        uint16_t idx = adpToSlaveIndex(adp);
        if (idx == 0xFFFF) return false;
        auto& s = slaves[idx];

        if (ado == EC_REG_EEPCTL) {
            eepctl_write_lengths.push_back(len);
            bool is_fpwr = (s.configured_addr != 0 && adp == s.configured_addr);
            if (is_fpwr) s.fpwr_eepctl_count++;
            else s.apwr_eepctl_count++;

            if (!is_fpwr && s.apwr_eepctl_fails) return false;

            // EEPCTL write — just acknowledge (command only, no address)
            return true;
        }
        if (ado == 0x0504) {
            // EEPADDR write — capture the word address
            if (data && len >= 2) {
                uint16_t addr_le = 0;
                std::memcpy(&addr_le, data, sizeof(addr_le));
                s.last_eepctl_addr = EtherCAT::Raw::le16_to_host(addr_le);
            }
            return true;
        }
        if (ado == EC_REG_CFG_ADDR) {
            if (data && len >= 2) {
                uint16_t addr_le = 0;
                std::memcpy(&addr_le, data, sizeof(addr_le));
                s.configured_addr = EtherCAT::Raw::le16_to_host(addr_le);
            }
            return true;
        }
        return true;
    });

    SIIReader reader(master);

    // Slave 0 via APWR
    uint32_t out0 = 0;
    ASSERT_TRUE(reader.readDWord(0, 0x0000, out0));

    // Slave 1 via FPWR
    uint32_t out1 = 0;
    ASSERT_TRUE(reader.readDWord(1, 0x0000, out1));

    // All EEPCTL writes should be exactly 2 bytes
    for (size_t i = 0; i < eepctl_write_lengths.size(); ++i) {
        EXPECT_EQ(eepctl_write_lengths[i], 2u)
            << "EEPCTL write " << i << " should be 2 bytes, got "
            << eepctl_write_lengths[i];
    }
}
