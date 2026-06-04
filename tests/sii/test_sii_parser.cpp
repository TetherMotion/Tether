/**
 * @file test_sii_parser.cpp
 * @brief Comprehensive unit tests for SII EEPROM mailbox parser
 * 
 * Tests cover:
 * - Valid mailbox configuration parsing
 * - Missing/invalid mailbox data
 * - Default fallback behavior
 * - Edge cases and corner cases
 * - Error handling
 */

#include <gtest/gtest.h>
#include "sii/SIIReader.hpp"
#include "sii/SIIParser.hpp"
#include "ethercat/raw/internal.hpp"
#include "tether/ethercat/EtherCATMaster.hpp"

#include <cstdint>
#include <cstring>
#include <map>

using namespace EtherCAT::SII;
using namespace EtherCAT::Raw;

// ============================================================================
// Test Helpers and Mocks
// ============================================================================

/**
 * @brief Helper class to simulate SII EEPROM data for testing
 */
class MockSIIEEPROM {
public:
    MockSIIEEPROM() {
        reset();
    }
    
    /**
     * @brief Reset to default empty EEPROM
     */
    void reset() {
        data_.clear();
        last_cmd_addr_ = 0xFFFF;
        simulate_failure_ = false;
    }
    
    /**
     * @brief Set 32-bit value at word address (two consecutive 16-bit words)
     */
    void setDWord(uint16_t word_addr, uint32_t value) {
        data_[word_addr] = static_cast<uint16_t>(value & 0xFFFF);
        data_[word_addr + 1] = static_cast<uint16_t>((value >> 16) & 0xFFFF);
    }
    
    /**
     * @brief Set 16-bit value at word address
     */
    void setWord(uint16_t word_addr, uint16_t value) {
        data_[word_addr] = value;
    }
    
    /**
     * @brief Configure valid mailbox in SII
     */
    void setValidMailbox(uint16_t rx_addr, uint16_t rx_size,
                         uint16_t tx_addr, uint16_t tx_size,
                         uint16_t protocols) {
        // Bootstrap mailbox (optional, set to zeros)
        setWord(0x14, 0);  // bootstrap_rx_offset
        setWord(0x15, 0);  // bootstrap_rx_size
        setWord(0x16, 0);  // bootstrap_tx_offset
        setWord(0x17, 0);  // bootstrap_tx_size
        
        // Standard mailbox
        setWord(0x18, rx_addr);   // std_rx_offset (slave RX = master write)
        setWord(0x19, rx_size);   // std_rx_size
        setWord(0x1A, tx_addr);   // std_tx_offset (slave TX = master read)
        setWord(0x1B, tx_size);   // std_tx_size
        setWord(0x1C, protocols); // mailbox_protocols
    }
    
    /**
     * @brief Configure no mailbox (all zeros)
     */
    void setNoMailbox() {
        setValidMailbox(0, 0, 0, 0, 0);
    }
    
    /**
     * @brief Simulate read/write failure
     */
    void setSimulateFailure(bool fail) {
        simulate_failure_ = fail;
    }
    
    /**
     * @brief Install mock callbacks
     */
    void installMocks(EtherCAT::EtherCATMaster& master) {
        master_ = &master;
        master.setApwrTestCallback([this](uint16_t adp, uint16_t ado,
                          const void* data, uint16_t len, unsigned int ms) {
            if (simulate_failure_) return false;
            
            // Capture EEPROM read command address
            if (ado == 0x0502 && data && len >= 4) {
                uint16_t addr_le = 0;
                std::memcpy(&addr_le, reinterpret_cast<const uint8_t*>(data) + 2, sizeof(addr_le));
                last_cmd_addr_ = addr_le;
                return true;
            }
            return true;
        });
        
        master.setAprdTestCallback([this](uint16_t adp, uint16_t ado,
                          void* out, uint16_t len, unsigned int ms) {
            if (simulate_failure_) return false;
            
            // EEPSTAT: indicate not busy
            if (ado == 0x0502) {
                if (out && len >= 2) {
                    uint16_t estat_le = 0;  // Not busy
                    std::memcpy(out, &estat_le, 2);
                }
                return true;
            }
            
            // EEPDAT: return stored 32-bit value (two 16-bit words)
            if (ado == 0x0508) {
                if (out && len >= 4) {
                    uint16_t lo = data_.count(last_cmd_addr_) ? data_[last_cmd_addr_] : 0;
                    uint16_t hi = data_.count(last_cmd_addr_ + 1) ? data_[last_cmd_addr_ + 1] : 0;
                    uint32_t val = static_cast<uint32_t>(lo) | (static_cast<uint32_t>(hi) << 16);
                    std::memcpy(out, &val, 4);
                }
                return true;
            }
            
            return false;
        });
    }
    
    /**
     * @brief Remove mock callbacks
     */
    void removeMocks() {
        if (master_) {
            master_->setAprdTestCallback(nullptr);
            master_->setApwrTestCallback(nullptr);
        }
    }

private:
    std::map<uint16_t, uint16_t> data_;  ///< Simulated EEPROM data (word address → value)
    uint16_t last_cmd_addr_{0xFFFF};     ///< Last requested read address
    bool simulate_failure_{false};        ///< Simulate read/write failures
    EtherCAT::EtherCATMaster* master_{nullptr};
};

// ============================================================================
// Test Fixture
// ============================================================================

class SIIMailboxParserTest : public ::testing::Test {
protected:
    void SetUp() override {
        mock_eeprom_.reset();
        mock_eeprom_.installMocks(master_);
    }
    
    void TearDown() override {
        mock_eeprom_.removeMocks();
    }
    
    MockSIIEEPROM mock_eeprom_;
    EtherCAT::EtherCATMaster master_;
};

// ============================================================================
// Basic Functionality Tests
// ============================================================================

/**
 * @test Valid mailbox configuration is parsed correctly
 */
TEST_F(SIIMailboxParserTest, ValidMailboxConfiguration) {
    // Setup: Configure valid mailbox in mock EEPROM
    // Standard EtherCAT convention: SM0 (Receive/MbxIn), SM1 (Send/MbxOut)
    // SII RX (slave receives, master writes, SM0) at LOWER address
    // SII TX (slave transmits, master reads, SM1) at HIGHER address
    mock_eeprom_.setValidMailbox(
        0x1000, 128,   // RX: addr=0x1000, size=128 (Receive/MbxIn/M→S, lower addr)
        0x1400, 64,    // TX: addr=0x1400, size=64  (Send/MbxOut/S→M, higher addr)
        MBX_PROTO_COE | MBX_PROTO_EOE  // Protocols: CoE + EoE
    );

    // Execute: Read mailbox configuration
    uint16_t wr_addr = 0, wr_len = 0, rd_addr = 0, rd_len = 0, proto = 0;
    bool result = configure_mailbox_from_sii(
        master_, 0x0000,  // slave 0
        &wr_addr, &wr_len, &rd_addr, &rd_len, &proto
    );

    // Verify
    EXPECT_TRUE(result);
    EXPECT_EQ(wr_addr, 0x1000);  // Master write = slave RX = Receive/MbxIn
    EXPECT_EQ(wr_len, 128);
    EXPECT_EQ(rd_addr, 0x1400);  // Master read = slave TX = Send/MbxOut
    EXPECT_EQ(rd_len, 64);
    EXPECT_EQ(proto, static_cast<uint16_t>(MBX_PROTO_COE | MBX_PROTO_EOE));
}

/**
 * @test SII with RX > TX addresses (non-standard ordering) is kept as-is
 * Some devices have mailbox addresses where SM0 comes at higher address
 * than SM1 — we log a warning but keep the SII values unchanged.
 */
TEST_F(SIIMailboxParserTest, ReversedAddressesKeptAsIs) {
    // Setup: RX (Receive/SM0) at higher address than TX (Send/SM1) — non-standard but valid
    mock_eeprom_.setValidMailbox(
        0x1400, 128,   // RX: addr=0x1400 (Receive/SM0 at higher address — unusual)
        0x1000, 64,    // TX: addr=0x1000 (Send/SM1 at lower address — unusual)
        MBX_PROTO_COE
    );
    
    // Execute: Read mailbox configuration
    uint16_t wr_addr = 0, wr_len = 0, rd_addr = 0, rd_len = 0;
    bool result = configure_mailbox_from_sii(
        master_, 0x0000,
        &wr_addr, &wr_len, &rd_addr, &rd_len, nullptr
    );
    
    // Verify: Addresses kept as-is from SII (no swap correction)
    EXPECT_TRUE(result);
    EXPECT_EQ(wr_addr, 0x1400);  // Kept from SII (RX/Receive)
    EXPECT_EQ(wr_len, 128);      // Kept from SII
    EXPECT_EQ(rd_addr, 0x1000);  // Kept from SII (TX/Send)
    EXPECT_EQ(rd_len, 64);       // Kept from SII
}

/**
 * @test Multiple protocol flags are preserved
 */
TEST_F(SIIMailboxParserTest, MultipleProtocols) {
    uint16_t all_protocols = MBX_PROTO_AOE | MBX_PROTO_EOE | MBX_PROTO_COE | 
                            MBX_PROTO_FOE | MBX_PROTO_SOE | MBX_PROTO_VOE;
    
    mock_eeprom_.setValidMailbox(0x1000, 128, 0x1400, 64, all_protocols);
    
    uint16_t proto = 0;
    bool result = configure_mailbox_from_sii(master_, 0, nullptr, nullptr, 
                                            nullptr, nullptr, &proto);
    
    EXPECT_TRUE(result);
    EXPECT_EQ(proto, all_protocols);
}

/**
 * @test Large mailbox sizes are accepted
 */
TEST_F(SIIMailboxParserTest, LargeMailboxSize) {
    mock_eeprom_.setValidMailbox(0x1000, 1024, 0x1400, 1024, MBX_PROTO_COE);
    
    uint16_t wr_len = 0, rd_len = 0;
    bool result = configure_mailbox_from_sii(master_, 0, nullptr, &wr_len, 
                                            nullptr, &rd_len, nullptr);
    
    EXPECT_TRUE(result);
    EXPECT_EQ(wr_len, 1024);
    EXPECT_EQ(rd_len, 1024);
}

/**
 * @test Different slave indices work correctly
 */
TEST_F(SIIMailboxParserTest, DifferentSlaveIndices) {
    mock_eeprom_.setValidMailbox(0x1000, 128, 0x1400, 64, MBX_PROTO_COE);
    
    // Slave 0 (ADP = 0x0000)
    uint16_t wr_addr = 0;
    EXPECT_TRUE(configure_mailbox_from_sii(master_, 0, &wr_addr, nullptr, 
                                          nullptr, nullptr, nullptr));
    EXPECT_EQ(wr_addr, 0x1000);
    
    // Slave 1 (ADP = 0xFFFF = -1)
    EXPECT_TRUE(configure_mailbox_from_sii(master_, 1, &wr_addr, nullptr,
                                          nullptr, nullptr, nullptr));
    EXPECT_EQ(wr_addr, 0x1000);

    // Slave 2 (ADP = 0xFFFE = -2)
    EXPECT_TRUE(configure_mailbox_from_sii(master_, 2, &wr_addr, nullptr,
                                          nullptr, nullptr, nullptr));
    EXPECT_EQ(wr_addr, 0x1000);
}

// ============================================================================
// Fallback to Defaults Tests
// ============================================================================

/**
 * @test No mailbox in SII falls back to defaults
 */
TEST_F(SIIMailboxParserTest, NoMailboxFallsBackToDefaults) {
    // Setup: No mailbox configured (all zeros)
    mock_eeprom_.setNoMailbox();
    
    // Execute
    uint16_t wr_addr = 0, wr_len = 0, rd_addr = 0, rd_len = 0, proto = 0;
    bool result = configure_mailbox_from_sii(master_, 0, &wr_addr, &wr_len, 
                                            &rd_addr, &rd_len, &proto);
    
    // Verify: Should return defaults
    EXPECT_TRUE(result);
    EXPECT_EQ(wr_addr, 0x1000);  // Default Receive addr (MbxIn/M→S, SM1 per ETG standard)
    EXPECT_EQ(wr_len, 256);      // Default Receive size
    EXPECT_EQ(rd_addr, 0x1400);  // Default Send addr (MbxOut/S→M, SM0 per ETG standard)
    EXPECT_EQ(rd_len, 256);      // Default Send size
    EXPECT_EQ(proto, static_cast<uint16_t>(MBX_PROTO_COE | MBX_PROTO_EOE | MBX_PROTO_AOE));
}

/**
 * @test Failed SII read falls back to defaults
 */
TEST_F(SIIMailboxParserTest, FailedSIIReadFallsBackToDefaults) {
    // Setup: Simulate read failure
    mock_eeprom_.setSimulateFailure(true);
    
    // Execute
    uint16_t wr_addr = 0, wr_len = 0, rd_addr = 0, rd_len = 0, proto = 0;
    bool result = configure_mailbox_from_sii(master_, 0, &wr_addr, &wr_len, 
                                            &rd_addr, &rd_len, &proto);
    
    // Verify: Should still succeed with defaults
    EXPECT_TRUE(result);
    EXPECT_EQ(wr_addr, 0x1000);
    EXPECT_EQ(wr_len, 256);
    EXPECT_EQ(rd_addr, 0x1400);
    EXPECT_EQ(rd_len, 256);
    EXPECT_EQ(proto, static_cast<uint16_t>(MBX_PROTO_COE | MBX_PROTO_EOE | MBX_PROTO_AOE));
}

// ============================================================================
// Edge Cases and Validation Tests
// ============================================================================

/**
 * @test Zero RX size triggers default fallback
 */
TEST_F(SIIMailboxParserTest, ZeroRxSizeFallsBack) {
    mock_eeprom_.setValidMailbox(0x1400, 0, 0x1000, 64, MBX_PROTO_COE);
    
    uint16_t wr_len = 0xAAAA;  // Sentinel value
    bool result = configure_mailbox_from_sii(master_, 0, nullptr, &wr_len, 
                                            nullptr, nullptr, nullptr);
    
    EXPECT_TRUE(result);
    EXPECT_EQ(wr_len, 256);  // Default, not 0 (updated fallback default)
}

/**
 * @test Zero TX size triggers default fallback
 */
TEST_F(SIIMailboxParserTest, ZeroTxSizeFallsBack) {
    mock_eeprom_.setValidMailbox(0x1400, 128, 0x1000, 0, MBX_PROTO_COE);
    
    uint16_t rd_len = 0xAAAA;  // Sentinel value
    bool result = configure_mailbox_from_sii(master_, 0, nullptr, nullptr, 
                                            nullptr, &rd_len, nullptr);
    
    EXPECT_TRUE(result);
    EXPECT_EQ(rd_len, 256);  // Default, not 0 (updated fallback default)
}

/**
 * @test Both zero sizes trigger default fallback
 */
TEST_F(SIIMailboxParserTest, BothZeroSizesFallBack) {
    mock_eeprom_.setValidMailbox(0x1400, 0, 0x1000, 0, MBX_PROTO_COE);
    
    uint16_t wr_addr = 0, wr_len = 0, rd_addr = 0, rd_len = 0;
    bool result = configure_mailbox_from_sii(master_, 0, &wr_addr, &wr_len, 
                                            &rd_addr, &rd_len, nullptr);
    
    EXPECT_TRUE(result);
    EXPECT_EQ(wr_addr, 0x1000);  // Default Receive addr (SM0)
    EXPECT_EQ(wr_len, 256);
    EXPECT_EQ(rd_addr, 0x1400);  // Default Send addr (SM1)
    EXPECT_EQ(rd_len, 256);
}

/**
 * @test Overlapping mailboxes (same address) trigger default fallback
 */
TEST_F(SIIMailboxParserTest, OverlappingMailboxesFallBack) {
    // RX and TX at same address - invalid configuration
    mock_eeprom_.setValidMailbox(0x1000, 128, 0x1000, 64, MBX_PROTO_COE);
    
    uint16_t wr_addr = 0, rd_addr = 0;
    bool result = configure_mailbox_from_sii(master_, 0, &wr_addr, nullptr, 
                                            &rd_addr, nullptr, nullptr);
    
    EXPECT_TRUE(result);
    EXPECT_EQ(wr_addr, 0x1000);  // Default Receive addr (SM0)
    EXPECT_EQ(rd_addr, 0x1400);  // Default Send addr (SM1)
}

/**
 * @test Small but valid mailboxes are accepted (with warning logged)
 */
TEST_F(SIIMailboxParserTest, SmallMailboxAccepted) {
    // 32 bytes is unusually small but should be accepted
    mock_eeprom_.setValidMailbox(0x1400, 32, 0x1000, 32, MBX_PROTO_COE);
    
    uint16_t wr_len = 0, rd_len = 0;
    bool result = configure_mailbox_from_sii(master_, 0, nullptr, &wr_len, 
                                            nullptr, &rd_len, nullptr);
    
    EXPECT_TRUE(result);
    EXPECT_EQ(wr_len, 32);  // Should accept small size (not use defaults)
    EXPECT_EQ(rd_len, 32);
}

/**
 * @test Very small mailboxes (< 32 bytes) are still accepted with warning
 */
TEST_F(SIIMailboxParserTest, VerySmallMailboxAcceptedWithWarning) {
    // 16 bytes is very small but should still be accepted
    mock_eeprom_.setValidMailbox(0x1400, 16, 0x1000, 16, MBX_PROTO_COE);
    
    uint16_t wr_len = 0, rd_len = 0;
    bool result = configure_mailbox_from_sii(master_, 0, nullptr, &wr_len, 
                                            nullptr, &rd_len, nullptr);
    
    EXPECT_TRUE(result);
    EXPECT_EQ(wr_len, 16);  // Should accept very small size
    EXPECT_EQ(rd_len, 16);
}

// ============================================================================
// NULL Pointer Tests
// ============================================================================

/**
 * @test NULL output pointers are handled gracefully
 */
TEST_F(SIIMailboxParserTest, NullPointersHandled) {
    mock_eeprom_.setValidMailbox(0x1400, 128, 0x1000, 64, MBX_PROTO_COE);
    
    // Should not crash with NULL pointers
    bool result = configure_mailbox_from_sii(master_, 0, nullptr, nullptr, 
                                            nullptr, nullptr, nullptr);
    
    EXPECT_TRUE(result);
}

/**
 * @test Partial NULL pointers work correctly
 */
TEST_F(SIIMailboxParserTest, PartialNullPointers) {
    mock_eeprom_.setValidMailbox(0x1400, 128, 0x1000, 64, MBX_PROTO_COE);
    
    uint16_t wr_addr = 0, proto = 0;
    
    // Only request write addr and protocols, others NULL
    bool result = configure_mailbox_from_sii(master_, 0, &wr_addr, nullptr, 
                                            nullptr, nullptr, &proto);
    
    EXPECT_TRUE(result);
    EXPECT_EQ(wr_addr, 0x1400);
    EXPECT_EQ(proto, static_cast<uint16_t>(MBX_PROTO_COE));
}

// ============================================================================
// Protocol Edge Cases
// ============================================================================

/**
 * @test No protocols specified (zero) is accepted
 */
TEST_F(SIIMailboxParserTest, NoProtocolsSpecified) {
    mock_eeprom_.setValidMailbox(0x1400, 128, 0x1000, 64, 0);
    
    uint16_t proto = 0xFFFF;  // Sentinel
    bool result = configure_mailbox_from_sii(master_, 0, nullptr, nullptr, 
                                            nullptr, nullptr, &proto);
    
    EXPECT_TRUE(result);
    EXPECT_EQ(proto, 0);  // Should preserve zero protocols
}

/**
 * @test Single protocol flag works correctly
 */
TEST_F(SIIMailboxParserTest, SingleProtocol) {
    mock_eeprom_.setValidMailbox(0x1400, 128, 0x1000, 64, MBX_PROTO_FOE);
    
    uint16_t proto = 0;
    bool result = configure_mailbox_from_sii(master_, 0, nullptr, nullptr, 
                                            nullptr, nullptr, &proto);
    
    EXPECT_TRUE(result);
    EXPECT_EQ(proto, static_cast<uint16_t>(MBX_PROTO_FOE));
}

// ============================================================================
// Address Edge Cases
// ============================================================================

/**
 * @test Maximum valid addresses are accepted
 */
TEST_F(SIIMailboxParserTest, MaximumAddresses) {
    mock_eeprom_.setValidMailbox(0xFFFF, 128, 0xFFF0, 64, MBX_PROTO_COE);
    
    uint16_t wr_addr = 0, rd_addr = 0;
    bool result = configure_mailbox_from_sii(master_, 0, &wr_addr, nullptr, 
                                            &rd_addr, nullptr, nullptr);
    
    EXPECT_TRUE(result);
    EXPECT_EQ(wr_addr, 0xFFFF);  // Should accept max address
    EXPECT_EQ(rd_addr, 0xFFF0);
}

/**
 * @test Zero addresses with non-zero sizes are accepted
 */
TEST_F(SIIMailboxParserTest, ZeroAddresses) {
    // RX at higher address, TX at zero (lower) - correct order
    mock_eeprom_.setValidMailbox(0x0100, 128, 0x0000, 64, MBX_PROTO_COE);
    
    uint16_t wr_addr = 0xFFFF, rd_addr = 0xFFFF;  // Sentinels
    bool result = configure_mailbox_from_sii(master_, 0, &wr_addr, nullptr, 
                                            &rd_addr, nullptr, nullptr);
    
    EXPECT_TRUE(result);
    EXPECT_EQ(wr_addr, 0x0100);  // RX (higher address)
    EXPECT_EQ(rd_addr, 0x0000);  // TX (zero address, lower)
}

// ============================================================================
// Size Edge Cases
// ============================================================================

/**
 * @test Maximum size values are accepted
 */
TEST_F(SIIMailboxParserTest, MaximumSizes) {
    mock_eeprom_.setValidMailbox(0x1000, 0xFFFF, 0x2000, 0xFFFF, MBX_PROTO_COE);
    
    uint16_t wr_len = 0, rd_len = 0;
    bool result = configure_mailbox_from_sii(master_, 0, nullptr, &wr_len, 
                                            nullptr, &rd_len, nullptr);
    
    EXPECT_TRUE(result);
    EXPECT_EQ(wr_len, 0xFFFF);  // Should accept max size
    EXPECT_EQ(rd_len, 0xFFFF);
}

/**
 * @test Odd sizes are accepted
 */
TEST_F(SIIMailboxParserTest, OddSizes) {
    mock_eeprom_.setValidMailbox(0x1400, 127, 0x1000, 63, MBX_PROTO_COE);
    
    uint16_t wr_len = 0, rd_len = 0;
    bool result = configure_mailbox_from_sii(master_, 0, nullptr, &wr_len, 
                                            nullptr, &rd_len, nullptr);
    
    EXPECT_TRUE(result);
    EXPECT_EQ(wr_len, 127);  // Odd sizes are valid
    EXPECT_EQ(rd_len, 63);
}

// ============================================================================
// Integration with SII Parser Tests
// ============================================================================

/**
 * @test Bootstrap mailbox fields are parsed but not used by configure_mailbox_from_sii
 * (function uses standard mailbox only)
 */
TEST_F(SIIMailboxParserTest, BootstrapMailboxParsedButNotUsed) {
    // Setup bootstrap mailbox data
    mock_eeprom_.setWord(0x14, 0x0800);  // bootstrap_rx_offset
    mock_eeprom_.setWord(0x15, 64);      // bootstrap_rx_size
    mock_eeprom_.setWord(0x16, 0x0900);  // bootstrap_tx_offset
    mock_eeprom_.setWord(0x17, 32);      // bootstrap_tx_size
    
    // Standard mailbox
    mock_eeprom_.setValidMailbox(0x1400, 128, 0x1000, 64, MBX_PROTO_COE);
    
    // The function should return standard mailbox, not bootstrap
    uint16_t wr_addr = 0, rd_addr = 0;
    bool result = configure_mailbox_from_sii(master_, 0, &wr_addr, nullptr, 
                                            &rd_addr, nullptr, nullptr);
    
    EXPECT_TRUE(result);
    EXPECT_EQ(wr_addr, 0x1400);  // Standard mailbox, not bootstrap
    EXPECT_EQ(rd_addr, 0x1000);
}

/**
 * @test Identity data parsing doesn't affect mailbox configuration
 */
TEST_F(SIIMailboxParserTest, IdentityDataDoesNotAffectMailbox) {
    // Set some identity data
    mock_eeprom_.setDWord(0x08, 0x11223344);  // vendor_id
    mock_eeprom_.setDWord(0x0A, 0x55667788);  // product_code
    
    // Set mailbox
    mock_eeprom_.setValidMailbox(0x1400, 128, 0x1000, 64, MBX_PROTO_COE);
    
    uint16_t wr_addr = 0;
    bool result = configure_mailbox_from_sii(master_, 0, &wr_addr, nullptr, 
                                            nullptr, nullptr, nullptr);
    
    EXPECT_TRUE(result);
    EXPECT_EQ(wr_addr, 0x1400);  // Mailbox unaffected by identity
}

// ============================================================================
// Consistency Tests
// ============================================================================

/**
 * @test Repeated calls with same data produce same results
 */
TEST_F(SIIMailboxParserTest, RepeatedCallsConsistent) {
    mock_eeprom_.setValidMailbox(0x1400, 128, 0x1000, 64, MBX_PROTO_COE);
    
    uint16_t wr_addr1 = 0, wr_len1 = 0, rd_addr1 = 0, rd_len1 = 0, proto1 = 0;
    bool result1 = configure_mailbox_from_sii(master_, 0, &wr_addr1, &wr_len1, 
                                             &rd_addr1, &rd_len1, &proto1);
    
    uint16_t wr_addr2 = 0, wr_len2 = 0, rd_addr2 = 0, rd_len2 = 0, proto2 = 0;
    bool result2 = configure_mailbox_from_sii(master_, 0, &wr_addr2, &wr_len2, 
                                             &rd_addr2, &rd_len2, &proto2);
    
    EXPECT_TRUE(result1);
    EXPECT_TRUE(result2);
    EXPECT_EQ(wr_addr1, wr_addr2);
    EXPECT_EQ(wr_len1, wr_len2);
    EXPECT_EQ(rd_addr1, rd_addr2);
    EXPECT_EQ(rd_len1, rd_len2);
    EXPECT_EQ(proto1, proto2);
}

/**
 * @test Changing mock data produces different results
 */
TEST_F(SIIMailboxParserTest, DifferentDataProducesDifferentResults) {
    // First configuration
    mock_eeprom_.setValidMailbox(0x1400, 128, 0x1000, 64, MBX_PROTO_COE);
    
    uint16_t wr_addr1 = 0, proto1 = 0;
    configure_mailbox_from_sii(master_, 0, &wr_addr1, nullptr, 
                              nullptr, nullptr, &proto1);
    
    // Second configuration (different)
    mock_eeprom_.setValidMailbox(0x2000, 256, 0x2200, 128, MBX_PROTO_FOE);
    
    uint16_t wr_addr2 = 0, proto2 = 0;
    configure_mailbox_from_sii(master_, 0, &wr_addr2, nullptr, 
                              nullptr, nullptr, &proto2);
    
    EXPECT_NE(wr_addr1, wr_addr2);
    EXPECT_NE(proto1, proto2);
}

// ============================================================================
// Stress Tests
// ============================================================================

/**
 * @test Function handles many consecutive calls without issues
 */
TEST_F(SIIMailboxParserTest, ManyConsecutiveCalls) {
    mock_eeprom_.setValidMailbox(0x1400, 128, 0x1000, 64, MBX_PROTO_COE);
    
    for (int i = 0; i < 100; i++) {
        uint16_t wr_addr = 0;
        bool result = configure_mailbox_from_sii(master_, 0, &wr_addr, nullptr, 
                                                nullptr, nullptr, nullptr);
        EXPECT_TRUE(result);
        EXPECT_EQ(wr_addr, 0x1400);
    }
}
