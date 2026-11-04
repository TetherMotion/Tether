/**
 * @file test_slave_pdo_registration.cpp
 * @brief Coverage for Slave PDO registration paths — fixed PDOs, SII- and
 *        ESI-driven registration and the CiA 301 PDO-assignment writes.
 */

#include <gtest/gtest.h>

#include <cstring>
#include <memory>
#include <unordered_map>
#include <vector>

#include "mocks/EmulatedBus.hpp"
#include "tether/ethercat/ESIFile.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/Slave.hpp"

using namespace EtherCAT;
using EtherCAT::Emulator::SlaveEmulator;

namespace {

class SlavePdoRegistrationTest : public ::testing::Test {
protected:
    void TearDown() override {
        bus_.shutdown();
        master_.packetRouter().shutdown();
    }

    void bringUp() {
        auto slave = std::make_unique<SlaveEmulator>();
        EtherCAT::Emulator::SIIConfig cfg;
        cfg.vendor_id = 0x00000002;
        cfg.product_code = 0x0FA13052;
        cfg.mailbox_protocol = 0x04;
        slave->setSIIConfig(cfg);
        bus_.net.addSlave(std::move(slave));

        bus_.attach(master_);
        master_.start(bus_.iface, kEmulatedMac);
        master_.initSlaves(1);
    }

    EmulatedBus bus_;
    Master master_;
};

}  // namespace

// ============================================================================
// registerFixedPDOs
// ============================================================================

TEST_F(SlavePdoRegistrationTest, RegisterFixedPDOsBothDirections) {
    bringUp();
    auto& sl = master_.slave(0);

    SIIPDOConfig cfg{};
    cfg.rxpdo_index = 0x1600;
    cfg.rxpdo_size = 8;
    cfg.has_rxpdo = true;
    cfg.txpdo_index = 0x1A00;
    cfg.txpdo_size = 12;
    cfg.has_txpdo = true;

    EXPECT_EQ(sl.registerFixedPDOs(cfg), SlaveError::Ok);

    EXPECT_EQ(master_.pdoForSlave(0).mapping().total_rxpdo_bytes(), 8u);
    EXPECT_EQ(master_.pdoForSlave(0).mapping().total_txpdo_bytes(), 12u);
}

TEST_F(SlavePdoRegistrationTest, RegisterFixedPDOsSingleDirections) {
    bringUp();
    auto& sl = master_.slave(0);

    SIIPDOConfig rx{};
    rx.rxpdo_index = 0x1601;
    rx.rxpdo_size = 4;
    rx.has_rxpdo = true;
    EXPECT_EQ(sl.registerFixedPDOs(rx), SlaveError::Ok);
    EXPECT_EQ(master_.pdoForSlave(0).mapping().total_rxpdo_bytes(), 4u);

    SIIPDOConfig tx{};
    tx.txpdo_index = 0x1A01;
    tx.txpdo_size = 6;
    tx.has_txpdo = true;
    EXPECT_EQ(sl.registerFixedPDOs(tx), SlaveError::Ok);
    EXPECT_EQ(master_.pdoForSlave(0).mapping().total_txpdo_bytes(), 6u);
}

TEST_F(SlavePdoRegistrationTest, RegisterFixedPDOsEmptyConfigIsNoop) {
    bringUp();
    auto& sl = master_.slave(0);
    SIIPDOConfig empty{};
    EXPECT_EQ(sl.registerFixedPDOs(empty), SlaveError::Ok);
    EXPECT_EQ(master_.pdoForSlave(0).mapping().total_rxpdo_bytes(), 0u);
    EXPECT_EQ(master_.pdoForSlave(0).mapping().total_txpdo_bytes(), 0u);
}

TEST_F(SlavePdoRegistrationTest, RegisterFixedPDOsOversizedFails) {
    bringUp();
    auto& sl = master_.slave(0);

    SIIPDOConfig cfg{};
    cfg.rxpdo_index = 0x1600;
    cfg.rxpdo_size = static_cast<uint16_t>(PDO::kMaxPDOSize + 1);
    cfg.has_rxpdo = true;
    EXPECT_EQ(sl.registerFixedPDOs(cfg), SlaveError::PDOMappingFailed);

    SIIPDOConfig tx{};
    tx.txpdo_index = 0x1A00;
    tx.txpdo_size = static_cast<uint16_t>(PDO::kMaxPDOSize + 1);
    tx.has_txpdo = true;
    EXPECT_EQ(sl.registerFixedPDOs(tx), SlaveError::PDOMappingFailed);
}

// ============================================================================
// registerPDOsFromSII
// ============================================================================

TEST_F(SlavePdoRegistrationTest, RegisterFromSiiWithoutPdoDataFails) {
    bringUp();
    auto& sl = master_.slave(0);

    // The emulated SII carries no PDO category data.
    SIIPDOConfig cfg{};
    EXPECT_EQ(sl.registerPDOsFromSII(cfg), SlaveError::PDOConfigFailed);
    EXPECT_FALSE(cfg.has_rxpdo);
    EXPECT_FALSE(cfg.has_txpdo);
}

TEST_F(SlavePdoRegistrationTest, RegisterFromSiiReadError) {
    bringUp();
    auto& sl = master_.slave(0);

    // Drop the cache populated during initSlaves, then fail every EEPROM
    // data read so the SII read itself fails.
    master_.slave(0).sii().invalidateCache();
    master_.setAprdTestCallback([](uint16_t, uint16_t ado, void* out,
                                   uint16_t len, unsigned int) -> bool {
        if (ado == 0x0508) return false;
        if (out && len >= 2) {
            uint16_t zero = 0;
            std::memcpy(out, &zero, 2);
        }
        return ado != 0x0504;
    });

    SIIPDOConfig cfg{};
    EXPECT_EQ(sl.registerPDOsFromSII(cfg), SlaveError::SIIReadError);
    master_.setAprdTestCallback(nullptr);
}

// ============================================================================
// registerPDOsFromESI
// ============================================================================

namespace {

ESI::DeviceInfo makeEsiDevice(bool with_rx, bool with_tx, int rx_sm = 2,
                              int tx_sm = 3) {
    ESI::DeviceInfo dev;
    dev.name = "TestDevice";
    dev.vendorId = 0x00000002;
    dev.productCode = 0x0FA13052;
    if (with_rx) {
        ESI::PDO rx;
        rx.index = 0x1600;
        rx.sm = rx_sm;
        rx.entries.push_back({0x7000, 1, 16, "Ctrl", "UINT"});
        rx.entries.push_back({0x7010, 1, 32, "Target", "DINT"});
        dev.rxPdos.push_back(rx);
    }
    if (with_tx) {
        ESI::PDO tx;
        tx.index = 0x1A00;
        tx.sm = tx_sm;
        tx.entries.push_back({0x6000, 1, 8, "Status", "USINT"});
        tx.entries.push_back({0x6010, 1, 16, "Actual", "INT"});
        dev.txPdos.push_back(tx);
    }
    return dev;
}

}  // namespace

TEST_F(SlavePdoRegistrationTest, RegisterFromEsiBothDirections) {
    bringUp();
    auto& sl = master_.slave(0);

    ESIFile esi(std::vector<ESI::DeviceInfo>{makeEsiDevice(true, true)});
    SIIPDOConfig cfg{};
    ASSERT_EQ(sl.registerPDOsFromESI(esi, cfg), SlaveError::Ok);
    EXPECT_TRUE(cfg.has_rxpdo);
    EXPECT_TRUE(cfg.has_txpdo);
    EXPECT_EQ(cfg.rxpdo_index, 0x1600);
    EXPECT_EQ(cfg.txpdo_index, 0x1A00);
    // 16 + 32 bits -> 6 bytes; 8 + 16 bits -> 3 bytes.
    EXPECT_EQ(cfg.rxpdo_size, 6u);
    EXPECT_EQ(cfg.txpdo_size, 3u);
    EXPECT_EQ(master_.pdoForSlave(0).mapping().total_rxpdo_bytes(), 6u);
    EXPECT_EQ(master_.pdoForSlave(0).mapping().total_txpdo_bytes(), 3u);
}

TEST_F(SlavePdoRegistrationTest, RegisterFromEsiWildcardSyncManagers) {
    bringUp();
    auto& sl = master_.slave(0);

    // sm == -1 entries are accepted as "unspecified".
    ESIFile esi(std::vector<ESI::DeviceInfo>{makeEsiDevice(true, true, -1, -1)});
    SIIPDOConfig cfg{};
    ASSERT_EQ(sl.registerPDOsFromESI(esi, cfg), SlaveError::Ok);
    EXPECT_TRUE(cfg.has_rxpdo);
    EXPECT_TRUE(cfg.has_txpdo);
}

TEST_F(SlavePdoRegistrationTest, RegisterFromEsiOnlyOneDirection) {
    bringUp();
    auto& sl = master_.slave(0);

    ESIFile rx_only(std::vector<ESI::DeviceInfo>{makeEsiDevice(true, false)});
    SIIPDOConfig cfg{};
    ASSERT_EQ(sl.registerPDOsFromESI(rx_only, cfg), SlaveError::Ok);
    EXPECT_TRUE(cfg.has_rxpdo);
    EXPECT_FALSE(cfg.has_txpdo);
}

TEST_F(SlavePdoRegistrationTest, RegisterFromEsiEmptyFileFails) {
    bringUp();
    auto& sl = master_.slave(0);

    ESIFile empty(std::vector<ESI::DeviceInfo>{});
    SIIPDOConfig cfg{};
    EXPECT_EQ(sl.registerPDOsFromESI(empty, cfg), SlaveError::PDOConfigFailed);
}

TEST_F(SlavePdoRegistrationTest, RegisterFromEsiWithoutPdosFails) {
    bringUp();
    auto& sl = master_.slave(0);

    ESIFile esi(std::vector<ESI::DeviceInfo>{makeEsiDevice(false, false)});
    SIIPDOConfig cfg{};
    EXPECT_EQ(sl.registerPDOsFromESI(esi, cfg), SlaveError::PDOConfigFailed);
}

TEST_F(SlavePdoRegistrationTest, RegisterFromEsiOversizedFails) {
    bringUp();
    auto& sl = master_.slave(0);

    ESI::DeviceInfo dev = makeEsiDevice(true, false);
    dev.rxPdos[0].entries.push_back(
        {0x7020, 1, static_cast<uint16_t>(PDO::kMaxPDOSize * 8), "Big", "UINT"});
    ESIFile esi(std::vector<ESI::DeviceInfo>{dev});

    SIIPDOConfig cfg{};
    EXPECT_EQ(sl.registerPDOsFromESI(esi, cfg), SlaveError::PDOMappingFailed);
}

// ============================================================================
// assignPDOs
// ============================================================================

TEST_F(SlavePdoRegistrationTest, AssignPdosWritesSmAssignRecords) {
    bringUp();
    auto& sl = master_.slave(0);

    SIIPDOConfig cfg{};
    cfg.rxpdo_index = 0x1600;
    cfg.txpdo_index = 0x1A00;
    cfg.has_rxpdo = true;
    cfg.has_txpdo = true;

    // The emulated slave does not implement the CiA 301 assign objects, so
    // the SDO writes fail — the call still reports success and continues.
    EXPECT_EQ(sl.assignPDOs(cfg), SlaveError::Ok);
}

TEST_F(SlavePdoRegistrationTest, AssignPdosEmptyConfigIsNoop) {
    bringUp();
    auto& sl = master_.slave(0);
    SIIPDOConfig empty{};
    EXPECT_EQ(sl.assignPDOs(empty), SlaveError::Ok);
}

// ============================================================================
// registerPDOsFromSII — success path with a crafted EEPROM image
// ============================================================================

namespace {

uint32_t pack4(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
    return static_cast<uint32_t>(a) |
           (static_cast<uint32_t>(b) << 8) |
           (static_cast<uint32_t>(c) << 16) |
           (static_cast<uint32_t>(d) << 24);
}

/// Install byte-level APRD/APWR handlers serving `mem` (word addr -> dword).
void installEeprom(const std::unordered_map<uint16_t, uint32_t>& mem,
                   Master& master) {
    auto byte_map = std::make_shared<std::unordered_map<uint32_t, uint8_t>>();
    for (const auto& [addr, val] : mem) {
        for (int b = 0; b < 4; ++b)
            (*byte_map)[addr * 2 + b] = static_cast<uint8_t>((val >> (8 * b)) & 0xFF);
    }
    auto last_cmd_addr = std::make_shared<uint16_t>(0xFFFF);

    master.setApwrTestCallback(
        [last_cmd_addr](uint16_t, uint16_t ado, const void* data, uint16_t len,
                        unsigned int) {
            if (ado == 0x0504 && data && len >= 2) {
                uint16_t addr_le = 0;
                std::memcpy(&addr_le, data, sizeof(addr_le));
                *last_cmd_addr = addr_le;  // little-endian host
            }
            return true;
        });

    master.setAprdTestCallback(
        [byte_map, last_cmd_addr](uint16_t, uint16_t ado, void* out, uint16_t len,
                                  unsigned int) {
            if (ado == 0x0500) {
                if (out && len >= 1) {
                    uint8_t cfg = 0;
                    std::memcpy(out, &cfg, 1);
                }
                return true;
            }
            if (ado == 0x0502) {
                uint16_t estat = 0;
                std::memcpy(out, &estat, 2);
                return true;
            }
            if (ado == 0x0508) {
                const uint32_t base = static_cast<uint32_t>(*last_cmd_addr) * 2;
                uint32_t val = 0;
                for (int b = 0; b < 4; ++b) {
                    auto it = byte_map->find(base + static_cast<uint32_t>(b));
                    const uint8_t vb = (it != byte_map->end()) ? it->second : 0u;
                    val |= static_cast<uint32_t>(vb) << (8 * b);
                }
                if (out && len >= 4) std::memcpy(out, &val, 4);
                return true;
            }
            return false;
        });
}

}  // namespace

TEST_F(SlavePdoRegistrationTest, RegisterFromSiiWithPdoCategories) {
    bringUp();
    auto& sl = master_.slave(0);

    std::unordered_map<uint16_t, uint32_t> mem;
    // Configuration area + identity.
    mem[0x00] = 0x00020001;
    mem[0x02] = 0x00040003;
    mem[0x04] = 0x00060005;
    mem[0x06] = 0x00080007;
    mem[0x08] = 0x00000002;  // vendor id
    mem[0x0A] = 0x0FA13052;  // product code
    mem[0x0C] = 0x00010000;  // revision
    mem[0x0E] = 0x00000001;  // serial

    // Category area: one RxPDO (SM2) and one TxPDO (SM3), one entry each.
    mem[0x40] = (8u << 16) | static_cast<uint32_t>(SII::CAT_RXPDO);
    mem[0x42] = pack4(0x00, 0x16, 0x01, 0x02);  // 0x1600, 1 entry, SM2
    mem[0x44] = pack4(0x00, 0x10, 0x03, 0x00);
    mem[0x46] = pack4(0x00, 0x20, 0x01, 0x04);  // 0x2000:1
    mem[0x48] = pack4(0x0C, 0x10, 0x00, 0x00);  // 16 bits

    mem[0x4A] = (8u << 16) | static_cast<uint32_t>(SII::CAT_TXPDO);
    mem[0x4C] = pack4(0x00, 0x1A, 0x01, 0x03);  // 0x1A00, 1 entry, SM3
    mem[0x4E] = pack4(0x00, 0x10, 0x03, 0x00);
    mem[0x50] = pack4(0x00, 0x21, 0x01, 0x04);  // 0x2100:1
    mem[0x52] = pack4(0x0C, 0x20, 0x00, 0x00);  // 32 bits

    mem[0x54] = static_cast<uint32_t>(SII::CAT_END);

    installEeprom(mem, master_);
    sl.sii().invalidateCache();

    SIIPDOConfig cfg{};
    ASSERT_EQ(sl.registerPDOsFromSII(cfg), SlaveError::Ok);
    EXPECT_TRUE(cfg.has_rxpdo);
    EXPECT_TRUE(cfg.has_txpdo);
    EXPECT_EQ(cfg.rxpdo_index, 0x1600);
    EXPECT_EQ(cfg.txpdo_index, 0x1A00);
    EXPECT_EQ(cfg.rxpdo_size, 2u);
    EXPECT_EQ(cfg.txpdo_size, 4u);

    master_.setAprdTestCallback(nullptr);
    master_.setApwrTestCallback(nullptr);
}
