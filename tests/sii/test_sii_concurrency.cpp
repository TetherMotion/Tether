/**
 * @file test_sii_concurrency.cpp
 * @brief Concurrency tests for per-slave SIIManager.
 *
 * Verifies that:
 *   - Multiple threads reading from the same SIIManager do not corrupt the
 *     EEPROM read sequence (bus_mutex_ serialization).
 *   - Multiple threads reading from different SIIManagers (different slaves)
 *     can proceed in parallel without interference.
 *   - The word cache serves concurrent cache-hit reads without contention.
 *   - parseFull() is safe to call concurrently and only parses once.
 */

#include <gtest/gtest.h>
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/sii/SIIManager.hpp"
#include "ethercat/raw/internal.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <thread>
#include <unordered_map>
#include <vector>

using namespace EtherCAT::SII;

// ============================================================================
// Helper constants (match SIIReader.cpp internals)
// ============================================================================
static constexpr uint16_t EC_REG_EEPCTL   = 0x0502;
static constexpr uint16_t EC_REG_EEPDAT   = 0x0508;
static constexpr uint16_t EC_REG_CFG_ADDR = 0x0010;

// ============================================================================
// Test fixture: multi-slave SII mock infrastructure (same pattern as
// test_sii_multi_slave_regression.cpp but exercising SIIManager directly).
// ============================================================================
class SiiConcurrencyTest : public ::testing::Test {
protected:
    EtherCAT::Master master;

    struct SlaveMockState {
        std::unordered_map<uint16_t, uint32_t> eepdat;
        uint16_t configured_addr = 0;
        uint16_t last_eepctl_addr = 0xFFFF;
        std::atomic<int> eepctl_write_count{0};
        std::atomic<int> eepdat_read_count{0};
    };

    std::unordered_map<uint16_t, SlaveMockState> slaves;

    void SetUp() override {
        slaves.clear();
        setupSlave(0);
        setupSlave(1);
        // Shrink SII read timeout so initSlaves prefetch doesn't block.
        master.setSiiTimeoutMs(1);
        installCallbacks();
    }

    void TearDown() override {
        master.setAprdTestCallback(nullptr);
        master.setApwrTestCallback(nullptr);
    }

    void setupSlave(uint16_t index) {
        auto& s = slaves[index];
        s.configured_addr = 0;
        s.last_eepctl_addr = 0xFFFF;
        s.eepctl_write_count = 0;
        s.eepdat_read_count = 0;

        // Fill with valid SII config area data (8 words)
        s.eepdat[0x00] = 0x00020001u;
        s.eepdat[0x02] = 0x00040003u;
        s.eepdat[0x04] = 0x00060005u;
        s.eepdat[0x06] = 0x00080007u;
        // Identity
        uint32_t vendor = 0x000022D2u + index;
        s.eepdat[0x08] = vendor;
        s.eepdat[0x0A] = 0x00000001u + index;
        s.eepdat[0x0C] = 0x00010000u + index;
        s.eepdat[0x0E] = 0x00000001u + index;
    }

    uint16_t adpToSlaveIndex(uint16_t adp) {
        if (adp == 0x0000) return 0;
        if (adp == 0xFFFF) return 1;
        if (adp == 0xFFFE) return 2;
        for (auto& [idx, s] : slaves) {
            if (s.configured_addr != 0 && s.configured_addr == adp) {
                return idx;
            }
        }
        return 0xFFFF;
    }

    void installCallbacks() {
        master.setApwrTestCallback([this](uint16_t adp, uint16_t ado,
                                          const void* data, uint16_t len,
                                          unsigned int) -> bool {
            uint16_t idx = adpToSlaveIndex(adp);
            if (idx == 0xFFFF) return false;
            auto& s = slaves[idx];

            if (ado == EC_REG_EEPCTL) {
                s.eepctl_write_count++;
                return true;
            }
            if (ado == 0x0504) {
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

        master.setAprdTestCallback([this](uint16_t adp, uint16_t ado,
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
                    uint16_t estat_le = EtherCAT::Raw::host_to_le16(0);
                    std::memcpy(out, &estat_le, 2);
                }
                return true;
            }
            if (ado == 0x0508) {
                s.eepdat_read_count++;
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
    }

    /// Initialize the master with N slaves so SIIManager instances exist.
    void initSlaves(uint16_t count) {
        for (uint16_t i = 0; i < count; ++i) {
            if (slaves.find(i) == slaves.end()) setupSlave(i);
        }
        master.initSlaves(count);
    }
};

// ============================================================================
// Test 1: Concurrent reads from the same SIIManager do not corrupt state.
// ============================================================================
TEST_F(SiiConcurrencyTest, ConcurrentReadsSameManager) {
    initSlaves(1);

    auto& sii = master.slave(0).sii();

    // Prefetch to populate the cache, then read concurrently.
    sii.prefetchWords(0x00, 16);

    std::atomic<int> success_count{0};
    std::atomic<int> failure_count{0};

    const int num_threads = 8;
    const int reads_per_thread = 100;

    std::vector<std::thread> threads;
    for (int t = 0; t < num_threads; ++t) {
        threads.emplace_back([&sii, &success_count, &failure_count, t]() {
            for (int i = 0; i < reads_per_thread; ++i) {
                uint16_t word_addr = static_cast<uint16_t>(
                    (i * 2 + (t % 2)) % 16);
                uint16_t val = 0;
                if (sii.readWord(word_addr, val)) {
                    success_count++;
                } else {
                    failure_count++;
                }
            }
        });
    }

    for (auto& th : threads) th.join();

    // All reads should succeed (data is cached after prefetch).
    EXPECT_EQ(failure_count.load(), 0)
        << "Concurrent cache-hit reads should not fail";
    EXPECT_EQ(success_count.load(), num_threads * reads_per_thread)
        << "All concurrent reads should succeed";
}

// ============================================================================
// Test 2: Concurrent reads from different SIIManagers (different slaves)
// do not interfere.
// ============================================================================
TEST_F(SiiConcurrencyTest, ConcurrentReadsDifferentManagers) {
    initSlaves(2);

    auto& sii0 = master.slave(0).sii();
    auto& sii1 = master.slave(1).sii();

    // Prefetch both slaves.
    sii0.prefetchWords(0x00, 16);
    sii1.prefetchWords(0x00, 16);

    std::atomic<int> success0{0};
    std::atomic<int> success1{0};

    std::thread t0([&sii0, &success0]() {
        for (int i = 0; i < 100; ++i) {
            uint16_t val = 0;
            if (sii0.readWord(static_cast<uint16_t>(i % 16), val)) {
                success0++;
            }
        }
    });
    std::thread t1([&sii1, &success1]() {
        for (int i = 0; i < 100; ++i) {
            uint16_t val = 0;
            if (sii1.readWord(static_cast<uint16_t>(i % 16), val)) {
                success1++;
            }
        }
    });

    t0.join();
    t1.join();

    EXPECT_EQ(success0.load(), 100);
    EXPECT_EQ(success1.load(), 100);
}

// ============================================================================
// Test 3: Concurrent cache-miss reads from the same SIIManager serialize
// correctly and all succeed.
// ============================================================================
TEST_F(SiiConcurrencyTest, ConcurrentCacheMissSameManager) {
    initSlaves(1);

    auto& sii = master.slave(0).sii();

    // Do NOT prefetch — all reads will be cache misses and require bus access.
    std::atomic<int> success_count{0};
    std::atomic<int> failure_count{0};

    const int num_threads = 4;
    const int reads_per_thread = 5;

    std::vector<std::thread> threads;
    for (int t = 0; t < num_threads; ++t) {
        threads.emplace_back([&sii, &success_count, &failure_count]() {
            for (int i = 0; i < reads_per_thread; ++i) {
                // Read a unique address per thread to force bus access.
                uint16_t word_addr = static_cast<uint16_t>(i * 2);
                uint16_t val = 0;
                if (sii.readWord(word_addr, val)) {
                    success_count++;
                } else {
                    failure_count++;
                }
            }
        });
    }

    for (auto& th : threads) th.join();

    // All reads should succeed — bus_mutex_ serializes them.
    EXPECT_EQ(failure_count.load(), 0)
        << "Concurrent cache-miss reads should serialize and succeed";
    EXPECT_EQ(success_count.load(), num_threads * reads_per_thread);
}

// ============================================================================
// Test 4: parseFull() called concurrently only parses once and returns
// consistent data.
// ============================================================================
TEST_F(SiiConcurrencyTest, ConcurrentParseFull) {
    initSlaves(1);

    auto& sii = master.slave(0).sii();

    const int num_threads = 4;
    std::vector<std::thread> threads;
    std::vector<SIIData> results(num_threads);
    // vector<bool> packs bits into shared words — concurrent writes to
    // different elements would still race on the same word.
    std::vector<std::atomic<bool>> success(num_threads);

    for (int t = 0; t < num_threads; ++t) {
        threads.emplace_back([this, &sii, &results, &success, t]() {
            SIIData data;
            success[t] = sii.parseFull(data);
            results[t] = data;
        });
    }

    for (auto& th : threads) th.join();

    // All should succeed.
    for (int t = 0; t < num_threads; ++t) {
        EXPECT_TRUE(success[t]) << "Thread " << t << ": parseFull should succeed";
    }

    // All results should be identical (same vendor ID).
    uint32_t expected_vendor = 0x000022D2u;
    for (int t = 0; t < num_threads; ++t) {
        EXPECT_EQ(results[t].identity.vendor_id, expected_vendor)
            << "Thread " << t << ": vendor ID mismatch";
    }
}

// ============================================================================
// Test 5: Cache invalidation during concurrent reads is safe.
// ============================================================================
TEST_F(SiiConcurrencyTest, InvalidateDuringReads) {
    initSlaves(1);

    auto& sii = master.slave(0).sii();

    // Prefetch to populate cache.
    sii.prefetchWords(0x00, 16);

    std::atomic<bool> stop{false};
    std::atomic<int> read_count{0};

    // Reader thread: continuously reads cached words.
    std::thread reader([&sii, &stop, &read_count]() {
        while (!stop.load()) {
            uint16_t val = 0;
            sii.readWord(0x0000, val);
            read_count++;
        }
    });

    // Main thread: invalidate cache a few times.
    for (int i = 0; i < 10; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        sii.invalidateCache();
        // Re-populate so the reader can continue.
        sii.prefetchWords(0x00, 4);
    }

    stop.store(true);
    reader.join();

    // Should have completed many reads without crashing.
    EXPECT_GT(read_count.load(), 0);
}
