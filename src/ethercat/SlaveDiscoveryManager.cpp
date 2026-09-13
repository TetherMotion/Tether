/**
 * @file SlaveDiscoveryManager.cpp
 * @brief Implementation of SlaveDiscoveryManager
 */

#include "tether/ethercat/SlaveDiscoveryManager.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/SlaveStatusPoller.hpp"
#include "tether/ethercat/SlaveSupervisor.hpp"
#include "tether/ethercat/FaultDetection.hpp"
#include "tether/ethercat/DebugFlags.hpp"
#include "tether/platform/Platform.hpp"
#include "ethercat/raw/internal.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <thread>
#include <vector>

#if TETHER_ENABLE_SII
#include "tether/sii/SIIParser.hpp"
#endif

namespace EtherCAT {

static const char* TAG = "discovery";

// ============================================================================
// Construction
// ============================================================================

SlaveDiscoveryManager::SlaveDiscoveryManager(Master& master)
    : master_(&master) {}

// ============================================================================
// Synchronous discovery
// ============================================================================

std::vector<DiscoveredSlave> SlaveDiscoveryManager::discover(DiscoveryOptions opts) {
    return discoverSync(opts, {}, true);
}

DiscoveredSlave SlaveDiscoveryManager::discoverOne(uint16_t index,
                                                    DiscoveryOptions opts) {
    std::array<uint16_t, 1> ids{index};
    auto result = discoverSync(opts, std::span<const uint16_t>(ids), false);
    if (result.empty()) {
        DiscoveredSlave empty;
        empty.index = index;
        return empty;
    }
    return result[0];
}

std::vector<DiscoveredSlave> SlaveDiscoveryManager::discover(
    std::span<const uint16_t> indices, DiscoveryOptions opts) {
    return discoverSync(opts, indices, false);
}

// ============================================================================
// Asynchronous discovery
// ============================================================================

std::future<std::vector<DiscoveredSlave>> SlaveDiscoveryManager::discoverAsync(
    DiscoveryOptions opts) {
    // Capture opts by value; the span overload copies indices into a vector.
    return std::async(std::launch::async, [this, opts] {
        return discover(opts);
    });
}

std::future<DiscoveredSlave> SlaveDiscoveryManager::discoverOneAsync(
    uint16_t index, DiscoveryOptions opts) {
    return std::async(std::launch::async, [this, index, opts] {
        return discoverOne(index, opts);
    });
}

std::future<std::vector<DiscoveredSlave>> SlaveDiscoveryManager::discoverAsync(
    std::span<const uint16_t> indices, DiscoveryOptions opts) {
    // Copy the span into a owning vector for the async thread.
    std::vector<uint16_t> ids(indices.begin(), indices.end());
    return std::async(std::launch::async, [this, ids = std::move(ids), opts] {
        return discover(std::span<const uint16_t>(ids), opts);
    });
}

// ============================================================================
// Core synchronous discovery
// ============================================================================

std::vector<DiscoveredSlave> SlaveDiscoveryManager::discoverSync(
    DiscoveryOptions opts, std::span<const uint16_t> indices, bool all_slaves) {

    if (!master_) return {};

    // ---- 1. BRD scan to discover slave count ----
    // Reuse the master's existing scan logic. The master's discoverSlaves()
    // performs the BRD scan, calls initSlaves(), and initialises
    // faults/poller/supervisor. We call it here; it is idempotent if slaves
    // are already initialised (it re-scans and re-inits).
    //
    // If the scan fails, return an empty result.
    if (!master_->discoverSlaves()) {
        TETHER_LOGW(TAG, "BRD scan failed, no slaves discovered");
        return {};
    }

    const uint16_t slave_count = master_->getDiscoveredSlaveCount();
    if (slave_count == 0) return {};

    // ---- 2. Determine which slave indices to read SII for ----
    std::vector<uint16_t> target_indices;
    if (all_slaves) {
        target_indices.reserve(slave_count);
        for (uint16_t i = 0; i < slave_count; ++i) {
            target_indices.push_back(i);
        }
    } else {
        for (uint16_t idx : indices) {
            if (idx < slave_count) {
                target_indices.push_back(idx);
            }
        }
    }

    // ---- 3. Concurrent SII prefetch ----
    // Extend the 128-word prefetch from initSlaves() by reading the
    // next 384 words (128..511) from ALL target slaves in parallel using
    // the batch register API.  This is the single-worker concurrent-router
    // pattern: one thread, multiple datagrams in flight per frame.
    // The subsequent per-slave parseCategories() calls then hit the cache
    // for most words, avoiding sequential per-slave EEPROM round-trips.
#if TETHER_ENABLE_SII
    if (target_indices.size() > 1) {
        concurrentPrefetchSii(target_indices, 128, 384);
    }
#endif

    // ---- 4. Read SII data for each target slave ----
    std::vector<DiscoveredSlave> results;
    results.reserve(target_indices.size());

    const uint16_t effective_threads =
        std::min<uint16_t>(max_threads_,
                           static_cast<uint16_t>(target_indices.size()));

    if (effective_threads <= 1 || target_indices.size() <= 1) {
        // Sequential path
        for (uint16_t idx : target_indices) {
            DiscoveredSlave ds;
            ds.index = idx;
            readSlaveSii(ds, opts);
            results.push_back(std::move(ds));
        }
    } else {
        // Parallel path: spawn up to max_threads_ worker threads, each
        // processing a subset of the target slaves. Each SIIManager is
        // per-slave and thread-safe, and Master::sendRawFrame is guarded
        // by send_mutex_, so concurrent SII reads are safe.
        std::vector<DiscoveredSlave> partial(target_indices.size());
        std::vector<std::jthread> threads;
        threads.reserve(effective_threads);

        const size_t n = target_indices.size();
        const size_t chunk = (n + effective_threads - 1) / effective_threads;

        auto worker = [&](size_t start, size_t end) {
            for (size_t i = start; i < end; ++i) {
                DiscoveredSlave ds;
                ds.index = target_indices[i];
                readSlaveSii(ds, opts);
                partial[i] = std::move(ds);
            }
        };

        for (uint16_t t = 0; t < effective_threads; ++t) {
            size_t start = t * chunk;
            size_t end = std::min(start + chunk, n);
            if (start >= end) break;
            threads.emplace_back(worker, start, end);
        }
        threads.clear();  // join all

        for (auto& ds : partial) {
            results.push_back(std::move(ds));
        }
    }

    return results;
}

// ============================================================================
// Per-slave SII reading
// ============================================================================

void SlaveDiscoveryManager::readSlaveSii(DiscoveredSlave& out,
                                          DiscoveryOptions opts) {
#if TETHER_ENABLE_SII
    if (!master_) return;

    const uint16_t idx = out.index;
    auto& sii = master_->slave(idx).sii();

    // "All" implies everything including deep options.
    const bool all = opts.has(DiscoveryOption::All);

    // Map DiscoveryOptions to SII category mask. Only the categories
    // actually requested will be read from the EEPROM.
    uint32_t cat_mask = SII::CAT_MASK_NONE;
    if (all || opts.has(DiscoveryOption::DeviceNames)) {
        cat_mask |= SII::CAT_MASK_STRINGS | SII::CAT_MASK_GENERAL;
    }
    if (all || opts.has(DiscoveryOption::SyncManagers)) {
        cat_mask |= SII::CAT_MASK_SYNC_MGR;
    }
    if (all || opts.has(DiscoveryOption::FMMUs)) {
        cat_mask |= SII::CAT_MASK_FMMU;
    }
    if (all || opts.has(DiscoveryOption::TxPDOs)) {
        cat_mask |= SII::CAT_MASK_TXPDO;
    }
    if (all || opts.has(DiscoveryOption::RxPDOs)) {
        cat_mask |= SII::CAT_MASK_RXPDO;
    }
    if (all || opts.has(DiscoveryOption::DistributedClocks)) {
        cat_mask |= SII::CAT_MASK_DC;
    }
    if (all || opts.has(DiscoveryOption::DeviceProfile) ||
        opts.has(DiscoveryOption::GeneralInfo) ||
        opts.has(DiscoveryOption::PhysicalPorts)) {
        cat_mask |= SII::CAT_MASK_GENERAL;
    }
    if (all) {
        cat_mask = SII::CAT_MASK_ALL;
    }

    const bool needs_deep = (cat_mask != SII::CAT_MASK_NONE);

    if (needs_deep) {
        // Selective SII parse — only reads the requested categories.
        SII::SIIData data;
        if (sii.parseCategories(data, cat_mask)) {
            // Identity
            if (all || opts.has(DiscoveryOption::VendorId))
                out.vendor_id = data.identity.vendor_id;
            if (all || opts.has(DiscoveryOption::ProductCode))
                out.product_code = data.identity.product_code;
            if (all || opts.has(DiscoveryOption::RevisionNumber))
                out.revision_number = data.identity.revision_number;
            if (all || opts.has(DiscoveryOption::SerialNumber))
                out.serial_number = data.identity.serial_number;

            // Configured address (alias)
            out.configured_address = data.alias_address;

            // Mailbox
            if (all || opts.has(DiscoveryOption::MailboxConfig))
                out.mailbox_config = data.mailbox;
            if (all || opts.has(DiscoveryOption::MailboxProtocols))
                out.mailbox_protocols = data.mailbox.protocols;

            // Strings / names
            if (all || opts.has(DiscoveryOption::DeviceNames)) {
                if (data.has_general) {
                    if (data.general.name_idx > 0) {
                        char buf[256] = {};
                        if (sii.readString(data.general.name_idx, buf, sizeof(buf))) {
                            out.device_name = std::string(buf);
                        }
                    }
                    if (data.general.group_idx > 0) {
                        char buf[256] = {};
                        if (sii.readString(data.general.group_idx, buf, sizeof(buf))) {
                            out.group_name = std::string(buf);
                        }
                    }
                    if (data.general.order_idx > 0) {
                        char buf[256] = {};
                        if (sii.readString(data.general.order_idx, buf, sizeof(buf))) {
                            out.order_code = std::string(buf);
                        }
                    }
                }
            }

            // Sync managers
            if (all || opts.has(DiscoveryOption::SyncManagers)) {
                std::vector<SII::SIISyncManager> sms;
                for (size_t i = 0; i < data.sm_count; ++i) {
                    sms.push_back(data.sync_managers[i]);
                }
                out.sync_managers = std::move(sms);
            }

            // FMMUs
            if (all || opts.has(DiscoveryOption::FMMUs)) {
                std::vector<SII::SIIFMMU> fmmus;
                for (size_t i = 0; i < data.fmmu_count; ++i) {
                    fmmus.push_back(data.fmmus[i]);
                }
                out.fmmus = std::move(fmmus);
            }

            // PDOs
            if (all || opts.has(DiscoveryOption::TxPDOs)) {
                out.tx_pdos = data.tx_pdos;
            }
            if (all || opts.has(DiscoveryOption::RxPDOs)) {
                out.rx_pdos = data.rx_pdos;
            }

            // Distributed Clocks
            if (all || opts.has(DiscoveryOption::DistributedClocks)) {
                out.dc_configs = data.dc_configs;
            }

            // General info (includes device profile data)
            if (all || opts.has(DiscoveryOption::DeviceProfile) ||
                opts.has(DiscoveryOption::GeneralInfo)) {
                if (data.has_general) {
                    out.general_info = data.general;
                }
            }

            // Physical ports
            if (all || opts.has(DiscoveryOption::PhysicalPorts)) {
                if (data.has_general) {
                    out.physical_ports = data.general.phys_port;
                }
            }

            // EEPROM size
            if (all || opts.has(DiscoveryOption::EepromSize)) {
                out.eeprom_size_kbits = data.eeprom_size_kbits;
            }

            return;
        }
        // parseCategories failed — fall through to shallow reads
    }

    // ---- Shallow reads (first 64 words, already prefetched by initSlaves) ----
    if (all || opts.has(DiscoveryOption::VendorId)) {
        uint32_t v = 0;
        if (sii.readDWord(SII::SII_VENDOR_ID, v)) {
            out.vendor_id = v;
        }
    }
    if (all || opts.has(DiscoveryOption::ProductCode)) {
        uint32_t v = 0;
        if (sii.readDWord(SII::SII_PRODUCT_CODE, v)) {
            out.product_code = v;
        }
    }
    if (all || opts.has(DiscoveryOption::RevisionNumber)) {
        uint32_t v = 0;
        if (sii.readDWord(SII::SII_REVISION, v)) {
            out.revision_number = v;
        }
    }
    if (all || opts.has(DiscoveryOption::SerialNumber)) {
        uint32_t v = 0;
        if (sii.readDWord(SII::SII_SERIAL_NUMBER, v)) {
            out.serial_number = v;
        }
    }

    // Configured address (alias) — word 0x0004
    {
        uint16_t alias = 0;
        if (sii.readWord(SII::SII_ALIAS_ADDRESS, alias)) {
            out.configured_address = alias;
        }
    }

    // Mailbox config
    if (all || opts.has(DiscoveryOption::MailboxConfig)) {
        SII::SIIMailboxConfig mbx;
        uint16_t w = 0;
        if (sii.readWord(SII::SII_BOOTSTRAP_RX_MBX_OFFSET, w)) mbx.bootstrap_rx_offset = w;
        if (sii.readWord(SII::SII_BOOTSTRAP_RX_MBX_SIZE, w))   mbx.bootstrap_rx_size = w;
        if (sii.readWord(SII::SII_BOOTSTRAP_TX_MBX_OFFSET, w)) mbx.bootstrap_tx_offset = w;
        if (sii.readWord(SII::SII_BOOTSTRAP_TX_MBX_SIZE, w))   mbx.bootstrap_tx_size = w;
        if (sii.readWord(SII::SII_STD_RX_MBX_OFFSET, w))       mbx.std_rx_offset = w;
        if (sii.readWord(SII::SII_STD_RX_MBX_SIZE, w))         mbx.std_rx_size = w;
        if (sii.readWord(SII::SII_STD_TX_MBX_OFFSET, w))       mbx.std_tx_offset = w;
        if (sii.readWord(SII::SII_STD_TX_MBX_SIZE, w))         mbx.std_tx_size = w;
        out.mailbox_config = mbx;
    }

    // Mailbox protocols
    if (all || opts.has(DiscoveryOption::MailboxProtocols)) {
        uint16_t proto = 0;
        if (sii.readWord(SII::SII_MAILBOX_PROTOCOLS, proto)) {
            out.mailbox_protocols = proto;
        }
    }

    // EEPROM size — word 0x002E
    if (all || opts.has(DiscoveryOption::EepromSize)) {
        uint16_t sz = 0;
        if (sii.readWord(SII::SII_SIZE_INFO, sz)) {
            out.eeprom_size_kbits = sz;
        }
    }
#else
    (void)opts;
    // SII disabled — only slave count is available.
#endif
}

#if TETHER_ENABLE_SII
// ============================================================================
// Concurrent EEPROM prefetch — single-worker, multi-slave parallel I/O
// ============================================================================

void SlaveDiscoveryManager::concurrentPrefetchSii(
    const std::vector<uint16_t>& slave_indices,
    uint16_t start_word, uint16_t word_count)
{
    if (slave_indices.empty() || word_count == 0 || !master_) return;

    const size_t n = slave_indices.size();

    // EEPROM register addresses (EtherCAT ESC, per ETG.1000.4)
    constexpr uint16_t REG_EEPCTL  = 0x0502;  // Control/status (shared)
    constexpr uint16_t REG_EEPADDR = 0x0504;  // EEPROM word address
    constexpr uint16_t REG_EEPDAT  = 0x0508;  // EEPROM data (32-bit)
    constexpr uint16_t ECMD_READ   = 0x0100;  // Read command
    constexpr uint16_t ESTAT_BUSY  = 0x8000;  // Busy bit

    // Reusable buffers for batch operations
    std::vector<SlaveAddress> addrs;
    addrs.reserve(n);
    for (size_t i = 0; i < n; ++i)
        addrs.push_back(SlaveAddress(slave_indices[i]));
    std::vector<uint16_t>      regs(n);
    std::vector<uint16_t>      lens(n);
    std::vector<const void*>  wdata(n);
    std::vector<BatchReadResult> results;

    // readRaw32 reads 2 words (32 bits) at a time, so round up to even
    const uint16_t end_word = start_word + (word_count + 1u) & ~1u;

    for (uint16_t wa = start_word; wa < end_word; wa += 2) {
        // --- Step 1: Write EEPADDR to all slaves (batch APWR) ---
        uint16_t eepaddr_le = Raw::host_to_le16(wa);
        for (size_t i = 0; i < n; ++i) {
            regs[i]  = REG_EEPADDR;
            wdata[i] = &eepaddr_le;
            lens[i]  = 2;
        }
        auto bw = master_->writeRegistersBatch(
            addrs.data(), regs.data(), wdata.data(), lens.data(), n);
        if (bw.count() == 0) break;
        bw.waitAll(200, results);

        // --- Step 2: Write EEPCTL READ command to all slaves (batch APWR) ---
        uint16_t eepctl_le = Raw::host_to_le16(ECMD_READ);
        for (size_t i = 0; i < n; ++i) {
            regs[i]  = REG_EEPCTL;
            wdata[i] = &eepctl_le;
            lens[i]  = 2;
        }
        bw = master_->writeRegistersBatch(
            addrs.data(), regs.data(), wdata.data(), lens.data(), n);
        if (bw.count() == 0) break;
        bw.waitAll(200, results);

        // --- Step 3: Poll EEPSTAT until not busy (all slaves) ---
        bool any_busy = true;
        int poll_iters = 0;
        while (any_busy && poll_iters < 100) {
            for (size_t i = 0; i < n; ++i) {
                regs[i] = REG_EEPCTL;  // EEPSTAT is same register (0x0502)
                lens[i] = 2;
            }
            auto br = master_->readRegistersBatch(
                addrs.data(), regs.data(), lens.data(), n);
            if (br.count() == 0) break;
            br.waitAll(200, results);

            any_busy = false;
            for (size_t i = 0; i < n; ++i) {
                if (results[i].success && results[i].datalen >= 2) {
                    uint16_t estat = Raw::le16_to_host(
                        *reinterpret_cast<const uint16_t*>(results[i].data));
                    if (estat & ESTAT_BUSY) any_busy = true;
                }
            }
            ++poll_iters;
        }

        // --- Step 4: Read EEPDAT from all slaves (batch APRD, 4 bytes) ---
        for (size_t i = 0; i < n; ++i) {
            regs[i] = REG_EEPDAT;
            lens[i] = 4;
        }
        auto br = master_->readRegistersBatch(
            addrs.data(), regs.data(), lens.data(), n);
        if (br.count() == 0) continue;
        br.waitAll(200, results);

        // --- Cache the results in each slave's SII cache ---
        for (size_t i = 0; i < n; ++i) {
            if (!results[i].success || results[i].datalen < 4) continue;
            uint32_t dword_le = 0;
            std::memcpy(&dword_le, results[i].data, 4);
            uint32_t dword = Raw::le32_to_host(dword_le);
            uint16_t lo = static_cast<uint16_t>(dword & 0xFFFF);
            uint16_t hi = static_cast<uint16_t>((dword >> 16) & 0xFFFF);
            master_->slave(slave_indices[i]).sii().cache().set(wa, lo);
            master_->slave(slave_indices[i]).sii().cache().set(
                static_cast<uint16_t>(wa + 1), hi);
        }
    }
}
#endif

} // namespace EtherCAT
