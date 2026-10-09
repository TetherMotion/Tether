/**
 * @file SlaveGroup_coe.cpp
 * @brief SlaveGroup — CoE/SDO group access and verification.
 *
 * TU split out of SlaveGroup.cpp.
 */

#include "tether/ethercat/SlaveGroup.hpp"

#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/TransactionRouter.hpp"
#include "tether/ethercat/FaultDetection.hpp"
#include "tether/platform/Platform.hpp"

#include "raw/RawWireFormat.hpp"

#include "logging/Logger.hpp"

#include <cstring>
#include <format>
#include <string>

namespace EtherCAT {

static const char* TAG = "SlaveGroup";

// ============================================================================
// CoE/SDO group access — one mailbox transaction per member
// ============================================================================

std::vector<uint16_t> SlaveGroup::memberIndices() const
{
    if (!broadcast_) return indices_;
    const uint16_t n = master_.getDiscoveredSlaveCount();
    std::vector<uint16_t> v(n);
    for (uint16_t i = 0; i < n; ++i) v[i] = i;
    return v;
}

std::vector<std::future<CoE::CoEResult<uint64_t>>>
SlaveGroup::readEntryAsync(
    const ObjectDictionary::ObjectDictionaryEntry& entry,
    CoE::CoETransactionOptions opts)
{
    const auto members = memberIndices();
    std::vector<std::future<CoE::CoEResult<uint64_t>>> futures;
    futures.reserve(members.size());
    for (const uint16_t idx : members) {
        futures.push_back(
            master_.sdoManager(idx).readEntryAsync(entry, opts));
    }
    return futures;
}

std::vector<std::future<CoE::CoEResult<void>>>
SlaveGroup::writeEntryAsync(
    const ObjectDictionary::ObjectDictionaryEntry& entry,
    uint64_t value, CoE::CoETransactionOptions opts)
{
    const auto members = memberIndices();
    std::vector<std::future<CoE::CoEResult<void>>> futures;
    futures.reserve(members.size());
    for (const uint16_t idx : members) {
        futures.push_back(
            master_.sdoManager(idx).writeEntryAsync(entry, value, opts));
    }
    return futures;
}

std::vector<std::future<CoE::CoEResult<void>>>
SlaveGroup::writeEntryAsync(
    const ObjectDictionary::ObjectDictionaryEntry& entry,
    std::span<const uint64_t> values, CoE::CoETransactionOptions opts)
{
    const auto members = memberIndices();
    std::vector<std::future<CoE::CoEResult<void>>> futures;
    futures.reserve(members.size());
    if (values.size() != members.size()) {
        TETHER_LOGE(TAG, "writeEntry: {} values for {} members — refusing",
                    values.size(), members.size());
        for (size_t i = 0; i < members.size(); ++i) {
            CoE::CoEWriteTransaction t;
            t.promise.set_value(
                std::unexpected(CoE::CoEErrorCode::NotConfigured));
            futures.push_back(t.promise.get_future());
        }
        return futures;
    }
    for (size_t i = 0; i < members.size(); ++i) {
        futures.push_back(
            master_.sdoManager(members[i])
                .writeEntryAsync(entry, values[i], opts));
    }
    return futures;
}

// Wait on all per-member futures against a shared deadline; members that
// miss it get CoEErrorCode::Timeout in their result slot.
template<typename T>
static std::vector<SlaveGroup::SlaveGroupSdoResult<T>>
collectGroupFutures(const std::vector<uint16_t>& members,
                    std::vector<std::future<CoE::CoEResult<T>>>& futures,
                    uint32_t timeout_ms)
{
    std::vector<SlaveGroup::SlaveGroupSdoResult<T>> out(members.size());
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    for (size_t i = 0; i < members.size(); ++i) {
        out[i].slave_index = members[i];
        const auto remaining = deadline - std::chrono::steady_clock::now();
        if (remaining > std::chrono::steady_clock::duration::zero() &&
            futures[i].wait_for(remaining) == std::future_status::ready) {
            out[i].result = futures[i].get();
        } else {
            out[i].result =
                std::unexpected(CoE::CoEError{CoE::CoEErrorCode::Timeout});
        }
    }
    return out;
}

std::vector<SlaveGroup::SlaveGroupReadResult>
SlaveGroup::readEntry(
    const ObjectDictionary::ObjectDictionaryEntry& entry,
    CoE::CoETransactionOptions opts)
{
    const auto members = memberIndices();
    const uint32_t timeout_ms =
        (opts.timeout_ms > 0) ? opts.timeout_ms : CoE::kDefaultTimeoutMs;
    auto futures = readEntryAsync(entry, opts);
    return collectGroupFutures<uint64_t>(members, futures, timeout_ms);
}

std::vector<SlaveGroup::SlaveGroupWriteResult>
SlaveGroup::writeEntry(
    const ObjectDictionary::ObjectDictionaryEntry& entry,
    uint64_t value, CoE::CoETransactionOptions opts)
{
    const auto members = memberIndices();
    const uint32_t timeout_ms =
        (opts.timeout_ms > 0) ? opts.timeout_ms : CoE::kDefaultTimeoutMs;
    auto futures = writeEntryAsync(entry, value, opts);
    return collectGroupFutures<void>(members, futures, timeout_ms);
}

std::vector<SlaveGroup::SlaveGroupWriteResult>
SlaveGroup::writeEntry(
    const ObjectDictionary::ObjectDictionaryEntry& entry,
    std::span<const uint64_t> values,
    CoE::CoETransactionOptions opts)
{
    const auto members = memberIndices();
    const uint32_t timeout_ms =
        (opts.timeout_ms > 0) ? opts.timeout_ms : CoE::kDefaultTimeoutMs;
    auto futures = writeEntryAsync(entry, values, opts);
    return collectGroupFutures<void>(members, futures, timeout_ms);
}

// ============================================================================
// verifyEntry / dumpEntries
// ============================================================================

bool SlaveGroup::verifyEntry(
    const ObjectDictionary::ObjectDictionaryEntry& entry,
    uint64_t expected, CoE::CoETransactionOptions opts,
    const char* log_tag)
{
    bool ok = true;
    for (const auto& r : readEntry(entry, opts)) {
        if (!r.result.has_value()) {
            TETHER_LOGE(log_tag,
                "Slave {} read '{}' (0x{:04X}:{}) failed",
                r.slave_index, entry.name ? entry.name : "?",
                entry.index, entry.subindex);
            ok = false;
        } else if (*r.result != expected) {
            TETHER_LOGW(log_tag,
                "Slave {} '{}' (0x{:04X}:{}) = {} (expected {})",
                r.slave_index, entry.name ? entry.name : "?",
                entry.index, entry.subindex,
                static_cast<int64_t>(*r.result),
                static_cast<int64_t>(expected));
            ok = false;
        }
    }
    return ok;
}

void SlaveGroup::dumpEntries(
    std::span<const ObjectDictionary::ObjectDictionaryEntry* const> entries,
    const char* log_tag, CoE::CoETransactionOptions opts)
{
    for (const auto* e : entries) {
        std::string per_slave;
        for (const auto& r : readEntry(*e, opts)) {
            per_slave += std::format(" s{}={}", r.slave_index,
                r.result.has_value() ? static_cast<int64_t>(*r.result)
                                     : int64_t(-1));
        }
        TETHER_LOGI(log_tag, "0x{:04X}:{} '{}':{}",
                    e->index, e->subindex, e->name ? e->name : "?",
                    per_slave);
    }
}

// ============================================================================
// waitForMailboxReady — batch-poll SM0/SM1 activation registers
// ============================================================================

bool SlaveGroup::waitForMailboxReady(uint32_t timeout_ms,
                                     uint32_t poll_interval_ms)
{
    // Sync Manager activation register: EC_REG_SM0 (0x0800) + sm*8 + 6.
    // Bit 0 (Channel Enable) is set by the slave once the mailbox channel
    // is operational.
    auto smActivateAddr = [](uint8_t sm) {
        return static_cast<uint16_t>(0x0800 + sm * 8 + 6);
    };

    auto& clock = Tether::Platform::Clock::instance();
    const int64_t deadline =
        clock.getMilliseconds() + static_cast<int64_t>(timeout_ms);

    while (clock.getMilliseconds() < deadline) {
        if (master_.isCancelRequested()) return false;

        const auto members = memberIndices();
        const size_t count = members.size();
        const size_t dg_count = count * 2;

        std::vector<SlaveAddress> addrs;
        addrs.reserve(dg_count);
        std::vector<uint16_t> regs(dg_count);
        std::vector<uint16_t> lens(dg_count, 1);
        for (size_t i = 0; i < count; ++i) {
            addrs.emplace_back(members[i]);
            regs[2 * i]     = smActivateAddr(0);
            addrs.emplace_back(members[i]);
            regs[2 * i + 1] = smActivateAddr(1);
        }

        auto batch = master_.readRegistersBatch(addrs.data(), regs.data(),
                                                lens.data(), dg_count);
        if (batch.count() == dg_count) {
            std::vector<BatchReadResult> results;
            batch.waitAll(poll_interval_ms + 50, results);

            bool all_ready = count > 0;
            for (size_t i = 0; i < count && all_ready; ++i) {
                for (size_t d = 0; d < 2; ++d) {
                    const auto& r = results[2 * i + d];
                    if (!r.success || r.wkc == 0 || !r.data ||
                        r.datalen < 1 || (r.data[0] & 0x01) == 0) {
                        all_ready = false;
                        break;
                    }
                }
            }
            if (all_ready) return true;
        }

        clock.delayMilliseconds(poll_interval_ms);
    }

    TETHER_LOGW(TAG, "Mailbox not ready on all members after {} ms",
                timeout_ms);
    return false;
}

} // namespace EtherCAT
