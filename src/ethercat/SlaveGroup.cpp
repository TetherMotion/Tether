/**
 * @file SlaveGroup.cpp
 * @brief Single-packet AL state control/query for slave groups — see header.
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
// Factories
// ============================================================================

SlaveGroup SlaveGroup::all(Master& master)
{
    SlaveGroup g(master, std::span<const uint16_t>{});
    g.broadcast_ = true;
    return g;
}

SlaveGroup SlaveGroup::allAddressed(Master& master)
{
    const uint16_t n = master.getDiscoveredSlaveCount();
    std::vector<uint16_t> indices(n);
    for (uint16_t i = 0; i < n; ++i) indices[i] = i;
    return SlaveGroup(master, indices);
}

// ============================================================================
// AL_CONTROL write — BWR for broadcast groups, one APWR-per-slave frame else
// ============================================================================

uint16_t SlaveGroup::requestAlControl(uint8_t al_control,
                                    uint32_t timeout_ms)
{
    if (broadcast_) {
        const uint8_t idx = master_.allocIdx();
        RxDatagram resp{};
        const size_t slot = master_.preRegisterResponseWaiter(
            idx, resp.data, sizeof(resp.data));
        if (slot >= TransactionRouter::kNumSlots) return 0;

        const uint16_t val_le =
            Raw::host_to_le16(static_cast<uint16_t>(al_control));
        if (!master_.sendSingleDatagram(Command::BWR, idx, /*adp=*/0x0000,
                                        reg::AL_CONTROL, &val_le,
                                        sizeof(val_le), /*roundtrip=*/true)) {
            return 0;
        }

        const WaitResult result =
            master_.waitForPreRegistered(slot, timeout_ms);
        return result.success ? result.wkc : 0;
    }

    const size_t count = indices_.size();
    if (count == 0) return 0;

    // Kick mode: piggyback an APRD(AL_STATUS) after each APWR(AL_CONTROL)
    // in the same frame — firmware-driven ESCs that only process register
    // work while servicing traffic get an immediate evaluation + reply.
    if (master_.alControlStatusKick()) {
        std::vector<uint16_t> vals_le(count);
        std::vector<MultiDatagramSpec> specs(count * 2);
        std::vector<RxDatagram> resps(count * 2);
        std::vector<size_t> slots(count * 2, TransactionRouter::kNumSlots);
        size_t n = 0;
        for (size_t i = 0; i < count; ++i) {
            vals_le[i] = Raw::host_to_le16(
                static_cast<uint16_t>(al_control));
            const uint16_t adp =
                SlaveAddress(indices_[i]).raw();
            const uint8_t idx_w = master_.allocIdx();
            const uint8_t idx_r = master_.allocIdx();
            slots[n] = master_.preRegisterResponseWaiter(
                idx_w, resps[n].data, sizeof(resps[n].data));
            ++n;
            slots[n] = master_.preRegisterResponseWaiter(
                idx_r, resps[n].data, sizeof(resps[n].data));
            ++n;
            if (slots[n - 2] >= TransactionRouter::kNumSlots ||
                slots[n - 1] >= TransactionRouter::kNumSlots) {
                for (size_t s = 0; s < n; ++s)
                    if (slots[s] < TransactionRouter::kNumSlots)
                        master_.packetRouter().cancelPreRegistered(slots[s]);
                return 0;
            }
            specs[n - 2] = {Command::APWR, idx_w, adp, reg::AL_CONTROL,
                            &vals_le[i], 2, true};
            specs[n - 1] = {Command::APRD, idx_r, adp, reg::AL_STATUS,
                            nullptr, 2, true};
        }
        if (master_.sendMultiDatagram(specs.data(), specs.size()) == 0)
            return 0;
        uint16_t acked = 0;
        for (size_t i = 0; i < count; ++i) {
            const WaitResult wr = master_.waitForPreRegistered(
                slots[2 * i], timeout_ms);
            const WaitResult rd = master_.waitForPreRegistered(
                slots[2 * i + 1], timeout_ms);
            if (rd.success) {
                ++acked;
            } else {
                TETHER_LOGW(TAG, "Slave {} AL_CONTROL=0x{:02X} kick: "
                                 "no status reply (write ok={} wkc={})",
                            indices_[i], al_control, wr.success, wr.wkc);
            }
        }
        return acked;
    }

    std::vector<SlaveAddress> addrs;
    addrs.reserve(count);
    std::vector<uint16_t>     regs(count, reg::AL_CONTROL);
    std::vector<uint16_t>     vals_le(count);
    std::vector<const void*>  data(count);
    std::vector<uint16_t>     lens(count, 2);

    for (size_t i = 0; i < count; ++i) {
        addrs.emplace_back(indices_[i]);
        vals_le[i] = Raw::host_to_le16(static_cast<uint16_t>(al_control));
        data[i]    = &vals_le[i];
    }

    auto batch = master_.writeRegistersBatch(addrs.data(), regs.data(),
                                             data.data(), lens.data(), count);
    if (batch.count() != count) return 0;

    std::vector<BatchReadResult> results;
    batch.waitAll(timeout_ms, results);

    uint16_t acked = 0;
    for (size_t i = 0; i < count; ++i) {
        if (results[i].success && results[i].wkc > 0) {
            ++acked;
        } else {
            TETHER_LOGW(TAG, "Slave {} did not acknowledge AL_CONTROL=0x{:02X} "
                             "(wkc={})",
                        indices_[i], al_control, results[i].wkc);
        }
    }
    return acked;
}

// ============================================================================
// Group AL_STATUS query — one frame, two APRD datagrams per slave
// ============================================================================

std::vector<SlaveGroupState> SlaveGroup::readStates(uint32_t timeout_ms)
{
    std::vector<uint16_t> query = indices_;
    if (broadcast_) {
        // Broadcast reads can't disaggregate per-slave data — poll every
        // discovered slave with one APRD-pair each inside the same frame.
        const uint16_t n = master_.getDiscoveredSlaveCount();
        query.resize(n);
        for (uint16_t i = 0; i < n; ++i) query[i] = i;
    }

    const size_t count = query.size();
    std::vector<SlaveGroupState> out(count);
    for (size_t i = 0; i < count; ++i) {
        out[i].slave_index = query[i];
    }
    if (count == 0) return out;

    // Two datagrams per slave: AL_STATUS (0x0130) + AL_STATUS_CODE (0x0134).
    const size_t dg_count = count * 2;
    std::vector<SlaveAddress> addrs;
    addrs.reserve(dg_count);
    std::vector<uint16_t>     regs(dg_count);
    std::vector<uint16_t>     lens(dg_count, 2);

    for (size_t i = 0; i < count; ++i) {
        addrs.emplace_back(query[i]);
        regs[2 * i]      = reg::AL_STATUS;
        addrs.emplace_back(query[i]);
        regs[2 * i + 1]  = reg::AL_STATUS_CODE;
    }

    auto batch = master_.readRegistersBatch(addrs.data(), regs.data(),
                                            lens.data(), dg_count);
    if (batch.count() != dg_count) return out;

    std::vector<BatchReadResult> results;
    batch.waitAll(timeout_ms, results);

    for (size_t i = 0; i < count; ++i) {
        const auto& st = results[2 * i];
        const auto& sc = results[2 * i + 1];
        auto& o = out[i];
        o.wkc = static_cast<uint16_t>(st.wkc + sc.wkc);
        if (st.success && st.wkc > 0 && st.data && st.datalen >= 2) {
            uint16_t v;
            std::memcpy(&v, st.data, 2);
            o.al_status = Raw::le16_to_host(v);
            o.responded = true;
        }
        if (sc.success && sc.wkc > 0 && sc.data && sc.datalen >= 2) {
            uint16_t v;
            std::memcpy(&v, sc.data, 2);
            o.al_status_code = Raw::le16_to_host(v);
        }
    }
    return out;
}

// ============================================================================
// waitForState
// ============================================================================

bool SlaveGroup::waitForState(SlaveState target, uint32_t timeout_ms,
                              uint32_t poll_interval_ms,
                              uint32_t resend_interval_ms)
{
    auto& clock = Tether::Platform::Clock::instance();
    const int64_t deadline =
        clock.getMilliseconds() + static_cast<int64_t>(timeout_ms);
    int64_t last_resend = 0;

    while (clock.getMilliseconds() < deadline) {
        if (master_.isCancelRequested()) return false;

        const auto states = readStates(poll_interval_ms + 50);
        bool all_in_target = true;
        for (const auto& s : states) {
            if (!s.responded) {
                all_in_target = false;
                continue;
            }
            if (s.errorFlag()) {
                TETHER_LOGE(TAG,
                    "{}: AL error during {} wait — state=0x{:02X} "
                    "status code: {} (0x{:04X})",
                    master_.slaveLogPrefix(s.slave_index).c_str(),
                    slaveStateToString(target), s.al_status,
                    getALStatusCodeName(s.al_status_code), s.al_status_code);
                return false;
            }
            if (s.state() != target) all_in_target = false;
        }
        if (all_in_target && !states.empty()) return true;

        if (resend_interval_ms > 0 &&
            clock.getMilliseconds() - last_resend >=
                static_cast<int64_t>(resend_interval_ms)) {
            last_resend = clock.getMilliseconds();
            requestState(target, /*ack_error=*/true, poll_interval_ms);
        }

        clock.delayMilliseconds(poll_interval_ms);
    }

    // Final diagnostic pass
    const auto states = readStates(200);
    for (const auto& s : states) {
        if (!s.responded || s.state() != target) {
            TETHER_LOGW(TAG,
                "{}: not in {} after {} ms — responded={} "
                "AL_STATUS=0x{:04X} code: {} (0x{:04X})",
                master_.slaveLogPrefix(s.slave_index).c_str(),
                slaveStateToString(target), timeout_ms, s.responded,
                s.al_status, getALStatusCodeName(s.al_status_code),
                s.al_status_code);
        }
    }
    return false;
}

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
