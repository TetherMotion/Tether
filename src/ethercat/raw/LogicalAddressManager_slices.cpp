/**
 * @file LogicalAddressManager_slices.cpp
 * @brief LogicalAddressManager — user-defined PDO slice
 *        define/plan/emit/collect/one-off exchange.
 *
 * TU split out of LogicalAddressManager.cpp; the class and its state are
 * declared in LogicalAddressManager.hpp.
 */

#include "tether/ethercat/LogicalAddressManager.hpp"
#include "tether/ethercat/ProcessImage.hpp"
#include "tether/ethercat/CyclicChannel.hpp"
#include "tether/ethercat/Types.hpp"
#include "raw/LogicalAddressManagerInternal.hpp"

#include <algorithm>
#include <cstring>
#include <format>

namespace EtherCAT {

static const char* TAG = "ec_logaddr";

// ============================================================================
// User-defined PDO slices — define/plan/emit/collect/one-off
// ============================================================================

size_t LogicalAddressManager::resolveSpecRuns(
    const PDO::PDOMapping& mapping, const PDOSliceSpec& spec,
    std::array<std::pair<uint32_t, uint32_t>, kMaxSliceRuns>& out) const
{
    size_t n = 0;
    if (!spec.entries.empty()) {
        // Resolve entry indices → logical placements → merge exact-adjacent
        // placements into contiguous runs (one LRW datagram per run).
        const auto all = describeEntries(mapping);
        std::array<std::pair<uint32_t, uint32_t>, kMaxSliceRuns * 2> segs{};
        size_t nseg = 0;
        for (const uint16_t ei : spec.entries) {
            if (ei >= all.size()) continue;
            const EntrySlice& e = all[ei];
            if (e.length == 0 || nseg >= segs.size()) continue;
            segs[nseg++] = {e.offset, e.length};
        }
        std::sort(segs.begin(), segs.begin() + nseg);
        for (size_t i = 0; i < nseg && n < out.size(); ++i) {
            if (n > 0 && out[n - 1].first + out[n - 1].second
                             == segs[i].first) {
                out[n - 1].second += segs[i].second;   // contiguous — merge
            } else {
                out[n++] = segs[i];
            }
        }
    } else {
        for (const auto& r : spec.ranges) {
            if (n >= out.size()) break;
            if (r.second == 0) continue;
            out[n++] = r;
        }
        std::sort(out.begin(), out.begin() + n);
    }
    return n;
}

uint32_t LogicalAddressManager::definePDOSlice(
    const PDO::PDOMapping& mapping, const PDOSliceSpec& spec)
{
    constexpr uint32_t kInvalid = 0xFFFFFFFFu;
    if (!initialized_ || slave_count_ == 0) {
        TETHER_LOGW(TAG, "definePDOSlice: LAM not initialized");
        return kInvalid;
    }
    if (spec.entries.empty() && spec.ranges.empty()) {
        TETHER_LOGW(TAG, "definePDOSlice: spec selects no bytes "
                         "(entries and ranges both empty)");
        return kInvalid;
    }

    // Validate against the CURRENT mapping: resolve the new spec and every
    // already-defined spec to count total runs against the slot pool.
    std::array<std::pair<uint32_t, uint32_t>, kMaxSliceRuns> rr{};
    const size_t n = resolveSpecRuns(mapping, spec, rr);
    if (n == 0) {
        TETHER_LOGW(TAG, "definePDOSlice: spec resolves to zero "
                         "contiguous runs");
        return kInvalid;
    }
    const uint32_t max_len = maxSliceLength();
    const uint32_t img_len = next_free_log_;
    size_t total = n;
    for (size_t i = 0; i < n; ++i) {
        if (rr[i].second > max_len) {
            TETHER_LOGE(TAG, "definePDOSlice: run [{}+{}B] exceeds one "
                             "datagram ({}B) — split the spec",
                        rr[i].first, rr[i].second, max_len);
            return kInvalid;
        }
        if (rr[i].first + rr[i].second > img_len) {
            TETHER_LOGE(TAG, "definePDOSlice: run [{}+{}B] exceeds the "
                             "logical image ({}B)",
                        rr[i].first, rr[i].second, img_len);
            return kInvalid;
        }
    }
    for (const auto& s : slices_) {
        std::array<std::pair<uint32_t, uint32_t>, kMaxSliceRuns> other{};
        total += resolveSpecRuns(mapping, s.spec, other);
    }
    if (total > IPDOTransport::kNumSliceSlots) {
        TETHER_LOGE(TAG, "definePDOSlice: {} runs requested, {} slice "
                         "slots available", total,
                    IPDOTransport::kNumSliceSlots);
        return kInvalid;
    }

    PDOSlice s;
    s.spec     = spec;
    s.every_n  = spec.every_n ? spec.every_n : 1;
    s.run_count = static_cast<uint8_t>(n);
    for (size_t i = 0; i < n; ++i) {
        s.runs[i].off = rr[i].first;
        s.runs[i].len = rr[i].second;
    }
    slices_.push_back(std::move(s));
    // Sentinel, not 0: a fresh mapping's epoch IS 0 (add_*pdo() doesn't
    // bump it), so 0 would falsely match and skip slot/WKC assignment.
    slice_epoch_ = 0xFFFFFFFFu;   // force (re)plan next send
    const uint32_t handle = static_cast<uint32_t>(slices_.size() - 1);
    TETHER_LOGI(TAG, "PDO slice {} defined: {} run(s), every {} cycle(s)",
                handle, n, s.every_n);
    return handle;
}

bool LogicalAddressManager::clearPDOSlices()
{
    slices_.clear();
    slice_epoch_ = 0xFFFFFFFFu;
    return true;
}

bool LogicalAddressManager::replanSlices(const PDO::PDOMapping& mapping)
{
    // Re-resolve every slice's runs against the current mapping and
    // re-assign slots in define-order — deterministic, so the same
    // mapping always yields the same slot layout.
    uint8_t next_slot = 0;
    const uint32_t max_len = maxSliceLength();
    const auto entries = describeEntries(mapping);   // WKC derivation
    for (auto& slice : slices_) {
        std::array<std::pair<uint32_t, uint32_t>, kMaxSliceRuns> rr{};
        const size_t n = resolveSpecRuns(mapping, slice.spec, rr);
        slice.run_count = static_cast<uint8_t>(n);
        for (size_t i = 0; i < n; ++i) {
            auto& run = slice.runs[i];
            run.off = rr[i].first;
            run.len = rr[i].second;
            const uint32_t img_len = next_free_log_;
            if (run.len > max_len || run.off + run.len > img_len) {
                TETHER_LOGE(TAG, "PDO slice replan: run [{}+{}B] exceeds "
                                 "one datagram or the image — slice "
                                 "disabled", run.off, run.len);
                slice.run_count = 0;
                break;
            }
            if (next_slot >= IPDOTransport::kNumSliceSlots) {
                TETHER_LOGE(TAG, "PDO slice replan: slice-slot pool "
                                 "exhausted — slice disabled");
                slice.run_count = 0;
                break;
            }
            run.slot  = next_slot++;
            run.sent  = 0;
            // Derive the run's expected WKC: +1 per slave writing output
            // bytes in range, +2 per slave reading input bytes.
            uint64_t rx_set = 0, tx_set = 0;
            for (const auto& e : entries) {
                if (e.length == 0 || e.slave_index >= 64) continue;
                if (e.offset >= run.off + run.len ||
                    e.offset + e.length <= run.off) continue;
                if (e.direction == PDO::PDODirection::RxPDO)
                    rx_set |= 1ull << e.slave_index;
                else
                    tx_set |= 1ull << e.slave_index;
            }
            const uint32_t wkc =
                std::popcount(rx_set) + 2u * std::popcount(tx_set);
            run.expected_wkc = wkc ? static_cast<uint16_t>(wkc)
                                   : kWkcUnknown;
        }
    }
    slice_epoch_ = mapping.epoch();
    return true;
}

void LogicalAddressManager::emitSlices(const PDO::PDOMapping& mapping,
                                       ProcessImage* image,
                                       uint32_t rx_timeout_ns)
{
    if (slices_.empty()) return;
    if (slice_epoch_ != mapping.epoch()) replanSlices(mapping);
    const bool img_active = image && image->configured() &&
                            image->mode() != ImageMode::Buffered;
    // Send-image source for slice payloads: acquireSendImage() covers
    // Direct/Double/Triple (snapshot semantics).  Rotating returns
    // nullptr there — its send image is the attached TX frame's payload
    // region, reached via outputWrite().  Null → staging-buffer gather.
    const uint8_t* send_img = nullptr;
    if (img_active) {
        send_img = (image->mode() == ImageMode::Rotating)
            ? image->outputWrite()
            : image->acquireSendImage();
    }
    for (auto& slice : slices_) {
        if (slice.run_count == 0) continue;
        if (slice.every_n > 1 && ++slice.cycle_mod < slice.every_n)
            continue;
        slice.cycle_mod = 0;
        if (slice.pending) continue;   // last collect still open — skip
        slice.deadline_ns = monoNowNs() + rx_timeout_ns;
        for (uint8_t r = 0; r < slice.run_count; ++r) {
            auto& run = slice.runs[r];
            const uint32_t logical_addr = base_logical_addr_ + run.off;
            const uint16_t adp = static_cast<uint16_t>(logical_addr & 0xFFFF);
            const uint16_t ado = static_cast<uint16_t>(
                (logical_addr >> 16) & 0xFFFF);

            const uint8_t* data;
            if (send_img) {
                data = send_img + run.off;
            } else {
                // Gather RxPDO bytes intersecting the run into staging —
                // the rest of the datagram reads as zeros (FMMU write
                // regions only accept their own slave's bytes anyway).
                if (run.len > slice_payload_size_) {
                    slice_payload_.reset(new uint8_t[run.len]);
                    slice_payload_size_ = run.len;
                }
                std::memset(slice_payload_.get(), 0, run.len);
                std::array<uint32_t, PDO::kMaxPDOSlaves> rx_running{};
                for (size_t i = 0; i < mapping.entry_count(); i++) {
                    const PDO::PDOEntry* e = mapping.get_entry(i);
                    if (!e || !e->enabled ||
                        e->direction != PDO::PDODirection::RxPDO) continue;
                    if (e->slave_index >= slave_count_ ||
                        !addr_map_[e->slave_index].active) continue;
                    const auto& addr = addr_map_[e->slave_index];
                    const uint32_t e0 =
                        addr.rxpdo_logical_addr - base_logical_addr_
                        + rx_running[e->slave_index];
                    rx_running[e->slave_index] += e->data_size;
                    const uint32_t s0 = run.off, s1 = run.off + run.len;
                    const uint32_t e1 = e0 + e->data_size;
                    if (e->data_size == 0 || e0 >= s1 || s0 >= e1) continue;
                    const uint32_t lo = std::max(e0, s0);
                    const uint32_t hi = std::min(e1, s1);
                    std::memcpy(slice_payload_.get() + (lo - s0),
                                e->storage + (lo - e0), hi - lo);
                }
                data = slice_payload_.get();
            }

            run.token = transport_.sliceSlotToken(run.slot);
            run.sent  = transport_.sendSliceDatagram(
                Command::LRW, run.slot, adp, ado, data,
                static_cast<uint16_t>(run.len), true) ? 1 : 0;
            if (!run.sent) {
                stats_.send_errors++;
                healthFail(run.health_packed, CyclicSliceStatus::SendError);
                continue;
            }
            run.gen = transport_.sliceSlotGen(run.slot);
        }
        slice.pending = true;
    }
}

bool LogicalAddressManager::collectSlices(const PDO::PDOMapping& mapping,
                                          ProcessImage* image)
{
    if (slices_.empty()) return true;
    bool ok = true;
    bool bank_touched = false;
    const bool img_active = image && image->configured() &&
                            image->mode() != ImageMode::Buffered;
    const uint32_t total_data = next_free_log_;
    const bool epoch_ok = (slice_epoch_ == mapping.epoch());

    for (auto& slice : slices_) {
        if (!slice.pending) continue;
        slice.pending = false;

        uint32_t mask = 0;
        std::array<uint64_t, IPDOTransport::kNumSliceSlots> tokens{};
        std::array<CyclicSlotView, IPDOTransport::kNumSliceSlots> views{};
        for (uint8_t r = 0; r < slice.run_count; ++r) {
            const auto& run = slice.runs[r];
            if (!run.sent) continue;
            mask |= 1u << run.slot;
            tokens[run.slot] = run.token;
        }
        if (!mask) continue;

        const uint64_t now = monoNowNs();
        const uint32_t remaining = now < slice.deadline_ns
            ? static_cast<uint32_t>(slice.deadline_ns - now) : 0;
        uint32_t arrived = transport_.waitSliceSlotMask(
            mask, tokens.data(), remaining, views.data());

        // Stale-generation retry — same guard as the image collect: a
        // late response echoes the previous send generation.
        uint32_t stale = 0;
        for (uint8_t r = 0; r < slice.run_count; ++r) {
            const auto& run = slice.runs[r];
            if (!run.sent) continue;
            if ((arrived & (1u << run.slot)) &&
                views[run.slot].gen != run.gen) {
                stale |= 1u << run.slot;
                stats_.stale_responses++;
            }
        }
        if (stale) {
            for (uint8_t r = 0; r < slice.run_count; ++r) {
                const auto& run = slice.runs[r];
                if (stale & (1u << run.slot))
                    tokens[run.slot] = transport_.sliceSlotToken(run.slot);
            }
            const uint64_t now2 = monoNowNs();
            const uint32_t rem2 = now2 < slice.deadline_ns
                ? static_cast<uint32_t>(slice.deadline_ns - now2) : 0;
            const uint32_t arrived2 = rem2
                ? transport_.waitSliceSlotMask(stale, tokens.data(),
                                               rem2, views.data())
                : 0;
            arrived = (arrived & ~stale) | (arrived2 & stale);
            for (uint8_t r = 0; r < slice.run_count; ++r) {
                auto& run = slice.runs[r];
                if ((arrived2 & (1u << run.slot)) &&
                    views[run.slot].gen != run.gen) {
                    arrived &= ~(1u << run.slot);
                    stats_.stale_responses++;
                    healthMarkStatus(run.health_packed,
                                     CyclicSliceStatus::Stale);
                }
            }
        }

        for (uint8_t r = 0; r < slice.run_count; ++r) {
            auto& run = slice.runs[r];
            if (!run.sent) continue;
            if (!(arrived & (1u << run.slot))) {
                stats_.timeout_errors++;
                healthTimeout(run.health_packed, run.expected_wkc);
                ok = false;
                continue;
            }
            const CyclicSlotView& resp = views[run.slot];
            if (resp.stamp_ns > cyclic_send_ns_) {
                const uint64_t rtt = resp.stamp_ns - cyclic_send_ns_;
                stats_.rtt_ns_sum += rtt;
                const uint32_t r = static_cast<uint32_t>(
                    std::min<uint64_t>(rtt, UINT32_MAX));
                if (stats_.rtt_samples == 0 || r < stats_.rtt_ns_min)
                    stats_.rtt_ns_min = r;
                if (r > stats_.rtt_ns_max) stats_.rtt_ns_max = r;
                ++stats_.rtt_samples;
            }
            if (resp.wkc == 0 ||
                (strict_wkc_ && run.expected_wkc != kWkcUnknown &&
                 resp.wkc != run.expected_wkc)) {
                stats_.wkc_errors++;
                markRunStatus(run.health_packed, CyclicSliceStatus::WkcError,
                              resp.wkc, run.expected_wkc);
                ok = false;
                continue;
            }
            if (run.expected_wkc == kWkcUnknown && resp.wkc != 0)
                run.expected_wkc = resp.wkc;
            markRunStatus(run.health_packed, CyclicSliceStatus::Ok,
                          resp.wkc, run.expected_wkc);
            // A mid-flight mapping change leaves run offsets stale — the
            // datagram executed; skip scatter/publish, replan next send.
            if (!epoch_ok) continue;

            // Scatter TxPDO bytes to entry storage — mirror of the image
            // collect's predicate (image-mapped entries live in the bank).
            std::array<uint32_t, PDO::kMaxPDOSlaves> tx_running{};
            for (size_t i = 0; i < mapping.entry_count(); i++) {
                const PDO::PDOEntry* e = mapping.get_entry(i);
                if (!e || !e->enabled ||
                    e->direction != PDO::PDODirection::TxPDO) continue;
                if (e->slave_index >= slave_count_ ||
                    !addr_map_[e->slave_index].active) continue;
                const auto& addr = addr_map_[e->slave_index];
                const uint32_t e0 =
                    addr.txpdo_logical_addr - base_logical_addr_
                    + tx_running[e->slave_index];
                tx_running[e->slave_index] += e->data_size;
                const uint32_t s0 = run.off;
                const uint32_t s1 = run.off + resp.datalen;
                const uint32_t e1 = e0 + e->data_size;
                if (e->data_size == 0 || e0 >= s1 || s0 >= e1) continue;
                if (img_active && !e->storage_bound &&
                    image->entryOffset(i) >= 0) continue;
                if (!resp.payload) break;
                const uint32_t lo = std::max(e0, s0);
                const uint32_t hi = std::min(e1, s1);
                std::memcpy(e->storage + (lo - e0),
                            resp.payload + (lo - s0), hi - lo);
            }
            // Publish into the input bank — overlays the full image's
            // data (this collect runs after the image publish).
            if (img_active && resp.payload) {
                uint8_t* bank = image->inputWriteBank();
                if (bank) {
                    const uint32_t ncp = std::min<uint32_t>(
                        resp.datalen, total_data - run.off);
                    std::memcpy(bank + run.off, resp.payload, ncp);
                    bank_touched = true;
                }
            }
            if (slice.spec.on_exchange && resp.payload) {
                slice.spec.on_exchange(r, resp.payload, resp.datalen,
                                       resp.wkc);
            }
        }
    }
    if (bank_touched) image->commitInput();
    return ok;
}

bool LogicalAddressManager::exchangePDOSlice(const PDO::PDOMapping& mapping,
                                             const PDOSliceSpec& spec)
{
    std::array<std::pair<uint32_t, uint32_t>, kMaxSliceRuns> rr{};
    const size_t n = resolveSpecRuns(mapping, spec, rr);
    if (n == 0) {
        TETHER_LOGW(TAG, "exchangePDOSlice: spec resolves to zero runs");
        return false;
    }
    bool ok = true;
    for (size_t i = 0; i < n; ++i) {
        if (!exchangeLRWSlice(mapping, rr[i].first, rr[i].second))
            ok = false;
    }
    return ok;
}

bool LogicalAddressManager::exchangeLRWSlice(const PDO::PDOMapping& mapping,
                                             uint32_t offset, uint32_t length) {
    return exchangeLRWImpl(mapping, offset, length,
                           /*enforce_slice_limit=*/true);
}

uint32_t LogicalAddressManager::maxSliceLength() const {
    // One LRW datagram = frame payload minus the per-datagram wire overhead
    // (datagram header: cmd+idx+adp+ado+len/flags+irq = 10 B, plus WKC = 2 B).
    static constexpr uint32_t kDatagramOverhead = 12;
    const size_t frame = transport_.maxEtherCATPayloadPerFrame();
    // The frame may allow more than a datagram's 11-bit length field can
    // express — a single LRW slice is still capped at kMaxDatagramDataSize.
    const size_t limit = std::min<size_t>(frame, kMaxDatagramDataSize
                                                   + kDatagramOverhead);
    return limit > kDatagramOverhead
         ? static_cast<uint32_t>(limit - kDatagramOverhead)
         : 0u;
}

std::vector<LogicalAddressManager::EntrySlice>
LogicalAddressManager::describeEntries(const PDO::PDOMapping& mapping) const {
    std::vector<EntrySlice> out;
    if (!initialized_ || slave_count_ == 0) return out;
    out.reserve(mapping.entry_count());

    std::array<uint32_t, PDO::kMaxPDOSlaves> rx_running{};
    std::array<uint32_t, PDO::kMaxPDOSlaves> tx_running{};

    for (size_t i = 0; i < mapping.entry_count(); i++) {
        const PDO::PDOEntry* e = mapping.get_entry(i);
        if (!e || !e->enabled) continue;
        if (e->slave_index >= slave_count_) continue;
        if (!addr_map_[e->slave_index].active) continue;

        const auto& addr = addr_map_[e->slave_index];
        EntrySlice s;
        s.entry_index = i;
        s.slave_index = e->slave_index;
        s.pdo_index   = e->pdo_index;
        s.direction   = e->direction;
        s.length      = e->data_size;
        if (e->direction == PDO::PDODirection::RxPDO) {
            s.offset = addr.rxpdo_logical_addr - base_logical_addr_
                     + rx_running[e->slave_index];
            rx_running[e->slave_index] += e->data_size;
        } else {
            s.offset = addr.txpdo_logical_addr - base_logical_addr_
                     + tx_running[e->slave_index];
            tx_running[e->slave_index] += e->data_size;
        }
        out.push_back(s);
    }
    return out;
}


} // namespace EtherCAT

