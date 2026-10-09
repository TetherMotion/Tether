/**
 * @file LogicalAddressManager_cyclic.cpp
 * @brief LogicalAddressManager — cyclic image exchange (cyclicSend/collect).
 *
 * TU split out of LogicalAddressManager.cpp.
 */

#include "tether/ethercat/LogicalAddressManager.hpp"
#include "tether/ethercat/ProcessImage.hpp"
#include "tether/ethercat/CyclicChannel.hpp"
#include "tether/ethercat/Types.hpp"
#include "raw/TxFailureDiagnostics.hpp"
#include "raw/LogicalAddressManagerInternal.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cstring>
#include <ctime>

namespace EtherCAT {

static const char* TAG = "ec_logaddr";

// ============================================================================
// cyclicSend — gather outputs + emit all LRW slices
// ============================================================================

bool LogicalAddressManager::cyclicSend(const PDO::PDOMapping& mapping,
                                       ProcessImage* image,
                                       uint32_t rx_timeout_ns) {
    if (!transport_.supportsCyclicFastPath()) {
        // Legacy path stays atomic — no split support.
        return exchangeAllLRW(mapping);
    }
    if (!initialized_ || slave_count_ == 0) {
        stats_.send_errors++;
        return false;
    }
    const uint32_t total_data = next_free_log_;
    // Early-out on zero LIVE bytes, not zero extent: with sticky windows a
    // slave reconfigured to no PDOs leaves its old window allocated as
    // dead space (next_free_log_ > 0), but there is nothing to exchange.
    if (total_rxpdo_bytes_ + total_txpdo_bytes_ == 0 &&
        slices_.empty()) return true;

    // Whole-image decimation: with image_every_n_ > 1 the full exchange
    // runs only every Nth cycle — user PDO slices carry the hot bytes on
    // the in-between cycles (10 kHz slice / 1 kHz image, e.g.).
    const uint64_t cycle = exchange_cycle_++;
    if (total_data == 0 || (cycle % image_every_n_) != 0) {
        emitSlices(mapping, image, rx_timeout_ns);
        return true;
    }
    if (total_data > cyclic_payload_size_) {
        ensureCyclicPayload(total_data);
        if (total_data > cyclic_payload_size_) {
            stats_.send_errors++;
            return false;
        }
    }

    // Slice count: one LRW datagram per maxSliceLength()-sized chunk,
    // sent on cyclic slots 0..N-1.  Wire time is the real constraint —
    // at 100 Mbit/s a max-size frame costs ~120 µs on the wire.
    const uint32_t max_slice = maxSliceLength();
    uint32_t nslices = max_slice ? (total_data + max_slice - 1) / max_slice
                                 : kMaxCyclicSlices + 1;
    if (nslices > kMaxCyclicSlices) {
        TETHER_LOGE(TAG, "cyclic image {}B needs {} slices > {} slots — "
                         "cannot exchange (raise slot count or shrink "
                         "the image)", total_data, nslices,
                    kMaxCyclicSlices);
        stats_.send_errors++;
        return false;
    }
    cyclic_slice_count_ = static_cast<uint8_t>(nslices);

    // Concurrent mapping mutation (slave recovery re-registers PDO entries
    // on the supervisor thread while this iterates) produces a torn gather
    // — wrong bytes on the wire for every slave.  Snapshot the epoch now
    // and re-check after the gather; a changed epoch aborts the emit
    // silently (a skipped cycle, not an error — the mapping owner is
    // mid-reconfiguration and the next cycle uses the new layout).
    const uint32_t map_epoch0 = mapping.epoch();

    // Derive per-slice expected WKC from the slave set (Q6) — overrides
    // any learned values; underivable slices keep the learn sentinel.
    deriveExpectedWkc(mapping);

    const bool img_active = image && image->configured() &&
                            image->mode() != ImageMode::Buffered;

    // ---- Choose the TX payload ------------------------------------------
    uint8_t* payload         = cyclic_payload_.get();
    uint8_t* rotating_frame  = nullptr;
    bool     rotating_send   = false;

    if (img_active && image->mode() == ImageMode::Rotating) {
        rotating_frame = image->rotatingFrameBase();
        if (!rotating_frame) {
            rotating_frame = transport_.acquireCyclicTxFrame();
            // Offset comes from the transport — VLAN-tagged cyclic frames
            // carry the 802.1Q tag inline, pushing the payload 4 deeper.
            image->attachTxFrame(rotating_frame,
                                 transport_.cyclicPayloadOffset());
        }
        if (rotating_frame && nslices == 1) {
            payload        = rotating_frame + transport_.cyclicPayloadOffset();
            rotating_send  = true;
        } else if (rotating_frame) {
            // Rotating cannot span multiple frames — stage instead.
            TETHER_LOGW(TAG, "Rotating mode with {} slices — staging "
                             "payload instead (image > one frame)",
                        nslices);
            payload = const_cast<uint8_t*>(image->acquireSendImage());
            rotating_frame = nullptr;
        }
    } else if (img_active) {
        payload = const_cast<uint8_t*>(image->acquireSendImage());
    }

    if (!payload) {
        payload = cyclic_payload_.get();
        rotating_send = false;
    }

    if (!img_active) {
        std::memset(payload, 0, total_data);
    }

    // Gather RxPDO bytes from app buffers for entries that are NOT image
    // mapped (all entries on the legacy path; only -1-offset entries in
    // image modes).
    std::array<uint32_t, PDO::kMaxPDOSlaves> rx_running{};
    for (size_t i = 0; i < mapping.entry_count(); i++) {
        const PDO::PDOEntry* e = mapping.get_entry(i);
        if (!e || !e->enabled || e->direction != PDO::PDODirection::RxPDO) continue;
        if (e->slave_index >= slave_count_) continue;
        if (!addr_map_[e->slave_index].active) continue;

        const auto& addr = addr_map_[e->slave_index];
        const uint32_t entry_off = addr.rxpdo_logical_addr - base_logical_addr_
                                 + rx_running[e->slave_index];
        rx_running[e->slave_index] += e->data_size;
        if (e->data_size == 0) continue;
        if (entry_off + e->data_size > total_data) break;  // layout guard
        // Image-mapped entries live in the payload already; storage-bound
        // ones bridge — the device layer's cached storage pointer must
        // reach the wire even when the offsets table image-maps it (e.g.
        // an accessor taken after configureProcessImage()).
        if (img_active && !e->storage_bound &&
            image->entryOffset(i) >= 0) continue; // in-image

        std::memcpy(payload + entry_off, e->storage, e->data_size);
    }

    // Mapping changed under the gather — the staged payload mixes two
    // layouts.  Drop this cycle's emit rather than write torn outputs.
    if (mapping.epoch() != map_epoch0) {
        TETHER_LOGW(TAG, "cyclic send skipped: PDO mapping changed "
                         "mid-gather (slave re-registration?)");
        cyclic_pending_count_ = 0;
        pending_image_ = nullptr;
        return true;
    }

    // ---- Emit slices -----------------------------------------------------
    // Collect the response deadline once — the collect half shares it
    // regardless of when it runs (split-phase overlap).
    cyclic_send_ns_     = monoNowNs();
    cyclic_deadline_ns_ = cyclic_send_ns_ + rx_timeout_ns;
    cyclic_pending_count_ = 0;
    pending_image_ = image;
    cnt_pending_ = false;
    dc_pending_  = false;

    // Rotating pool: each LRW slice draws a fresh position (0..81) —
    // responses are waited for on the very positions they were sent on,
    // and a position is only re-armed when the allocator wraps back to
    // it, i.e. a response up to ~82 sends late still lands in its own
    // untouched mailbox instead of being dropped or aliasing a newer
    // request.  Nothing else ever clears these slots: re-arming is the
    // only invalidation, and it only touches the position being sent.
    uint8_t lrw_pos[kMaxCyclicSlices];
    for (uint32_t s = 0; s < nslices; ++s) lrw_pos[s] = allocPoolPos();

    // The counter trailer rides a dedicated datagram (LRW over the first
    // unmapped logical address — slaves pass it through verbatim, the
    // echo proves the frame belongs to THIS send) plus, when configured,
    // an APRD of the DC slave's System Time register.  Both trail the
    // last LRW slice inside its frame whenever the bytes fit; otherwise
    // they go in a second frame of the same send.
    const uint64_t cnt_value = ++lrw_counter_;
    cnt_value_ = cnt_value;
    cnt_pos_   = allocPoolPos();
    uint8_t cnt_bytes[8];
    std::memcpy(cnt_bytes, &cnt_value, 8);   // little-endian per host ABI
    const uint32_t cnt_addr = base_logical_addr_ + total_data;

    CyclicDgramSpec cnt_spec{};
    // LRD (not LRW): read-only — if the "unmapped" address were ever
    // wrong, an LRW could write counter bytes into slave outputs.
    cnt_spec.cmd      = Command::LRD;
    cnt_spec.idx      = cyclicPoolWireIdx(cnt_pos_);
    cnt_spec.adp      = static_cast<uint16_t>(cnt_addr & 0xFFFF);
    cnt_spec.ado      = static_cast<uint16_t>((cnt_addr >> 16) & 0xFFFF);
    cnt_spec.data     = cnt_bytes;
    cnt_spec.datalen  = kLrwCounterTrailerBytes;
    cnt_spec.roundtrip = true;
    cnt_spec.stamp_gen = true;

    CyclicDgramSpec dc_spec{};
    const bool want_dc = dc_slave_pos_ >= 0;
    if (want_dc) {
        dc_spec.cmd      = Command::APRD;
        dc_spec.idx      = kDcTimeIdx;
        dc_spec.adp      = static_cast<uint16_t>(0 - dc_slave_pos_);
        dc_spec.ado      = reg::DC_SYS_TIME;
        dc_spec.data     = nullptr;   // reads send zeros
        dc_spec.datalen  = 8;
        dc_spec.roundtrip = true;
        dc_spec.stamp_gen = true;
    }
    const uint8_t ntrailer = want_dc ? 2 : 1;

    auto fail_send = [&](uint8_t s) {
        stats_.send_errors++;
        healthFail(slice_health_[s], CyclicSliceStatus::SendError);
        cyclic_pending_count_ = 0;
        pending_image_ = nullptr;
        return false;
    };

    for (uint32_t s = 0; s < nslices; ++s) {
        const uint32_t off = s * max_slice;
        const uint32_t len = std::min(max_slice, total_data - off);
        const uint32_t logical_addr = base_logical_addr_ + off;
        const uint16_t adp = static_cast<uint16_t>(logical_addr & 0xFFFF);
        const uint16_t ado = static_cast<uint16_t>((logical_addr >> 16)
                                                 & 0xFFFF);
        const uint8_t pos = lrw_pos[s];
        const bool last   = (s + 1 == nslices);

        cyclic_pending_[s].token = transport_.cyclicSlotToken(pos);
        cyclic_pending_[s].off   = off;
        cyclic_pending_[s].len   = len;
        cyclic_pending_[s].pos   = pos;

        bool sent = false;
        if (last && !rotating_send) {
            // Trail counter (+DC) in the same frame when it fits; the
            // datagram bytes are ec-hdr-free math — 12 B wire cost each
            // (10 hdr + 2 wkc) plus payload.
            const uint32_t lrw_dgram  = 12 + len;
            const uint32_t tail_bytes = 12 + 8 + (want_dc ? 12 + 8 : 0);
            if (lrw_dgram + tail_bytes <=
                transport_.maxEtherCATPayloadPerFrame() + 12) {
                CyclicDgramSpec frame[3]{};
                frame[0].cmd      = Command::LRW;
                frame[0].idx      = cyclicPoolWireIdx(pos);
                frame[0].adp      = adp;
                frame[0].ado      = ado;
                frame[0].data     = payload + off;
                frame[0].datalen  = static_cast<uint16_t>(len);
                frame[0].roundtrip = true;
                frame[0].stamp_gen = true;
                frame[1] = cnt_spec;
                if (want_dc) frame[2] = dc_spec;
                sent = transport_.sendPoolFrame(frame,
                                                1u + ntrailer);
                if (sent) {
                    cnt_token_ = transport_.cyclicSlotToken(cnt_pos_);
                    cnt_gen_   = transport_.cyclicSlotGen(cnt_pos_);
                    cnt_pending_ = true;
                    if (want_dc) {
                        dc_token_ = transport_.cyclicSlotToken(
                            kCyclicDcPoolPos);
                        dc_gen_   = transport_.cyclicSlotGen(
                            kCyclicDcPoolPos);
                        dc_pending_ = true;
                    }
                }
                // sendPoolFrame may be unimplemented (test transports) —
                // fall back to single datagrams below.
            }
        }
        if (!sent && rotating_send) {
            const uint32_t poff = transport_.cyclicPayloadOffset();
            transport_.composeCyclicHeader(rotating_frame, Command::LRW,
                                           pos, adp, ado,
                                           static_cast<uint16_t>(len),
                                           true);
            *reinterpret_cast<uint16_t*>(
                rotating_frame + poff + len) = 0;
            image->detachTxFrame();
            sent = transport_.sendCyclicFrame(
                poff + len + sizeof(uint16_t));
            image->attachTxFrame(transport_.acquireCyclicTxFrame(),
                                 poff);
            rotating_send = false;   // single-slice path only
        }
        if (!sent) {
            sent = transport_.sendCyclicDatagram(
                Command::LRW, pos, adp, ado, payload + off,
                static_cast<uint16_t>(len), true);
        }
        if (!sent) return fail_send(static_cast<uint8_t>(s));
        // Send-generation stamped by the transport — collect rejects
        // deposits echoing the previous generation (stale-deposit ABA).
        cyclic_pending_[s].gen = transport_.cyclicSlotGen(pos);
        ++cyclic_pending_count_;
    }

    // Counter/DC trailers not yet sent (the last frame had no room, or
    // the transport cannot compose multi-datagram frames): emit them
    // now — a second frame of the same send, still in-flight together.
    if (!cnt_pending_) {
        CyclicDgramSpec tail[2]{};
        tail[0] = cnt_spec;
        if (want_dc) tail[1] = dc_spec;
        bool tail_sent = transport_.sendPoolFrame(tail, ntrailer);
        if (!tail_sent) {
            tail_sent = transport_.sendCyclicDatagram(
                Command::LRD, cnt_pos_,
                static_cast<uint16_t>(cnt_addr & 0xFFFF),
                static_cast<uint16_t>((cnt_addr >> 16) & 0xFFFF),
                cnt_bytes, kLrwCounterTrailerBytes, true);
            if (tail_sent && want_dc) {
                tail_sent = transport_.sendCyclicDatagram(
                    Command::APRD, kCyclicDcPoolPos,
                    static_cast<uint16_t>(0 - dc_slave_pos_),
                    reg::DC_SYS_TIME, nullptr, 8, true);
            }
        }
        if (tail_sent) {
            cnt_token_ = transport_.cyclicSlotToken(cnt_pos_);
            cnt_gen_   = transport_.cyclicSlotGen(cnt_pos_);
            cnt_pending_ = true;
            if (want_dc) {
                dc_token_ = transport_.cyclicSlotToken(kCyclicDcPoolPos);
                dc_gen_   = transport_.cyclicSlotGen(kCyclicDcPoolPos);
                dc_pending_ = true;
            }
        } else {
            // Trailer emit failed — the LRW slices are still live; the
            // collect just can't prove frame identity this cycle.
            stats_.send_errors++;
        }
    }

    emitSlices(mapping, image, rx_timeout_ns);
    return true;
}

void LogicalAddressManager::deriveExpectedWkc(
    const PDO::PDOMapping& mapping)
{
    // LRW WKC: +1 per slave that writes output bytes in the slice's
    // logical range, +2 per slave that reads input bytes.  Distinct
    // slaves per direction per slice — bitsets, then popcount.
    const uint32_t nslices = cyclic_slice_count_;
    const uint32_t max_slice = maxSliceLength();
    std::array<uint64_t, kMaxCyclicSlices> rx_set{}, tx_set{};
    for (const auto& e : describeEntries(mapping)) {
        if (e.length == 0 || e.slave_index >= 64) continue;
        for (uint32_t s = 0; s < nslices; ++s) {
            const uint32_t off = s * max_slice;
            const uint32_t end = off + max_slice;
            if (e.offset >= end || e.offset + e.length <= off) continue;
            if (e.direction == PDO::PDODirection::RxPDO)
                rx_set[s] |= 1ull << e.slave_index;
            else
                tx_set[s] |= 1ull << e.slave_index;
        }
    }
    for (uint32_t s = 0; s < nslices && s < kMaxCyclicSlices; ++s) {
        const uint32_t wkc =
            std::popcount(rx_set[s]) + 2u * std::popcount(tx_set[s]);
        // A 0 derivation means "no entry mapped into this slice" — bogus
        // for a live image slice, so keep the learn sentinel (Q7).
        expected_wkc_[s] = wkc ? static_cast<uint16_t>(wkc) : kWkcUnknown;
    }
    for (uint32_t s = nslices; s < kMaxCyclicSlices; ++s)
        expected_wkc_[s] = kWkcUnknown;
}

// ============================================================================
// cyclicCollect — wait slices, verify WKC, publish + scatter
// ============================================================================

bool LogicalAddressManager::cyclicCollect(const PDO::PDOMapping& mapping,
                                          ProcessImage* image) {
    if (!transport_.supportsCyclicFastPath()) {
        return true;   // cyclicSend already ran the atomic legacy exchange
    }
    const uint32_t total_data = next_free_log_;
    const uint8_t nslices = cyclic_pending_count_;
    // Nothing pending means the send half didn't emit (skipped, paused, or
    // failed — already counted there).  An empty collect is not an error —
    // but user PDO slices may still be in flight.
    if (nslices == 0) {
        return collectSlices(mapping, image);
    }
    image = pending_image_;
    const bool img_active = image && image->configured() &&
                            image->mode() != ImageMode::Buffered;

    // See cyclicSend: a mapping mutation during the wait/publish/scatter
    // window leaves entry offsets inconsistent — skip this cycle's publish
    // and scatter when the epoch moved.
    const uint32_t map_epoch0 = mapping.epoch();

    bool ok = true;
    std::array<const CyclicSlotView*, kMaxCyclicSlices> resps{};

    // Wait list: the nslices LRW positions, then the counter-trailer
    // position, then the DC position (entries n_extra ≤ 2).
    const uint8_t icnt = cnt_pending_ ? 1 : 0;
    const uint8_t idc  = dc_pending_ ? 1 : 0;
    const uint8_t nwait = static_cast<uint8_t>(nslices + icnt + idc);
    const uint8_t i_cnt = nslices;          // wait-list index of counter
    const uint8_t i_dc  = nslices + icnt;   // wait-list index of DC

    std::array<uint8_t,  kMaxCyclicSlices + 2> positions{};
    std::array<uint64_t, kMaxCyclicSlices + 2> tokens{};
    std::array<CyclicSlotView, kMaxCyclicSlices + 2> views{};
    std::array<bool,     kMaxCyclicSlices + 2> arrived{};
    for (uint8_t s = 0; s < nslices; ++s) {
        positions[s] = cyclic_pending_[s].pos;
        tokens[s]    = cyclic_pending_[s].token;
    }
    if (icnt) {
        positions[i_cnt] = cnt_pos_;
        tokens[i_cnt]    = cnt_token_;
    }
    if (idc) {
        positions[i_dc] = kCyclicDcPoolPos;
        tokens[i_dc]    = dc_token_;
    }
    const uint64_t now = monoNowNs();
    const uint32_t remaining = now < cyclic_deadline_ns_
        ? static_cast<uint32_t>(cyclic_deadline_ns_ - now) : 0;

    // One wake for the whole list — the transport's fallback still
    // walks per-position, so correctness never depends on the fast path.
    transport_.waitCyclicPool(positions.data(), tokens.data(), nwait,
                              remaining, views.data(), arrived.data());

    // Stale-deposit guard: a response deposited late — past its own
    // cycle's timeout — echoes the PREVIOUS send generation (lenFlags
    // res-bit 13).  Detect it, consume the deposit (refresh the token
    // baseline) and give the real response the remaining deadline once.
    // A second mismatch is pathological (two stales in flight) → miss.
    auto expected_gen = [&](uint8_t i) -> uint8_t {
        if (i < nslices) return cyclic_pending_[i].gen;
        if (i == i_cnt && icnt) return cnt_gen_;
        return dc_gen_;
    };
    uint8_t nstale = 0;
    for (uint8_t i = 0; i < nwait; ++i) {
        if (arrived[i] && views[i].gen != expected_gen(i)) {
            arrived[i] = false;
            ++nstale;
            stats_.stale_responses++;
            tokens[i] = transport_.cyclicSlotToken(positions[i]);
        }
    }
    if (nstale) {
        std::array<uint8_t, kMaxCyclicSlices + 2> re_pos{};
        std::array<uint64_t, kMaxCyclicSlices + 2> re_tok{};
        std::array<CyclicSlotView, kMaxCyclicSlices + 2> re_views{};
        std::array<bool, kMaxCyclicSlices + 2> re_arrived{};
        uint8_t nre = 0;
        for (uint8_t i = 0; i < nwait; ++i) {
            if (arrived[i]) continue;
            re_pos[nre] = positions[i];
            re_tok[nre] = tokens[i];
            ++nre;
        }
        const uint64_t now2 = monoNowNs();
        const uint32_t remain2 = now2 < cyclic_deadline_ns_
            ? static_cast<uint32_t>(cyclic_deadline_ns_ - now2) : 0;
        if (nre && remain2) {
            transport_.waitCyclicPool(re_pos.data(), re_tok.data(), nre,
                                      remain2, re_views.data(),
                                      re_arrived.data());
            uint8_t ri = 0;
            for (uint8_t i = 0; i < nwait; ++i) {
                if (arrived[i]) continue;
                const bool got = re_arrived[ri];
                const CyclicSlotView& rv = re_views[ri];
                ++ri;
                // The retry winner must carry THIS send's generation; a
                // second stale or a timeout leaves the position missed.
                if (!got) continue;
                if (rv.gen != expected_gen(i)) {
                    stats_.stale_responses++;
                    if (i < nslices)
                        healthMarkStatus(slice_health_[i],
                                         CyclicSliceStatus::Stale);
                    continue;
                }
                arrived[i] = true;
                views[i]   = rv;
            }
        }
    }

    for (uint8_t s = 0; s < nslices; ++s) {
        uint64_t& hw = slice_health_[s];
        if (!arrived[s]) {
            stats_.timeout_errors++;
            healthTimeout(hw, expected_wkc_[s]);
            ok = false;
            continue;
        }
        const CyclicSlotView& resp = views[s];
        // Wire RTT for diagnostics: emit -> kernel RX stamp.  Only
        // in-generation deposits count (a stale echoes an older send).
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
        const uint16_t exp = expected_wkc_[s];
        if (resp.wkc == 0 ||
            (strict_wkc_ && exp != kWkcUnknown && resp.wkc != exp)) {
            stats_.wkc_errors++;
            markRunStatus(hw, CyclicSliceStatus::WkcError, resp.wkc, exp);
            ok = false;
            continue;
        }
        // Sentinel fallback (Q7): a slice we couldn't derive learns its
        // expectation from the first non-error response.
        if (exp == kWkcUnknown && resp.wkc != 0)
            expected_wkc_[s] = resp.wkc;
        markRunStatus(hw, CyclicSliceStatus::Ok, resp.wkc,
                      expected_wkc_[s]);
        resps[s] = &views[s];
    }

    // ---- Counter trailer: verify the frame belongs to THIS send -------
    // The trailer datagram's payload IS the counter — an echo returning
    // anything else (older send, corrupted frame) is rejected: the PDO
    // data of the last frame is suspect and the exchange fails closed.
    if (icnt) {
        bool cnt_ok = false;
        if (arrived[i_cnt] && views[i_cnt].payload &&
            views[i_cnt].datalen >= kLrwCounterTrailerBytes) {
            uint64_t echoed = 0;
            std::memcpy(&echoed, views[i_cnt].payload, 8);
            cnt_ok = (echoed == cnt_value_);
        }
        if (cnt_ok) {
            lrw_counter_ok_ = cnt_value_;
        } else {
            ++stats_.counter_mismatches;
            // The trailer protects the LAST frame — fail that slice's
            // data rather than trust a frame of unproven provenance.
            if (nslices > 0) {
                resps[nslices - 1] = nullptr;
                markRunStatus(slice_health_[nslices - 1],
                              CyclicSliceStatus::Stale,
                              arrived[i_cnt] ? views[i_cnt].wkc : 0,
                              expected_wkc_[nslices - 1]);
            }
            ok = false;
        }
    }

    // ---- DC timepoint: same-frame APRD of the configured slave --------
    if (idc) {
        if (arrived[i_dc] && views[i_dc].payload &&
            views[i_dc].datalen >= 8 && views[i_dc].wkc >= 1) {
            std::memcpy(&dc_time_ns_, views[i_dc].payload, 8);
            dc_time_valid_ = true;
        } else {
            ++stats_.dc_timeouts;
            dc_time_valid_ = false;
        }
    }
    cnt_pending_ = false;
    dc_pending_  = false;
    cyclic_pending_count_ = 0;
    pending_image_ = nullptr;
    if (!ok) {
        // Still drain the user slices — an image-collect failure must not
        // wedge their pipelines (they own separate slots).
        collectSlices(mapping, image);
        return false;
    }

    // The mapping changed while the responses were in flight — entry
    // offsets (and image layout) no longer match the gathered mapping.
    // Drop the publish/scatter; the datagram already executed on the wire.
    if (mapping.epoch() != map_epoch0) {
        TETHER_LOGW(TAG, "cyclic collect skipped: PDO mapping changed "
                         "mid-exchange (slave re-registration?)");
        stats_.success++;
        collectSlices(mapping, image);
        return true;
    }

    // ---- Publish the input image --------------------------------------
    if (img_active) {
        // View publish is only safe when no user slices exist — slices
        // publish by overlaying the input bank, which requires the bank
        // to hold the canonical image.  With slices defined, always take
        // the bank path so later slice data lands on top.
        if (nslices == 1 && resps[0] && resps[0]->payload &&
            slices_.empty()) {
            const CyclicSlotView& resp = *resps[0];
            if (resp.channel) {
                image->publishInputView(resp.payload, resp.datalen,
                                        resp.cookie, resp.channel);
            } else {
                image->publishInputCopy(resp.payload, resp.datalen);
            }
        } else {
            // Multi-slice: stage each slice at its offset, one publish.
            uint8_t* bank = image->inputWriteBank();
            if (bank) {
                for (uint8_t s = 0; s < nslices; ++s) {
                    if (!resps[s] || !resps[s]->payload) continue;
                    const uint32_t off = cyclic_pending_[s].off;
                    const uint32_t n = std::min<uint32_t>(resps[s]->datalen,
                                                          total_data - off);
                    std::memcpy(bank + off, resps[s]->payload, n);
                }
                image->commitInput();
            }
        }
    }

    // ---- Scatter TxPDO data into app buffers (non-image entries) ------
    std::array<uint32_t, PDO::kMaxPDOSlaves> tx_running{};
    for (size_t i = 0; i < mapping.entry_count(); i++) {
        const PDO::PDOEntry* e = mapping.get_entry(i);
        if (!e || !e->enabled || e->direction != PDO::PDODirection::TxPDO) continue;
        if (e->slave_index >= slave_count_) continue;
        if (!addr_map_[e->slave_index].active) continue;

        const auto& addr = addr_map_[e->slave_index];
        const uint32_t entry_off = addr.txpdo_logical_addr - base_logical_addr_
                                 + tx_running[e->slave_index];
        tx_running[e->slave_index] += e->data_size;
        if (e->data_size == 0) continue;
        // Mirror of the gather predicate: storage-bound entries scatter
        // back into storage even when the offsets table image-maps them.
        if (img_active && !e->storage_bound &&
            image->entryOffset(i) >= 0) continue;

        // Copy from whichever slice(s) cover the entry range.
        uint32_t done = 0;
        for (uint8_t s = 0; s < nslices && done < e->data_size; ++s) {
            if (!resps[s] || !resps[s]->payload) continue;
            const uint32_t s0 = cyclic_pending_[s].off;
            const uint32_t s1 = s0 + resps[s]->datalen;
            const uint32_t e0 = entry_off, e1 = entry_off + e->data_size;
            if (e0 >= s1 || s0 >= e1) continue;
            const uint32_t lo = std::max(e0, s0);
            const uint32_t hi = std::min(e1, s1);
            std::memcpy(e->storage + (lo - e0),
                        resps[s]->payload + (lo - s0), hi - lo);
            done += hi - lo;
        }
    }

    stats_.success++;

    // User PDO slices collect AFTER the image publish — their (fresher)
    // bytes overlay the input bank on top of the full-image data.
    if (!collectSlices(mapping, image)) ok = false;
    return ok;
}

} // namespace EtherCAT

