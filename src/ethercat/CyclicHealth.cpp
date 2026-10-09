/**
 * @file CyclicHealth.cpp
 * @brief CyclicHealth::describe() — human-readable health dump.
 *
 * Split out of Master_loops.cpp; the type is declared in CyclicHealth.hpp.
 */

#include "tether/ethercat/CyclicHealth.hpp"

#include <format>

namespace EtherCAT {

std::string CyclicHealth::describe() const
{
    std::string out = std::format(
        "cycles={} ok={} wire_loss={} wkc_err={} stale={} send_err={} "
        "rx_bank_drops={} tp_drops={} unrouted={} rxq_overflow={} "
        "missed_deadlines={} collect_calls={} wait_calls={} "
        "dispatched={} disp_unrouted={}",
        cycles, exchanges_ok, wire_loss, wkc_errors, stale_responses,
        send_errors, rx_bank_drops, kernel_rx_drops, unrouted_datagrams,
        rx_queue_overflow, missed_deadlines, collect_calls, wait_calls,
        dispatch_frames, dispatch_unrouted);
    // Late-vs-lost split: a stale deposit is a reply that arrived AFTER
    // its cycle deadline (the send counted wire_loss, then the echo
    // surfaced next cycle).  wire_loss - stale ≈ true non-return.
    const uint32_t true_loss =
        wire_loss > stale_responses ? wire_loss - stale_responses : 0;
    out += std::format(
        "\n  loss split: never_returned≈{} late_replies={}"
        "{}",
        true_loss, stale_responses,
        rtt_samples
            ? std::format("  rtt_us=min/{}/avg/{}/max/{} ({} samples)",
                          rtt_us_min, rtt_us_avg, rtt_us_max, rtt_samples)
            : std::string{});
    if (slices_stuck_stale)
        out += std::format(
            "\n  {} slice(s) stuck Stale — wire RTT exceeds a full "
            "cycle period; every cycle only sees the previous echo",
            slices_stuck_stale);
    auto slice_line = [&out](const char* kind, uint32_t i,
                             const CyclicSliceHealth& s) {
        if (s.last_status == CyclicSliceStatus::Ok &&
            s.consecutive_failures == 0)
            return;
        out += std::format("\n  {} slice {}: status={} wkc={}/{} "
                           "consec_fail={}",
                           kind, i, toString(s.last_status), s.last_wkc,
                           s.expected_wkc, s.consecutive_failures);
    };
    for (uint8_t i = 0; i < image_slice_count; ++i)
        slice_line("image", i, image_slices[i]);
    for (uint32_t i = 0; i < pdo_slice_health.size(); ++i)
        slice_line("pdo", i, pdo_slice_health[i]);
    return out;
}

} // namespace EtherCAT
