/**
 * @file SlaveGroup.cpp
 * @brief Single-packet AL state control/query for slave groups — see header.
 */

#include "tether/ethercat/SlaveGroup.hpp"

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/TransactionRouter.hpp"
#include "tether/ethercat/FaultDetection.hpp"
#include "tether/platform/Platform.hpp"

#include "raw/RawWireFormat.hpp"

#include "logging/Logger.hpp"

#include <cstring>

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

} // namespace EtherCAT
