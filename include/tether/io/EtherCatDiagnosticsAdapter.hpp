#pragma once

/**
 * @file EtherCatDiagnosticsAdapter.hpp
 * @brief IMachineDiagnosticsSource over a running EtherCAT::Master.
 *
 * Plugging the optional machine.diagnostics signal into a real master is:
 *
 * @code
 *   EtherCatDiagnosticsAdapter diagnostics(master);
 *   machineService.setDiagnosticsSource(&diagnostics);   // before install()
 * @endcode
 *
 * Dependency direction: tether/io depends on tether/ethercat here, never the
 * reverse — CiA402/DS402 drivers do not pull in the IO framework.
 *
 * The adapter reads only non-blocking cumulative counters (cyclic executive,
 * async loop, link stats, last WKC). `slavesDetected` reports the last
 * discovery result; per-slave OP counting uses Slave::ALState() which is a
 * synchronous register read on the calling IO session thread — the same
 * threading contract as EtherCatSdoAccess.
 *
 * These counters are observability data for commissioning and service. They
 * are not a protective channel and must never gate functional-safety
 * behavior — that stays in hardware (E-stop, STO, interlocks).
 */

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/Types.hpp"
#include "tether/io/MachineService.hpp"

namespace tether::io::machine {

class EtherCatDiagnosticsAdapter final : public IMachineDiagnosticsSource {
public:
    explicit EtherCatDiagnosticsAdapter(EtherCAT::Master& master) : master_(master) {}

    MachineDiagnosticsV1 read() override {
        MachineDiagnosticsV1 d;
        d.timestampUs = detail::nowUs();

        const auto cyclic = master_.getCyclicLoopStats();
        d.cycleCount = cyclic.cycle_count;
        d.missedDeadlines = cyclic.missed_deadlines;
        d.maxCycleWorkUs = cyclic.max_cycle_work_us;
        d.jitterMaxUs = cyclic.jitter.max_jitter_us;
        d.jitterAvgUs = cyclic.jitter.avg_jitter_us;
        d.dcSyncCount = cyclic.dc_sync_count;
        d.dcSyncErrors = cyclic.dc_sync_errors;
        d.dcJitterMaxUs = cyclic.dc_jitter.max_jitter_us;

        const auto async = master_.getAsyncLoopStats();
        d.mbxSends = async.sends;
        d.mbxSendErrors = async.send_errors;
        d.mbxCollects = async.collects;
        d.mbxCollectErrors = async.collect_errors;
        d.maxSendLatencyNs = async.max_send_latency_ns;

#if TETHER_ENABLE_ETHERCAT_STATS
        const auto link = master_.getStats();
        d.txRetries = link.tx_retry_count;
        d.txFailures = link.tx_fail_count;
        d.rxFrames = link.rx_frame_count;
#endif

        d.lastWkc = master_.lastWkc();
        d.slavesDetected = master_.getDiscoveredSlaveCount();
        uint16_t operational = 0;
        for (uint16_t i = 0; i < d.slavesDetected; ++i) {
            if (master_.slave(i).ALState() == EtherCAT::SlaveState::OP) ++operational;
        }
        d.slavesOperational = operational;
        return d;
    }

private:
    EtherCAT::Master& master_;
};

} // namespace tether::io::machine
