/**
 * @file EtherCATExposer.hpp
 * @brief Exposes EtherCAT master/slave state to the IO protocol.
 *
 * Signals:
 *  - `discovered_slave_count`, `is_running` (master level)
 *  - `slave<N>.ec_state`, `slave<N>.has_fault` (per slave)
 *  - `cyclic.stats` — CyclicExecutive::Stats as a self-describing Binary
 *    snapshot plus scalar field signals (cycle count, exchange/task
 *    errors, missed deadlines, max work time, wake-up jitter, DC sync
 *    counters and jitter).
 *  - `slave<N>.recovery` — SlaveSupervisor per-slave recovery snapshot
 *    (state, suspended, recovering, attempt count).
 *
 * Parameters:
 *  - `enable_mailbox_fallback` (Bool, read/write)
 *
 * `ec_state` performs a bounded AL-status register read (~200 µs timeout)
 * on the calling session thread; the remaining entries are lock-free
 * reads of master-internal state.
 *
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#pragma once

#include "tether/io/ParameterExposer.hpp"
#include "tether/io/exposers/SnapshotExposer.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/CyclicExecutive.hpp"
#include "tether/ethercat/SlaveSupervisor.hpp"
#include <cstring>

namespace tether { namespace io { namespace exposers {

class EtherCATExposer : public IParameterExposer {
public:
    /**
     * @param master   Reference to the Master instance.  Must outlive the
     *                 registered entries.
     * @param numSlaves  Number of slaves to expose (0 = master only).
     */
    explicit EtherCATExposer(EtherCAT::Master& master,
                             uint16_t numSlaves = 0)
        : master_(master), numSlaves_(numSlaves), snapshots_("ethercat") {}

    const char* moduleName() const override { return "ethercat"; }

    void expose(Registry& registry, uint64_t idBase) override {
        using namespace EtherCAT;

        // -- Master-level signals --
        registry.addSignal({
            makeId(idBase, 0x0001),
            "discovered_slave_count",
            "Number of discovered EtherCAT slaves",
            "ethercat.master",
            ValueType::U16,
            [this](void* d) {
                uint16_t v = master_.getDiscoveredSlaveCount();
                std::memcpy(d, &v, sizeof(v));
            }
        });

        registry.addSignal({
            makeId(idBase, 0x0002),
            "is_running",
            "Whether the EtherCAT master is running",
            "ethercat.master",
            ValueType::Bool,
            [this](void* d) {
                uint8_t v = master_.isRunning() ? 1 : 0;
                std::memcpy(d, &v, 1);
            }
        });

        // -- Master-level parameters --
        registry.addParam({
            makeId(idBase, 0x0101),
            "enable_mailbox_fallback",
            "Enable mailbox fallback mode on SDO errors",
            "ethercat.master",
            ValueType::Bool,
            [this](void* d) {
                uint8_t v = master_.isMailboxFallbackEnabled() ? 1 : 0;
                std::memcpy(d, &v, 1);
            },
            [this](const void* s) {
                uint8_t v;
                std::memcpy(&v, s, 1);
                master_.setEnableMailboxFallback(v != 0);
            }
        });

        // -- Per-slave signals --
        for (uint16_t i = 0; i < numSlaves_; ++i) {
            std::string prefix = "slave" + std::to_string(i);
            std::string group  = "ethercat.slave" + std::to_string(i);

            // ec_state performs a wire read (AL status register, ~200 µs
            // timeout) — keep it out of stream collect plans.
            SignalEntry ecState;
            ecState.id          = makeId(idBase, 0x1000 + i * 0x10 + 0x01);
            ecState.name        = prefix + ".ec_state";
            ecState.description =
                "EtherCAT AL state of slave " + std::to_string(i);
            ecState.group       = group;
            ecState.valueType   = ValueType::U8;
            ecState.extraFlags  = EntryFlags::NoStream;
            ecState.readFn      = [this, i](void* d) {
                uint8_t state = 0;
                master_.readSlaveApplicationLayerState(i, state);
                std::memcpy(d, &state, 1);
            };
            registry.addSignal(std::move(ecState));

            registry.addSignal({
                makeId(idBase, 0x1000 + i * 0x10 + 0x02),
                prefix + ".has_fault",
                "Whether a fault was diagnosed on slave " + std::to_string(i),
                group,
                ValueType::Bool,
                [this, i](void* d) {
                    uint8_t v = master_.wasFaultDiagnosed(i) ? 1 : 0;
                    std::memcpy(d, &v, 1);
                }
            });
        }

        // -- Snapshot structs (self-describing Binary + scalar fields) --
        buildSnapshots();
        snapshots_.expose(registry, idBase);
    }

private:
    /// Packed per-slave recovery snapshot filled from SlaveSupervisor.
    struct SlaveRecoverySnapshot {
        uint8_t  state;         ///< SlaveRecoveryState enum value
        uint8_t  suspended;     ///< PDO data suspended for this slave
        uint8_t  recovering;    ///< Recovery currently in progress
        uint8_t  _reserved;
        uint32_t attempt_count; ///< Recovery attempts so far
    };

    void buildSnapshots() {
        using Stats = EtherCAT::CyclicExecutive::Stats;
        using JS    = EtherCAT::JitterStats;
        constexpr uint16_t J  = offsetof(Stats, jitter);
        constexpr uint16_t DJ = offsetof(Stats, dc_jitter);

        snapshots_.addSnapshot(
            0x2000, "cyclic.stats",
            "Cyclic executive statistics (cycle counters, timing, jitter, DC sync)",
            "ethercat.cyclic",
            {
                {"cycle_count",        ValueType::U64, offsetof(Stats, cycle_count),        8, "cycles"},
                {"exchange_errors",    ValueType::U64, offsetof(Stats, exchange_errors),    8, "cycles"},
                {"task_errors",        ValueType::U64, offsetof(Stats, task_errors),        8, "cycles"},
                {"missed_deadlines",   ValueType::U64, offsetof(Stats, missed_deadlines),   8, "cycles"},
                {"max_cycle_work_us",  ValueType::U32, offsetof(Stats, max_cycle_work_us),  4, "us"},
                {"deadline_active",    ValueType::Bool,offsetof(Stats, deadline_active),    1},
                {"dl_runtime_ns",      ValueType::U64, offsetof(Stats, dl_runtime_ns),      8, "ns"},
                {"jitter.cycle_count", ValueType::U64, J + offsetof(JS, cycle_count),     8, "cycles"},
                {"jitter.max_us",      ValueType::U32, J + offsetof(JS, max_jitter_us),   4, "us"},
                {"jitter.avg_us",      ValueType::U32, J + offsetof(JS, avg_jitter_us),   4, "us"},
                {"jitter.warnings",    ValueType::U64, J + offsetof(JS, warning_count),   8, "cycles"},
                {"jitter.criticals",   ValueType::U64, J + offsetof(JS, critical_count),  8, "cycles"},
                {"jitter.last_period_us", ValueType::U32, J + offsetof(JS, last_period_us), 4, "us"},
                {"jitter.realtime_ok", ValueType::Bool,J + offsetof(JS, realtime_ok),     1},
                {"dc_sync_count",      ValueType::U64, offsetof(Stats, dc_sync_count),      8, "cycles"},
                {"dc_sync_errors",     ValueType::U64, offsetof(Stats, dc_sync_errors),     8, "cycles"},
                {"dc_jitter.cycle_count", ValueType::U64, DJ + offsetof(JS, cycle_count), 8, "cycles"},
                {"dc_jitter.max_us",   ValueType::U32, DJ + offsetof(JS, max_jitter_us),  4, "us"},
                {"dc_jitter.avg_us",   ValueType::U32, DJ + offsetof(JS, avg_jitter_us),  4, "us"},
                {"dc_jitter.warnings", ValueType::U64, DJ + offsetof(JS, warning_count),  8, "cycles"},
                {"dc_jitter.criticals",ValueType::U64, DJ + offsetof(JS, critical_count), 8, "cycles"},
                {"dc_jitter.last_period_us", ValueType::U32, DJ + offsetof(JS, last_period_us), 4, "us"},
                {"dc_jitter.realtime_ok", ValueType::Bool, DJ + offsetof(JS, realtime_ok),1},
            },
            sizeof(Stats),
            [this](void* d) {
                Stats s = master_.getCyclicLoopStats();
                std::memcpy(d, &s, sizeof(s));
            });

        for (uint16_t i = 0; i < numSlaves_; ++i) {
            const std::string group = "ethercat.slave" + std::to_string(i);
            snapshots_.addSnapshot(
                0x2100 + i * 0x20,
                "slave" + std::to_string(i) + ".recovery",
                "SlaveSupervisor recovery state for slave " + std::to_string(i),
                group,
                {
                    {"state",         ValueType::U8,  offsetof(SlaveRecoverySnapshot, state),         1, "",
                     "0=Normal,1=Critical,2=Recovering,3=Recovered,4=Failed"},
                    {"suspended",     ValueType::Bool,offsetof(SlaveRecoverySnapshot, suspended),     1},
                    {"recovering",    ValueType::Bool,offsetof(SlaveRecoverySnapshot, recovering),    1},
                    {"attempt_count", ValueType::U32, offsetof(SlaveRecoverySnapshot, attempt_count), 4, "attempts"},
                },
                sizeof(SlaveRecoverySnapshot),
                [this, i](void* d) {
                    auto& sup = master_.slaveSupervisor();
                    SlaveRecoverySnapshot s{};
                    s.state         = static_cast<uint8_t>(sup.slaveState(i));
                    s.suspended     = sup.isSlaveSuspended(i) ? 1 : 0;
                    s.recovering    = sup.isSlaveRecovering(i) ? 1 : 0;
                    s.attempt_count = static_cast<uint32_t>(sup.slaveAttemptCount(i));
                    std::memcpy(d, &s, sizeof(s));
                });
        }
    }

    EtherCAT::Master& master_;
    uint16_t          numSlaves_;
    SnapshotExposer   snapshots_;
};

}}} // namespace tether::io::exposers
