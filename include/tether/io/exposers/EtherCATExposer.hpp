/**
 * @file EtherCATExposer.hpp
 * @brief Exposes Master parameters and signals to the IO protocol.
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#pragma once

#include "tether/io/ParameterExposer.hpp"
#include "tether/ethercat/Master.hpp"

namespace tether { namespace io { namespace exposers {

/**
 * @class EtherCATExposer
 * @brief Exposes EtherCAT master's runtime info, per-slave state, DC, PDO stats.
 *
 * ## Signals
 *  - `discovered_slave_count` (U16): Total slave count from scanning.
 *  - `is_running` (Bool): Whether the master is running.
 *  - `slave[N].ec_state` (U8): AL state of slave N.
 *  - `slave[N].has_fault` (Bool): Whether slave N had a fault diagnosed.
 *
 * ## Parameters
 *  - `enable_mailbox_fallback` (Bool): Toggle mailbox fallback mode.
 */
class EtherCATExposer : public IParameterExposer {
public:
    /**
     * @param master   Reference to the Master instance.
     * @param numSlaves  Number of slaves to expose (0 = just master-level entries).
     */
    explicit EtherCATExposer(tether::ethercat::Master& master,
                             uint16_t numSlaves = 0)
        : master_(master), numSlaves_(numSlaves) {}

    const char* moduleName() const override { return "ethercat"; }

    void expose(Registry& registry, uint64_t idBase) override {
        using namespace tether::ethercat;

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

            registry.addSignal({
                makeId(idBase, 0x1000 + i * 0x10 + 0x01),
                prefix + ".ec_state",
                "EtherCAT AL state of slave " + std::to_string(i),
                group,
                ValueType::U8,
                [this, i](void* d) {
                    uint8_t state = 0;
                    master_.readSlaveApplicationLayerState(i, state);
                    std::memcpy(d, &state, 1);
                }
            });

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
    }

private:
    tether::ethercat::Master& master_;
    uint16_t numSlaves_;
};

}}} // namespace tether::io::exposers
