/**
 * @file CiA402Exposer.hpp
 * @brief Exposes CiA 402 drive parameters and signals to the IO protocol.
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#pragma once

#include "tether/io/ParameterExposer.hpp"
#include "tether/profiles/cia402/CiA402Drive.hpp"
#include <cstring>
#include <functional>

namespace tether { namespace io { namespace exposers {

/**
 * @class CiA402Exposer
 * @brief Exposes a single CiA 402 drive's state machine, control/status words, PDO info.
 *
 * ## Signals
 *  - `statusword` (U16): CiA 402 statusword.
 *  - `drive_state` (U8): DriveState enum value.
 *  - `ec_state` (U8): EtherCAT AL state.
 *  - `is_enabled` (Bool), `is_faulted` (Bool), `target_reached` (Bool).
 *  - `operating_mode` (I8): Current mode of operation.
 *  - `rxpdo_size` (U16), `txpdo_size` (U16): PDO buffer sizes.
 *  - `rxpdo_index` (U16), `txpdo_index` (U16): PDO SM indices.
 *
 * ## Parameters
 *  - `controlword` (U16): CiA 402 controlword (read/write) — only when the
 *    `exposeControlword` ctor flag is set.
 *
 * ## Blocking reads
 * Most reads are cheap PDO-buffer decodes while the drive is in OP, but
 * several fall back to wire transactions when the drive is not in OP:
 * `readStatusword()` issues an SDO upload (~ms timeout) and `getECState()`
 * issues an AL-status register read (~200 µs).  These run on the calling
 * IO session thread — do NOT wrap the WithDrive accessor in a mutex that
 * a realtime thread also takes, or an SDO round-trip can stall the RT loop.
 */
class CiA402Exposer : public IParameterExposer {
public:
    /**
     * Serialized drive accessor for the recovery-tolerant ctor.  Must invoke
     * `fn(drive)` while the caller's synchronization guarantees the drive
     * stays alive, and return false when no drive is currently present.
     */
    using WithDrive =
        std::function<bool(const std::function<void(EtherCAT::CiA402Drive&)>&)>;

    /**
     * @param drive      Reference to the CiA402Drive (must outlive the entries).
     * @param driveIndex Index for naming, e.g. "drive0", "drive1".
     */
    explicit CiA402Exposer(EtherCAT::CiA402Drive& drive,
                           uint16_t driveIndex = 0)
        : driveIndex_(driveIndex), exposeControlword_(true),
          withDrive_([&drive](const std::function<void(EtherCAT::CiA402Drive&)>& fn) {
              fn(drive);
              return true;
          }) {}

    /**
     * Recovery-tolerant variant for adapters that destroy and recreate the
     * CiA402Drive during EtherCAT recovery.
     *
     * @param withDrive          Serializing accessor (see WithDrive).
     * @param driveIndex         Index for naming.
     * @param exposeControlword  When false the writable `controlword`
     *                           parameter is not registered — use this when
     *                           a cyclic loop owns the PDO controlword.
     */
    CiA402Exposer(WithDrive withDrive, uint16_t driveIndex,
                  bool exposeControlword)
        : driveIndex_(driveIndex), exposeControlword_(exposeControlword),
          withDrive_(std::move(withDrive)) {}

    const char* moduleName() const override { return "cia402"; }

    void expose(Registry& registry, uint64_t idBase) override {
        using namespace EtherCAT;
        std::string prefix = "drive" + std::to_string(driveIndex_);
        std::string group  = "cia402." + prefix;

        // Offset per drive to avoid ID collision
        uint32_t off = static_cast<uint32_t>(driveIndex_) * 0x100;

        // -- Signals --
        registry.addSignal({
            makeId(idBase, off + 0x01), prefix + ".statusword",
            "CiA 402 statusword", group, ValueType::U16,
            [this](void* d) {
                uint16_t v = 0;
                withDrive_([&](CiA402Drive& drv) { v = drv.getStatusword(); });
                std::memcpy(d, &v, sizeof(v));
            }
        });

        registry.addSignal({
            makeId(idBase, off + 0x02), prefix + ".drive_state",
            "CiA 402 drive state enum", group, ValueType::U8,
            [this](void* d) {
                uint8_t v = 0;
                withDrive_([&](CiA402Drive& drv) {
                    v = static_cast<uint8_t>(drv.getDriveState());
                });
                std::memcpy(d, &v, 1);
            }
        });

        registry.addSignal({
            makeId(idBase, off + 0x03), prefix + ".ec_state",
            "EtherCAT AL state", group, ValueType::U8,
            [this](void* d) {
                uint8_t v = 0;
                withDrive_([&](CiA402Drive& drv) {
                    v = static_cast<uint8_t>(drv.getECState());
                });
                std::memcpy(d, &v, 1);
            }
        });

        registry.addSignal({
            makeId(idBase, off + 0x04), prefix + ".is_enabled",
            "Whether the drive is enabled", group, ValueType::Bool,
            [this](void* d) {
                uint8_t v = 0;
                withDrive_([&](CiA402Drive& drv) { v = drv.isEnabled() ? 1 : 0; });
                std::memcpy(d, &v, 1);
            }
        });

        registry.addSignal({
            makeId(idBase, off + 0x05), prefix + ".is_faulted",
            "Whether the drive is in fault state", group, ValueType::Bool,
            [this](void* d) {
                uint8_t v = 0;
                withDrive_([&](CiA402Drive& drv) { v = drv.isFaulted() ? 1 : 0; });
                std::memcpy(d, &v, 1);
            }
        });

        registry.addSignal({
            makeId(idBase, off + 0x06), prefix + ".target_reached",
            "Whether the target position/velocity is reached", group, ValueType::Bool,
            [this](void* d) {
                uint8_t v = 0;
                withDrive_([&](CiA402Drive& drv) {
                    v = drv.isTargetReached() ? 1 : 0;
                });
                std::memcpy(d, &v, 1);
            }
        });

        registry.addSignal({
            makeId(idBase, off + 0x07), prefix + ".operating_mode",
            "Current mode of operation (CSP=8, CSV=9, CST=10) — SDO read",
            group, ValueType::I8,
            [this](void* d) {
                int8_t v = 0;
                withDrive_([&](CiA402Drive& drv) { v = drv.getOperatingMode(); });
                std::memcpy(d, &v, 1);
            }
        });

        registry.addSignal({
            makeId(idBase, off + 0x08), prefix + ".slave_index",
            "EtherCAT slave index", group, ValueType::U16,
            [this](void* d) {
                uint16_t v = 0;
                withDrive_([&](CiA402Drive& drv) { v = drv.slaveIndex(); });
                std::memcpy(d, &v, sizeof(v));
            }
        });

        registry.addSignal({
            makeId(idBase, off + 0x09), prefix + ".pdo_registered",
            "Whether PDO buffers are registered", group, ValueType::Bool,
            [this](void* d) {
                uint8_t v = 0;
                withDrive_([&](CiA402Drive& drv) {
                    v = drv.isPDORegistered() ? 1 : 0;
                });
                std::memcpy(d, &v, 1);
            }
        });

        registry.addSignal({
            makeId(idBase, off + 0x0A), prefix + ".rxpdo_size",
            "RxPDO buffer size in bytes", group, ValueType::U16,
            [this](void* d) {
                uint16_t v = 0;
                withDrive_([&](CiA402Drive& drv) { v = drv.getRxPDOSize(); });
                std::memcpy(d, &v, sizeof(v));
            }
        });

        registry.addSignal({
            makeId(idBase, off + 0x0B), prefix + ".txpdo_size",
            "TxPDO buffer size in bytes", group, ValueType::U16,
            [this](void* d) {
                uint16_t v = 0;
                withDrive_([&](CiA402Drive& drv) { v = drv.getTxPDOSize(); });
                std::memcpy(d, &v, sizeof(v));
            }
        });

        // -- Parameters --
        if (exposeControlword_) {
            registry.addParam({
                makeId(idBase, off + 0x81), prefix + ".controlword",
                "CiA 402 controlword", group, ValueType::U16,
                [this](void* d) {
                    uint16_t v = 0;
                    withDrive_([&](CiA402Drive& drv) { v = drv.getControlword(); });
                    std::memcpy(d, &v, sizeof(v));
                },
                [this](const void* s) {
                    uint16_t v;
                    std::memcpy(&v, s, sizeof(v));
                    withDrive_([&](CiA402Drive& drv) { drv.setControlword(v); });
                }
            });
        }
    }

private:
    uint16_t  driveIndex_;
    bool      exposeControlword_;
    WithDrive withDrive_;
};

}}} // namespace tether::io::exposers
