/**
 * @file CiA402Exposer.hpp
 * @brief Exposes CiA 402 drive parameters and signals to the IO protocol.
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#pragma once

#include "tether/io/ParameterExposer.hpp"
#include "tether/profiles/cia402/CiA402Drive.hpp"

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
 *  - `controlword` (U16): CiA 402 controlword (read/write).
 *  - `sdo_timeout` (U32): SDO timeout in ms (write-only by effect).
 */
class CiA402Exposer : public IParameterExposer {
public:
    /**
     * @param drive      Reference to the CiA402Drive.
     * @param driveIndex Index for naming, e.g. "drive0", "drive1".
     */
    explicit CiA402Exposer(tether::cia402::CiA402Drive& drive,
                           uint16_t driveIndex = 0)
        : drive_(drive), driveIndex_(driveIndex) {}

    const char* moduleName() const override { return "cia402"; }

    void expose(Registry& registry, uint64_t idBase) override {
        using namespace tether::cia402;
        std::string prefix = "drive" + std::to_string(driveIndex_);
        std::string group  = "cia402." + prefix;

        // Offset per drive to avoid ID collision
        uint32_t off = static_cast<uint32_t>(driveIndex_) * 0x100;

        // -- Signals --
        registry.addSignal({
            makeId(idBase, off + 0x01), prefix + ".statusword",
            "CiA 402 statusword", group, ValueType::U16,
            [this](void* d) {
                uint16_t v = drive_.getStatusword();
                std::memcpy(d, &v, sizeof(v));
            }
        });

        registry.addSignal({
            makeId(idBase, off + 0x02), prefix + ".drive_state",
            "CiA 402 drive state enum", group, ValueType::U8,
            [this](void* d) {
                uint8_t v = static_cast<uint8_t>(drive_.getDriveState());
                std::memcpy(d, &v, 1);
            }
        });

        registry.addSignal({
            makeId(idBase, off + 0x03), prefix + ".ec_state",
            "EtherCAT AL state", group, ValueType::U8,
            [this](void* d) {
                uint8_t v = static_cast<uint8_t>(drive_.getECState());
                std::memcpy(d, &v, 1);
            }
        });

        registry.addSignal({
            makeId(idBase, off + 0x04), prefix + ".is_enabled",
            "Whether the drive is enabled", group, ValueType::Bool,
            [this](void* d) {
                uint8_t v = drive_.isEnabled() ? 1 : 0;
                std::memcpy(d, &v, 1);
            }
        });

        registry.addSignal({
            makeId(idBase, off + 0x05), prefix + ".is_faulted",
            "Whether the drive is in fault state", group, ValueType::Bool,
            [this](void* d) {
                uint8_t v = drive_.isFaulted() ? 1 : 0;
                std::memcpy(d, &v, 1);
            }
        });

        registry.addSignal({
            makeId(idBase, off + 0x06), prefix + ".target_reached",
            "Whether the target position/velocity is reached", group, ValueType::Bool,
            [this](void* d) {
                uint8_t v = drive_.isTargetReached() ? 1 : 0;
                std::memcpy(d, &v, 1);
            }
        });

        registry.addSignal({
            makeId(idBase, off + 0x07), prefix + ".operating_mode",
            "Current mode of operation (CSP=8, CSV=9, CST=10)", group, ValueType::I8,
            [this](void* d) {
                int8_t v = drive_.getOperatingMode();
                std::memcpy(d, &v, 1);
            }
        });

        registry.addSignal({
            makeId(idBase, off + 0x08), prefix + ".slave_index",
            "EtherCAT slave index", group, ValueType::U16,
            [this](void* d) {
                uint16_t v = drive_.slaveIndex();
                std::memcpy(d, &v, sizeof(v));
            }
        });

        registry.addSignal({
            makeId(idBase, off + 0x09), prefix + ".pdo_registered",
            "Whether PDO buffers are registered", group, ValueType::Bool,
            [this](void* d) {
                uint8_t v = drive_.isPDORegistered() ? 1 : 0;
                std::memcpy(d, &v, 1);
            }
        });

        registry.addSignal({
            makeId(idBase, off + 0x0A), prefix + ".rxpdo_size",
            "RxPDO buffer size in bytes", group, ValueType::U16,
            [this](void* d) {
                uint16_t v = drive_.getRxPDOSize();
                std::memcpy(d, &v, sizeof(v));
            }
        });

        registry.addSignal({
            makeId(idBase, off + 0x0B), prefix + ".txpdo_size",
            "TxPDO buffer size in bytes", group, ValueType::U16,
            [this](void* d) {
                uint16_t v = drive_.getTxPDOSize();
                std::memcpy(d, &v, sizeof(v));
            }
        });

        // -- Parameters --
        registry.addParam({
            makeId(idBase, off + 0x81), prefix + ".controlword",
            "CiA 402 controlword", group, ValueType::U16,
            [this](void* d) {
                uint16_t v = drive_.getControlword();
                std::memcpy(d, &v, sizeof(v));
            },
            [this](const void* s) {
                uint16_t v;
                std::memcpy(&v, s, sizeof(v));
                drive_.setControlword(v);
            }
        });
    }

private:
    tether::cia402::CiA402Drive& drive_;
    uint16_t driveIndex_;
};

}}} // namespace tether::io::exposers
