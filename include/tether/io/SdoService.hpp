#pragma once

/**
 * @file SdoService.hpp
 * @brief Pluggable backends for the optional machine.sdo.* surface.
 *
 * MachineService only sees IMachineSdoAccess (defined in MachineService.hpp
 * alongside the other service interfaces). This header provides the
 * device-independent in-memory backend; EtherCatSdoAdapter.hpp provides the
 * real EtherCAT::Master backend. Applications without SDO simply do not
 * attach a backend and the wire functions are never registered — including
 * SdoService.hpp has no effect on the catalog until it is plugged in.
 */

#include "tether/io/MachineService.hpp"

#include <cstdint>
#include <map>
#include <vector>

namespace tether::io::machine {

/**
 * In-memory OD backend used by the simulated stack and tests. Each slave
 * gets an independent byte map; `listObjects` returns a caller-supplied
 * static object table so the inspector stays meaningful without hardware.
 */
class SimulatedSdoAccess final : public IMachineSdoAccess {
public:
    explicit SimulatedSdoAccess(std::vector<SdoObjectInfoV1> objectTable = {})
        : objectTable_(std::move(objectTable)) {}

    std::vector<SdoObjectInfoV1> listObjects(uint16_t slave) override {
        (void)slave;
        return objectTable_;
    }

    SdoTransferResult read(uint16_t slave, uint16_t index, uint8_t subindex,
                           size_t maxBytes) override {
        SdoTransferResult result;
        const auto it = objects_[slave].find(key(index, subindex));
        if (it == objects_[slave].end()) {
            result.abortCode = 0x06020000;  // object does not exist
            result.error = "Object does not exist in the simulated dictionary";
            return result;
        }
        result.ok = true;
        result.data.assign(it->second.begin(),
                           it->second.begin() +
                               static_cast<ptrdiff_t>(std::min(it->second.size(), maxBytes)));
        return result;
    }

    SdoTransferResult write(uint16_t slave, uint16_t index, uint8_t subindex,
                            const std::vector<uint8_t>& data) override {
        SdoTransferResult result;
        objects_[slave][key(index, subindex)] = data;
        result.ok = true;
        return result;
    }

    void seed(uint16_t slave, uint16_t index, uint8_t subindex,
              std::vector<uint8_t> value) {
        objects_[slave][key(index, subindex)] = std::move(value);
    }

private:
    static uint32_t key(uint16_t index, uint8_t subindex) {
        return (static_cast<uint32_t>(index) << 8U) | subindex;
    }

    std::vector<SdoObjectInfoV1> objectTable_;
    std::map<uint16_t, std::map<uint32_t, std::vector<uint8_t>>> objects_;
};

} // namespace tether::io::machine
