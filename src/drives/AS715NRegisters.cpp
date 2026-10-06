#include "tether/drives/AS715N.hpp"
#include "tether/drives/AS715NRegisters.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/platform/Platform.hpp"

static const char* TAG = "AS715N";

namespace EtherCAT {
namespace Drives {

uint16_t AS715NFaultHandler::readManufacturerFault(EtherCAT::CoE::CoEManager& sdo, uint16_t slave_idx) {
    const auto ext = readManufacturerFaultExtended(sdo, slave_idx);
    return ext.external_code;
}

AS715NManufacturerFault203F AS715NFaultHandler::readManufacturerFaultExtended(EtherCAT::CoE::CoEManager& sdo, uint16_t slave_idx) {
    auto result = sdo.readU32(AS715NDevice::kManufacturerFaultIndex, 0x00, {.timeout_ms = 3000});
    if (!result.has_value()) {
        TETHER_LOGW(TAG, "Failed to read manufacturer fault (0x{:04X}) from slave {}",
                    AS715NDevice::kManufacturerFaultIndex, slave_idx);
        return AS715NManufacturerFault203F{};
    }
    return AS715NManufacturerFault203F::fromU32(result.value());
}

bool AS715NConfig::configureBleederResistor(
    EtherCAT::CoE::CoEManager& sdo,
    Registers::AS715N::C00::BleederResistorSelectionOptions selection,
    uint16_t power_w, uint16_t resistance_ohm,
    uint32_t timeout_ms) {
    namespace C00 = Registers::AS715N::C00;
    bool ok = true;
    const struct { uint8_t sub; uint16_t value; } params[] = {
        {C00::BleederResistorSelection.subindex,
         static_cast<uint16_t>(selection)},
        {C00::BleederResistorPower.subindex,      power_w},
        {C00::BleederResistorResistance.subindex, resistance_ohm},
    };
    for (const auto& p : params) {
        if (!sdo.writeU16(C00::C00ObjectIndex, p.sub, p.value,
                          {.timeout_ms = timeout_ms}).has_value()) {
            TETHER_LOGW(TAG, "{}: failed to write 0x{:04X}:{:02X}={}",
                        sdo.logPrefix().c_str(), C00::C00ObjectIndex,
                        p.sub, p.value);
            ok = false;
        }
    }
    return ok;
}

uint16_t AS715NFaultHandler::readCiA402Error(EtherCAT::CoE::CoEManager& sdo, uint16_t slave_idx) {
    auto result = sdo.readU16(AS715NDevice::kCiA402ErrorIndex, 0x00, {.timeout_ms = 3000});
    if (!result.has_value()) {
        TETHER_LOGW(TAG, "Failed to read CiA 402 error code (0x{:04X}) from slave {}",
                    AS715NDevice::kCiA402ErrorIndex, slave_idx);
        return 0;
    }
    return result.value();
}

} // namespace Drives
} // namespace EtherCAT
