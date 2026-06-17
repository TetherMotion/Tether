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
    auto result = sdo.readU32(slave_idx, AS715NDevice::kManufacturerFaultIndex, 0x00, {.timeout_ms = 3000});
    if (!result.has_value()) {
        TETHER_LOGW(TAG, "Failed to read manufacturer fault (0x%04X) from slave %u",
                    AS715NDevice::kManufacturerFaultIndex, slave_idx);
        return AS715NManufacturerFault203F{};
    }
    return AS715NManufacturerFault203F::fromU32(result.value());
}

uint16_t AS715NFaultHandler::readCiA402Error(EtherCAT::CoE::CoEManager& sdo, uint16_t slave_idx) {
    auto result = sdo.readU16(slave_idx, AS715NDevice::kCiA402ErrorIndex, 0x00, {.timeout_ms = 3000});
    if (!result.has_value()) {
        TETHER_LOGW(TAG, "Failed to read CiA 402 error code (0x%04X) from slave %u",
                    AS715NDevice::kCiA402ErrorIndex, slave_idx);
        return 0;
    }
    return result.value();
}

} // namespace Drives
} // namespace EtherCAT
