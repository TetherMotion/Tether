#include "tether/drives/AS715N.hpp"
#include "tether/drives/AS715NRegisters.hpp"
#include "tether/ethercat/EtherCATSDO.hpp"
#include "tether/platform/Platform.hpp"

static const char* TAG = "AS715N";

namespace EtherCAT {
namespace Drives {

uint16_t AS715NFaultHandler::readManufacturerFault(EtherCAT::SDO::SDOManager& sdo, uint16_t slave_idx) {
    const auto ext = readManufacturerFaultExtended(sdo, slave_idx);
    return ext.external_code;
}

AS715NManufacturerFault203F AS715NFaultHandler::readManufacturerFaultExtended(EtherCAT::SDO::SDOManager& sdo, uint16_t slave_idx) {
    uint32_t raw = 0;
    if (!sdo.readU32(slave_idx, AS715NDevice::kManufacturerFaultIndex, 0x00, raw, 3000)) {
        TETHER_LOGW(TAG, "Failed to read manufacturer fault (0x%04X) from slave %u",
                    AS715NDevice::kManufacturerFaultIndex, slave_idx);
        return AS715NManufacturerFault203F{};
    }
    return AS715NManufacturerFault203F::fromU32(raw);
}

uint16_t AS715NFaultHandler::readCiA402Error(EtherCAT::SDO::SDOManager& sdo, uint16_t slave_idx) {
    uint16_t value = 0;
    if (!sdo.readU16(slave_idx, AS715NDevice::kCiA402ErrorIndex, 0x00, value, 3000)) {
        TETHER_LOGW(TAG, "Failed to read CiA 402 error code (0x%04X) from slave %u",
                    AS715NDevice::kCiA402ErrorIndex, slave_idx);
        return 0;
    }
    return value;
}

} // namespace Drives
} // namespace EtherCAT
