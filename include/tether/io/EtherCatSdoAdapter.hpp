#pragma once

/**
 * @file EtherCatSdoAdapter.hpp
 * @brief IMachineSdoAccess backend over a running EtherCAT::Master.
 *
 * Plugging the optional machine.sdo.* surface into a real master is:
 *
 * @code
 *   EtherCatSdoAccess sdoAccess(master);                    // CiA402 table by default
 *   MachineSdoService sdoService(sdoAccess, &eventJournal); // audited writes
 *   machineService.setSdoService(&sdoService);              // before install()
 * @endcode
 *
 * Dependency direction: tether/io depends on tether/ethercat here, never the
 * reverse — CiA402/DS402 drivers do not pull in the IO framework.
 *
 * Threading: sdoRead/sdoWrite perform synchronous mailbox round-trips on the
 * calling IO session thread (multi-ms). Do not hold a mutex shared with the
 * cyclic RT loop while the web surface is attached, or an SDO transfer can
 * stall the PDO exchange.
 */

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/ObjectDictionary.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/io/MachineService.hpp"

#include <functional>
#include <limits>
#include <vector>

namespace tether::io::machine {

/// Supplies the object list shown by machine.sdo.list for a slave. Returning
/// an empty vector is valid — explicit-index transfers still work.
using SdoObjectTableProvider =
    std::function<std::vector<EtherCAT::ObjectDictionary::ObjectDictionaryEntry>(
        uint16_t slave)>;

/// Common CiA 301 + CiA 402 objects, usable as the default listing for DS402
/// drives. Device vendors may supply richer tables via the provider argument.
inline std::vector<EtherCAT::ObjectDictionary::ObjectDictionaryEntry>
cia402SdoObjectTable(uint16_t /*slave*/) {
    using namespace EtherCAT::ObjectDictionary;
    using D = ObjectDictionaryDataType;
    using M = ModificationMode;
    return {
        {0x1000, 0, "Device type",            D::Unsigned32, 0, Unit_None, nullptr, 0, 0, M::ReadOnly, EffectiveTime::Immediately},
        {0x1008, 0, "Device name",            D::VisibleString, 0, Unit_None, nullptr, 0, 0, M::ReadOnly, EffectiveTime::Immediately},
        {0x1018, 1, "Vendor ID",              D::Unsigned32, 0, Unit_None, nullptr, 0, 0, M::ReadOnly, EffectiveTime::Immediately},
        {0x1018, 2, "Product code",           D::Unsigned32, 0, Unit_None, nullptr, 0, 0, M::ReadOnly, EffectiveTime::Immediately},
        {0x6040, 0, "Controlword",            D::Unsigned16, 0, Unit_None, nullptr, 0, 0xFFFF, M::DuringOperation, EffectiveTime::Immediately},
        {0x6041, 0, "Statusword",             D::Unsigned16, 0, Unit_None, nullptr, 0, 0, M::ReadOnly, EffectiveTime::Immediately},
        {0x6060, 0, "Modes of operation",     D::Integer8, 0, Unit_None, nullptr, -128, 127, M::DuringOperation, EffectiveTime::Immediately},
        {0x6061, 0, "Modes of operation display", D::Integer8, 0, Unit_None, nullptr, 0, 0, M::ReadOnly, EffectiveTime::Immediately},
        {0x6064, 0, "Position actual value",  D::Integer32, 0, Unit_Inc, nullptr, 0, 0, M::ReadOnly, EffectiveTime::Immediately},
        {0x606C, 0, "Velocity actual value",  D::Integer32, 0, Unit_Inc, nullptr, 0, 0, M::ReadOnly, EffectiveTime::Immediately},
        {0x6078, 0, "Current actual value",   D::Integer16, 0, Unit_None, nullptr, 0, 0, M::ReadOnly, EffectiveTime::Immediately},
        {0x607A, 0, "Target position",        D::Integer32, 0, Unit_Inc, nullptr, INT32_MIN, INT32_MAX, M::DuringOperation, EffectiveTime::Immediately},
        {0x6081, 0, "Profile velocity",       D::Unsigned32, 0, Unit_Inc, nullptr, 0, INT32_MAX, M::DuringOperation, EffectiveTime::Immediately},
        {0x6083, 0, "Profile acceleration",   D::Unsigned32, 0, Unit_None, nullptr, 0, INT32_MAX, M::DuringOperation, EffectiveTime::Immediately},
        {0x6084, 0, "Profile deceleration",   D::Unsigned32, 0, Unit_None, nullptr, 0, INT32_MAX, M::DuringOperation, EffectiveTime::Immediately},
        {0x60FF, 0, "Target velocity",        D::Integer32, 0, Unit_Inc, nullptr, INT32_MIN, INT32_MAX, M::DuringOperation, EffectiveTime::Immediately},
        {0x6502, 0, "Supported drive modes",  D::Unsigned32, 0, Unit_None, nullptr, 0, 0, M::ReadOnly, EffectiveTime::Immediately},
        {0x1C12, 0, "RxPDO assignment",       D::Unsigned8, 0, Unit_None, nullptr, 0, 0, M::ReadOnly, EffectiveTime::Immediately},
        {0x1C13, 0, "TxPDO assignment",       D::Unsigned8, 0, Unit_None, nullptr, 0, 0, M::ReadOnly, EffectiveTime::Immediately},
    };
}

class EtherCatSdoAccess final : public IMachineSdoAccess {
public:
    /// @param master   Running EtherCAT master (must outlive this adapter).
    /// @param objects  Known-OD provider for machine.sdo.list; defaults to
    ///                 the common CiA 402 table. Swap in a device-specific
    ///                 table for non-drive slaves.
    explicit EtherCatSdoAccess(EtherCAT::Master& master,
                               SdoObjectTableProvider objects = cia402SdoObjectTable)
        : master_(master), objects_(std::move(objects)) {}

    std::vector<SdoObjectInfoV1> listObjects(uint16_t slave) override {
        std::vector<SdoObjectInfoV1> entries;
        if (!objects_) return entries;
        for (const auto& entry : objects_(slave)) {
            entries.push_back({
                .index = entry.index,
                .subindex = entry.subindex,
                .dataType = static_cast<uint16_t>(entry.data_type),
                .access = entry.modification_mode ==
                        EtherCAT::ObjectDictionary::ModificationMode::ReadOnly
                    ? uint8_t{1} : uint8_t{3},
                .name = entry.name ? entry.name : "",
                .description = entry.comment ? entry.comment : "",
                .minValue = entry.min_value <= entry.max_value
                    ? static_cast<double>(entry.min_value)
                    : std::numeric_limits<double>::quiet_NaN(),
                .maxValue = entry.min_value <= entry.max_value
                    ? static_cast<double>(entry.max_value)
                    : std::numeric_limits<double>::quiet_NaN(),
            });
        }
        return entries;
    }

    SdoTransferResult read(uint16_t slave, uint16_t index, uint8_t subindex,
                           size_t maxBytes) override {
        SdoTransferResult result;
        std::vector<uint8_t> buffer(std::min(maxBytes, MachineSdoService::kMaxTransferBytes));
        size_t size = buffer.size();
        auto& target = master_.slave(slave);  // NonExistingSlave when out of range
        const auto error = target.sdoRead(index, subindex, buffer.data(), size);
        if (error != EtherCAT::SlaveError::Ok) {
            translate(target, error, result);
            return result;
        }
        result.ok = true;
        result.data.assign(buffer.begin(),
                           buffer.begin() + static_cast<ptrdiff_t>(size));
        return result;
    }

    SdoTransferResult write(uint16_t slave, uint16_t index, uint8_t subindex,
                            const std::vector<uint8_t>& data) override {
        SdoTransferResult result;
        auto& target = master_.slave(slave);
        const auto error = target.sdoWrite(index, subindex, data.data(), data.size());
        if (error != EtherCAT::SlaveError::Ok) {
            translate(target, error, result);
            return result;
        }
        result.ok = true;
        return result;
    }

private:
    static void translate(EtherCAT::Slave& slave, EtherCAT::SlaveError error,
                          SdoTransferResult& result) {
        if (error == EtherCAT::SlaveError::SDOAborted) {
            result.abortCode = slave.lastSdoAbortCode();
            result.error = "SDO aborted by slave";
        } else {
            result.error = EtherCAT::slaveErrorToString(error);
        }
    }

    EtherCAT::Master& master_;
    SdoObjectTableProvider objects_;
};

} // namespace tether::io::machine
