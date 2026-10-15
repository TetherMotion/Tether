#include "tether/drives/NexcobotESC211/RSAPMonitoringProcessor.hpp"

#include "logging/Logger.hpp"
#include "tether/ethercat/SdoCommandChannel.hpp"

namespace EtherCAT {
namespace Drives {
namespace ESC211 {

namespace {
constexpr const char* TAG = "RSAPMonProcessor";
}

// {index, subindex, name, is_signed} — every data entry of the
// 0x4100-0x4115 records plus the scalar limit/value objects.  Record
// sub-index 0 (the USINT entry count) is not polled.
const std::array<RSAPMonitoringProcessor::ObjectInfo,
                 RSAPMonitoringProcessor::kObjectCount>
RSAPMonitoringProcessor::kObjects = {{
    // 0x4100 — TCP 0 position (X/Y/Z + Z-director X/Y/Z), DINT
    {0x4100, 0x01, "TCP 0 Pos X Calculated",        true},
    {0x4100, 0x02, "TCP 0 Pos Y Calculated",        true},
    {0x4100, 0x03, "TCP 0 Pos Z Calculated",        true},
    {0x4100, 0x04, "TCP 0 Pos Z Director X",        true},
    {0x4100, 0x05, "TCP 0 Pos Z Director Y",        true},
    {0x4100, 0x06, "TCP 0 Pos Z Director Z",        true},
    // 0x4101 — TCP monitoring speed limit (shared by all TCPs)
    {0x4101, 0x00, "TCP Monitoring Speed Limit",    false},
    // 0x4102 — TCP 0..8 speed calculated, UDINT
    {0x4102, 0x01, "TCP 0 Speed Calculated",        false},
    {0x4102, 0x02, "TCP 1 Speed Calculated",        false},
    {0x4102, 0x03, "TCP 2 Speed Calculated",        false},
    {0x4102, 0x04, "TCP 3 Speed Calculated",        false},
    {0x4102, 0x05, "TCP 4 Speed Calculated",        false},
    {0x4102, 0x06, "TCP 5 Speed Calculated",        false},
    {0x4102, 0x07, "TCP 6 Speed Calculated",        false},
    {0x4102, 0x08, "TCP 7 Speed Calculated",        false},
    {0x4102, 0x09, "TCP 8 Speed Calculated",        false},
    // 0x4103 — TCP monitoring force limit
    {0x4103, 0x00, "TCP Monitoring Force Limit",    false},
    // 0x4104 — TCP 0 + safe TCP 1..8 force calculated, UDINT
    {0x4104, 0x01, "TCP 0 Force Calculated",        false},
    {0x4104, 0x02, "Safe TCP 1 Force Calculated",   false},
    {0x4104, 0x03, "Safe TCP 2 Force Calculated",   false},
    {0x4104, 0x04, "Safe TCP 3 Force Calculated",   false},
    {0x4104, 0x05, "Safe TCP 4 Force Calculated",   false},
    {0x4104, 0x06, "Safe TCP 5 Force Calculated",   false},
    {0x4104, 0x07, "Safe TCP 6 Force Calculated",   false},
    {0x4104, 0x08, "Safe TCP 7 Force Calculated",   false},
    {0x4104, 0x09, "Safe TCP 8 Force Calculated",   false},
    // 0x4105 — TCP 0 force vector (X/Y/Z), DINT
    {0x4105, 0x01, "TCP 0 Force Vector X",          true},
    {0x4105, 0x02, "TCP 0 Force Vector Y",          true},
    {0x4105, 0x03, "TCP 0 Force Vector Z",          true},
    // 0x4106/0x4107 — TCP power limit + calculated
    {0x4106, 0x00, "TCP Monitoring Power Limit",    false},
    {0x4107, 0x00, "TCP 0 Power Calculated",        false},
    // 0x4108 — endpoints monitoring speed limit
    {0x4108, 0x00, "Endpoints Mon. Speed Limit",    false},
    // 0x4109 — endpoint 1..7 speed calculated, UDINT
    {0x4109, 0x01, "Endpoint 1 Speed Calculated",   false},
    {0x4109, 0x02, "Endpoint 2 Speed Calculated",   false},
    {0x4109, 0x03, "Endpoint 3 Speed Calculated",   false},
    {0x4109, 0x04, "Endpoint 4 Speed Calculated",   false},
    {0x4109, 0x05, "Endpoint 5 Speed Calculated",   false},
    {0x4109, 0x06, "Endpoint 6 Speed Calculated",   false},
    {0x4109, 0x07, "Endpoint 7 Speed Calculated",   false},
    // 0x410A/0x410B — endpoints force limit + EP2 calculated
    {0x410A, 0x00, "Endpoints Mon. Force Limit",    false},
    {0x410B, 0x00, "Endpoint 2 Force Calculated",   false},
    // 0x410C — axis 1..7 position calculated, DINT
    {0x410C, 0x01, "Axis 1 Position Calculated",    true},
    {0x410C, 0x02, "Axis 2 Position Calculated",    true},
    {0x410C, 0x03, "Axis 3 Position Calculated",    true},
    {0x410C, 0x04, "Axis 4 Position Calculated",    true},
    {0x410C, 0x05, "Axis 5 Position Calculated",    true},
    {0x410C, 0x06, "Axis 6 Position Calculated",    true},
    {0x410C, 0x07, "Axis 7 Position Calculated",    true},
    // 0x410D — axis 1..7 monitoring speed limit, UDINT
    {0x410D, 0x01, "Axis 1 Mon. Speed Limit",       false},
    {0x410D, 0x02, "Axis 2 Mon. Speed Limit",       false},
    {0x410D, 0x03, "Axis 3 Mon. Speed Limit",       false},
    {0x410D, 0x04, "Axis 4 Mon. Speed Limit",       false},
    {0x410D, 0x05, "Axis 5 Mon. Speed Limit",       false},
    {0x410D, 0x06, "Axis 6 Mon. Speed Limit",       false},
    {0x410D, 0x07, "Axis 7 Mon. Speed Limit",       false},
    // 0x410E — axis speed calculated, UDINT.  The manual numbers the
    // entries "Axis 0 .. Axis 6" (0-based) — subs pair positionally
    // with the 1-based axis records.
    {0x410E, 0x01, "Axis 0 Speed Calculated",       false},
    {0x410E, 0x02, "Axis 1 Speed Calculated",       false},
    {0x410E, 0x03, "Axis 2 Speed Calculated",       false},
    {0x410E, 0x04, "Axis 3 Speed Calculated",       false},
    {0x410E, 0x05, "Axis 4 Speed Calculated",       false},
    {0x410E, 0x06, "Axis 5 Speed Calculated",       false},
    {0x410E, 0x07, "Axis 6 Speed Calculated",       false},
    // 0x4110 — axis 1..7 acceleration calculated, UDINT
    {0x4110, 0x01, "Axis 1 Acceleration Calculated", false},
    {0x4110, 0x02, "Axis 2 Acceleration Calculated", false},
    {0x4110, 0x03, "Axis 3 Acceleration Calculated", false},
    {0x4110, 0x04, "Axis 4 Acceleration Calculated", false},
    {0x4110, 0x05, "Axis 5 Acceleration Calculated", false},
    {0x4110, 0x06, "Axis 6 Acceleration Calculated", false},
    {0x4110, 0x07, "Axis 7 Acceleration Calculated", false},
    // 0x4111 — axis 1..7 monitoring torque limit, UDINT
    {0x4111, 0x01, "Axis 1 Mon. Torque Limit",      false},
    {0x4111, 0x02, "Axis 2 Mon. Torque Limit",      false},
    {0x4111, 0x03, "Axis 3 Mon. Torque Limit",      false},
    {0x4111, 0x04, "Axis 4 Mon. Torque Limit",      false},
    {0x4111, 0x05, "Axis 5 Mon. Torque Limit",      false},
    {0x4111, 0x06, "Axis 6 Mon. Torque Limit",      false},
    {0x4111, 0x07, "Axis 7 Mon. Torque Limit",      false},
    // 0x4112 — axis 1..7 torque calculated, UDINT
    {0x4112, 0x01, "Axis 1 Torque Calculated",      false},
    {0x4112, 0x02, "Axis 2 Torque Calculated",      false},
    {0x4112, 0x03, "Axis 3 Torque Calculated",      false},
    {0x4112, 0x04, "Axis 4 Torque Calculated",      false},
    {0x4112, 0x05, "Axis 5 Torque Calculated",      false},
    {0x4112, 0x06, "Axis 6 Torque Calculated",      false},
    {0x4112, 0x07, "Axis 7 Torque Calculated",      false},
    // 0x4113 — joint 1..7 inverse dynamics torque, DINT
    {0x4113, 0x01, "Joint 1 Inverse Dyn. Torque",   true},
    {0x4113, 0x02, "Joint 2 Inverse Dyn. Torque",   true},
    {0x4113, 0x03, "Joint 3 Inverse Dyn. Torque",   true},
    {0x4113, 0x04, "Joint 4 Inverse Dyn. Torque",   true},
    {0x4113, 0x05, "Joint 5 Inverse Dyn. Torque",   true},
    {0x4113, 0x06, "Joint 6 Inverse Dyn. Torque",   true},
    {0x4113, 0x07, "Joint 7 Inverse Dyn. Torque",   true},
    // 0x4114 — joint 1..7 friction torque, DINT
    {0x4114, 0x01, "Joint 1 Friction Torque",       true},
    {0x4114, 0x02, "Joint 2 Friction Torque",       true},
    {0x4114, 0x03, "Joint 3 Friction Torque",       true},
    {0x4114, 0x04, "Joint 4 Friction Torque",       true},
    {0x4114, 0x05, "Joint 5 Friction Torque",       true},
    {0x4114, 0x06, "Joint 6 Friction Torque",       true},
    {0x4114, 0x07, "Joint 7 Friction Torque",       true},
    // 0x4115 — axis 1..7 lever arm length to TCP 0, UDINT
    {0x4115, 0x01, "Axis 1 Lever Arm to TCP 0",     false},
    {0x4115, 0x02, "Axis 2 Lever Arm to TCP 0",     false},
    {0x4115, 0x03, "Axis 3 Lever Arm to TCP 0",     false},
    {0x4115, 0x04, "Axis 4 Lever Arm to TCP 0",     false},
    {0x4115, 0x05, "Axis 5 Lever Arm to TCP 0",     false},
    {0x4115, 0x06, "Axis 6 Lever Arm to TCP 0",     false},
    {0x4115, 0x07, "Axis 7 Lever Arm to TCP 0",     false},
}};

// Object table for the generic poll engine — every entry is 4 bytes.
const std::array<EtherCAT::SdoObjectEntry,
                 RSAPMonitoringProcessor::kObjectCount>
kSdoEntries = [] {
    std::array<EtherCAT::SdoObjectEntry,
               RSAPMonitoringProcessor::kObjectCount> t{};
    for (size_t i = 0; i < t.size(); ++i) {
        t[i] = {RSAPMonitoringProcessor::kObjects[i].index,
                RSAPMonitoringProcessor::kObjects[i].subindex, 4};
    }
    return t;
}();

RSAPMonitoringProcessor::RSAPMonitoringProcessor(EtherCAT::Slave& slave)
    : SdoObjectPoller(slave, kSdoEntries,
                      +[](size_t i) { return kObjects[i].name; },
                      TAG) {}

std::optional<uint32_t>
RSAPMonitoringSnapshot::get(uint16_t index, uint8_t subindex) const {
    for (size_t i = 0; i < RSAPMonitoringProcessor::kObjectCount; ++i) {
        const auto& o = RSAPMonitoringProcessor::kObjects[i];
        if (o.index == index && o.subindex == subindex) {
            return ok[i] ? std::optional<uint32_t>(value[i])
                         : std::nullopt;
        }
    }
    return std::nullopt;
}

std::optional<int32_t>
RSAPMonitoringSnapshot::getSigned(uint16_t index,
                                  uint8_t subindex) const {
    const auto v = get(index, subindex);
    if (!v) return std::nullopt;
    return static_cast<int32_t>(*v);
}

} // namespace ESC211
} // namespace Drives
} // namespace EtherCAT
