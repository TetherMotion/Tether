#pragma once

#include <cstdint>
#include "tether/drives/NexcobotESC211/Registers/Common.hpp"

namespace EtherCAT {
namespace Drives {
namespace Registers {
namespace NexcobotESC211 {
namespace RSAPMonitoring {

// ESI v0.9: the 0x4100-range is the RSAP calculated-values / monitoring-limits
// dictionary.  Record objects (DT41xx) are modeled by their top-level entry;
// the scalar limits are UDINT.
//
//   0x4100  TCP 0 Position Calculated by RSAP            DT4100 (7 subs, 26 B)
//   0x4101  TCP Monitoring Speed Limit                   UDINT
//   0x4102  TCP 0 Speed Calculated by RSAP               DT4102 (10 subs, 38 B)
//   0x4103  TCP Monitoring Force Limit                   UDINT
//   0x4104  TCP Force Calculated by RSAP                 DT4104 (10 subs, 38 B)
//   0x4105  TCP 0 Force Vector Calculated by RSAP        DT4105 (4 subs, 14 B)
//   0x4106  TCP Monitoring Power Limit                   UDINT
//   0x4107  TCP 0 Power Calculated by RSAP               UDINT
//   0x4108  Endpoints Monitoring Speed Limit             UDINT
//   0x4109  Endpoint Speed Calculated by RSAP            DT4109 (8 subs, 30 B)
//   0x410A  Endpoints Monitoring Force Limit             UDINT
//   0x410B  Endpoint 2 Force Calculated by RSAP          UDINT
//   0x410C  Axis Position Calculated by RSAP             DT410C (8 subs, 30 B)
//   0x410D  Axis Monitoring Speed Limit                  DT410D (8 subs, 30 B)
//   0x410E  Axis Speed Calculated by RSAP                DT410E (8 subs, 30 B)
//   0x4110  Axis Acceleration Calculated by RSAP         DT4110 (8 subs, 30 B)
//   0x4111  Axis Monitoring Torque Limit                 DT4111 (8 subs, 30 B)
//   0x4112  Axis Torque Calculated by RSAP               DT4112 (8 subs, 30 B)
//   0x4113  Joint Inverse Dynamics Torque Calculated     DT4113 (8 subs, 30 B)
//   0x4114  Joint Friction Torque Calculated by RSAP     DT4114 (8 subs, 30 B)
//   0x4115  Axis Lever Arm Length to TCP 0               DT4115 (8 subs, 30 B)

#define NEXCOBOT_RSAP_UDINT(NAME, IDX, DESC) \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry NAME = { \
        .index = (IDX), .subindex = 0x00, .name = (DESC), \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned32, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFFFFFFFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, .comment = (DESC), \
    }

#define NEXCOBOT_RSAP_RECORD(NAME, IDX, DESC) \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry NAME = { \
        .index = (IDX), .subindex = 0x00, .name = (DESC), \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Domain, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, .comment = (DESC), \
    }

NEXCOBOT_RSAP_RECORD(TCP0PositionCalculated,        0x4100, "TCP 0 Position Calculated by RSAP");
NEXCOBOT_RSAP_UDINT (TCPMonitoringSpeedLimit,       0x4101, "TCP Monitoring Speed Limit");
NEXCOBOT_RSAP_RECORD(TCP0SpeedCalculated,           0x4102, "TCP 0 Speed Calculated by RSAP");
NEXCOBOT_RSAP_UDINT (TCPMonitoringForceLimit,       0x4103, "TCP Monitoring Force Limit");
NEXCOBOT_RSAP_RECORD(TCPForceCalculated,            0x4104, "TCP Force Calculated by RSAP");
NEXCOBOT_RSAP_RECORD(TCP0ForceVectorCalculated,     0x4105, "TCP 0 Force Vector Calculated by RSAP");
NEXCOBOT_RSAP_UDINT (TCPMonitoringPowerLimit,       0x4106, "TCP Monitoring Power Limit");
NEXCOBOT_RSAP_UDINT (TCP0PowerCalculated,           0x4107, "TCP 0 Power Calculated by RSAP");
NEXCOBOT_RSAP_UDINT (EndpointsMonitoringSpeedLimit, 0x4108, "Endpoints Monitoring Speed Limit");
NEXCOBOT_RSAP_RECORD(EndpointSpeedCalculated,       0x4109, "Endpoint Speed Calculated by RSAP");
NEXCOBOT_RSAP_UDINT (EndpointsMonitoringForceLimit, 0x410A, "Endpoints Monitoring Force Limit");
NEXCOBOT_RSAP_UDINT (Endpoint2ForceCalculated,      0x410B, "Endpoint 2 Force Calculated by RSAP");
NEXCOBOT_RSAP_RECORD(AxisPositionCalculated,        0x410C, "Axis Position Calculated by RSAP");
NEXCOBOT_RSAP_RECORD(AxisMonitoringSpeedLimit,      0x410D, "Axis Monitoring Speed Limit");
NEXCOBOT_RSAP_RECORD(AxisSpeedCalculated,           0x410E, "Axis Speed Calculated by RSAP");
NEXCOBOT_RSAP_RECORD(AxisAccelerationCalculated,    0x4110, "Axis Acceleration Calculated by RSAP");
NEXCOBOT_RSAP_RECORD(AxisMonitoringTorqueLimit,     0x4111, "Axis Monitoring Torque Limit");
NEXCOBOT_RSAP_RECORD(AxisTorqueCalculated,          0x4112, "Axis Torque Calculated by RSAP");
NEXCOBOT_RSAP_RECORD(JointInverseDynamicsTorque,    0x4113, "Joint Inverse Dynamics Torque Calculated by RSAP");
NEXCOBOT_RSAP_RECORD(JointFrictionTorque,           0x4114, "Joint Friction Torque Calculated by RSAP");
NEXCOBOT_RSAP_RECORD(AxisLeverArmLengthTCP0,        0x4115, "Axis Lever Arm Length to TCP 0");

#undef NEXCOBOT_RSAP_UDINT
#undef NEXCOBOT_RSAP_RECORD

inline const RegisterList kRegisterList = {
    &TCP0PositionCalculated, &TCPMonitoringSpeedLimit, &TCP0SpeedCalculated,
    &TCPMonitoringForceLimit, &TCPForceCalculated, &TCP0ForceVectorCalculated,
    &TCPMonitoringPowerLimit, &TCP0PowerCalculated, &EndpointsMonitoringSpeedLimit,
    &EndpointSpeedCalculated, &EndpointsMonitoringForceLimit, &Endpoint2ForceCalculated,
    &AxisPositionCalculated, &AxisMonitoringSpeedLimit, &AxisSpeedCalculated,
    &AxisAccelerationCalculated, &AxisMonitoringTorqueLimit, &AxisTorqueCalculated,
    &JointInverseDynamicsTorque, &JointFrictionTorque, &AxisLeverArmLengthTCP0,
};

} // namespace RSAPMonitoring
} // namespace NexcobotESC211
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
