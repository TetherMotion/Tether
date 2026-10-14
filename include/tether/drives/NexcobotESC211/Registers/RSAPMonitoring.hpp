#pragma once

// Nexcobot ESC211 — RSAP Monitoring (0x4100–0x4115)
// Calculated-value / monitoring-limit objects published by the RSAP.
// Read-only; all entries are PDO-mappable.
// Sources: ESC211_v0_9_13-2.xml (object names/types/sub counts)
//          "ESC CoE Object (Manual).xlsx" (record sub-entry names)
//
// Record convention (verified on-device): sub-index 0 is the USINT entry
// count and data entries live at sub-index 1..N.  The vendor manual lists
// record data starting at "Sub 0" — i.e. its Sub N = device sub N+1.
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
//   0x410F  (not in ESI v0.9 — reserved/unused)
//   0x4110  Axis Acceleration Calculated by RSAP         DT4110 (8 subs, 30 B)
//   0x4111  Axis Monitoring Torque Limit                 DT4111 (8 subs, 30 B)
//   0x4112  Axis Torque Calculated by RSAP               DT4112 (8 subs, 30 B)
//   0x4113  Joint Inverse Dynamics Torque Calculated     DT4113 (8 subs, 30 B)
//   0x4114  Joint Friction Torque Calculated by RSAP     DT4114 (8 subs, 30 B)
//   0x4115  Axis Lever Arm Length to TCP 0               DT4115 (8 subs, 30 B)

#include <cstdint>
#include "tether/drives/NexcobotESC211/Registers/Common.hpp"

namespace EtherCAT {
namespace Drives {
namespace Registers {
namespace NexcobotESC211 {
namespace RSAPMonitoring {

using ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry;
using ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType;

#define NEXCOBOT_RSAP_ENTRY(NAME, IDX, SUB, TYPE, DESC)                        \
    constexpr ObjectDictionaryEntry NAME = {                                   \
        .index = (IDX), .subindex = (SUB), .name = (DESC),                     \
        .data_type = ObjectDictionaryDataType::TYPE,                           \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr,        \
        .min_value = 0, .max_value = 0xFFFFFFFF,                               \
        .modification_mode = ModificationMode::ReadOnly,                       \
        .effective_time = EffectiveTime::Immediately, .comment = (DESC),       \
    }

#define NEXCOBOT_RSAP_UDINT(NAME, IDX, DESC)                                   \
    NEXCOBOT_RSAP_ENTRY(NAME, IDX, 0x00, Unsigned32, DESC)
#define NEXCOBOT_RSAP_DINT(NAME, IDX, DESC)                                    \
    NEXCOBOT_RSAP_ENTRY(NAME, IDX, 0x00, Integer32, DESC)
#define NEXCOBOT_RSAP_USINT(NAME, IDX, DESC)                                   \
    NEXCOBOT_RSAP_ENTRY(NAME, IDX, 0x00, Unsigned8, DESC)
#define NEXCOBOT_RSAP_UDINT_SUB(NAME, IDX, SUB, DESC)                          \
    NEXCOBOT_RSAP_ENTRY(NAME, IDX, SUB, Unsigned32, DESC)
#define NEXCOBOT_RSAP_DINT_SUB(NAME, IDX, SUB, DESC)                           \
    NEXCOBOT_RSAP_ENTRY(NAME, IDX, SUB, Integer32, DESC)

// Record top-level entry (the object itself; data lives in the sub-entries).
#define NEXCOBOT_RSAP_RECORD(NAME, IDX, DESC)                                  \
    constexpr ObjectDictionaryEntry NAME = {                                   \
        .index = (IDX), .subindex = 0x00, .name = (DESC),                      \
        .data_type = ObjectDictionaryDataType::Domain,                         \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr,        \
        .min_value = 0, .max_value = 0,                                        \
        .modification_mode = ModificationMode::ReadOnly,                       \
        .effective_time = EffectiveTime::Immediately, .comment = (DESC),       \
    }

// ============================================================
// TCP monitoring objects
// ============================================================

// 0x4100: TCP 0 Position Calculated by RSAP — record of 6 DINT values
//         (X/Y/Z + Z-director X/Y/Z)
NEXCOBOT_RSAP_RECORD(TCP0PositionCalculated,        0x4100, "TCP 0 Position Calculated by RSAP");
NEXCOBOT_RSAP_DINT_SUB(TCP0PosXCalculated,          0x4100, 0x01, "TCP 0 Pos X Calculated by RSAP");
NEXCOBOT_RSAP_DINT_SUB(TCP0PosYCalculated,          0x4100, 0x02, "TCP 0 Pos Y Calculated by RSAP");
NEXCOBOT_RSAP_DINT_SUB(TCP0PosZCalculated,          0x4100, 0x03, "TCP 0 Pos Z Calculated by RSAP");
NEXCOBOT_RSAP_DINT_SUB(TCP0PosZDirectorXCalculated, 0x4100, 0x04, "TCP 0 Pos Z Director X Calculated by RSAP");
NEXCOBOT_RSAP_DINT_SUB(TCP0PosZDirectorYCalculated, 0x4100, 0x05, "TCP 0 Pos Z Director Y Calculated by RSAP");
NEXCOBOT_RSAP_DINT_SUB(TCP0PosZDirectorZCalculated, 0x4100, 0x06, "TCP 0 Pos Z Director Z Calculated by RSAP");

// 0x4101: TCP Monitoring Speed Limit — scalar UDINT
NEXCOBOT_RSAP_UDINT (TCPMonitoringSpeedLimit,       0x4101, "TCP Monitoring Speed Limit");

// 0x4102: TCP N Speed Calculated by RSAP — record of 9 UDINT values
//         (TCP0–TCP8; note: one firmware observed count=8 → TCP0–TCP7)
NEXCOBOT_RSAP_RECORD(TCP0SpeedCalculated,           0x4102, "TCP 0 Speed Calculated by RSAP");
NEXCOBOT_RSAP_UDINT_SUB(TCP0SpeedCalculatedSub,     0x4102, 0x01, "TCP 0 Speed Calculated");
NEXCOBOT_RSAP_UDINT_SUB(TCP1SpeedCalculated,        0x4102, 0x02, "TCP 1 Speed Calculated");
NEXCOBOT_RSAP_UDINT_SUB(TCP2SpeedCalculated,        0x4102, 0x03, "TCP 2 Speed Calculated");
NEXCOBOT_RSAP_UDINT_SUB(TCP3SpeedCalculated,        0x4102, 0x04, "TCP 3 Speed Calculated");
NEXCOBOT_RSAP_UDINT_SUB(TCP4SpeedCalculated,        0x4102, 0x05, "TCP 4 Speed Calculated");
NEXCOBOT_RSAP_UDINT_SUB(TCP5SpeedCalculated,        0x4102, 0x06, "TCP 5 Speed Calculated");
NEXCOBOT_RSAP_UDINT_SUB(TCP6SpeedCalculated,        0x4102, 0x07, "TCP 6 Speed Calculated");
NEXCOBOT_RSAP_UDINT_SUB(TCP7SpeedCalculated,        0x4102, 0x08, "TCP 7 Speed Calculated");
NEXCOBOT_RSAP_UDINT_SUB(TCP8SpeedCalculated,        0x4102, 0x09, "TCP 8 Speed Calculated");

// 0x4103: TCP Monitoring Force Limit — scalar UDINT
NEXCOBOT_RSAP_UDINT (TCPMonitoringForceLimit,       0x4103, "TCP Monitoring Force Limit");

// 0x4104: TCP Force Calculated by RSAP — record of 9 UDINT values
//         (TCP0 force + safe TCP1–TCP8 forces)
NEXCOBOT_RSAP_RECORD(TCPForceCalculated,            0x4104, "TCP Force Calculated by RSAP");
NEXCOBOT_RSAP_UDINT_SUB(TCP0ForceCalculated,        0x4104, 0x01, "TCP 0 Force Calculated");
NEXCOBOT_RSAP_UDINT_SUB(SafeTCP1ForceCalculated,    0x4104, 0x02, "Safe TCP 1 Force Calculated");
NEXCOBOT_RSAP_UDINT_SUB(SafeTCP2ForceCalculated,    0x4104, 0x03, "Safe TCP 2 Force Calculated");
NEXCOBOT_RSAP_UDINT_SUB(SafeTCP3ForceCalculated,    0x4104, 0x04, "Safe TCP 3 Force Calculated");
NEXCOBOT_RSAP_UDINT_SUB(SafeTCP4ForceCalculated,    0x4104, 0x05, "Safe TCP 4 Force Calculated");
NEXCOBOT_RSAP_UDINT_SUB(SafeTCP5ForceCalculated,    0x4104, 0x06, "Safe TCP 5 Force Calculated");
NEXCOBOT_RSAP_UDINT_SUB(SafeTCP6ForceCalculated,    0x4104, 0x07, "Safe TCP 6 Force Calculated");
NEXCOBOT_RSAP_UDINT_SUB(SafeTCP7ForceCalculated,    0x4104, 0x08, "Safe TCP 7 Force Calculated");
NEXCOBOT_RSAP_UDINT_SUB(SafeTCP8ForceCalculated,    0x4104, 0x09, "Safe TCP 8 Force Calculated");

// 0x4105: TCP 0 Force Vector Calculated by RSAP — record of 3 DINT values
NEXCOBOT_RSAP_RECORD(TCP0ForceVectorCalculated,     0x4105, "TCP 0 Force Vector Calculated by RSAP");
NEXCOBOT_RSAP_DINT_SUB(TCP0ForceVectorXCalculated,  0x4105, 0x01, "TCP 0 Force Vector X Calculated");
NEXCOBOT_RSAP_DINT_SUB(TCP0ForceVectorYCalculated,  0x4105, 0x02, "TCP 0 Force Vector Y Calculated");
NEXCOBOT_RSAP_DINT_SUB(TCP0ForceVectorZCalculated,  0x4105, 0x03, "TCP 0 Force Vector Z Calculated");

// 0x4106-0x4108: scalar limits / calculated power
NEXCOBOT_RSAP_UDINT (TCPMonitoringPowerLimit,       0x4106, "TCP Monitoring Power Limit");
NEXCOBOT_RSAP_UDINT (TCP0PowerCalculated,           0x4107, "TCP 0 Power Calculated by RSAP");
NEXCOBOT_RSAP_UDINT (EndpointsMonitoringSpeedLimit, 0x4108, "Endpoints Monitoring Speed Limit");

// 0x4109: Endpoint N Speed Calculated by RSAP — record of 7 UDINT values
NEXCOBOT_RSAP_RECORD(EndpointSpeedCalculated,       0x4109, "Endpoint Speed Calculated by RSAP");
NEXCOBOT_RSAP_UDINT_SUB(Endpoint1SpeedCalculated,   0x4109, 0x01, "Endpoint 1 Speed Calculated");
NEXCOBOT_RSAP_UDINT_SUB(Endpoint2SpeedCalculated,   0x4109, 0x02, "Endpoint 2 Speed Calculated");
NEXCOBOT_RSAP_UDINT_SUB(Endpoint3SpeedCalculated,   0x4109, 0x03, "Endpoint 3 Speed Calculated");
NEXCOBOT_RSAP_UDINT_SUB(Endpoint4SpeedCalculated,   0x4109, 0x04, "Endpoint 4 Speed Calculated");
NEXCOBOT_RSAP_UDINT_SUB(Endpoint5SpeedCalculated,   0x4109, 0x05, "Endpoint 5 Speed Calculated");
NEXCOBOT_RSAP_UDINT_SUB(Endpoint6SpeedCalculated,   0x4109, 0x06, "Endpoint 6 Speed Calculated");
NEXCOBOT_RSAP_UDINT_SUB(Endpoint7SpeedCalculated,   0x4109, 0x07, "Endpoint 7 Speed Calculated");

// 0x410A-0x410B: scalar limit / calculated force
NEXCOBOT_RSAP_UDINT (EndpointsMonitoringForceLimit, 0x410A, "Endpoints Monitoring Force Limit");
NEXCOBOT_RSAP_UDINT (Endpoint2ForceCalculated,      0x410B, "Endpoint 2 Force Calculated by RSAP");

// ============================================================
// Axis monitoring objects
// ============================================================

// 0x410C: Axis N Position Calculated by RSAP — record of 7 DINT values
NEXCOBOT_RSAP_RECORD(AxisPositionCalculated,        0x410C, "Axis Position Calculated by RSAP");
NEXCOBOT_RSAP_DINT_SUB(Axis1PositionCalculated,     0x410C, 0x01, "Axis 1 Position Calculated");
NEXCOBOT_RSAP_DINT_SUB(Axis2PositionCalculated,     0x410C, 0x02, "Axis 2 Position Calculated");
NEXCOBOT_RSAP_DINT_SUB(Axis3PositionCalculated,     0x410C, 0x03, "Axis 3 Position Calculated");
NEXCOBOT_RSAP_DINT_SUB(Axis4PositionCalculated,     0x410C, 0x04, "Axis 4 Position Calculated");
NEXCOBOT_RSAP_DINT_SUB(Axis5PositionCalculated,     0x410C, 0x05, "Axis 5 Position Calculated");
NEXCOBOT_RSAP_DINT_SUB(Axis6PositionCalculated,     0x410C, 0x06, "Axis 6 Position Calculated");
NEXCOBOT_RSAP_DINT_SUB(Axis7PositionCalculated,     0x410C, 0x07, "Axis 7 Position Calculated");

// 0x410D: Axis N Monitoring Speed Limit — record of 7 UDINT values
NEXCOBOT_RSAP_RECORD(AxisMonitoringSpeedLimit,      0x410D, "Axis Monitoring Speed Limit");
NEXCOBOT_RSAP_UDINT_SUB(Axis1MonitoringSpeedLimit,  0x410D, 0x01, "Axis 1 Monitoring Speed Limit");
NEXCOBOT_RSAP_UDINT_SUB(Axis2MonitoringSpeedLimit,  0x410D, 0x02, "Axis 2 Monitoring Speed Limit");
NEXCOBOT_RSAP_UDINT_SUB(Axis3MonitoringSpeedLimit,  0x410D, 0x03, "Axis 3 Monitoring Speed Limit");
NEXCOBOT_RSAP_UDINT_SUB(Axis4MonitoringSpeedLimit,  0x410D, 0x04, "Axis 4 Monitoring Speed Limit");
NEXCOBOT_RSAP_UDINT_SUB(Axis5MonitoringSpeedLimit,  0x410D, 0x05, "Axis 5 Monitoring Speed Limit");
NEXCOBOT_RSAP_UDINT_SUB(Axis6MonitoringSpeedLimit,  0x410D, 0x06, "Axis 6 Monitoring Speed Limit");
NEXCOBOT_RSAP_UDINT_SUB(Axis7MonitoringSpeedLimit,  0x410D, 0x07, "Axis 7 Monitoring Speed Limit");

// 0x410E: Axis N Speed Calculated by RSAP — record of 7 UDINT values.
//         The manual names the entries "Axis 0 Speed … Axis 6 Speed"
//         (0-based numbering, unlike the other axis records).
NEXCOBOT_RSAP_RECORD(AxisSpeedCalculated,           0x410E, "Axis Speed Calculated by RSAP");
NEXCOBOT_RSAP_UDINT_SUB(Axis0SpeedCalculated,       0x410E, 0x01, "Axis 0 Speed Calculated");
NEXCOBOT_RSAP_UDINT_SUB(Axis1SpeedCalculated,       0x410E, 0x02, "Axis 1 Speed Calculated");
NEXCOBOT_RSAP_UDINT_SUB(Axis2SpeedCalculated,       0x410E, 0x03, "Axis 2 Speed Calculated");
NEXCOBOT_RSAP_UDINT_SUB(Axis3SpeedCalculated,       0x410E, 0x04, "Axis 3 Speed Calculated");
NEXCOBOT_RSAP_UDINT_SUB(Axis4SpeedCalculated,       0x410E, 0x05, "Axis 4 Speed Calculated");
NEXCOBOT_RSAP_UDINT_SUB(Axis5SpeedCalculated,       0x410E, 0x06, "Axis 5 Speed Calculated");
NEXCOBOT_RSAP_UDINT_SUB(Axis6SpeedCalculated,       0x410E, 0x07, "Axis 6 Speed Calculated");

// 0x4110: Axis N Acceleration Calculated by RSAP — record of 7 UDINT values
NEXCOBOT_RSAP_RECORD(AxisAccelerationCalculated,    0x4110, "Axis Acceleration Calculated by RSAP");
NEXCOBOT_RSAP_UDINT_SUB(Axis1AccelerationCalculated, 0x4110, 0x01, "Axis 1 Acceleration Calculated");
NEXCOBOT_RSAP_UDINT_SUB(Axis2AccelerationCalculated, 0x4110, 0x02, "Axis 2 Acceleration Calculated");
NEXCOBOT_RSAP_UDINT_SUB(Axis3AccelerationCalculated, 0x4110, 0x03, "Axis 3 Acceleration Calculated");
NEXCOBOT_RSAP_UDINT_SUB(Axis4AccelerationCalculated, 0x4110, 0x04, "Axis 4 Acceleration Calculated");
NEXCOBOT_RSAP_UDINT_SUB(Axis5AccelerationCalculated, 0x4110, 0x05, "Axis 5 Acceleration Calculated");
NEXCOBOT_RSAP_UDINT_SUB(Axis6AccelerationCalculated, 0x4110, 0x06, "Axis 6 Acceleration Calculated");
NEXCOBOT_RSAP_UDINT_SUB(Axis7AccelerationCalculated, 0x4110, 0x07, "Axis 7 Acceleration Calculated");

// 0x4111: Axis N Monitoring Torque Limit — record of 7 UDINT values
NEXCOBOT_RSAP_RECORD(AxisMonitoringTorqueLimit,     0x4111, "Axis Monitoring Torque Limit");
NEXCOBOT_RSAP_UDINT_SUB(Axis1MonitoringTorqueLimit, 0x4111, 0x01, "Axis 1 Monitoring Torque Limit");
NEXCOBOT_RSAP_UDINT_SUB(Axis2MonitoringTorqueLimit, 0x4111, 0x02, "Axis 2 Monitoring Torque Limit");
NEXCOBOT_RSAP_UDINT_SUB(Axis3MonitoringTorqueLimit, 0x4111, 0x03, "Axis 3 Monitoring Torque Limit");
NEXCOBOT_RSAP_UDINT_SUB(Axis4MonitoringTorqueLimit, 0x4111, 0x04, "Axis 4 Monitoring Torque Limit");
NEXCOBOT_RSAP_UDINT_SUB(Axis5MonitoringTorqueLimit, 0x4111, 0x05, "Axis 5 Monitoring Torque Limit");
NEXCOBOT_RSAP_UDINT_SUB(Axis6MonitoringTorqueLimit, 0x4111, 0x06, "Axis 6 Monitoring Torque Limit");
NEXCOBOT_RSAP_UDINT_SUB(Axis7MonitoringTorqueLimit, 0x4111, 0x07, "Axis 7 Monitoring Torque Limit");

// 0x4112: Axis N Torque Calculated by RSAP — record of 7 UDINT values
NEXCOBOT_RSAP_RECORD(AxisTorqueCalculated,          0x4112, "Axis Torque Calculated by RSAP");
NEXCOBOT_RSAP_UDINT_SUB(Axis1TorqueCalculated,      0x4112, 0x01, "Axis 1 Torque Calculated");
NEXCOBOT_RSAP_UDINT_SUB(Axis2TorqueCalculated,      0x4112, 0x02, "Axis 2 Torque Calculated");
NEXCOBOT_RSAP_UDINT_SUB(Axis3TorqueCalculated,      0x4112, 0x03, "Axis 3 Torque Calculated");
NEXCOBOT_RSAP_UDINT_SUB(Axis4TorqueCalculated,      0x4112, 0x04, "Axis 4 Torque Calculated");
NEXCOBOT_RSAP_UDINT_SUB(Axis5TorqueCalculated,      0x4112, 0x05, "Axis 5 Torque Calculated");
NEXCOBOT_RSAP_UDINT_SUB(Axis6TorqueCalculated,      0x4112, 0x06, "Axis 6 Torque Calculated");
NEXCOBOT_RSAP_UDINT_SUB(Axis7TorqueCalculated,      0x4112, 0x07, "Axis 7 Torque Calculated");

// 0x4113: Joint N Inverse Dynamics Torque — record of 7 DINT values
NEXCOBOT_RSAP_RECORD(JointInverseDynamicsTorque,    0x4113, "Joint Inverse Dynamics Torque Calculated by RSAP");
NEXCOBOT_RSAP_DINT_SUB(Joint1InverseDynamicsTorque, 0x4113, 0x01, "Joint 1 Inverse Dynamics Torque Calculated");
NEXCOBOT_RSAP_DINT_SUB(Joint2InverseDynamicsTorque, 0x4113, 0x02, "Joint 2 Inverse Dynamics Torque Calculated");
NEXCOBOT_RSAP_DINT_SUB(Joint3InverseDynamicsTorque, 0x4113, 0x03, "Joint 3 Inverse Dynamics Torque Calculated");
NEXCOBOT_RSAP_DINT_SUB(Joint4InverseDynamicsTorque, 0x4113, 0x04, "Joint 4 Inverse Dynamics Torque Calculated");
NEXCOBOT_RSAP_DINT_SUB(Joint5InverseDynamicsTorque, 0x4113, 0x05, "Joint 5 Inverse Dynamics Torque Calculated");
NEXCOBOT_RSAP_DINT_SUB(Joint6InverseDynamicsTorque, 0x4113, 0x06, "Joint 6 Inverse Dynamics Torque Calculated");
NEXCOBOT_RSAP_DINT_SUB(Joint7InverseDynamicsTorque, 0x4113, 0x07, "Joint 7 Inverse Dynamics Torque Calculated");

// 0x4114: Joint N Friction Torque — record of 7 DINT values
NEXCOBOT_RSAP_RECORD(JointFrictionTorque,           0x4114, "Joint Friction Torque Calculated by RSAP");
NEXCOBOT_RSAP_DINT_SUB(Joint1FrictionTorque,        0x4114, 0x01, "Joint 1 Friction Torque Calculated");
NEXCOBOT_RSAP_DINT_SUB(Joint2FrictionTorque,        0x4114, 0x02, "Joint 2 Friction Torque Calculated");
NEXCOBOT_RSAP_DINT_SUB(Joint3FrictionTorque,        0x4114, 0x03, "Joint 3 Friction Torque Calculated");
NEXCOBOT_RSAP_DINT_SUB(Joint4FrictionTorque,        0x4114, 0x04, "Joint 4 Friction Torque Calculated");
NEXCOBOT_RSAP_DINT_SUB(Joint5FrictionTorque,        0x4114, 0x05, "Joint 5 Friction Torque Calculated");
NEXCOBOT_RSAP_DINT_SUB(Joint6FrictionTorque,        0x4114, 0x06, "Joint 6 Friction Torque Calculated");
NEXCOBOT_RSAP_DINT_SUB(Joint7FrictionTorque,        0x4114, 0x07, "Joint 7 Friction Torque Calculated");

// 0x4115: Axis N Lever Arm Length to TCP 0 — record of 7 UDINT values
NEXCOBOT_RSAP_RECORD(AxisLeverArmLengthTCP0,        0x4115, "Axis Lever Arm Length to TCP 0");
NEXCOBOT_RSAP_UDINT_SUB(Axis1LeverArmLengthTCP0,    0x4115, 0x01, "Axis 1 Lever Arm Length to TCP 0");
NEXCOBOT_RSAP_UDINT_SUB(Axis2LeverArmLengthTCP0,    0x4115, 0x02, "Axis 2 Lever Arm Length to TCP 0");
NEXCOBOT_RSAP_UDINT_SUB(Axis3LeverArmLengthTCP0,    0x4115, 0x03, "Axis 3 Lever Arm Length to TCP 0");
NEXCOBOT_RSAP_UDINT_SUB(Axis4LeverArmLengthTCP0,    0x4115, 0x04, "Axis 4 Lever Arm Length to TCP 0");
NEXCOBOT_RSAP_UDINT_SUB(Axis5LeverArmLengthTCP0,    0x4115, 0x05, "Axis 5 Lever Arm Length to TCP 0");
NEXCOBOT_RSAP_UDINT_SUB(Axis6LeverArmLengthTCP0,    0x4115, 0x06, "Axis 6 Lever Arm Length to TCP 0");
NEXCOBOT_RSAP_UDINT_SUB(Axis7LeverArmLengthTCP0,    0x4115, 0x07, "Axis 7 Lever Arm Length to TCP 0");

#undef NEXCOBOT_RSAP_ENTRY
#undef NEXCOBOT_RSAP_UDINT
#undef NEXCOBOT_RSAP_DINT
#undef NEXCOBOT_RSAP_USINT
#undef NEXCOBOT_RSAP_UDINT_SUB
#undef NEXCOBOT_RSAP_DINT_SUB
#undef NEXCOBOT_RSAP_RECORD

inline const RegisterList kRegisterList = {
    // 0x4100 — TCP 0 position
    &TCP0PositionCalculated,
    &TCP0PosXCalculated, &TCP0PosYCalculated, &TCP0PosZCalculated,
    &TCP0PosZDirectorXCalculated, &TCP0PosZDirectorYCalculated,
    &TCP0PosZDirectorZCalculated,
    // 0x4101-0x4103
    &TCPMonitoringSpeedLimit,
    &TCP0SpeedCalculated,
    &TCP0SpeedCalculatedSub, &TCP1SpeedCalculated, &TCP2SpeedCalculated,
    &TCP3SpeedCalculated, &TCP4SpeedCalculated, &TCP5SpeedCalculated,
    &TCP6SpeedCalculated, &TCP7SpeedCalculated, &TCP8SpeedCalculated,
    &TCPMonitoringForceLimit,
    // 0x4104 — TCP forces
    &TCPForceCalculated,
    &TCP0ForceCalculated, &SafeTCP1ForceCalculated, &SafeTCP2ForceCalculated,
    &SafeTCP3ForceCalculated, &SafeTCP4ForceCalculated, &SafeTCP5ForceCalculated,
    &SafeTCP6ForceCalculated, &SafeTCP7ForceCalculated, &SafeTCP8ForceCalculated,
    // 0x4105 — TCP 0 force vector
    &TCP0ForceVectorCalculated,
    &TCP0ForceVectorXCalculated, &TCP0ForceVectorYCalculated,
    &TCP0ForceVectorZCalculated,
    // 0x4106-0x4108
    &TCPMonitoringPowerLimit, &TCP0PowerCalculated,
    &EndpointsMonitoringSpeedLimit,
    // 0x4109 — endpoint speeds
    &EndpointSpeedCalculated,
    &Endpoint1SpeedCalculated, &Endpoint2SpeedCalculated,
    &Endpoint3SpeedCalculated, &Endpoint4SpeedCalculated,
    &Endpoint5SpeedCalculated, &Endpoint6SpeedCalculated,
    &Endpoint7SpeedCalculated,
    // 0x410A-0x410B
    &EndpointsMonitoringForceLimit, &Endpoint2ForceCalculated,
    // 0x410C — axis positions
    &AxisPositionCalculated,
    &Axis1PositionCalculated, &Axis2PositionCalculated,
    &Axis3PositionCalculated, &Axis4PositionCalculated,
    &Axis5PositionCalculated, &Axis6PositionCalculated,
    &Axis7PositionCalculated,
    // 0x410D — axis monitoring speed limits
    &AxisMonitoringSpeedLimit,
    &Axis1MonitoringSpeedLimit, &Axis2MonitoringSpeedLimit,
    &Axis3MonitoringSpeedLimit, &Axis4MonitoringSpeedLimit,
    &Axis5MonitoringSpeedLimit, &Axis6MonitoringSpeedLimit,
    &Axis7MonitoringSpeedLimit,
    // 0x410E — axis speeds
    &AxisSpeedCalculated,
    &Axis0SpeedCalculated, &Axis1SpeedCalculated, &Axis2SpeedCalculated,
    &Axis3SpeedCalculated, &Axis4SpeedCalculated, &Axis5SpeedCalculated,
    &Axis6SpeedCalculated,
    // 0x4110 — axis accelerations
    &AxisAccelerationCalculated,
    &Axis1AccelerationCalculated, &Axis2AccelerationCalculated,
    &Axis3AccelerationCalculated, &Axis4AccelerationCalculated,
    &Axis5AccelerationCalculated, &Axis6AccelerationCalculated,
    &Axis7AccelerationCalculated,
    // 0x4111 — axis monitoring torque limits
    &AxisMonitoringTorqueLimit,
    &Axis1MonitoringTorqueLimit, &Axis2MonitoringTorqueLimit,
    &Axis3MonitoringTorqueLimit, &Axis4MonitoringTorqueLimit,
    &Axis5MonitoringTorqueLimit, &Axis6MonitoringTorqueLimit,
    &Axis7MonitoringTorqueLimit,
    // 0x4112 — axis torques
    &AxisTorqueCalculated,
    &Axis1TorqueCalculated, &Axis2TorqueCalculated, &Axis3TorqueCalculated,
    &Axis4TorqueCalculated, &Axis5TorqueCalculated, &Axis6TorqueCalculated,
    &Axis7TorqueCalculated,
    // 0x4113 — joint inverse dynamics torques
    &JointInverseDynamicsTorque,
    &Joint1InverseDynamicsTorque, &Joint2InverseDynamicsTorque,
    &Joint3InverseDynamicsTorque, &Joint4InverseDynamicsTorque,
    &Joint5InverseDynamicsTorque, &Joint6InverseDynamicsTorque,
    &Joint7InverseDynamicsTorque,
    // 0x4114 — joint friction torques
    &JointFrictionTorque,
    &Joint1FrictionTorque, &Joint2FrictionTorque, &Joint3FrictionTorque,
    &Joint4FrictionTorque, &Joint5FrictionTorque, &Joint6FrictionTorque,
    &Joint7FrictionTorque,
    // 0x4115 — lever arm lengths
    &AxisLeverArmLengthTCP0,
    &Axis1LeverArmLengthTCP0, &Axis2LeverArmLengthTCP0,
    &Axis3LeverArmLengthTCP0, &Axis4LeverArmLengthTCP0,
    &Axis5LeverArmLengthTCP0, &Axis6LeverArmLengthTCP0,
    &Axis7LeverArmLengthTCP0,
};

} // namespace RSAPMonitoring
} // namespace NexcobotESC211
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT
