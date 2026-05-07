/**
 * @file CiA417Defs.hpp
 * @brief CiA 417 Lift Controller Device Profile Object Dictionary
 *
 * Defines object dictionary entries for CiA 417 compliant lift/elevator
 * controllers including safety systems, position tracking, and door control.
 *
 * Features:
 * - Cabin position and speed control
 * - Door control (main and counter doors)
 * - Safety systems (overspeed, buffer, UCM)
 * - Floor management and call handling
 * - Load weighing integration
 * - Inspection and emergency modes
 */

#pragma once

#include <cstdint>

namespace CiA417 {

// ============================================================================
// Profile Identification
// ============================================================================

constexpr uint16_t PROFILE_NUMBER = 417;
constexpr uint16_t PROFILE_VERSION = 0x0200;

// ============================================================================
// Object Dictionary Indices
// ============================================================================

namespace Object {

// Lift Identification (0x6000-0x600F)
constexpr uint16_t LiftType           = 0x6000;
constexpr uint16_t NumberOfFloors     = 0x6001;
constexpr uint16_t NominalSpeed       = 0x6002;
constexpr uint16_t NominalLoad        = 0x6003;
constexpr uint16_t NumberOfDoors      = 0x6004;
constexpr uint16_t DriveType          = 0x6005;
constexpr uint16_t SafetyClass        = 0x6006;

// Position (0x6010-0x601F)
constexpr uint16_t PositionActual     = 0x6010;
constexpr uint16_t PositionSetpoint   = 0x6011;
constexpr uint16_t FloorActual        = 0x6012;
constexpr uint16_t FloorTarget        = 0x6013;
constexpr uint16_t FloorHeightTable   = 0x6014;  // Array
constexpr uint16_t LevelingZone       = 0x6015;
constexpr uint16_t DoorZone           = 0x6016;
constexpr uint16_t RelevellngRange    = 0x6017;

// Velocity (0x6020-0x602F)
constexpr uint16_t VelocityActual     = 0x6020;
constexpr uint16_t VelocitySetpoint   = 0x6021;
constexpr uint16_t VelocityMax        = 0x6022;
constexpr uint16_t AccelerationMax    = 0x6023;
constexpr uint16_t JerkMax            = 0x6024;
constexpr uint16_t LevelingSpeed      = 0x6025;
constexpr uint16_t InspectionSpeed    = 0x6026;

// Control/Status (0x6030-0x603F)
constexpr uint16_t Statusword         = 0x6030;
constexpr uint16_t Controlword        = 0x6031;
constexpr uint16_t OperatingMode      = 0x6032;
constexpr uint16_t MotionState        = 0x6033;
constexpr uint16_t DoorState          = 0x6034;
constexpr uint16_t SafetyState        = 0x6035;

// Door Control (0x6040-0x604F)
constexpr uint16_t DoorCommand        = 0x6040;
constexpr uint16_t DoorStatusMain     = 0x6041;
constexpr uint16_t DoorStatusCounter  = 0x6042;
constexpr uint16_t DoorOpenTime       = 0x6043;
constexpr uint16_t DoorCloseTime      = 0x6044;
constexpr uint16_t DoorNudgingDelay   = 0x6045;
constexpr uint16_t DoorReopenCount    = 0x6046;
constexpr uint16_t DoorForce          = 0x6047;
constexpr uint16_t PhotoeyeStatus     = 0x6048;
constexpr uint16_t SafetyEdgeStatus   = 0x6049;

// Call Management (0x6050-0x605F)
constexpr uint16_t CarCallsUp         = 0x6050;  // Bitmask
constexpr uint16_t CarCallsDown       = 0x6051;
constexpr uint16_t HallCallsUp        = 0x6052;
constexpr uint16_t HallCallsDown      = 0x6053;
constexpr uint16_t NextStop           = 0x6054;
constexpr uint16_t Direction          = 0x6055;
constexpr uint16_t FloorLocked        = 0x6056;  // Bitmask
constexpr uint16_t FloorEnabled       = 0x6057;

// Load Weighing (0x6060-0x606F)
constexpr uint16_t LoadActual         = 0x6060;
constexpr uint16_t LoadCapacity       = 0x6061;
constexpr uint16_t OverloadThreshold  = 0x6062;
constexpr uint16_t BypassThreshold    = 0x6063;
constexpr uint16_t LoadCompensation   = 0x6064;
constexpr uint16_t PassengerCount     = 0x6065;

// Safety Systems (0x6070-0x607F)
constexpr uint16_t SafetyInputs       = 0x6070;
constexpr uint16_t SafetyOutputs      = 0x6071;
constexpr uint16_t OverspeedStatus    = 0x6072;
constexpr uint16_t BufferStatus       = 0x6073;
constexpr uint16_t UCMStatus          = 0x6074;  // Unintended Car Movement
constexpr uint16_t RopeSlackStatus    = 0x6075;
constexpr uint16_t EmergencyStop      = 0x6076;
constexpr uint16_t SafetyCircuit      = 0x6077;
constexpr uint16_t GovernorTripped    = 0x6078;
constexpr uint16_t FinalLimits        = 0x6079;

// Brake Control (0x6080-0x608F)
constexpr uint16_t BrakeCommand       = 0x6080;
constexpr uint16_t BrakeStatus        = 0x6081;
constexpr uint16_t BrakeMonitoring    = 0x6082;
constexpr uint16_t BrakeWearStatus    = 0x6083;
constexpr uint16_t BrakeTestResult    = 0x6084;
constexpr uint16_t ParkingBrake       = 0x6085;

// Inspection Mode (0x6090-0x609F)
constexpr uint16_t InspectionStatus   = 0x6090;
constexpr uint16_t InspectionControl  = 0x6091;
constexpr uint16_t CarTopControl      = 0x6092;
constexpr uint16_t PitControl         = 0x6093;
constexpr uint16_t RescueOperation    = 0x6094;

// Diagnostics (0x60A0-0x60AF)
constexpr uint16_t FaultCode          = 0x60A0;
constexpr uint16_t FaultHistory       = 0x60A1;  // Array
constexpr uint16_t WarningCode        = 0x60A2;
constexpr uint16_t TripCount          = 0x60A3;
constexpr uint16_t OperatingHours     = 0x60A4;
constexpr uint16_t DoorCycles         = 0x60A5;
constexpr uint16_t LastMaintenance    = 0x60A6;
constexpr uint16_t EnergyConsumption  = 0x60A7;
constexpr uint16_t DriveTemperature   = 0x60A8;
constexpr uint16_t MotorTemperature   = 0x60A9;

// Display/Communication (0x60B0-0x60BF)
constexpr uint16_t DisplayFloor       = 0x60B0;
constexpr uint16_t DisplayDirection   = 0x60B1;
constexpr uint16_t DisplayMessage     = 0x60B2;
constexpr uint16_t VoiceAnnouncement  = 0x60B3;
constexpr uint16_t EmergencyIntercom  = 0x60B4;

} // namespace Object

// ============================================================================
// Statusword Bits
// ============================================================================

namespace StatuswordBits {
constexpr uint32_t Ready              = 0x00000001;
constexpr uint32_t Running            = 0x00000002;
constexpr uint32_t AtFloor            = 0x00000004;
constexpr uint32_t InLevelingZone     = 0x00000008;
constexpr uint32_t InDoorZone         = 0x00000010;
constexpr uint32_t DoorsOpen          = 0x00000020;
constexpr uint32_t DoorsClosed        = 0x00000040;
constexpr uint32_t DirectionUp        = 0x00000080;
constexpr uint32_t DirectionDown      = 0x00000100;
constexpr uint32_t Overloaded         = 0x00000200;
constexpr uint32_t SafetyOK           = 0x00000400;
constexpr uint32_t BrakesApplied      = 0x00000800;
constexpr uint32_t InspectionMode     = 0x00001000;
constexpr uint32_t EmergencyMode      = 0x00002000;
constexpr uint32_t OutOfService       = 0x00004000;
constexpr uint32_t Fault              = 0x00008000;
constexpr uint32_t Warning            = 0x00010000;
constexpr uint32_t FireService        = 0x00020000;
constexpr uint32_t EarthquakeMode     = 0x00040000;
constexpr uint32_t RescueActive       = 0x00080000;
} // namespace StatuswordBits

// ============================================================================
// Controlword Bits
// ============================================================================

namespace ControlwordBits {
constexpr uint32_t Enable             = 0x00000001;
constexpr uint32_t Start              = 0x00000002;
constexpr uint32_t Stop               = 0x00000004;
constexpr uint32_t GoUp               = 0x00000008;
constexpr uint32_t GoDown             = 0x00000010;
constexpr uint32_t OpenDoor           = 0x00000020;
constexpr uint32_t CloseDoor          = 0x00000040;
constexpr uint32_t ResetFault         = 0x00000080;
constexpr uint32_t EnableBypass       = 0x00000100;
constexpr uint32_t InspectionEnable   = 0x00000200;
constexpr uint32_t RescueMode         = 0x00000400;
constexpr uint32_t FireServiceMode    = 0x00000800;
constexpr uint32_t RelevelEnable      = 0x00001000;
} // namespace ControlwordBits

// ============================================================================
// Operating Modes
// ============================================================================

namespace OperatingMode {
constexpr uint8_t Normal              = 0x00;
constexpr uint8_t Inspection          = 0x01;
constexpr uint8_t FireService         = 0x02;
constexpr uint8_t Emergency           = 0x03;
constexpr uint8_t Rescue              = 0x04;
constexpr uint8_t Independent         = 0x05;
constexpr uint8_t Attendant           = 0x06;
constexpr uint8_t Earthquake          = 0x07;
constexpr uint8_t Parking             = 0x08;
constexpr uint8_t Maintenance         = 0x09;
constexpr uint8_t CarTopControl       = 0x0A;
constexpr uint8_t PitControl          = 0x0B;
} // namespace OperatingMode

// ============================================================================
// Motion States
// ============================================================================

namespace MotionState {
constexpr uint8_t Stopped             = 0x00;
constexpr uint8_t Accelerating        = 0x01;
constexpr uint8_t FullSpeed           = 0x02;
constexpr uint8_t Decelerating        = 0x03;
constexpr uint8_t Leveling            = 0x04;
constexpr uint8_t Releveling          = 0x05;
constexpr uint8_t MovingUp            = 0x10;
constexpr uint8_t MovingDown          = 0x20;
} // namespace MotionState

// ============================================================================
// Door States
// ============================================================================

namespace DoorState {
constexpr uint8_t FullyClosed         = 0x00;
constexpr uint8_t Opening             = 0x01;
constexpr uint8_t FullyOpen           = 0x02;
constexpr uint8_t Closing             = 0x03;
constexpr uint8_t Nudging             = 0x04;
constexpr uint8_t Blocked             = 0x05;
constexpr uint8_t Fault               = 0x06;
constexpr uint8_t Locked              = 0x07;
} // namespace DoorState

// ============================================================================
// Door Commands
// ============================================================================

namespace DoorCommand {
constexpr uint8_t None                = 0x00;
constexpr uint8_t Open                = 0x01;
constexpr uint8_t Close               = 0x02;
constexpr uint8_t Stop                = 0x03;
constexpr uint8_t Nudge               = 0x04;
constexpr uint8_t Lock                = 0x05;
constexpr uint8_t Unlock              = 0x06;
constexpr uint8_t ForceClose          = 0x07;  // Fire service
} // namespace DoorCommand

// ============================================================================
// Direction
// ============================================================================

namespace Direction {
constexpr uint8_t None                = 0x00;
constexpr uint8_t Up                  = 0x01;
constexpr uint8_t Down                = 0x02;
} // namespace Direction

// ============================================================================
// Safety Input Bits
// ============================================================================

namespace SafetyInputBits {
constexpr uint32_t GovernorOK         = 0x00000001;
constexpr uint32_t SafetyCircuitOK    = 0x00000002;
constexpr uint32_t DoorClosedMain     = 0x00000004;
constexpr uint32_t DoorClosedCounter  = 0x00000008;
constexpr uint32_t DoorLockMain       = 0x00000010;
constexpr uint32_t DoorLockCounter    = 0x00000020;
constexpr uint32_t UpperFinalLimit    = 0x00000040;
constexpr uint32_t LowerFinalLimit    = 0x00000080;
constexpr uint32_t BufferSwitch       = 0x00000100;
constexpr uint32_t EmergencyStopCar   = 0x00000200;
constexpr uint32_t EmergencyStopMR    = 0x00000400;
constexpr uint32_t EmergencyStopPit   = 0x00000800;
constexpr uint32_t PitSwitch          = 0x00001000;
constexpr uint32_t CarTopSwitch       = 0x00002000;
constexpr uint32_t RopeSlack          = 0x00004000;
constexpr uint32_t CounterweightScreen = 0x00008000;
} // namespace SafetyInputBits

// ============================================================================
// Fault Codes
// ============================================================================

namespace FaultCode {
constexpr uint16_t NoFault            = 0x0000;
constexpr uint16_t DriveFailure       = 0x0001;
constexpr uint16_t BrakeFailure       = 0x0002;
constexpr uint16_t DoorFailure        = 0x0003;
constexpr uint16_t SafetyCircuitOpen  = 0x0004;
constexpr uint16_t GovernorTrip       = 0x0005;
constexpr uint16_t Overspeed          = 0x0006;
constexpr uint16_t UCMDetected        = 0x0007;
constexpr uint16_t PositionLost       = 0x0008;
constexpr uint16_t OverTemperature    = 0x0009;
constexpr uint16_t Overload           = 0x000A;
constexpr uint16_t CommunicationError = 0x000B;
constexpr uint16_t EncoderFailure     = 0x000C;
constexpr uint16_t RopeSlackDetected  = 0x000D;
constexpr uint16_t LevelingTimeout    = 0x000E;
constexpr uint16_t DoorTimeout        = 0x000F;
constexpr uint16_t InverterFault      = 0x0010;
} // namespace FaultCode

// ============================================================================
// Unit Conversions
// ============================================================================

// Position in mm
inline float rawToMM(int32_t raw) { return static_cast<float>(raw); }
inline int32_t mmToRaw(float mm) { return static_cast<int32_t>(mm); }

// Velocity in mm/s
inline float rawToMMPerSec(int16_t raw) { return static_cast<float>(raw); }
inline int16_t mmPerSecToRaw(float v) { return static_cast<int16_t>(v); }

// Load in kg
inline float rawToKg(uint16_t raw) { return static_cast<float>(raw); }
inline uint16_t kgToRaw(float kg) { return static_cast<uint16_t>(kg); }

// Temperature in 0.1°C
inline float rawToTempC(int16_t raw) { return raw / 10.0f; }

// ============================================================================
// PDO Structures
// ============================================================================

#pragma pack(push, 1)

// Basic status TxPDO
struct TxPDO_Basic {
    uint32_t statusword;
    int32_t  position_actual;  // mm
    uint8_t  floor_actual;
    uint8_t  door_state;
};

// Extended TxPDO
struct TxPDO_Extended {
    uint32_t statusword;
    int32_t  position_actual;
    int16_t  velocity_actual;  // mm/s
    uint8_t  floor_actual;
    uint8_t  floor_target;
    uint8_t  door_state_main;
    uint8_t  door_state_counter;
    uint16_t load_actual;      // kg
    uint32_t safety_inputs;
};

// Full TxPDO
struct TxPDO_Full {
    uint32_t statusword;
    int32_t  position_actual;
    int16_t  velocity_actual;
    uint8_t  floor_actual;
    uint8_t  floor_target;
    uint8_t  motion_state;
    uint8_t  operating_mode;
    uint8_t  door_state_main;
    uint8_t  door_state_counter;
    uint16_t load_actual;
    uint32_t safety_inputs;
    uint64_t car_calls;       // Floor bitmask
    uint64_t hall_calls_up;
    uint64_t hall_calls_down;
    uint16_t fault_code;
    int16_t  drive_temperature;
};

// Basic control RxPDO
struct RxPDO_Basic {
    uint32_t controlword;
    uint8_t  floor_target;
    uint8_t  door_command;
};

// Extended control RxPDO
struct RxPDO_Extended {
    uint32_t controlword;
    uint8_t  floor_target;
    uint8_t  door_command;
    uint8_t  operating_mode;
    int16_t  velocity_setpoint;
    uint64_t car_call_clear;  // Bitmask to clear
};

#pragma pack(pop)

} // namespace CiA417
