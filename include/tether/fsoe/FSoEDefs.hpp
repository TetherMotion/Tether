/**
 * @file FSoEDefs.hpp
 * @brief FSoE (Fail-Safe over EtherCAT) Definitions
 *
 * Defines constants and structures for the FSoE safety protocol
 * as specified in ETG.5100.
 *
 * FSoE provides a black-channel safety layer over EtherCAT for
 * safety-related communication up to SIL3/PLe.
 */

#pragma once

#include <cstdint>
#include <cstddef>

namespace FSoE {

// ============================================================================
// FSoE Message Types
// ============================================================================

namespace MessageType {
    constexpr uint8_t Data           = 0x00;  // Normal safety data
    constexpr uint8_t SessionReset   = 0x01;  // Session reset request
    constexpr uint8_t Parameter      = 0x02;  // Parameter transfer
    constexpr uint8_t FailSafe       = 0x03;  // Fail-safe command
}

// ============================================================================
// FSoE Command Codes
// ============================================================================

namespace Command {
    constexpr uint8_t ProcessData    = 0x00;  // Normal process data
    constexpr uint8_t Reset          = 0x36;  // Reset command
    constexpr uint8_t Session        = 0x6A;  // Session command
    constexpr uint8_t Connection     = 0x95;  // Connection command
    constexpr uint8_t ParameterReq   = 0xC3;  // Parameter request
    constexpr uint8_t ParameterResp  = 0x3C;  // Parameter response
}

// ============================================================================
// FSoE Connection States
// ============================================================================

namespace ConnectionState {
    constexpr uint8_t Reset          = 0x00;  // Connection reset
    constexpr uint8_t Session        = 0x01;  // Session establishment
    constexpr uint8_t Connection     = 0x02;  // Connection setup
    constexpr uint8_t Parameter      = 0x03;  // Parameter exchange
    constexpr uint8_t Data           = 0x04;  // Data exchange
    constexpr uint8_t FailSafe       = 0x05;  // Fail-safe state
    constexpr uint8_t Error          = 0x06;  // Error state
}

// ============================================================================
// FSoE Error Codes
// ============================================================================

namespace ErrorCode {
    constexpr uint16_t NoError           = 0x0000;
    constexpr uint16_t CommandError      = 0x0001;
    constexpr uint16_t CRCError          = 0x0002;
    constexpr uint16_t WatchdogError     = 0x0003;
    constexpr uint16_t SequenceError     = 0x0004;
    constexpr uint16_t ConnectionIDError = 0x0005;
    constexpr uint16_t DataLengthError   = 0x0006;
    constexpr uint16_t ParameterError    = 0x0007;
    constexpr uint16_t ApplicationError  = 0x0008;
    constexpr uint16_t TimeoutError      = 0x0009;
    constexpr uint16_t UnexpectedData    = 0x000A;
    constexpr uint16_t SessionError      = 0x000B;
    constexpr uint16_t MasterTimeout     = 0x000C;
    constexpr uint16_t SlaveTimeout      = 0x000D;
    constexpr uint16_t StartupError      = 0x000E;
    constexpr uint16_t CommChannelError  = 0x000F;
}

// ============================================================================
// Safety Integrity Levels
// ============================================================================

namespace SIL {
    constexpr uint8_t None   = 0x00;
    constexpr uint8_t SIL1   = 0x01;
    constexpr uint8_t SIL2   = 0x02;
    constexpr uint8_t SIL3   = 0x03;
}

namespace PL {
    constexpr uint8_t None   = 0x00;
    constexpr uint8_t PLa    = 0x01;
    constexpr uint8_t PLb    = 0x02;
    constexpr uint8_t PLc    = 0x03;
    constexpr uint8_t PLd    = 0x04;
    constexpr uint8_t PLe    = 0x05;
}

// ============================================================================
// FSoE Frame Structure
// ============================================================================

#pragma pack(push, 1)

/**
 * @brief FSoE frame header
 */
struct FSoEHeader {
    uint8_t  command;         // FSoE command code
    uint8_t  conn_id_low;     // Connection ID low byte
    uint8_t  conn_id_high;    // Connection ID high byte
    // Followed by safety data and CRC
};

/**
 * @brief FSoE safety data frame (1 byte)
 */
struct FSoEData1 {
    FSoEHeader header;
    uint8_t    data;          // 1 byte safety data
    uint16_t   crc;           // CRC-16
};

/**
 * @brief FSoE safety data frame (2 bytes)
 */
struct FSoEData2 {
    FSoEHeader header;
    uint8_t    data[2];       // 2 bytes safety data
    uint16_t   crc;
};

/**
 * @brief FSoE safety data frame (4 bytes)
 */
struct FSoEData4 {
    FSoEHeader header;
    uint8_t    data[4];       // 4 bytes safety data
    uint16_t   crc;
};

/**
 * @brief FSoE safety data frame (8 bytes)
 */
struct FSoEData8 {
    FSoEHeader header;
    uint8_t    data[8];       // 8 bytes safety data
    uint16_t   crc;
};

/**
 * @brief FSoE safety data frame (16 bytes)
 */
struct FSoEData16 {
    FSoEHeader header;
    uint8_t    data[16];      // 16 bytes safety data
    uint16_t   crc;
};

/**
 * @brief FSoE session reset frame
 */
struct FSoESessionReset {
    FSoEHeader header;
    uint16_t   session_id;
    uint16_t   crc;
};

/**
 * @brief FSoE connection frame
 */
struct FSoEConnectionFrame {
    FSoEHeader header;
    uint16_t   conn_id;
    uint16_t   slave_addr;
    uint8_t    sl_param_crc;
    uint8_t    reserved;
    uint16_t   crc;
};

// Backwards compatibility: old name
// using FSoEConnection = FSoEConnectionFrame;

/**
 * @brief FSoE parameter frame
 */
struct FSoEParameter {
    FSoEHeader header;
    uint16_t   param_id;
    uint32_t   param_value;
    uint16_t   crc;
};

/**
 * @brief FSoE fail-safe values frame
 */
struct FSoEFailSafe {
    FSoEHeader header;
    uint8_t    fail_safe_data[8];  // Fail-safe output values
    uint16_t   crc;
};

#pragma pack(pop)

// ============================================================================
// Safety I/O Types
// ============================================================================

namespace SafeIOType {
    constexpr uint8_t SafeInput1Bit      = 0x01;
    constexpr uint8_t SafeInput8Bit      = 0x02;
    constexpr uint8_t SafeInput16Bit     = 0x03;
    constexpr uint8_t SafeOutput1Bit     = 0x11;
    constexpr uint8_t SafeOutput8Bit     = 0x12;
    constexpr uint8_t SafeOutput16Bit    = 0x13;
    constexpr uint8_t SafeEncoder        = 0x21;
    constexpr uint8_t SafeDrive          = 0x31;
    constexpr uint8_t SafeMotionMonitor  = 0x32;
}

// ============================================================================
// Safe Motion Functions (per IEC 61800-5-2)
// ============================================================================

namespace SafeMotion {
    constexpr uint16_t STO   = 0x0001;  // Safe Torque Off
    constexpr uint16_t SBC   = 0x0002;  // Safe Brake Control
    constexpr uint16_t SS1   = 0x0004;  // Safe Stop 1
    constexpr uint16_t SS2   = 0x0008;  // Safe Stop 2
    constexpr uint16_t SOS   = 0x0010;  // Safe Operating Stop
    constexpr uint16_t SLS   = 0x0020;  // Safely Limited Speed
    constexpr uint16_t SDI   = 0x0040;  // Safe Direction
    constexpr uint16_t SLI   = 0x0080;  // Safely Limited Increment
    constexpr uint16_t SLP   = 0x0100;  // Safely Limited Position
    constexpr uint16_t SLA   = 0x0200;  // Safely Limited Acceleration
    constexpr uint16_t SMS   = 0x0400;  // Safe Maximum Speed
    constexpr uint16_t SCA   = 0x0800;  // Safe Cam
    constexpr uint16_t SMT   = 0x1000;  // Safe Maximum Torque
}

// ============================================================================
// FSoE Configuration Parameters
// ============================================================================

namespace ConfigParam {
    constexpr uint16_t WatchdogTimeout   = 0x0001;  // Watchdog timeout (ms)
    constexpr uint16_t ConnectionTimeout = 0x0002;  // Connection timeout (ms)
    constexpr uint16_t MasterAddress     = 0x0003;  // Master safety address
    constexpr uint16_t SlaveAddress      = 0x0004;  // Slave safety address
    constexpr uint16_t SafetyLevel       = 0x0005;  // SIL/PL level
    constexpr uint16_t InputDataSize     = 0x0006;  // Safety input data size
    constexpr uint16_t OutputDataSize    = 0x0007;  // Safety output data size
    constexpr uint16_t FailSafeValues    = 0x0008;  // Fail-safe output values
    constexpr uint16_t ParameterCRC      = 0x0009;  // Parameter CRC
    constexpr uint16_t ConnectionID      = 0x000A;  // Connection identifier
}

// ============================================================================
// Safety Process Data Objects
// ============================================================================

namespace SafePDO {
    // Input mapping objects (safe inputs from slave)
    constexpr uint16_t SafeInputs1       = 0x7000;  // Safe inputs group 1
    constexpr uint16_t SafeInputs2       = 0x7001;  // Safe inputs group 2
    constexpr uint16_t SafeInputs3       = 0x7002;  // Safe inputs group 3
    constexpr uint16_t SafeInputs4       = 0x7003;  // Safe inputs group 4
    constexpr uint16_t SafeEncoderPos    = 0x7010;  // Safe encoder position
    constexpr uint16_t SafeEncoderVel    = 0x7011;  // Safe encoder velocity
    constexpr uint16_t SafeDriveStatus   = 0x7020;  // Safe drive status
    constexpr uint16_t SafeActualSpeed   = 0x7021;  // Safe actual speed
    constexpr uint16_t SafeActualTorque  = 0x7022;  // Safe actual torque
    
    // Output mapping objects (safe outputs to slave)
    constexpr uint16_t SafeOutputs1      = 0x7100;  // Safe outputs group 1
    constexpr uint16_t SafeOutputs2      = 0x7101;  // Safe outputs group 2
    constexpr uint16_t SafeOutputs3      = 0x7102;  // Safe outputs group 3
    constexpr uint16_t SafeOutputs4      = 0x7103;  // Safe outputs group 4
    constexpr uint16_t SafeDriveControl  = 0x7120;  // Safe drive control
    constexpr uint16_t SafeSpeedLimit    = 0x7121;  // Safe speed limit
    constexpr uint16_t SafeTorqueLimit   = 0x7122;  // Safe torque limit
    
    // Status and control
    constexpr uint16_t FSoEStatus        = 0x7200;  // FSoE connection status
    constexpr uint16_t FSoEControl       = 0x7201;  // FSoE control word
    constexpr uint16_t FSoEError         = 0x7202;  // FSoE error code
    constexpr uint16_t FSoEDiag          = 0x7203;  // FSoE diagnostic info
}

// ============================================================================
// Connection Statistics
// ============================================================================

struct ConnectionStats {
    uint32_t frames_sent;
    uint32_t frames_received;
    uint32_t crc_errors;
    uint32_t sequence_errors;
    uint32_t timeout_events;
    uint32_t reset_events;
    uint32_t watchdog_events;
    uint64_t uptime_ms;
    uint64_t last_comm_time_ms;
};

} // namespace FSoE

namespace EtherCAT {
namespace FSoE = ::FSoE;
}
