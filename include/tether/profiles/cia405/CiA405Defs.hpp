/**
 * @file CiA405Defs.hpp
 * @brief CiA 405 IEC 61131-3 Programmable Device Profile - Object Dictionary
 *
 * Defines the complete object dictionary for CiA 405 profile which covers
 * programmable devices following IEC 61131-3 (PLCs, soft PLCs, etc.)
 *
 * CiA 405 Profile Features:
 * - Program control (start, stop, reset)
 * - Variable access (read/write program variables)
 * - Task management
 * - Exception handling
 * - Communication services
 * - Online debugging support
 */

#pragma once

#include <cstdint>

namespace CiA405 {

// ============================================================================
// Profile Identification
// ============================================================================

constexpr uint16_t PROFILE_NUMBER = 405;

// ============================================================================
// Object Dictionary Indices - Device Control (0x6000-0x60FF)
// ============================================================================

// Program Control
constexpr uint16_t ProgramControl         = 0x6000;
constexpr uint16_t ProgramStatus          = 0x6001;
constexpr uint16_t ProgramError           = 0x6002;
constexpr uint16_t ProgramName            = 0x6003;
constexpr uint16_t ProgramVersion         = 0x6004;
constexpr uint16_t ProgramChecksum        = 0x6005;

// Task Control
constexpr uint16_t TaskControl1           = 0x6010;
constexpr uint16_t TaskControl2           = 0x6011;
constexpr uint16_t TaskControl3           = 0x6012;
constexpr uint16_t TaskControl4           = 0x6013;
constexpr uint16_t TaskStatus1            = 0x6020;
constexpr uint16_t TaskStatus2            = 0x6021;
constexpr uint16_t TaskStatus3            = 0x6022;
constexpr uint16_t TaskStatus4            = 0x6023;

// Task Configuration
constexpr uint16_t TaskPriority1          = 0x6030;
constexpr uint16_t TaskPriority2          = 0x6031;
constexpr uint16_t TaskPriority3          = 0x6032;
constexpr uint16_t TaskPriority4          = 0x6033;
constexpr uint16_t TaskInterval1          = 0x6040;
constexpr uint16_t TaskInterval2          = 0x6041;
constexpr uint16_t TaskInterval3          = 0x6042;
constexpr uint16_t TaskInterval4          = 0x6043;

// Exception Handling
constexpr uint16_t ExceptionStatus        = 0x6050;
constexpr uint16_t ExceptionCode          = 0x6051;
constexpr uint16_t ExceptionInfoIndex      = 0x6052;
constexpr uint16_t ExceptionHistory       = 0x6053;

// System Timing
constexpr uint16_t SystemTime             = 0x6060;
constexpr uint16_t CycleTime              = 0x6061;
constexpr uint16_t MaxCycleTime           = 0x6062;
constexpr uint16_t MinCycleTime           = 0x6063;
constexpr uint16_t WatchdogTime           = 0x6064;

// ============================================================================
// Object Dictionary Indices - Variable Access (0x6100-0x61FF)
// ============================================================================

// Variable Directory
constexpr uint16_t VariableDirectory      = 0x6100;
constexpr uint16_t VariableCount          = 0x6101;
constexpr uint16_t VariableName           = 0x6102;
constexpr uint16_t VariableType           = 0x6103;
constexpr uint16_t VariableAddress        = 0x6104;
constexpr uint16_t VariableSize           = 0x6105;

// Variable Read/Write
constexpr uint16_t VariableReadByIndex    = 0x6110;
constexpr uint16_t VariableWriteByIndex   = 0x6111;
constexpr uint16_t VariableReadByName     = 0x6112;
constexpr uint16_t VariableWriteByName    = 0x6113;

// Bulk Variable Access
constexpr uint16_t BulkVariableRead       = 0x6120;
constexpr uint16_t BulkVariableWrite      = 0x6121;
constexpr uint16_t BulkVariableMapping    = 0x6122;

// Input Variables (Memory Area %I)
constexpr uint16_t InputVariables         = 0x6130;
constexpr uint16_t InputByte              = 0x6131;
constexpr uint16_t InputWord              = 0x6132;
constexpr uint16_t InputDWord             = 0x6133;

// Output Variables (Memory Area %Q)
constexpr uint16_t OutputVariables        = 0x6140;
constexpr uint16_t OutputByte             = 0x6141;
constexpr uint16_t OutputWord             = 0x6142;
constexpr uint16_t OutputDWord            = 0x6143;

// Memory Variables (Memory Area %M)
constexpr uint16_t MemoryVariables        = 0x6150;
constexpr uint16_t MemoryByte             = 0x6151;
constexpr uint16_t MemoryWord             = 0x6152;
constexpr uint16_t MemoryDWord            = 0x6153;

// ============================================================================
// Object Dictionary Indices - Communication (0x6200-0x62FF)
// ============================================================================

// Communication Parameters
constexpr uint16_t CommParameters         = 0x6200;
constexpr uint16_t CommStatus             = 0x6201;
constexpr uint16_t CommError              = 0x6202;

// PDO Communication
constexpr uint16_t PDOCommunication1      = 0x6210;
constexpr uint16_t PDOCommunication2      = 0x6211;
constexpr uint16_t PDOCommunication3      = 0x6212;
constexpr uint16_t PDOCommunication4      = 0x6213;

// SDO Communication
constexpr uint16_t SDOCommunication       = 0x6220;
constexpr uint16_t SDOTimeout             = 0x6221;
constexpr uint16_t SDORetries             = 0x6222;

// ============================================================================
// Object Dictionary Indices - Debug Interface (0x6300-0x63FF)
// ============================================================================

// Online Debugging
constexpr uint16_t DebugControl           = 0x6300;
constexpr uint16_t DebugStatus            = 0x6301;
constexpr uint16_t BreakpointEnable       = 0x6302;
constexpr uint16_t BreakpointAddress      = 0x6303;
constexpr uint16_t BreakpointCondition    = 0x6304;

// Single Step
constexpr uint16_t SingleStepControl      = 0x6310;
constexpr uint16_t StepMode               = 0x6311;
constexpr uint16_t CurrentPOU             = 0x6312;
constexpr uint16_t CurrentLine            = 0x6313;

// Watch Variables
constexpr uint16_t WatchVariable1         = 0x6320;
constexpr uint16_t WatchVariable2         = 0x6321;
constexpr uint16_t WatchVariable3         = 0x6322;
constexpr uint16_t WatchVariable4         = 0x6323;
constexpr uint16_t WatchVariable5         = 0x6324;
constexpr uint16_t WatchVariable6         = 0x6325;
constexpr uint16_t WatchVariable7         = 0x6326;
constexpr uint16_t WatchVariable8         = 0x6327;

// Trace/Recording
constexpr uint16_t TraceControl           = 0x6330;
constexpr uint16_t TraceTrigger           = 0x6331;
constexpr uint16_t TraceBuffer            = 0x6332;
constexpr uint16_t TraceSampleRate        = 0x6333;

// ============================================================================
// Object Dictionary Indices - Resource Management (0x6400-0x64FF)
// ============================================================================

// Memory Management
constexpr uint16_t MemoryTotal            = 0x6400;
constexpr uint16_t MemoryUsed             = 0x6401;
constexpr uint16_t MemoryFree             = 0x6402;
constexpr uint16_t StackSize              = 0x6403;
constexpr uint16_t StackUsed              = 0x6404;

// CPU Information
constexpr uint16_t CPULoad                = 0x6410;
constexpr uint16_t CPULoadMax             = 0x6411;
constexpr uint16_t CPUTemperature         = 0x6412;

// Resource Configuration
constexpr uint16_t ResourceList           = 0x6420;
constexpr uint16_t ResourceStatus         = 0x6421;

// ============================================================================
// Object Dictionary Indices - File Transfer (0x6500-0x65FF)
// ============================================================================

constexpr uint16_t FileTransferControl    = 0x6500;
constexpr uint16_t FileTransferStatusIndex = 0x6501;
constexpr uint16_t FileTransferProgress   = 0x6502;
constexpr uint16_t FileName               = 0x6503;
constexpr uint16_t FileSize               = 0x6504;
constexpr uint16_t FileData               = 0x6505;
constexpr uint16_t FileChecksum           = 0x6506;

// ============================================================================
// Subindex Definitions
// ============================================================================

namespace ProgControlSub {
    constexpr uint8_t NumberOfEntries     = 0x00;
    constexpr uint8_t Command             = 0x01;
    constexpr uint8_t State               = 0x02;
    constexpr uint8_t Error               = 0x03;
}

namespace TaskControlSub {
    constexpr uint8_t NumberOfEntries     = 0x00;
    constexpr uint8_t Command             = 0x01;
    constexpr uint8_t Status              = 0x02;
    constexpr uint8_t Priority            = 0x03;
    constexpr uint8_t Interval            = 0x04;
    constexpr uint8_t WatchdogTime        = 0x05;
}

namespace VariableSub {
    constexpr uint8_t NumberOfEntries     = 0x00;
    constexpr uint8_t Index               = 0x01;
    constexpr uint8_t Name                = 0x02;
    constexpr uint8_t DataType            = 0x03;
    constexpr uint8_t Value               = 0x04;
    constexpr uint8_t Address             = 0x05;
    constexpr uint8_t Size                = 0x06;
}

namespace DebugSub {
    constexpr uint8_t NumberOfEntries     = 0x00;
    constexpr uint8_t Enable              = 0x01;
    constexpr uint8_t Address             = 0x02;
    constexpr uint8_t Condition           = 0x03;
    constexpr uint8_t HitCount            = 0x04;
}

// ============================================================================
// Command Definitions
// ============================================================================

namespace ProgramCommands {
    constexpr uint8_t NoOp                = 0x00;
    constexpr uint8_t Start               = 0x01;
    constexpr uint8_t Stop                = 0x02;
    constexpr uint8_t Reset               = 0x03;
    constexpr uint8_t Halt                = 0x04;
    constexpr uint8_t Continue            = 0x05;
    constexpr uint8_t ColdStart           = 0x10;
    constexpr uint8_t WarmStart           = 0x11;
    constexpr uint8_t HotStart            = 0x12;
    constexpr uint8_t Download            = 0x20;
    constexpr uint8_t Upload              = 0x21;
    constexpr uint8_t Verify              = 0x22;
}

namespace TaskCommands {
    constexpr uint8_t NoOp                = 0x00;
    constexpr uint8_t Start               = 0x01;
    constexpr uint8_t Stop                = 0x02;
    constexpr uint8_t Suspend             = 0x03;
    constexpr uint8_t Resume              = 0x04;
    constexpr uint8_t SingleCycle         = 0x05;
}

namespace DebugCommands {
    constexpr uint8_t NoOp                = 0x00;
    constexpr uint8_t SetBreakpoint       = 0x01;
    constexpr uint8_t ClearBreakpoint     = 0x02;
    constexpr uint8_t ClearAll            = 0x03;
    constexpr uint8_t EnableBreakpoints   = 0x04;
    constexpr uint8_t DisableBreakpoints  = 0x05;
}

namespace StepCommands {
    constexpr uint8_t NoOp                = 0x00;
    constexpr uint8_t StepInto            = 0x01;
    constexpr uint8_t StepOver            = 0x02;
    constexpr uint8_t StepOut             = 0x03;
    constexpr uint8_t RunToCursor         = 0x04;
}

namespace FileCommands {
    constexpr uint8_t NoOp                = 0x00;
    constexpr uint8_t StartDownload       = 0x01;
    constexpr uint8_t StartUpload         = 0x02;
    constexpr uint8_t WriteBlock          = 0x03;
    constexpr uint8_t ReadBlock           = 0x04;
    constexpr uint8_t EndTransfer         = 0x05;
    constexpr uint8_t Abort               = 0x06;
    constexpr uint8_t Delete              = 0x07;
}

// ============================================================================
// Status Definitions
// ============================================================================

namespace ProgramState {
    constexpr uint8_t Unknown             = 0x00;
    constexpr uint8_t Stopped             = 0x01;
    constexpr uint8_t Running             = 0x02;
    constexpr uint8_t Halted              = 0x03;
    constexpr uint8_t Exception           = 0x04;
    constexpr uint8_t Downloading         = 0x05;
    constexpr uint8_t Uploading           = 0x06;
    constexpr uint8_t Debug               = 0x07;
    constexpr uint8_t SingleStep          = 0x08;
    constexpr uint8_t BreakPoint          = 0x09;
}

namespace TaskState {
    constexpr uint8_t Unknown             = 0x00;
    constexpr uint8_t Stopped             = 0x01;
    constexpr uint8_t Running             = 0x02;
    constexpr uint8_t Suspended           = 0x03;
    constexpr uint8_t Exception           = 0x04;
    constexpr uint8_t Waiting             = 0x05;
    constexpr uint8_t Ready               = 0x06;
}

namespace ExceptionCodes {
    constexpr uint16_t None               = 0x0000;
    constexpr uint16_t DivisionByZero     = 0x0001;
    constexpr uint16_t Overflow           = 0x0002;
    constexpr uint16_t ArrayBounds        = 0x0003;
    constexpr uint16_t InvalidPointer     = 0x0004;
    constexpr uint16_t StackOverflow      = 0x0005;
    constexpr uint16_t OutOfMemory        = 0x0006;
    constexpr uint16_t Watchdog           = 0x0007;
    constexpr uint16_t CycleOverrun       = 0x0008;
    constexpr uint16_t IOError            = 0x0009;
    constexpr uint16_t CommunicationError = 0x000A;
    constexpr uint16_t UserException      = 0x0100;
}

namespace FileTransferState {
    constexpr uint8_t Idle                = 0x00;
    constexpr uint8_t Downloading         = 0x01;
    constexpr uint8_t Uploading           = 0x02;
    constexpr uint8_t Complete            = 0x03;
    constexpr uint8_t Error               = 0x04;
}

// ============================================================================
// IEC 61131-3 Data Types
// ============================================================================

namespace IEC61131Types {
    constexpr uint8_t BOOL                = 0x01;
    constexpr uint8_t SINT                = 0x02;
    constexpr uint8_t INT                 = 0x03;
    constexpr uint8_t DINT                = 0x04;
    constexpr uint8_t LINT                = 0x05;
    constexpr uint8_t USINT               = 0x06;
    constexpr uint8_t UINT                = 0x07;
    constexpr uint8_t UDINT               = 0x08;
    constexpr uint8_t ULINT               = 0x09;
    constexpr uint8_t REAL                = 0x0A;
    constexpr uint8_t LREAL               = 0x0B;
    constexpr uint8_t TIME                = 0x0C;
    constexpr uint8_t DATE                = 0x0D;
    constexpr uint8_t TOD                 = 0x0E;
    constexpr uint8_t DT                  = 0x0F;
    constexpr uint8_t STRING              = 0x10;
    constexpr uint8_t WSTRING             = 0x11;
    constexpr uint8_t BYTE                = 0x12;
    constexpr uint8_t WORD                = 0x13;
    constexpr uint8_t DWORD               = 0x14;
    constexpr uint8_t LWORD               = 0x15;
    constexpr uint8_t ARRAY               = 0x80;
    constexpr uint8_t STRUCT              = 0x81;
    constexpr uint8_t FB_INSTANCE         = 0x82;
}

// ============================================================================
// PDO Structures
// ============================================================================

#pragma pack(push, 1)

// Basic status PDO
struct TxPDO_Status {
    uint8_t  program_state;
    uint8_t  task1_state;
    uint8_t  task2_state;
    uint8_t  task3_state;
    uint8_t  task4_state;
    uint16_t exception_code;
    uint8_t  reserved;
};
static_assert(sizeof(TxPDO_Status) == 8, "TxPDO_Status size mismatch");

// Program control PDO
struct RxPDO_Control {
    uint8_t  program_command;
    uint8_t  task1_command;
    uint8_t  task2_command;
    uint8_t  task3_command;
    uint8_t  task4_command;
    uint8_t  reserved[3];
};
static_assert(sizeof(RxPDO_Control) == 8, "RxPDO_Control size mismatch");

// Variable access PDO (request)
struct RxPDO_VariableRequest {
    uint16_t var_index;
    uint8_t  operation;  // 0=read, 1=write
    uint8_t  reserved;
    uint32_t write_value;
};
static_assert(sizeof(RxPDO_VariableRequest) == 8, "RxPDO_VariableRequest size mismatch");

// Variable access PDO (response)
struct TxPDO_VariableResponse {
    uint16_t var_index;
    uint8_t  status;
    uint8_t  data_type;
    uint32_t value;
};
static_assert(sizeof(TxPDO_VariableResponse) == 8, "TxPDO_VariableResponse size mismatch");

// Timing PDO
struct TxPDO_Timing {
    uint32_t system_time;      // System time in ms
    uint16_t cycle_time;       // Current cycle time in µs
    uint16_t max_cycle_time;   // Max cycle time in µs
};
static_assert(sizeof(TxPDO_Timing) == 8, "TxPDO_Timing size mismatch");

// Debug PDO
struct TxPDO_Debug {
    uint32_t current_address;
    uint16_t current_line;
    uint8_t  debug_state;
    uint8_t  breakpoint_hit;
};
static_assert(sizeof(TxPDO_Debug) == 8, "TxPDO_Debug size mismatch");

// Resource usage PDO
struct TxPDO_Resources {
    uint16_t cpu_load;         // CPU load in 0.01%
    uint16_t memory_used;      // Memory usage in KB
    uint16_t stack_used;       // Stack usage in bytes
    uint16_t reserved;
};
static_assert(sizeof(TxPDO_Resources) == 8, "TxPDO_Resources size mismatch");

// Extended status PDO with I/O mapping
struct TxPDO_ExtendedStatus {
    uint8_t  program_state;
    uint8_t  exception_active;
    uint16_t cycle_time_us;
    uint32_t input_image[8];   // 256 bits of inputs
};

// I/O update PDO
struct RxPDO_IOUpdate {
    uint32_t output_image[8];  // 256 bits of outputs
};

#pragma pack(pop)

// ============================================================================
// Helper Functions
// ============================================================================

inline const char* getProgramStateName(uint8_t state) {
    switch (state) {
        case ProgramState::Stopped:     return "Stopped";
        case ProgramState::Running:     return "Running";
        case ProgramState::Halted:      return "Halted";
        case ProgramState::Exception:   return "Exception";
        case ProgramState::Downloading: return "Downloading";
        case ProgramState::Uploading:   return "Uploading";
        case ProgramState::Debug:       return "Debug";
        case ProgramState::SingleStep:  return "SingleStep";
        case ProgramState::BreakPoint:  return "Breakpoint";
        default:                        return "Unknown";
    }
}

inline const char* getTaskStateName(uint8_t state) {
    switch (state) {
        case TaskState::Stopped:   return "Stopped";
        case TaskState::Running:   return "Running";
        case TaskState::Suspended: return "Suspended";
        case TaskState::Exception: return "Exception";
        case TaskState::Waiting:   return "Waiting";
        case TaskState::Ready:     return "Ready";
        default:                   return "Unknown";
    }
}

inline const char* getExceptionName(uint16_t code) {
    switch (code) {
        case ExceptionCodes::None:               return "None";
        case ExceptionCodes::DivisionByZero:     return "Division by Zero";
        case ExceptionCodes::Overflow:           return "Overflow";
        case ExceptionCodes::ArrayBounds:        return "Array Bounds";
        case ExceptionCodes::InvalidPointer:     return "Invalid Pointer";
        case ExceptionCodes::StackOverflow:      return "Stack Overflow";
        case ExceptionCodes::OutOfMemory:        return "Out of Memory";
        case ExceptionCodes::Watchdog:           return "Watchdog";
        case ExceptionCodes::CycleOverrun:       return "Cycle Overrun";
        case ExceptionCodes::IOError:            return "I/O Error";
        case ExceptionCodes::CommunicationError: return "Communication Error";
        default:
            if (code >= ExceptionCodes::UserException) return "User Exception";
            return "Unknown";
    }
}

inline const char* getTypeName(uint8_t type) {
    switch (type) {
        case IEC61131Types::BOOL:   return "BOOL";
        case IEC61131Types::SINT:   return "SINT";
        case IEC61131Types::INT:    return "INT";
        case IEC61131Types::DINT:   return "DINT";
        case IEC61131Types::LINT:   return "LINT";
        case IEC61131Types::USINT:  return "USINT";
        case IEC61131Types::UINT:   return "UINT";
        case IEC61131Types::UDINT:  return "UDINT";
        case IEC61131Types::ULINT:  return "ULINT";
        case IEC61131Types::REAL:   return "REAL";
        case IEC61131Types::LREAL:  return "LREAL";
        case IEC61131Types::TIME:   return "TIME";
        case IEC61131Types::DATE:   return "DATE";
        case IEC61131Types::STRING: return "STRING";
        case IEC61131Types::BYTE:   return "BYTE";
        case IEC61131Types::WORD:   return "WORD";
        case IEC61131Types::DWORD:  return "DWORD";
        case IEC61131Types::LWORD:  return "LWORD";
        case IEC61131Types::ARRAY:  return "ARRAY";
        case IEC61131Types::STRUCT: return "STRUCT";
        default:                    return "Unknown";
    }
}

inline uint8_t getTypeSize(uint8_t type) {
    switch (type) {
        case IEC61131Types::BOOL:
        case IEC61131Types::SINT:
        case IEC61131Types::USINT:
        case IEC61131Types::BYTE:   return 1;
        
        case IEC61131Types::INT:
        case IEC61131Types::UINT:
        case IEC61131Types::WORD:   return 2;
        
        case IEC61131Types::DINT:
        case IEC61131Types::UDINT:
        case IEC61131Types::REAL:
        case IEC61131Types::TIME:
        case IEC61131Types::DATE:
        case IEC61131Types::TOD:
        case IEC61131Types::DWORD:  return 4;
        
        case IEC61131Types::LINT:
        case IEC61131Types::ULINT:
        case IEC61131Types::LREAL:
        case IEC61131Types::DT:
        case IEC61131Types::LWORD:  return 8;
        
        default:                    return 0;  // Variable size
    }
}

} // namespace CiA405
