#pragma once

#include <cstdint>
#include <vector>

namespace EtherCAT {
namespace Drives {
namespace ErrorCodes {
namespace NexcobotESC211 {

// Error categories
enum class ErrorCategory : uint8_t {
    None = 0,
    RSAP = 1,
    CONFIG = 2,
    SAFETY = 3,
    SYSMGR = 4,
    FSoE = 5
};

const char* ErrorCategoryToString(ErrorCategory category);

// Error code entry structure (similar to register entries)
struct ErrorEntry {
    int32_t code;
    ErrorCategory category;
    const char* name;
    const char* description;
    const char* error_handling;
    bool is_recoverable;
};

using ErrorEntryPtr = const ErrorEntry*;
using ErrorList = std::vector<ErrorEntryPtr>;

// Parsed error structure
struct NexcobotESC211Error {
    int32_t raw_code = 0;
    ErrorCategory category = ErrorCategory::None;
    bool is_recoverable = false;
    const char* name = "NoError";
    const char* description = "No error";
    const char* error_handling = "No action required";

    static NexcobotESC211Error parse(int32_t raw_code);
    static const ErrorEntry* findEntry(int32_t raw_code);
};

// ---- RSAP Error Codes (0 to -17) -----------------------------------
constexpr ErrorEntry NoError = {
    .code = 0,
    .category = ErrorCategory::None,
    .name = "NoError",
    .description = "The operation completed successfully.",
    .error_handling = "No action required.",
    .is_recoverable = false
};

constexpr ErrorEntry RSAP_StateInvalid = {
    .code = -1,
    .category = ErrorCategory::RSAP,
    .name = "RSAP_StateInvalid",
    .description = "In current state, the system cannot accept the operation.",
    .error_handling = "Check RSAP state machine; retry in valid state.",
    .is_recoverable = true
};

constexpr ErrorEntry RSAP_UnexpectedException = {
    .code = -2,
    .category = ErrorCategory::RSAP,
    .name = "RSAP_UnexpectedException",
    .description = "The operation occurred unexpected exception.",
    .error_handling = "Reboot your device. If the error persists, please contact us.",
    .is_recoverable = true
};

constexpr ErrorEntry RSAP_NotReady = {
    .code = -3,
    .category = ErrorCategory::RSAP,
    .name = "RSAP_NotReady",
    .description = "The system is not ready.",
    .error_handling = "Wait for initialization complete; retry later.",
    .is_recoverable = true
};

constexpr ErrorEntry RSAP_InvalidParameterNumber = {
    .code = -4,
    .category = ErrorCategory::RSAP,
    .name = "RSAP_InvalidParameterNumber",
    .description = "The parameter number is invalid.",
    .error_handling = "After confirming the parameters, please reset them again.",
    .is_recoverable = true
};

constexpr ErrorEntry RSAP_InvalidParameterValue = {
    .code = -5,
    .category = ErrorCategory::RSAP,
    .name = "RSAP_InvalidParameterValue",
    .description = "The parameter value is invalid.",
    .error_handling = "After confirming the parameters, please reset them again.",
    .is_recoverable = true
};

constexpr ErrorEntry RSAP_InvalidAreaAccess = {
    .code = -6,
    .category = ErrorCategory::RSAP,
    .name = "RSAP_InvalidAreaAccess",
    .description = "The operation attempted to access invalid area.",
    .error_handling = "Reboot your device. If the error persists, please contact us.",
    .is_recoverable = true
};

constexpr ErrorEntry RSAP_NullInPointer = {
    .code = -7,
    .category = ErrorCategory::RSAP,
    .name = "RSAP_NullInPointer",
    .description = "The In pointer variable is null.",
    .error_handling = "Reboot your device. If the error persists, please contact us.",
    .is_recoverable = true
};

constexpr ErrorEntry RSAP_NullFunctionPointer = {
    .code = -8,
    .category = ErrorCategory::RSAP,
    .name = "RSAP_NullFunctionPointer",
    .description = "The function pointer variable is null.",
    .error_handling = "Reboot your device. If the error persists, please contact us.",
    .is_recoverable = true
};

constexpr ErrorEntry RSAP_StructureSizeIncompatible = {
    .code = -9,
    .category = ErrorCategory::RSAP,
    .name = "RSAP_StructureSizeIncompatible",
    .description = "The size of structure is incompatible.",
    .error_handling = "Confirm struct version compatibility. If the error persists, please contact us.",
    .is_recoverable = true
};

constexpr ErrorEntry RSAP_StateIncompatible = {
    .code = -10,
    .category = ErrorCategory::RSAP,
    .name = "RSAP_StateIncompatible",
    .description = "The current state is incompatible.",
    .error_handling = "Check RSAP state machine; retry in valid state.",
    .is_recoverable = true
};

constexpr ErrorEntry RSAP_CONFIG_DIO_EnaInvalid = {
    .code = -11,
    .category = ErrorCategory::RSAP,
    .name = "RSAP_CONFIG_DIO_EnaInvalid",
    .description = "The RSAP CONFIG DIO MUST Ena invalid.",
    .error_handling = "After confirming the parameters, please reset them again.",
    .is_recoverable = true
};

constexpr ErrorEntry RSAP_InputArgumentOutOfRange = {
    .code = -12,
    .category = ErrorCategory::RSAP,
    .name = "RSAP_InputArgumentOutOfRange",
    .description = "The input argument of RSAP function is out of range.",
    .error_handling = "Reboot your device. If the error persists, please contact us.",
    .is_recoverable = true
};

constexpr ErrorEntry RSAP_CannotExecuteCommand = {
    .code = -13,
    .category = ErrorCategory::RSAP,
    .name = "RSAP_CannotExecuteCommand",
    .description = "In its current state, RSAP cannot execute the command.",
    .error_handling = "Ensure RSAP state allows this command.",
    .is_recoverable = true
};

constexpr ErrorEntry RSAP_CommandOutOfRange = {
    .code = -14,
    .category = ErrorCategory::RSAP,
    .name = "RSAP_CommandOutOfRange",
    .description = "The command is out of range.",
    .error_handling = "Check command ID.",
    .is_recoverable = true
};

constexpr ErrorEntry RSAP_OPMODEOutOfRange = {
    .code = -15,
    .category = ErrorCategory::RSAP,
    .name = "RSAP_OPMODEOutOfRange",
    .description = "The OPMODE is out of range.",
    .error_handling = "Reboot your device. If the error persists, please contact us.",
    .is_recoverable = true
};

constexpr ErrorEntry RSAP_StopTypeStateOutOfRange = {
    .code = -16,
    .category = ErrorCategory::RSAP,
    .name = "RSAP_StopTypeStateOutOfRange",
    .description = "The StopTypeState is out of range.",
    .error_handling = "Reboot your device. If the error persists, please contact us.",
    .is_recoverable = true
};

constexpr ErrorEntry RSAP_StopCatOutOfRange = {
    .code = -17,
    .category = ErrorCategory::RSAP,
    .name = "RSAP_StopCatOutOfRange",
    .description = "The StopCat is out of range.",
    .error_handling = "Reboot your device. If the error persists, please contact us.",
    .is_recoverable = true
};

// ---- CONFIG Error Codes (-101 to -112, -201 to -202) ----------------
constexpr ErrorEntry CONFIG_CRCInvalid = {
    .code = -101,
    .category = ErrorCategory::CONFIG,
    .name = "CONFIG_CRCInvalid",
    .description = "The RSAP config info crc is invalid.",
    .error_handling = "Re-generate config; reload configuration file.",
    .is_recoverable = true
};

constexpr ErrorEntry CONFIG_HeaderInvalid = {
    .code = -102,
    .category = ErrorCategory::CONFIG,
    .name = "CONFIG_HeaderInvalid",
    .description = "The RSAP config info header is invalid.",
    .error_handling = "Validate config header; re-download configuration.",
    .is_recoverable = true
};

constexpr ErrorEntry CONFIG_VersionInvalid = {
    .code = -103,
    .category = ErrorCategory::CONFIG,
    .name = "CONFIG_VersionInvalid",
    .description = "The RSAP config info version is invalid.",
    .error_handling = "Update configuration version; ensure version match; re-download configuration.",
    .is_recoverable = true
};

constexpr ErrorEntry CONFIG_ParameterIndexInvalid = {
    .code = -104,
    .category = ErrorCategory::CONFIG,
    .name = "CONFIG_ParameterIndexInvalid",
    .description = "The RSAP config info parameter index is invalid.",
    .error_handling = "Update configuration version; ensure version match; re-download configuration.",
    .is_recoverable = true
};

constexpr ErrorEntry CONFIG_ParameterCntInvalid = {
    .code = -105,
    .category = ErrorCategory::CONFIG,
    .name = "CONFIG_ParameterCntInvalid",
    .description = "The RSAP config info parameter Cnt is invalid.",
    .error_handling = "Update configuration version; ensure version match; re-download configuration.",
    .is_recoverable = true
};

constexpr ErrorEntry CONFIG_ParameterLengthInvalid = {
    .code = -106,
    .category = ErrorCategory::CONFIG,
    .name = "CONFIG_ParameterLengthInvalid",
    .description = "The RSAP config info parameter length is invalid.",
    .error_handling = "Update configuration version; ensure version match; re-download configuration.",
    .is_recoverable = true
};

constexpr ErrorEntry CONFIG_ParameterSubindexInvalid = {
    .code = -107,
    .category = ErrorCategory::CONFIG,
    .name = "CONFIG_ParameterSubindexInvalid",
    .description = "The RSAP config info parameter subindex is invalid.",
    .error_handling = "Update configuration version; ensure version match; re-download configuration.",
    .is_recoverable = true
};

constexpr ErrorEntry CONFIG_ParameterDatatypeInvalid = {
    .code = -108,
    .category = ErrorCategory::CONFIG,
    .name = "CONFIG_ParameterDatatypeInvalid",
    .description = "The RSAP config info parameter datatype is invalid.",
    .error_handling = "Update configuration version; ensure version match; re-download configuration.",
    .is_recoverable = true
};

constexpr ErrorEntry CONFIG_ConfigParameterCntInvalid = {
    .code = -109,
    .category = ErrorCategory::CONFIG,
    .name = "CONFIG_ConfigParameterCntInvalid",
    .description = "The RSAP config parameter Cnt is invalid.",
    .error_handling = "Update configuration version; ensure version match; re-download configuration.",
    .is_recoverable = true
};

constexpr ErrorEntry CONFIG_CmdInvalid = {
    .code = -110,
    .category = ErrorCategory::CONFIG,
    .name = "CONFIG_CmdInvalid",
    .description = "The RSAP config cmd is invalid.",
    .error_handling = "Check command format; correct configuration command.",
    .is_recoverable = true
};

constexpr ErrorEntry CONFIG_InvalidAreaAccess = {
    .code = -111,
    .category = ErrorCategory::CONFIG,
    .name = "CONFIG_InvalidAreaAccess",
    .description = "The RSAP CONFIG to access invalid area.",
    .error_handling = "After confirming the parameters, please reset them again.",
    .is_recoverable = true
};

constexpr ErrorEntry CONFIG_DIO_EnaInvalid = {
    .code = -112,
    .category = ErrorCategory::CONFIG,
    .name = "CONFIG_DIO_EnaInvalid",
    .description = "The RSAP CONFIG DIO Ena invalid.",
    .error_handling = "After confirming the parameters, please reset them again.",
    .is_recoverable = true
};

constexpr ErrorEntry CONFIG_STO_NoReason = {
    .code = -201,
    .category = ErrorCategory::CONFIG,
    .name = "CONFIG_STO_NoReason",
    .description = "The drive switches to STO state for no reason.",
    .error_handling = "To resolve the STO issue with the safety driver, reset drive.",
    .is_recoverable = true
};

constexpr ErrorEntry CONFIG_STO_Timeout = {
    .code = -202,
    .category = ErrorCategory::CONFIG,
    .name = "CONFIG_STO_Timeout",
    .description = "Drive does not switch to STO status within the specified time.",
    .error_handling = "To resolve the STO issue with the safety driver, reset drive. Reboot your device. If the error persists, please contact us.",
    .is_recoverable = true
};

// ---- SAFETY Error Codes (-300 to -310, -320 to -330) ----------------
constexpr ErrorEntry SAFETY_DiagnosisError = {
    .code = -300,
    .category = ErrorCategory::SAFETY,
    .name = "SAFETY_DiagnosisError",
    .description = "The Safety DI Diagnosis Error (codes -300 to -310).",
    .error_handling = "Enter safe state; inspect DI wiring, OSSD, short/stuck fault.",
    .is_recoverable = true
};

constexpr ErrorEntry SAFETY_DODiagnosisError = {
    .code = -320,
    .category = ErrorCategory::SAFETY,
    .name = "SAFETY_DODiagnosisError",
    .description = "The Safety DO Diagnosis Error (codes -320 to -330).",
    .error_handling = "Enter safe state; check DO wiring, output stage, short faults.",
    .is_recoverable = true
};

// ---- SYSMGR Error Codes (-10000001 to -10000401) ------------------
constexpr ErrorEntry SYSMGR_FatalSystemError = {
    .code = -10000001,
    .category = ErrorCategory::SYSMGR,
    .name = "SYSMGR_FatalSystemError",
    .description = "Fatal system error.",
    .error_handling = "Internal exception occurred; record the code and contact the vendor.",
    .is_recoverable = false
};

constexpr ErrorEntry SYSMGR_NullPointer = {
    .code = -10000002,
    .category = ErrorCategory::SYSMGR,
    .name = "SYSMGR_NullPointer",
    .description = "Null pointer.",
    .error_handling = "Internal exception occurred; record the code and contact the vendor.",
    .is_recoverable = false
};

constexpr ErrorEntry SYSMGR_FlashMgrStateUnexpected = {
    .code = -10000009,
    .category = ErrorCategory::SYSMGR,
    .name = "SYSMGR_FlashMgrStateUnexpected",
    .description = "FlashMgr state unexpected.",
    .error_handling = "Internal exception occurred; record the code and contact the vendor.",
    .is_recoverable = false
};

constexpr ErrorEntry SYSMGR_FlashMgr_FNI_CRCError = {
    .code = -10000010,
    .category = ErrorCategory::SYSMGR,
    .name = "SYSMGR_FlashMgr_FNI_CRCError",
    .description = "FlashMgr FNI CRC error.",
    .error_handling = "Initialize the flash with command (9001). If errors remain, the flash hardware may be defective.",
    .is_recoverable = true
};

constexpr ErrorEntry SYSMGR_FlashMgr_ConfigCRCError = {
    .code = -10000011,
    .category = ErrorCategory::SYSMGR,
    .name = "SYSMGR_FlashMgr_ConfigCRCError",
    .description = "FlashMgr Config CRC error.",
    .error_handling = "Initialize the flash with command (9001). If errors remain, the flash hardware may be defective.",
    .is_recoverable = true
};

constexpr ErrorEntry SYSMGR_StateDeniesCommand = {
    .code = -10000100,
    .category = ErrorCategory::SYSMGR,
    .name = "SYSMGR_StateDeniesCommand",
    .description = "System state denies command.",
    .error_handling = "Verify that the system is in a valid state that allows the command to execute.",
    .is_recoverable = true
};

constexpr ErrorEntry SYSMGR_UnknownControlCommand = {
    .code = -10000101,
    .category = ErrorCategory::SYSMGR,
    .name = "SYSMGR_UnknownControlCommand",
    .description = "Unknown control command.",
    .error_handling = "Unknown command; check and correct it before retrying.",
    .is_recoverable = true
};

constexpr ErrorEntry SYSMGR_ControlCommandTimeout = {
    .code = -10000102,
    .category = ErrorCategory::SYSMGR,
    .name = "SYSMGR_ControlCommandTimeout",
    .description = "Control command timeout.",
    .error_handling = "Check the connection status, Slave-to-Slave configuration, and FSoE settings.",
    .is_recoverable = true
};

constexpr ErrorEntry SYSMGR_RSAPSafetyParameterVerificationFailed = {
    .code = -10000201,
    .category = ErrorCategory::SYSMGR,
    .name = "SYSMGR_RSAPSafetyParameterVerificationFailed",
    .description = "RSAP safety parameter verification failed.",
    .error_handling = "Check the RSAP Safety parameter settings.",
    .is_recoverable = true
};

constexpr ErrorEntry SYSMGR_SyncDataMismatch = {
    .code = -10000301,
    .category = ErrorCategory::SYSMGR,
    .name = "SYSMGR_SyncDataMismatch",
    .description = "Sync data mismatch.",
    .error_handling = "Dual-MPU crosscheck failed; please ensure both MPUs have the same software version.",
    .is_recoverable = true
};

constexpr ErrorEntry SYSMGR_RSAPCommandTimeout = {
    .code = -10000401,
    .category = ErrorCategory::SYSMGR,
    .name = "SYSMGR_RSAPCommandTimeout",
    .description = "RSAP command timeout.",
    .error_handling = "Internal exception occurred; record the code and contact the vendor.",
    .is_recoverable = false
};

// ---- FSoE Error Codes (-20001001 to -20007012) ----------------------
constexpr ErrorEntry FSoE_NullPointer = {
    .code = -20001001,
    .category = ErrorCategory::FSoE,
    .name = "FSoE_NullPointer",
    .description = "Null pointer.",
    .error_handling = "Internal exception occurred; record the code and contact the vendor.",
    .is_recoverable = false
};

constexpr ErrorEntry FSoE_TPGInfoArrayFull = {
    .code = -20001002,
    .category = ErrorCategory::FSoE,
    .name = "FSoE_TPGInfoArrayFull",
    .description = "TPG info array full.",
    .error_handling = "Internal exception occurred; record the code and contact the vendor.",
    .is_recoverable = false
};

constexpr ErrorEntry FSoE_RX_PDUQueueFull = {
    .code = -20001003,
    .category = ErrorCategory::FSoE,
    .name = "FSoE_RX_PDUQueueFull",
    .description = "RX PDU queue full.",
    .error_handling = "Internal exception occurred; record the code and contact the vendor.",
    .is_recoverable = false
};

constexpr ErrorEntry FSoE_RX_PDUQueueEmpty = {
    .code = -20001004,
    .category = ErrorCategory::FSoE,
    .name = "FSoE_RX_PDUQueueEmpty",
    .description = "RX PDU queue empty.",
    .error_handling = "Internal exception occurred; record the code and contact the vendor.",
    .is_recoverable = false
};

constexpr ErrorEntry FSoE_TX_PDUQueueFull = {
    .code = -20001005,
    .category = ErrorCategory::FSoE,
    .name = "FSoE_TX_PDUQueueFull",
    .description = "TX PDU queue full.",
    .error_handling = "Internal exception occurred; record the code and contact the vendor.",
    .is_recoverable = false
};

constexpr ErrorEntry FSoE_TX_PDUQueueEmpty = {
    .code = -20001006,
    .category = ErrorCategory::FSoE,
    .name = "FSoE_TX_PDUQueueEmpty",
    .description = "TX PDU queue empty.",
    .error_handling = "Internal exception occurred; record the code and contact the vendor.",
    .is_recoverable = false
};

constexpr ErrorEntry FSoE_AddressUnknown = {
    .code = -20001007,
    .category = ErrorCategory::FSoE,
    .name = "FSoE_AddressUnknown",
    .description = "FSoE address unknown.",
    .error_handling = "Please verify that the FNI parameters are set correctly.",
    .is_recoverable = true
};

constexpr ErrorEntry FSoE_NullPointer2 = {
    .code = -20002001,
    .category = ErrorCategory::FSoE,
    .name = "FSoE_NullPointer2",
    .description = "Null pointer.",
    .error_handling = "Internal exception occurred; record the code and contact the vendor.",
    .is_recoverable = false
};

constexpr ErrorEntry FSoE_CRCCheckFailed = {
    .code = -20002004,
    .category = ErrorCategory::FSoE,
    .name = "FSoE_CRCCheckFailed",
    .description = "CRC check failed.",
    .error_handling = "Please verify that the FNI parameters are set correctly.",
    .is_recoverable = true
};

constexpr ErrorEntry FSoE_ConnectionCountMismatch = {
    .code = -20002005,
    .category = ErrorCategory::FSoE,
    .name = "FSoE_ConnectionCountMismatch",
    .description = "FSoE connection count mismatch.",
    .error_handling = "Please verify that the FNI parameters are set correctly.",
    .is_recoverable = true
};

constexpr ErrorEntry FSoE_ConnectionDoesNotExist = {
    .code = -20002006,
    .category = ErrorCategory::FSoE,
    .name = "FSoE_ConnectionDoesNotExist",
    .description = "FSoE connection does not exist.",
    .error_handling = "Please verify that the FNI parameters are set correctly.",
    .is_recoverable = true
};

constexpr ErrorEntry FSoE_InvalidCommParameterLength = {
    .code = -20002007,
    .category = ErrorCategory::FSoE,
    .name = "FSoE_InvalidCommParameterLength",
    .description = "Invalid communication parameter length.",
    .error_handling = "Please verify that the FNI parameters are set correctly.",
    .is_recoverable = true
};

constexpr ErrorEntry FSoE_InvalidAppParameterLength = {
    .code = -20002008,
    .category = ErrorCategory::FSoE,
    .name = "FSoE_InvalidAppParameterLength",
    .description = "Invalid application parameter length.",
    .error_handling = "Please verify that the FNI parameters are set correctly.",
    .is_recoverable = true
};

constexpr ErrorEntry FSoE_AddressIsZero = {
    .code = -20002009,
    .category = ErrorCategory::FSoE,
    .name = "FSoE_AddressIsZero",
    .description = "FSoE address is zero.",
    .error_handling = "Please verify that the FNI parameters are set correctly.",
    .is_recoverable = true
};

constexpr ErrorEntry FSoE_DuplicateAddress = {
    .code = -20002010,
    .category = ErrorCategory::FSoE,
    .name = "FSoE_DuplicateAddress",
    .description = "Duplicate FSoE address.",
    .error_handling = "Please verify that the FNI parameters are set correctly.",
    .is_recoverable = true
};

constexpr ErrorEntry FSoE_ConnectionIDIsZero = {
    .code = -20002011,
    .category = ErrorCategory::FSoE,
    .name = "FSoE_ConnectionIDIsZero",
    .description = "Connection ID is zero.",
    .error_handling = "Please verify that the FNI parameters are set correctly.",
    .is_recoverable = true
};

constexpr ErrorEntry FSoE_DuplicateConnectionID = {
    .code = -20002012,
    .category = ErrorCategory::FSoE,
    .name = "FSoE_DuplicateConnectionID",
    .description = "Duplicate connection ID.",
    .error_handling = "Please verify that the FNI parameters are set correctly.",
    .is_recoverable = true
};

constexpr ErrorEntry FSoE_WatchdogValueIsZero = {
    .code = -20002013,
    .category = ErrorCategory::FSoE,
    .name = "FSoE_WatchdogValueIsZero",
    .description = "Watchdog value is zero.",
    .error_handling = "Please verify that the FNI parameters are set correctly.",
    .is_recoverable = true
};

constexpr ErrorEntry FSoE_InvalidInputParameter = {
    .code = -20006002,
    .category = ErrorCategory::FSoE,
    .name = "FSoE_InvalidInputParameter",
    .description = "Invalid input parameter.",
    .error_handling = "Internal exception occurred; record the code and contact the vendor.",
    .is_recoverable = false
};

constexpr ErrorEntry FSoE_InvalidDataLength = {
    .code = -20006003,
    .category = ErrorCategory::FSoE,
    .name = "FSoE_InvalidDataLength",
    .description = "Invalid data length.",
    .error_handling = "Internal exception occurred; record the code and contact the vendor.",
    .is_recoverable = false
};

constexpr ErrorEntry FSoE_NullPointer3 = {
    .code = -20006004,
    .category = ErrorCategory::FSoE,
    .name = "FSoE_NullPointer3",
    .description = "Null pointer.",
    .error_handling = "Internal exception occurred; record the code and contact the vendor.",
    .is_recoverable = false
};

constexpr ErrorEntry FSoE_FatalSystemError = {
    .code = -20007001,
    .category = ErrorCategory::FSoE,
    .name = "FSoE_FatalSystemError",
    .description = "Fatal system error.",
    .error_handling = "Internal exception occurred; record the code and contact the vendor.",
    .is_recoverable = false
};

constexpr ErrorEntry FSoE_InvalidOrNullPointer = {
    .code = -20007002,
    .category = ErrorCategory::FSoE,
    .name = "FSoE_InvalidOrNullPointer",
    .description = "Invalid or null pointer.",
    .error_handling = "Internal exception occurred; record the code and contact the vendor.",
    .is_recoverable = false
};

constexpr ErrorEntry FSoE_InvalidOperation = {
    .code = -20007003,
    .category = ErrorCategory::FSoE,
    .name = "FSoE_InvalidOperation",
    .description = "Invalid operation.",
    .error_handling = "Internal exception occurred; record the code and contact the vendor.",
    .is_recoverable = false
};

constexpr ErrorEntry FSoE_FailedToInitialize = {
    .code = -20007004,
    .category = ErrorCategory::FSoE,
    .name = "FSoE_FailedToInitialize",
    .description = "Failed to initialize FSoE.",
    .error_handling = "Internal exception occurred; record the code and contact the vendor.",
    .is_recoverable = false
};

constexpr ErrorEntry FSoE_InvalidOrMissingConnection = {
    .code = -20007005,
    .category = ErrorCategory::FSoE,
    .name = "FSoE_InvalidOrMissingConnection",
    .description = "Invalid or missing FSoE connection.",
    .error_handling = "Internal exception occurred; record the code and contact the vendor.",
    .is_recoverable = false
};

constexpr ErrorEntry FSoE_FailedToRetrieveState = {
    .code = -20007006,
    .category = ErrorCategory::FSoE,
    .name = "FSoE_FailedToRetrieveState",
    .description = "Failed to retrieve FSoE state.",
    .error_handling = "Internal exception occurred; record the code and contact the vendor.",
    .is_recoverable = false
};

constexpr ErrorEntry FSoE_InvalidFSoEIndex = {
    .code = -20007007,
    .category = ErrorCategory::FSoE,
    .name = "FSoE_InvalidFSoEIndex",
    .description = "Invalid FSoE index.",
    .error_handling = "Internal exception occurred; record the code and contact the vendor.",
    .is_recoverable = false
};

constexpr ErrorEntry FSoE_SAPLNotInitialized = {
    .code = -20007008,
    .category = ErrorCategory::FSoE,
    .name = "FSoE_SAPLNotInitialized",
    .description = "SAPL not initialized.",
    .error_handling = "Internal exception occurred; record the code and contact the vendor.",
    .is_recoverable = false
};

constexpr ErrorEntry FSoE_NotInitialized = {
    .code = -20007009,
    .category = ErrorCategory::FSoE,
    .name = "FSoE_NotInitialized",
    .description = "FSoE not initialized.",
    .error_handling = "Internal exception occurred; record the code and contact the vendor.",
    .is_recoverable = false
};

constexpr ErrorEntry FSoE_CriticalError = {
    .code = -20007010,
    .category = ErrorCategory::FSoE,
    .name = "FSoE_CriticalError",
    .description = "Critical FSoE error.",
    .error_handling = "FSoE communication error; please check the connection status, Slave-to-Slave configuration, and FSoE settings.",
    .is_recoverable = true
};

constexpr ErrorEntry FSoE_CommunicationError = {
    .code = -20007011,
    .category = ErrorCategory::FSoE,
    .name = "FSoE_CommunicationError",
    .description = "FSoE communication error.",
    .error_handling = "FSoE communication error; please check the connection status, Slave-to-Slave configuration, and FSoE settings.",
    .is_recoverable = true
};

constexpr ErrorEntry FSoE_OperationDeniedInCurrentState = {
    .code = -20007012,
    .category = ErrorCategory::FSoE,
    .name = "FSoE_OperationDeniedInCurrentState",
    .description = "Operation denied in current state.",
    .error_handling = "Internal exception occurred; record the code and contact the vendor.",
    .is_recoverable = false
};

// Complete list of all error entries
inline const ErrorList kAllErrorCodes = {
    &NoError,
    &RSAP_StateInvalid,
    &RSAP_UnexpectedException,
    &RSAP_NotReady,
    &RSAP_InvalidParameterNumber,
    &RSAP_InvalidParameterValue,
    &RSAP_InvalidAreaAccess,
    &RSAP_NullInPointer,
    &RSAP_NullFunctionPointer,
    &RSAP_StructureSizeIncompatible,
    &RSAP_StateIncompatible,
    &RSAP_CONFIG_DIO_EnaInvalid,
    &RSAP_InputArgumentOutOfRange,
    &RSAP_CannotExecuteCommand,
    &RSAP_CommandOutOfRange,
    &RSAP_OPMODEOutOfRange,
    &RSAP_StopTypeStateOutOfRange,
    &RSAP_StopCatOutOfRange,
    &CONFIG_CRCInvalid,
    &CONFIG_HeaderInvalid,
    &CONFIG_VersionInvalid,
    &CONFIG_ParameterIndexInvalid,
    &CONFIG_ParameterCntInvalid,
    &CONFIG_ParameterLengthInvalid,
    &CONFIG_ParameterSubindexInvalid,
    &CONFIG_ParameterDatatypeInvalid,
    &CONFIG_ConfigParameterCntInvalid,
    &CONFIG_CmdInvalid,
    &CONFIG_InvalidAreaAccess,
    &CONFIG_DIO_EnaInvalid,
    &CONFIG_STO_NoReason,
    &CONFIG_STO_Timeout,
    &SAFETY_DiagnosisError,
    &SAFETY_DODiagnosisError,
    &SYSMGR_FatalSystemError,
    &SYSMGR_NullPointer,
    &SYSMGR_FlashMgrStateUnexpected,
    &SYSMGR_FlashMgr_FNI_CRCError,
    &SYSMGR_FlashMgr_ConfigCRCError,
    &SYSMGR_StateDeniesCommand,
    &SYSMGR_UnknownControlCommand,
    &SYSMGR_ControlCommandTimeout,
    &SYSMGR_RSAPSafetyParameterVerificationFailed,
    &SYSMGR_SyncDataMismatch,
    &SYSMGR_RSAPCommandTimeout,
    &FSoE_NullPointer,
    &FSoE_TPGInfoArrayFull,
    &FSoE_RX_PDUQueueFull,
    &FSoE_RX_PDUQueueEmpty,
    &FSoE_TX_PDUQueueFull,
    &FSoE_TX_PDUQueueEmpty,
    &FSoE_AddressUnknown,
    &FSoE_NullPointer2,
    &FSoE_CRCCheckFailed,
    &FSoE_ConnectionCountMismatch,
    &FSoE_ConnectionDoesNotExist,
    &FSoE_InvalidCommParameterLength,
    &FSoE_InvalidAppParameterLength,
    &FSoE_AddressIsZero,
    &FSoE_DuplicateAddress,
    &FSoE_ConnectionIDIsZero,
    &FSoE_DuplicateConnectionID,
    &FSoE_WatchdogValueIsZero,
    &FSoE_InvalidInputParameter,
    &FSoE_InvalidDataLength,
    &FSoE_NullPointer3,
    &FSoE_FatalSystemError,
    &FSoE_InvalidOrNullPointer,
    &FSoE_InvalidOperation,
    &FSoE_FailedToInitialize,
    &FSoE_InvalidOrMissingConnection,
    &FSoE_FailedToRetrieveState,
    &FSoE_InvalidFSoEIndex,
    &FSoE_SAPLNotInitialized,
    &FSoE_NotInitialized,
    &FSoE_CriticalError,
    &FSoE_CommunicationError,
    &FSoE_OperationDeniedInCurrentState
};

}  // namespace NexcobotESC211
}  // namespace ErrorCodes

}  // namespace Drives
}  // namespace EtherCAT
