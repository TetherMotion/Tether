#include "tether/drives/NexcobotESC211Errors.hpp"

namespace EtherCAT {
namespace Drives {
namespace ErrorCodes {
namespace NexcobotESC211 {

const char* ErrorCategoryToString(ErrorCategory category) {
    switch (category) {
        case ErrorCategory::None:   return "None";
        case ErrorCategory::RSAP:   return "RSAP";
        case ErrorCategory::CONFIG: return "CONFIG";
        case ErrorCategory::SAFETY: return "SAFETY";
        case ErrorCategory::SYSMGR: return "SYSMGR";
        case ErrorCategory::FSoE:   return "FSoE";
        default:                    return "Unknown";
    }
}

const ErrorEntry* NexcobotESC211Error::findEntry(int32_t raw_code) {
    // Check for range-based error codes (SAFETY DI: -300 to -310, SAFETY DO: -320 to -330)
    if (raw_code >= -310 && raw_code <= -300) {
        return &SAFETY_DiagnosisError;
    }
    if (raw_code >= -330 && raw_code <= -320) {
        return &SAFETY_DODiagnosisError;
    }

    // Search through the complete error list
    for (const auto* entry : kAllErrorCodes) {
        if (entry->code == raw_code) {
            return entry;
        }
    }

    return nullptr; // Not found
}

NexcobotESC211Error NexcobotESC211Error::parse(int32_t raw_code) {
    NexcobotESC211Error e{};
    e.raw_code = raw_code;

    const ErrorEntry* entry = findEntry(raw_code);
    if (entry != nullptr) {
        e.category = entry->category;
        e.name = entry->name;
        e.description = entry->description;
        e.error_handling = entry->error_handling;
        e.is_recoverable = entry->is_recoverable;
    } else {
        e.category = ErrorCategory::None;
        e.name = "UnknownError";
        e.description = "Unknown error code";
        e.error_handling = "See manual";
        e.is_recoverable = false;
    }

    return e;
}

} // namespace NexcobotESC211
} // namespace ErrorCodes
} // namespace Drives
} // namespace EtherCAT
