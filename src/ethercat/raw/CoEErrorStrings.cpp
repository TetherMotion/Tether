/**
 * @file CoEErrorStrings.cpp
 * @brief CoE error-string helpers (coeErrorStr + coeErrorToAbortCode).
 *
 * TU split out of CoEManager.cpp.
 */

#include "raw/CoEErrorStrings.hpp"

#include "tether/ethercat/CoETypes.hpp"
#include "tether/ethercat/SDOAbortCodes.hpp"

namespace EtherCAT {
namespace CoE {


const char* coeErrorStr(CoEError error) {
    // Aborts carry the slave's real SDO abort code — decode it so callers
    // get e.g. "Object does not exist" instead of a bare "Aborted".
    if (error.code == CoEErrorCode::Aborted && error.abort_code != 0) {
        return sdoAbortCodeStr(error.abort_code);
    }
    switch (error.code) {
        case CoEErrorCode::Ok:              return "Ok";
        case CoEErrorCode::Timeout:         return "Timeout";
        case CoEErrorCode::Aborted:         return "Aborted";
        case CoEErrorCode::TransportError:  return "Transport error";
        case CoEErrorCode::QueueFull:       return "Queue full";
        case CoEErrorCode::NotConfigured:   return "Mailbox not configured";
        case CoEErrorCode::ShuttingDown:    return "Shutting down";
        case CoEErrorCode::SlaveNotFound:   return "Slave not found";
        case CoEErrorCode::InternalError:   return "Internal error";
        default:                            return "Unknown error";
    }
}

// ============================================================================
// CoEError → SDOAbortCode mapping
// ============================================================================

SDOAbortCode coeErrorToAbortCode(CoEError err) {
    switch (err.code) {
        case CoEErrorCode::Ok:             return SDOAbortCode::Success;
        case CoEErrorCode::Timeout:        return SDOAbortCode::Timeout;
        case CoEErrorCode::NotConfigured:  return SDOAbortCode::DeviceStateError;
        case CoEErrorCode::TransportError: return SDOAbortCode::GeneralError;
        case CoEErrorCode::QueueFull:      return SDOAbortCode::OutOfMemory;
        // Preserve the slave's actual abort code instead of a generic
        // TransferAborted so legacy-queue callers see the real reason.
        case CoEErrorCode::Aborted:        return err.abort_code != 0
                                             ? err.abort
                                             : SDOAbortCode::TransferAborted;
        case CoEErrorCode::ShuttingDown:   return SDOAbortCode::DeviceStateError;
        case CoEErrorCode::SlaveNotFound:  return SDOAbortCode::ObjectNotFound;
        case CoEErrorCode::InternalError:  return SDOAbortCode::InternalError;
    }
    return SDOAbortCode::GeneralError;
}

} // namespace CoE
} // namespace EtherCAT

