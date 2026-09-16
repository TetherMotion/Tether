/**
 * @file SDOAbortCodes.cpp
 * @brief SDO abort code string table — shared by master and slave components
 */

#include "tether/ethercat/SDOAbortCodes.hpp"

namespace EtherCAT {

const char* sdoAbortCodeStr(SDOAbortCode code) {
    switch (code) {
        case SDOAbortCode::Success:              return "Success";
        case SDOAbortCode::ToggleBitNotChanged:  return "Toggle bit not alternated";
        case SDOAbortCode::Timeout:              return "SDO protocol timeout";
        case SDOAbortCode::InvalidCommand:       return "Invalid command";
        case SDOAbortCode::InvalidBlockSize:     return "Invalid block size";
        case SDOAbortCode::InvalidSequenceNumber:return "Invalid sequence number";
        case SDOAbortCode::CrcError:             return "CRC error";
        case SDOAbortCode::OutOfMemory:          return "Out of memory";
        case SDOAbortCode::UnsupportedAccess:    return "Unsupported access";
        case SDOAbortCode::ReadOnlyObject:       return "Write to read-only object";
        case SDOAbortCode::WriteOnlyObject:      return "Read from write-only object";
        case SDOAbortCode::ObjectNotFound:       return "Object does not exist";
        case SDOAbortCode::PdoMappingError:      return "Object cannot be mapped to PDO";
        case SDOAbortCode::PdoLengthExceeded:    return "PDO length exceeded";
        case SDOAbortCode::ParameterIncompatible:return "Parameter incompatibility";
        case SDOAbortCode::InternalError:        return "General internal incompatibility";
        case SDOAbortCode::HardwareError:        return "Hardware error";
        case SDOAbortCode::DataTypeMismatch:     return "Data type mismatch, length mismatch";
        case SDOAbortCode::DataTypeTooLong:      return "Data type mismatch, length too high";
        case SDOAbortCode::DataTypeTooShort:     return "Data type mismatch, length too low";
        case SDOAbortCode::SubindexNotFound:     return "Subindex does not exist";
        case SDOAbortCode::InvalidValue:         return "Invalid value for parameter";
        case SDOAbortCode::ValueTooHigh:         return "Value too high";
        case SDOAbortCode::ValueTooLow:          return "Value too low";
        case SDOAbortCode::MaxLessThanMin:       return "Maximum less than minimum";
        case SDOAbortCode::ResourceNotAvailable: return "Resource not available";
        case SDOAbortCode::GeneralError:         return "General error";
        case SDOAbortCode::TransferAborted:      return "Data transfer aborted";
        case SDOAbortCode::LocalControlError:    return "Local control error";
        case SDOAbortCode::DeviceStateError:     return "Wrong device state";
        case SDOAbortCode::DictionaryNotPresent: return "Object dictionary not present";
        case SDOAbortCode::NoDataAvailable:      return "No data available";
        default:                                 return "Unknown abort code";
    }
}

} // namespace EtherCAT
