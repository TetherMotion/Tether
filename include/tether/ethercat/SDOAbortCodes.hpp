/**
 * @file SDOAbortCodes.hpp
 * @brief Canonical CoE SDO abort codes shared by master and slave code
 *
 * These are the standard abort code values returned by a slave in an SDO
 * abort response (ETG.1000.6 / CiA 301).  A single enum at EtherCAT scope
 * is used by the master-side SDO/CoE machinery (namespace EtherCAT::SDO /
 * EtherCAT::CoE) and by the slave-emulation object dictionary
 * (namespace EtherCAT::slave) — unqualified references inside any of those
 * namespaces resolve here via enclosing-namespace lookup.
 */

#pragma once

#include <cstdint>

namespace EtherCAT {

/**
 * @brief SDO abort codes (CoE specification)
 *
 * The underlying value is the raw 32-bit code sent on the wire.  A value
 * not listed here (e.g. a vendor-specific abort code) can be carried as
 * static_cast<SDOAbortCode>(raw); sdoAbortCodeStr() maps it to
 * "Unknown abort code".
 */
enum class SDOAbortCode : uint32_t {
    Success                  = 0x00000000, ///< No error
    ToggleBitNotChanged      = 0x05030000, ///< Toggle bit not alternated
    Timeout                  = 0x05040000, ///< SDO protocol timeout
    InvalidCommand           = 0x05040001, ///< Command specifier unknown
    InvalidBlockSize         = 0x05040002, ///< Invalid block size
    InvalidSequenceNumber    = 0x05040003, ///< Invalid sequence number
    CrcError                 = 0x05040004, ///< CRC error
    OutOfMemory              = 0x05040005, ///< Out of memory
    UnsupportedAccess        = 0x06010000, ///< Unsupported access
    ReadOnlyObject           = 0x06010001, ///< Write to read-only object
    WriteOnlyObject          = 0x06010002, ///< Read from write-only object
    ObjectNotFound           = 0x06020000, ///< Object does not exist
    PdoMappingError          = 0x06040041, ///< Object cannot be mapped to PDO
    PdoLengthExceeded        = 0x06040042, ///< Number/length would exceed PDO
    ParameterIncompatible    = 0x06040043, ///< Parameter incompatibility
    InternalError            = 0x06040047, ///< General internal incompatibility
    HardwareError            = 0x06060000, ///< Hardware error
    DataTypeMismatch         = 0x06070010, ///< Data type mismatch, length mismatch
    DataTypeTooLong          = 0x06070012, ///< Data type mismatch, length too high
    DataTypeTooShort         = 0x06070013, ///< Data type mismatch, length too low
    SubindexNotFound         = 0x06090011, ///< Subindex does not exist
    InvalidValue             = 0x06090030, ///< Invalid value for parameter
    ValueTooHigh             = 0x06090031, ///< Value too high
    ValueTooLow              = 0x06090032, ///< Value too low
    MaxLessThanMin           = 0x06090036, ///< Maximum less than minimum
    ResourceNotAvailable     = 0x060A0023, ///< Resource not available
    GeneralError             = 0x08000000, ///< General error
    TransferAborted          = 0x08000020, ///< Data transfer aborted
    LocalControlError        = 0x08000021, ///< Local control error
    DeviceStateError         = 0x08000022, ///< Wrong device state
    DictionaryNotPresent     = 0x08000023, ///< Object dictionary not present
    NoDataAvailable          = 0x08000024, ///< No data available
};

/**
 * @brief Convert an SDO abort code to a human-readable string.
 *
 * Never returns null; unlisted (vendor-specific) codes map to
 * "Unknown abort code".
 */
const char* sdoAbortCodeStr(SDOAbortCode code);

/// @brief Overload taking the raw 32-bit abort code.
inline const char* sdoAbortCodeStr(uint32_t code) {
    return sdoAbortCodeStr(static_cast<SDOAbortCode>(code));
}

} // namespace EtherCAT
