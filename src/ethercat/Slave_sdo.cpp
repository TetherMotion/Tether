/**
 * @file Slave_sdo.cpp
 * @brief Slave — SDO convenience and object-dictionary access.
 *
 * TU split out of Slave.cpp.
 */

#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/SDOManager.hpp"
#include "tether/ethercat/Types.hpp"
#include "tether/ethercat/DebugFlags.hpp"
#include "tether/ethercat/FaultDetection.hpp"
#include "SlaveESIHelpers.hpp"
#include "raw/RawConstants.hpp"
#include "tether/platform/Platform.hpp"
#include "tether/ethercat/ObjectDictionary.hpp"

#include <cstring>
#include <bit>
#include <format>

namespace EtherCAT {

static const char* TAG = "Slave";

// -- SDO convenience ---------------------------------------------------------

SlaveError Slave::sdoRead(uint16_t index, uint8_t subindex,
                                   void* data, size_t& size) {
    if (!mailboxAvailable()) return SlaveError::MailboxNotConfigured;

    auto& sdo = master_->sdoManager(index_);
    size_t actual = 0;
    if (!sdo.readSync(index, subindex,
                      data, size, SDO::kDefaultSDOTimeoutMs, &actual)) {
        return (sdo.lastSdoAbortCode() != 0) ? SlaveError::SDOAborted
                                              : SlaveError::SDOError;
    }
    size = actual;
    return SlaveError::Ok;
}

SlaveError Slave::sdoWrite(uint16_t index, uint8_t subindex,
                                    const void* data, size_t size) {
    if (!mailboxAvailable()) return SlaveError::MailboxNotConfigured;

    auto& sdo = master_->sdoManager(index_);
    if (!sdo.writeSync(index, subindex,
                       data, size, {.timeout_ms = SDO::kDefaultSDOTimeoutMs})) {
        return (sdo.lastSdoAbortCode() != 0) ? SlaveError::SDOAborted
                                              : SlaveError::SDOError;
    }
    return SlaveError::Ok;
}

// Map a failed CoE typed read/write to a SlaveError.  ShuttingDown (master
// cancellation via requestCancel()) maps to Cancelled so callers can tell
// an expected shutdown failure apart from a real SDO problem.
static SlaveError sdoCoeErrorToSlaveError(CoE::CoEManager& sdo, CoE::CoEError err) {
    if (err.code == CoE::CoEErrorCode::ShuttingDown) return SlaveError::Cancelled;
    return (sdo.lastSdoAbortCode() != 0) ? SlaveError::SDOAborted
                                        : SlaveError::SDOError;
}

SlaveError Slave::sdoReadU8(uint16_t index, uint8_t sub, uint8_t& out) {
    if (!mailboxAvailable()) return SlaveError::MailboxNotConfigured;

    auto& sdo = master_->sdoManager(index_);
    auto result = sdo.readU8(index, sub);
    if (!result.has_value()) {
        return sdoCoeErrorToSlaveError(sdo, result.error());
    }
    out = result.value();
    return SlaveError::Ok;
}

SlaveError Slave::sdoReadU16(uint16_t index, uint8_t sub, uint16_t& out) {
    if (!mailboxAvailable()) return SlaveError::MailboxNotConfigured;

    auto& sdo = master_->sdoManager(index_);
    auto result = sdo.readU16(index, sub);
    if (!result.has_value()) {
        return sdoCoeErrorToSlaveError(sdo, result.error());
    }
    out = result.value();
    return SlaveError::Ok;
}

SlaveError Slave::sdoReadU32(uint16_t index, uint8_t sub, uint32_t& out) {
    if (!mailboxAvailable()) return SlaveError::MailboxNotConfigured;

    auto& sdo = master_->sdoManager(index_);
    auto result = sdo.readU32(index, sub);
    if (!result.has_value()) {
        return sdoCoeErrorToSlaveError(sdo, result.error());
    }
    out = result.value();
    return SlaveError::Ok;
}

SlaveError Slave::sdoWriteU8(uint16_t index, uint8_t sub, uint8_t val) {
    if (!mailboxAvailable()) return SlaveError::MailboxNotConfigured;

    auto& sdo = master_->sdoManager(index_);
    auto result = sdo.writeU8(index, sub, val);
    if (!result.has_value()) {
        return sdoCoeErrorToSlaveError(sdo, result.error());
    }
    return SlaveError::Ok;
}

SlaveError Slave::sdoWriteU16(uint16_t index, uint8_t sub, uint16_t val) {
    if (!mailboxAvailable()) return SlaveError::MailboxNotConfigured;

    auto& sdo = master_->sdoManager(index_);
    auto result = sdo.writeU16(index, sub, val);
    if (!result.has_value()) {
        return sdoCoeErrorToSlaveError(sdo, result.error());
    }
    return SlaveError::Ok;
}

SlaveError Slave::sdoWriteU32(uint16_t index, uint8_t sub, uint32_t val) {
    if (!mailboxAvailable()) return SlaveError::MailboxNotConfigured;

    auto& sdo = master_->sdoManager(index_);
    auto result = sdo.writeU32(index, sub, val);
    if (!result.has_value()) {
        return sdoCoeErrorToSlaveError(sdo, result.error());
    }
    return SlaveError::Ok;
}

// -- Object-dictionary entry access -----------------------------------------

// Dispatch goes through the virtual sdoReadUx/sdoWriteUx/sdoRead/sdoWrite
// calls so NonExistingSlave and test doubles keep their overridden behaviour.

SlaveError Slave::sdoReadEntry(
    const ObjectDictionary::ObjectDictionaryEntry& entry, uint64_t& out) {
    using D = ObjectDictionary::ObjectDictionaryDataType;
    out = 0;

    switch (entry.data_type) {
        case D::Boolean:
        case D::Unsigned8: {
            uint8_t v = 0;
            const auto e = sdoReadU8(entry.index, entry.subindex, v);
            if (e != SlaveError::Ok) return e;
            out = v;
            return SlaveError::Ok;
        }
        case D::Unsigned16: {
            uint16_t v = 0;
            const auto e = sdoReadU16(entry.index, entry.subindex, v);
            if (e != SlaveError::Ok) return e;
            out = v;
            return SlaveError::Ok;
        }
        case D::Unsigned32:
        case D::Real32: {
            uint32_t v = 0;
            const auto e = sdoReadU32(entry.index, entry.subindex, v);
            if (e != SlaveError::Ok) return e;
            out = v;
            return SlaveError::Ok;
        }
        case D::Integer8: {
            uint8_t v = 0;
            const auto e = sdoReadU8(entry.index, entry.subindex, v);
            if (e != SlaveError::Ok) return e;
            out = static_cast<uint64_t>(
                static_cast<int64_t>(static_cast<int8_t>(v)));
            return SlaveError::Ok;
        }
        case D::Integer16: {
            uint16_t v = 0;
            const auto e = sdoReadU16(entry.index, entry.subindex, v);
            if (e != SlaveError::Ok) return e;
            out = static_cast<uint64_t>(
                static_cast<int64_t>(static_cast<int16_t>(v)));
            return SlaveError::Ok;
        }
        case D::Integer32: {
            uint32_t v = 0;
            const auto e = sdoReadU32(entry.index, entry.subindex, v);
            if (e != SlaveError::Ok) return e;
            out = static_cast<uint64_t>(
                static_cast<int64_t>(static_cast<int32_t>(v)));
            return SlaveError::Ok;
        }
        default: {
            // 5-8 byte types (Integer40..64, Unsigned24..64, Real64, ...):
            // raw little-endian read, assembled into the result.
            const uint8_t width = inferByteSize(entry.data_type);
            if (width == 0) return SlaveError::SDOError;
            uint8_t buf[8] = {};
            size_t size = width;
            const auto e = sdoRead(entry.index, entry.subindex, buf, size);
            if (e != SlaveError::Ok || size != width) {
                return (e != SlaveError::Ok) ? e : SlaveError::SDOError;
            }
            for (size_t i = 0; i < width; ++i) {
                out |= static_cast<uint64_t>(buf[i]) << (i * 8);
            }
            return SlaveError::Ok;
        }
    }
}

SlaveError Slave::sdoWriteEntry(
    const ObjectDictionary::ObjectDictionaryEntry& entry, uint64_t value) {
    using D = ObjectDictionary::ObjectDictionaryDataType;

    switch (entry.data_type) {
        case D::Boolean:
        case D::Unsigned8:
        case D::Integer8:
            return sdoWriteU8(entry.index, entry.subindex,
                              static_cast<uint8_t>(value));
        case D::Unsigned16:
        case D::Integer16:
            return sdoWriteU16(entry.index, entry.subindex,
                               static_cast<uint16_t>(value));
        case D::Unsigned32:
        case D::Real32:
        case D::Integer32:
            return sdoWriteU32(entry.index, entry.subindex,
                               static_cast<uint32_t>(value));
        default: {
            const uint8_t width = inferByteSize(entry.data_type);
            if (width == 0) return SlaveError::SDOError;
            uint8_t buf[8] = {};
            for (size_t i = 0; i < width; ++i) {
                buf[i] = static_cast<uint8_t>(value >> (i * 8));
            }
            return sdoWrite(entry.index, entry.subindex, buf, width);
        }
    }
}

SlaveError Slave::sdoReadEntry(
    const ObjectDictionary::ObjectDictionaryEntry& entry,
    uint8_t subindex, uint64_t& out) {
    ObjectDictionary::ObjectDictionaryEntry e = entry;
    e.subindex = subindex;
    return sdoReadEntry(e, out);
}

SlaveError Slave::sdoWriteEntry(
    const ObjectDictionary::ObjectDictionaryEntry& entry,
    uint8_t subindex, uint64_t value) {
    ObjectDictionary::ObjectDictionaryEntry e = entry;
    e.subindex = subindex;
    return sdoWriteEntry(e, value);
}

uint32_t Slave::lastSdoAbortCode() const {
    return master_->sdoManager(index_).lastSdoAbortCode();
}

bool Slave::lastSdoWasDownload() const {
    return master_->sdoManager(index_).lastSdoWasDownload();
}

size_t Slave::lastSdoAttemptedLength() const {
    return master_->sdoManager(index_).lastSdoAttemptedLength();
}

} // namespace EtherCAT

