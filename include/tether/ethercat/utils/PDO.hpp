#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>
#include "tether/ethercat/CoEManager.hpp"  // for CoEManager

namespace EtherCAT {
namespace Utils {

struct PDOFieldDescriptor {
    uint16_t index;
    uint8_t subindex;
    uint16_t offset;
    uint8_t size;
    const char* description;
};

/// Represents a single entry in a PDO mapping object (0x1600/0x1700/0x1A00/0x1B00 etc).
/// The `bit_length` field is read directly from the slave; `byte_offset` is
/// computed by the helper and indicates where the entry falls within the PDO
/// data buffer when objects are packed sequentially.
struct PDOMappingEntry {
    uint16_t index;      ///< object dictionary index
    uint8_t subindex;    ///< subindex within the object
    uint8_t bit_length;  ///< length in bits (low 8 of mapping value)
    uint16_t byte_offset;///< computed offset in bytes from start of PDO
};

/// Read PDO mapping from a slave via SDO.
///
/// This helper encapsulates the common pattern of reading the entry count
/// (subindex 0) followed by each mapping value.  It stops on read failures but
/// returns `true` if the count itself was successfully obtained.  The
/// `out_entries` array is cleared on entry and filled with any successfully
/// read entries; callers may inspect its size to determine how many entries
/// were retrieved.  Byte offsets are computed assuming contiguous packing (8
/// bits per byte) and truncated when `bit_length` is not a multiple of 8.
///
/// @param sdo        Reference to an initialized SDOManager
/// @param slave      Slave address/alias to query
/// @param pdo_index  PDO mapping object index (e.g. 0x1705 or 0x1B04)
/// @param out_entries Vector to fill with mapping entries
/// @param timeout_ms Timeout passed to individual SDO reads
/// @return `true` if the entry count could be read; `false` otherwise
bool readPDOMapping(CoE::CoEManager& coe,
                    uint16_t pdo_index,
                    std::vector<PDOMappingEntry>& out_entries,
                    uint32_t timeout_ms = SDO::kDefaultSDOTimeoutMs);

/// Format a list of mapping entries into a human-readable string suitable for
/// logging.  The output mirrors the style previously used in examples where
/// each entry was printed with its index/subindex/size and offset.
std::string pdoMappingToString(bool is_tx,
                               uint16_t pdo_index,
                               const std::vector<PDOMappingEntry>& entries);

/// Read a PDO mapping and log the formatted results.
///
/// This convenience wrapper combines `readPDOMapping` and
/// `pdoMappingToString` and emits an INFO log on success or a WARNING when
/// the count read fails.  It mirrors the pattern used by other utility
/// modules (SyncManager/Utils, DC/Utils) and lets examples perform a
/// one-line diagnostic dump.
///
/// @param sdo        Reference to an SDO manager.
/// @param slave      Slave address/alias to query.
/// @param is_tx      True if the PDO is a TxPDO (process data sent by the
///                   slave), false for RxPDO.
/// @param pdo_index  PDO mapping index (e.g. 0x1705 or 0x1B04).
/// @param tag        Logging tag; defaults to "PDO".
/// @param timeout_ms SDO timeout in milliseconds.
void printPDOMapping(CoE::CoEManager& coe,
                     bool is_tx,
                     uint16_t pdo_index,
                     const char* tag = "PDO",
                     uint32_t timeout_ms = SDO::kDefaultSDOTimeoutMs);

/// Format a PDO buffer into a human-readable multi-line string.
///
/// If `fields` is null or `field_count` is 0, a raw hexdump is produced.
std::string pdoToString(bool is_tx,
                        uint16_t pdo_index,
                        const uint8_t* buffer,
                        size_t buffer_size,
                        const PDOFieldDescriptor* fields = nullptr,
                        size_t field_count = 0);

} // namespace Utils
} // namespace EtherCAT
