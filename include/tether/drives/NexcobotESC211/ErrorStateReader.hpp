#pragma once

#include <cstdint>
#include <string>

#include "tether/ethercat/Master.hpp"

// Forward declaration to avoid pulling the full Slave header into the public
// interface; the implementation only needs the CoEManager exposed by Master.
namespace EtherCAT { class Slave; }

namespace EtherCAT {
namespace Drives {
namespace ESC211 {

/**
 * @brief One-shot reader for the ESC211 error-state snapshot.
 *
 * Mirrors the official Nexcobot `GetEscErrorInfo` (Python reference id
 * `0xBF5F4`) routine.  Reads, in order, the four objects that the official
 * software consults whenever an ESC command reports `FAILED`:
 *
 *   - 0xF101:0x00  System Current State   (Unsigned32)
 *   - 0xF102:0x00  System Error Code      (Integer32)
 *   - 0xF103:0x00  System Error Message   (VisibleString, up to 256 B)
 *   - 0xF110:0x01  ESC Debug Msg entry 1  (VisibleString, up to 256 B,
 *                                          read with Complete Access)
 *
 * Each field is logged via TETHER_LOG as it is read, exactly like the
 * reference implementation.  The assembled Snapshot is also returned to
 * the caller so that examples can branch on the result (e.g. abort an
 * FNI apply when the device already reports an error).
 *
 * This component is deliberately synchronous and stateless — construct it,
 * call `readAndLog()` once, inspect the returned snapshot.  For continuous
 * background polling of 0xF102/0xF103/0xF104 use ESC211::SystemErrorManager
 * instead.
 */
class ErrorStateReader {
public:
    /// Aggregated error-state snapshot.  `valid` is true only if every
    /// individual SDO upload completed without a transport/abort error.
    /// `valid == false` does NOT mean the device is in an error state — it
    /// means the snapshot could not be fully read.
    struct Snapshot {
        bool        valid = false;
        uint32_t    system_state = 0;        ///< 0xF101:0x00
        int32_t     error_code = 0;          ///< 0xF102:0x00
        std::string error_message;           ///< 0xF103:0x00 (NUL-trimmed)
        std::string diagnostic_message;      ///< 0xF110:0x01 (NUL-trimmed, CA)
    };

    /**
     * @param master       EtherCAT master (for CoEManager / complete-access reads)
     * @param slave_index  Zero-based slave index whose error state is read
     * @param tag          ESP-style log tag
     */
    ErrorStateReader(EtherCAT::Master& master,
                     uint16_t slave_index,
                     const char* tag = "ESC211ErrorStateReader");

    /**
     * @brief Read the full error-state snapshot and log each field.
     *
     * Mirrors the order of operations in the reference Python
     * `get_esc_error_info`: state, error code, error message, diagnostic.
     * On a per-field SDO abort the exact abort code is reported via
     * `Tether::Examples::reportSdoAbort` (when available) and the snapshot
     * is marked invalid, but the remaining fields are still attempted so
     * the operator gets as much diagnostic context as possible.
     */
    Snapshot readAndLog();

    /**
     * @brief True if the snapshot indicates the device is in a clean state:
     *        fully read, error code zero, and no error message.
     *
     * A non-empty diagnostic message alone does NOT count as an error —
     * the ESC debug log frequently carries informational text.
     */
    static bool isOk(const Snapshot& s);

private:
    EtherCAT::Master& master_;
    uint16_t          slave_index_;
    const char*       tag_;
};

} // namespace ESC211
} // namespace Drives
} // namespace EtherCAT
