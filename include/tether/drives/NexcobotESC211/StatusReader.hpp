#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace EtherCAT { class Slave; }

namespace EtherCAT {
namespace Drives {
namespace ESC211 {

/**
 * @brief Best-effort reader for the ESC211 system-state and debug objects.
 *
 * Stateless synchronous reads (no background thread) of:
 *   - 0xF100:0x02  last control-command response code
 *   - 0xF101:0x00  system current state
 *   - 0xF102:0x00  system error code
 *   - 0xF103:0x00  system error message (VisibleString)
 *   - 0xF110       ESC debug message table (count + up to 16 string entries)
 *
 * Every read reports an `*_ok` flag so callers can distinguish "device has
 * no error" from "the object could not be read".  For continuous
 * background polling of the same objects use SystemErrorManager; for the
 * one-shot vendor-mirror `GetEscErrorInfo` routine use ErrorStateReader.
 */
class StatusReader {
public:
    /// Snapshot of the ESC system state (0xF100/0xF101/0xF102/0xF103).
    struct SystemStateSnapshot {
        uint32_t last_command_response = 0;
        bool last_command_response_ok = false;
        uint32_t system_state = 0;
        bool system_state_ok = false;
        int32_t  error_code = 0;
        bool error_code_ok = false;
        std::string error_message;
        bool error_message_ok = false;
    };

    /// Read the four system-state objects into `snap` (best effort).
    static void readSystemState(EtherCAT::Slave& slave,
                                SystemStateSnapshot& snap);

    /// Read the 0xF110 debug message table into `out` (one string per
    /// entry; read failures are pushed as "read failed" lines so the
    /// caller can display them verbatim).
    static void readDebugMessages(EtherCAT::Slave& slave,
                                  std::vector<std::string>& out);
};

} // namespace ESC211
} // namespace Drives
} // namespace EtherCAT
