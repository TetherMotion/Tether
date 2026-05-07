#include "tether/drives/AS715NErrors.hpp"
#include <cstdio>

namespace EtherCAT {
namespace Drives {

AS715NError AS715NError::parse(uint16_t raw_code) {
    AS715NError e{};
    e.raw_code = raw_code;

    if (raw_code == 0) {
        e.class_code = 0;
        e.sub_code = 0;
        e.is_recoverable = false;
        e.name = "NoError";
        e.description = "No error";
        return e;
    }

    // Hex-nibble encoding: each nibble is a hex digit, displayed as-is.
    //   Format: 0x0XYZ  →  "ErXY.Z"
    //   class_code = (raw_code >> 4) & 0xFF  (nibbles X and Y packed as a byte)
    //   sub_code   =  raw_code & 0x0F        (nibble Z)
    //
    // Examples:
    //   0x0C11  →  class=0xC1, sub=1  →  "ErC1.1"  (Synchronization loss)
    //   0x0874  →  class=0x87, sub=4  →  "Er87.4"
    //   0x0741  →  class=0x74, sub=1  →  "Er74.1"  (legacy NoSync)
    e.class_code = static_cast<uint8_t>((raw_code >> 4) & 0xFFu);
    e.sub_code   = static_cast<uint8_t>(raw_code & 0x0Fu);

    static thread_local char name_buf[16];
    if (e.class_code == 0u) {
        std::snprintf(name_buf, sizeof(name_buf), "Er0.%X",
                      static_cast<unsigned>(e.sub_code));
    } else {
        std::snprintf(name_buf, sizeof(name_buf), "Er%X.%X",
                      static_cast<unsigned>(e.class_code),
                      static_cast<unsigned>(e.sub_code));
    }
    e.name = name_buf;

    e.is_recoverable = false;
    e.description = "AS715N fault (see manual)";

    switch (raw_code) {
        // ---- Legacy 0x074x codes (older firmware) ---------------------------
        case ErrorCodes::DCSyncCycleSettingError:  // 0x0740  Er74.0
            e.description = "EtherCAT synchronization cycle setting error";
            e.is_recoverable = true;
            break;
        case ErrorCodes::NoSync:                   // 0x0741  Er74.1
            e.description = "No sync signal";
            e.is_recoverable = true;
            break;
        case ErrorCodes::ChipSyncIncompleteInOp:   // 0x0742  Er74.2
            e.description = "Chip synchronization process uncompleted in OP";
            e.is_recoverable = true;
            break;

        // ---- EtherCAT Class-C errors (ErC1.x / ErC2.x) ---------------------
        case ErrorCodes::EthSyncCycleError:        // 0x0C10  ErC1.0
            e.description = "Excessive EtherCAT synchronization period error";
            e.is_recoverable = true;
            break;
        case ErrorCodes::SyncLoss:                 // 0x0C11  ErC1.1
            e.description = "Synchronization loss";
            e.is_recoverable = true;
            break;
        case ErrorCodes::NetworkStatusSwitchover:  // 0x0C12  ErC1.2
            e.description = "Network status switchover error";
            e.is_recoverable = true;
            break;
        case ErrorCodes::NetworkCableUnreliable:   // 0x0C14  ErC1.4
            e.description = "Network cable connection unreliable";
            e.is_recoverable = true;
            break;
        case ErrorCodes::DataFrameLossProtection:  // 0x0C15  ErC1.5
            e.description = "Data frame loss protection error";
            e.is_recoverable = true;
            break;
        case ErrorCodes::DataFrameForwarding:      // 0x0C16  ErC1.6
            e.description = "Data frame forwarding error";
            e.is_recoverable = true;
            break;
        case ErrorCodes::DataUpdateTimeout:        // 0x0C17  ErC1.7
            e.description = "Data update timeout";
            e.is_recoverable = true;
            break;
        case ErrorCodes::WatchdogExpired:          // 0x0C18  ErC1.8
            e.description = "Watchdog expired";
            e.is_recoverable = true;
            break;
        case ErrorCodes::SYNCSIgnalLoss:           // 0x0C20  ErC2.0
            e.description = "SYNC signal loss";
            e.is_recoverable = true;
            break;

        default:
            break;
    }

    return e;
}

void AS715NError::format(char* buffer, size_t size, uint8_t class_code, uint8_t sub_code) {
    if (buffer == nullptr || size == 0) return;
    // Use uppercase hex: matches the hex-nibble encoding used in parse() and on
    // the drive display.  E.g. class_code=0xC1, sub_code=1 → "ErC1.1".
    if (class_code == 0u) {
        std::snprintf(buffer, size, "Er0.%X", static_cast<unsigned int>(sub_code));
    } else {
        std::snprintf(buffer, size, "Er%X.%X",
                      static_cast<unsigned int>(class_code),
                      static_cast<unsigned int>(sub_code));
    }
}

} // namespace Drives
} // namespace EtherCAT
