#pragma once

#include <cstdint>

namespace EtherCAT {
namespace Drives {
namespace ESC211 {

/// Human-readable name for the ESC211 system current state (0xF101:0x00).
///
/// State mapping:
///   0          -> INIT
///   2          -> READY
///   6          -> MONITOR
///   3–5, 7–13  -> CONFIG
///   14         -> APP_ERR
///   15         -> SYS_ERR
///   99         -> OS_ERR
inline constexpr uint32_t kSystemStateInit      = 0;
inline constexpr uint32_t kSystemStateReady     = 2;
inline constexpr uint32_t kSystemStateMonitoring = 6;
inline constexpr uint32_t kSystemStateAppError  = 14;
inline constexpr uint32_t kSystemStateSysError  = 15;

constexpr const char* systemStateName(uint32_t state) {
    switch (state) {
        case 0:  return "INIT";
        case 2:  return "READY";
        case 6:  return "MONITOR";
        case 3:
        case 4:
        case 5:
        case 7:
        case 8:
        case 9:
        case 10:
        case 11:
        case 12:
        case 13: return "CONFIG";
        case 14: return "APP_ERR";
        case 15: return "SYS_ERR";
        case 99: return "OS_ERR";
        default: return "UNKNOWN";
    }
}

} // namespace ESC211
} // namespace Drives
} // namespace EtherCAT
