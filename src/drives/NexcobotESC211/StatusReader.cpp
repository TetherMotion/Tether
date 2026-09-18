#include "tether/drives/NexcobotESC211/StatusReader.hpp"

#include <algorithm>
#include <cstring>
#include <format>

#include "tether/ethercat/Slave.hpp"

namespace EtherCAT {
namespace Drives {
namespace ESC211 {

void StatusReader::readSystemState(EtherCAT::Slave& slave,
                                   SystemStateSnapshot& snap) {
    constexpr uint16_t kControlCmdIndex = 0xF100;
    constexpr uint16_t kSystemStateIndex = 0xF101;
    constexpr uint16_t kErrorCodeIndex = 0xF102;
    constexpr uint16_t kErrorMsgIndex = 0xF103;

    snap.last_command_response_ok =
        (slave.sdoReadU32(kControlCmdIndex, 0x02, snap.last_command_response) ==
         EtherCAT::SlaveError::Ok);
    snap.system_state_ok =
        (slave.sdoReadU32(kSystemStateIndex, 0x00, snap.system_state) ==
         EtherCAT::SlaveError::Ok);
    uint32_t raw_error = 0;
    snap.error_code_ok =
        (slave.sdoReadU32(kErrorCodeIndex, 0x00, raw_error) ==
         EtherCAT::SlaveError::Ok);
    snap.error_code = static_cast<int32_t>(raw_error);
    if (snap.error_code_ok && snap.error_code != 0) {
        char buf[256] = {};
        size_t actual = sizeof(buf);
        auto e = slave.sdoRead(kErrorMsgIndex, 0x00, buf, actual);
        snap.error_message_ok = (e == EtherCAT::SlaveError::Ok);
        if (snap.error_message_ok && actual > 0) {
            snap.error_message.assign(buf, strnlen(buf, actual));
        }
    } else {
        snap.error_message.clear();
        snap.error_message_ok = false;
    }
}

void StatusReader::readDebugMessages(EtherCAT::Slave& slave,
                                     std::vector<std::string>& out) {
    constexpr uint16_t kDebugMsgIndex = 0xF110;
    out.clear();
    uint8_t dbg_count = 0;
    auto dbg_err = slave.sdoReadU8(kDebugMsgIndex, 0x00, dbg_count);
    if (dbg_err != EtherCAT::SlaveError::Ok) {
        out.push_back(std::format("0xF110:0x00 read failed: {}",
                                  std::string(EtherCAT::slaveErrorToString(dbg_err))));
        return;
    }
    out.reserve(dbg_count + 1);
    out.push_back(std::format("0xF110 entry count = {}",
                              static_cast<unsigned>(dbg_count)));
    const uint8_t n = std::min<uint8_t>(dbg_count, 16);
    for (uint8_t sub = 1; sub <= n; ++sub) {
        char buf[512] = {};
        size_t actual = sizeof(buf);
        auto e = slave.sdoRead(kDebugMsgIndex, sub, buf, actual);
        if (e != EtherCAT::SlaveError::Ok) {
            out.push_back(std::format("0xF110:0x{:02X}: read failed: {}",
                                      static_cast<unsigned>(sub),
                                      std::string(EtherCAT::slaveErrorToString(e))));
            continue;
        }
        const size_t len = strnlen(buf, actual);
        if (len == 0) {
            out.push_back(std::format("0xF110:0x{:02X}: (empty)",
                                      static_cast<unsigned>(sub)));
        } else {
            out.push_back(std::format("0xF110:0x{:02X}: {}",
                                      static_cast<unsigned>(sub),
                                      std::string(buf, len)));
        }
    }
}

} // namespace ESC211
} // namespace Drives
} // namespace EtherCAT
