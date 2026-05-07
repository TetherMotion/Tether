#include "tether/ethercat/Controlword.hpp"

#include <cstdio>
#include <cstddef>
#include <string>

namespace EtherCAT {

std::string describeControlword(uint16_t cw) {
    // Identify the canonical CiA-402 command from the key bits
    const char* cmd;
    uint16_t key = cw & 0x008Fu;
    if      (key == 0x0080u)                              cmd = "FaultReset";
    else if (key == 0x000Fu)                              cmd = "EnableOperation";
    else if (key == 0x0007u)                              cmd = "SwitchOn";
    else if (key == 0x0006u)                              cmd = "Shutdown";
    else if ((cw & 0x0002u) && !(cw & 0x0004u))           cmd = "QuickStop";
    else if (key == 0x0000u)                              cmd = "DisableVoltage";
    else                                                  cmd = "?";

    char buf[128];
    std::snprintf(buf, sizeof(buf),
                  "0x%04X (%s) [%s,%s,%s,%s%s%s]",
                  cw, cmd,
                  (cw & (1u << 0)) ? "SwitchOn" : "!SwitchOn",
                  (cw & (1u << 1)) ? "EnVolt"   : "!EnVolt",
                  (cw & (1u << 2)) ? "!QStop"   : "QSTOP!",
                  (cw & (1u << 3)) ? "EnOp"     : "!EnOp",
                  (cw & (1u << 7)) ? ",FaultRst" : "",
                  (cw & (1u << 8)) ? ",HALT"     : ""
    );

    return std::string(buf);
}

} // namespace EtherCAT
