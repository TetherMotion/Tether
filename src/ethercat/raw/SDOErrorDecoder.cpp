#include "tether/ethercat/SDOErrorDecoder.hpp"
#include "tether/ethercat/SDOAbortCodes.hpp"

namespace EtherCAT {
namespace Raw {

const char* SDOErrorDecoder::sdoAbortCodeStr(uint32_t code) const {
    return EtherCAT::sdoAbortCodeStr(code);
}

const char* SDOErrorDecoder::mbxErrorCodeStr(uint16_t code) const {
    switch (code) {
        case 0x0000: return "No error";
        case 0x0001: return "Syntax error in mailbox message";
        case 0x0002: return "Unsupported protocol";
        case 0x0003: return "Invalid channel";
        case 0x0004: return "Service not supported";
        case 0x0005: return "Invalid header";
        case 0x0006: return "Size too short";
        case 0x0007: return "No more memory";
        case 0x0008: return "Invalid size";
        case 0x0009: return "Service in work";
        default:     return "Unknown mailbox error";
    }
}

const char* SDOErrorDecoder::mbxErrorDetailStr(uint16_t errCode, uint16_t detail) const {
    if (errCode == 0x0001) {
        if (detail < 6) return "offset in mailbox header";
        if (detail < 8) return "offset in CoE header";
        return "offset in SDO payload";
    }
    return "protocol-specific detail";
}

} // namespace Raw
} // namespace EtherCAT
