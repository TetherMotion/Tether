#include "tether/ethercat/SDOErrorDecoder.hpp"

namespace EtherCAT {
namespace Raw {

const char* SDOErrorDecoder::sdoAbortCodeStr(uint32_t code) const {
    switch (code) {
        case 0x05030000: return "Toggle bit not alternated";
        case 0x05040000: return "SDO protocol timeout";
        case 0x05040001: return "Invalid command";
        case 0x05040002: return "Invalid block size";
        case 0x05040003: return "Invalid sequence number";
        case 0x05040004: return "CRC error";
        case 0x05040005: return "Out of memory";
        case 0x06010000: return "Unsupported access";
        case 0x06010001: return "Write to read-only object";
        case 0x06010002: return "Read from write-only object";
        case 0x06020000: return "Object does not exist";
        case 0x06040041: return "Object cannot be mapped to PDO";
        case 0x06040042: return "PDO length exceeded";
        case 0x06040043: return "Parameter incompatibility";
        case 0x06040047: return "General internal incompatibility";
        case 0x06060000: return "Hardware error";
        case 0x06070010: return "Data type mismatch, length mismatch";
        case 0x06070012: return "Data type mismatch, length too high";
        case 0x06070013: return "Data type mismatch, length too low";
        case 0x06090011: return "Subindex does not exist";
        case 0x06090030: return "Invalid value for parameter";
        case 0x06090031: return "Value too high";
        case 0x06090032: return "Value too low";
        case 0x06090036: return "Maximum less than minimum";
        case 0x060A0023: return "Resource not available";
        case 0x08000000: return "General error";
        case 0x08000020: return "Data transfer aborted";
        case 0x08000021: return "Local control error";
        case 0x08000022: return "Wrong device state";
        case 0x08000023: return "Object dictionary not present";
        case 0x08000024: return "No data available";
        default:        return "Unknown abort code";
    }
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
