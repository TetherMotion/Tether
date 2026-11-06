/**
 * @file FunctionCode.hpp
 * @brief Modbus function-code enum + classification helpers (plan §3.3/§6.4).
 */
#pragma once

#include <cstdint>
#include <optional>
#include <string_view>

namespace tether::modbus {

/**@brief Wire function codes — normative values, never renumber. */
enum class FunctionCode : uint8_t {
    ReadCoils              = 0x01,
    ReadDiscreteInputs     = 0x02,
    ReadHoldingRegisters   = 0x03,
    ReadInputRegisters     = 0x04,
    WriteSingleCoil        = 0x05,
    WriteSingleRegister    = 0x06,
    ReadExceptionStatus    = 0x07,
    Diagnostics            = 0x08,
    GetCommEventCounter    = 0x0B,
    GetCommEventLog        = 0x0C,
    WriteMultipleCoils     = 0x0F,
    WriteMultipleRegisters = 0x10,
    ReportServerId         = 0x11,
    ReadFileRecord         = 0x14,
    WriteFileRecord        = 0x15,
    MaskWriteRegister      = 0x16,
    ReadWriteMultipleRegs  = 0x17,
    ReadFifoQueue          = 0x18,
    EncapsulatedInterface  = 0x2B,
};

constexpr std::optional<FunctionCode> fcFromByte(uint8_t b) {
    switch (b) {
        case 0x01: return FunctionCode::ReadCoils;
        case 0x02: return FunctionCode::ReadDiscreteInputs;
        case 0x03: return FunctionCode::ReadHoldingRegisters;
        case 0x04: return FunctionCode::ReadInputRegisters;
        case 0x05: return FunctionCode::WriteSingleCoil;
        case 0x06: return FunctionCode::WriteSingleRegister;
        case 0x07: return FunctionCode::ReadExceptionStatus;
        case 0x08: return FunctionCode::Diagnostics;
        case 0x0B: return FunctionCode::GetCommEventCounter;
        case 0x0C: return FunctionCode::GetCommEventLog;
        case 0x0F: return FunctionCode::WriteMultipleCoils;
        case 0x10: return FunctionCode::WriteMultipleRegisters;
        case 0x11: return FunctionCode::ReportServerId;
        case 0x14: return FunctionCode::ReadFileRecord;
        case 0x15: return FunctionCode::WriteFileRecord;
        case 0x16: return FunctionCode::MaskWriteRegister;
        case 0x17: return FunctionCode::ReadWriteMultipleRegs;
        case 0x18: return FunctionCode::ReadFifoQueue;
        case 0x2B: return FunctionCode::EncapsulatedInterface;
        default:   return std::nullopt;
    }
}

constexpr std::string_view fcName(FunctionCode fc) {
    switch (fc) {
        case FunctionCode::ReadCoils:              return "ReadCoils";
        case FunctionCode::ReadDiscreteInputs:     return "ReadDiscreteInputs";
        case FunctionCode::ReadHoldingRegisters:   return "ReadHoldingRegisters";
        case FunctionCode::ReadInputRegisters:     return "ReadInputRegisters";
        case FunctionCode::WriteSingleCoil:        return "WriteSingleCoil";
        case FunctionCode::WriteSingleRegister:    return "WriteSingleRegister";
        case FunctionCode::ReadExceptionStatus:    return "ReadExceptionStatus";
        case FunctionCode::Diagnostics:            return "Diagnostics";
        case FunctionCode::GetCommEventCounter:    return "GetCommEventCounter";
        case FunctionCode::GetCommEventLog:        return "GetCommEventLog";
        case FunctionCode::WriteMultipleCoils:     return "WriteMultipleCoils";
        case FunctionCode::WriteMultipleRegisters: return "WriteMultipleRegisters";
        case FunctionCode::ReportServerId:         return "ReportServerId";
        case FunctionCode::ReadFileRecord:         return "ReadFileRecord";
        case FunctionCode::WriteFileRecord:        return "WriteFileRecord";
        case FunctionCode::MaskWriteRegister:      return "MaskWriteRegister";
        case FunctionCode::ReadWriteMultipleRegs:  return "ReadWriteMultipleRegs";
        case FunctionCode::ReadFifoQueue:          return "ReadFifoQueue";
        case FunctionCode::EncapsulatedInterface:  return "EncapsulatedInterface";
    }
    return "?";
}

/**@brief INI/config name → FunctionCode. */
std::optional<FunctionCode> parseFunctionCode(std::string_view name);

constexpr bool isReadFc(FunctionCode fc) {
    switch (fc) {
        case FunctionCode::ReadCoils:
        case FunctionCode::ReadDiscreteInputs:
        case FunctionCode::ReadHoldingRegisters:
        case FunctionCode::ReadInputRegisters:
        case FunctionCode::ReadExceptionStatus:
        case FunctionCode::GetCommEventCounter:
        case FunctionCode::GetCommEventLog:
        case FunctionCode::ReportServerId:
        case FunctionCode::ReadFileRecord:
        case FunctionCode::ReadFifoQueue:
        case FunctionCode::EncapsulatedInterface:
            return true;
        default: return false;
    }
}

constexpr bool isWriteFc(FunctionCode fc) {
    switch (fc) {
        case FunctionCode::WriteSingleCoil:
        case FunctionCode::WriteSingleRegister:
        case FunctionCode::WriteMultipleCoils:
        case FunctionCode::WriteMultipleRegisters:
        case FunctionCode::MaskWriteRegister:
        case FunctionCode::WriteFileRecord:
            return true;
        default: return false;
    }
}

/// FCs whose payloads address bit tables.
constexpr bool isBitFc(FunctionCode fc) {
    return fc == FunctionCode::ReadCoils ||
           fc == FunctionCode::ReadDiscreteInputs ||
           fc == FunctionCode::WriteSingleCoil ||
           fc == FunctionCode::WriteMultipleCoils;
}

/// Serial-line only FCs (not valid on TCP).
constexpr bool isSerialOnlyFc(FunctionCode fc) {
    return fc == FunctionCode::ReadExceptionStatus ||
           fc == FunctionCode::Diagnostics ||
           fc == FunctionCode::GetCommEventCounter ||
           fc == FunctionCode::GetCommEventLog ||
           fc == FunctionCode::ReportServerId;
}

/// FCs permitted in broadcast (unit 0) requests.
constexpr bool isBroadcastAllowed(FunctionCode fc) {
    return isWriteFc(fc);
}

/// Bit-table read FC for a given bit-table write FC (confirm path).
constexpr FunctionCode confirmReadFcFor(FunctionCode writeFc) {
    switch (writeFc) {
        case FunctionCode::WriteSingleCoil:
        case FunctionCode::WriteMultipleCoils:
            return FunctionCode::ReadCoils;
        default:
            return FunctionCode::ReadHoldingRegisters;
    }
}

/// Expected response payload shape for decode dispatch.
enum class ResponseKind { Bits, Registers, Echo, RangeAck, Custom };
constexpr ResponseKind expectedResponseKind(FunctionCode fc) {
    switch (fc) {
        case FunctionCode::ReadCoils:
        case FunctionCode::ReadDiscreteInputs:    return ResponseKind::Bits;
        case FunctionCode::ReadHoldingRegisters:
        case FunctionCode::ReadInputRegisters:
        case FunctionCode::ReadWriteMultipleRegs: return ResponseKind::Registers;
        case FunctionCode::WriteSingleCoil:
        case FunctionCode::WriteSingleRegister:
        case FunctionCode::MaskWriteRegister:
        case FunctionCode::WriteFileRecord:       return ResponseKind::Echo;
        case FunctionCode::WriteMultipleCoils:
        case FunctionCode::WriteMultipleRegisters: return ResponseKind::RangeAck;
        default:                                   return ResponseKind::Custom;
    }
}

} // namespace tether::modbus
