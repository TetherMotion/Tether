/**
 * @file Requests.hpp
 * @brief Typed request records + variant (plan AB.1).
 */
#pragma once

#include <cstdint>
#include <variant>
#include <vector>

namespace tether::modbus {

struct ReadRangeRequest       { uint16_t addr = 0, qty = 0; };
struct WriteSingleCoilRequest { uint16_t addr = 0; bool on = false; };
struct WriteSingleRegRequest  { uint16_t addr = 0, val = 0; };
struct WriteMultiCoilsRequest { uint16_t addr = 0, qty = 0;
                                std::vector<bool> bits; };
struct WriteMultiRegsRequest  { uint16_t addr = 0, qty = 0;
                                std::vector<uint16_t> regs; };
struct MaskWriteRequest       { uint16_t addr = 0, andMask = 0, orMask = 0; };
struct ReadWriteMultiRequest  { uint16_t raddr = 0, rqty = 0, waddr = 0,
                                wqty = 0; std::vector<uint16_t> regs; };

/**@brief FC20/FC21 file-record reference (type fixed to 6). */
struct FileRef {
    uint8_t  type = 6;
    uint16_t file = 0, record = 0, len = 0;
    std::vector<uint16_t> data;   ///< write only
    bool operator==(const FileRef&) const = default;
};
struct FileRequest { std::vector<FileRef> refs; };

struct DiagRequest   { uint16_t sub = 0; std::vector<uint8_t> data; };
struct DevIdRequest  { uint8_t readCode = 1, objectId = 0; };
struct FifoRequest   { uint16_t addr = 0; };
struct UnknownPdu    { uint8_t fc = 0; std::vector<uint8_t> body; };

using Request = std::variant<
    ReadRangeRequest, WriteSingleCoilRequest, WriteSingleRegRequest,
    WriteMultiCoilsRequest, WriteMultiRegsRequest, MaskWriteRequest,
    ReadWriteMultiRequest, FileRequest, DiagRequest, DevIdRequest,
    FifoRequest, UnknownPdu>;

} // namespace tether::modbus
