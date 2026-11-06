/**
 * @file Responses.hpp
 * @brief Typed response records + variant + device identification.
 */
#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <variant>
#include <vector>

#include "tether/modbus/protocol/Requests.hpp"

namespace tether::modbus {

struct BitsResponse       { std::vector<bool> bits; };
struct RegistersResponse  { std::vector<uint16_t> regs; };
struct EchoResponse       { uint16_t addr = 0, val = 0; }; // raw addr+word
struct RangeAckResponse   { uint16_t addr = 0, qty = 0; };
struct ExceptionStatusRsp { uint8_t status = 0; };
struct DiagResponse       { uint16_t sub = 0; std::vector<uint8_t> data; };
struct EventCtrResponse   { uint16_t status = 0, count = 0; };
struct EventLogResponse   { uint16_t status = 0, eventCnt = 0, msgCnt = 0;
                            std::vector<uint8_t> events; };
struct ServerIdResponse   { std::vector<uint8_t> id; };
struct FileRecordResponse { std::vector<FileRef> refs; };
struct FifoResponse       { std::vector<uint16_t> regs; };

/**@brief FC43/14 device-identification object map. */
struct DeviceIdentification {
    /// Object id → string value.
    std::map<uint8_t, std::string> objects;
    uint8_t conformity = 0;      ///< conformity level reported by device
    bool    more = false;        ///< more objects follow (need follow-up)
    uint8_t nextObjectId = 0;    ///< id to query next when more==true

    /// Convenience accessors for basic objects 0x00-0x02.
    std::string vendorName() const { return get(0x00); }
    std::string productCode() const { return get(0x01); }
    std::string revision() const { return get(0x02); }
    std::string get(uint8_t id) const {
        auto it = objects.find(id);
        return it == objects.end() ? std::string{} : it->second;
    }
};

struct MeiResponse {
    uint8_t mei = 0;
    std::vector<uint8_t> data;
    DeviceIdentification devid;   ///< populated when mei == 0x0E
};

using Response = std::variant<
    BitsResponse, RegistersResponse, EchoResponse, RangeAckResponse,
    ExceptionStatusRsp, DiagResponse, EventCtrResponse, EventLogResponse,
    ServerIdResponse, FileRecordResponse, FifoResponse, MeiResponse>;

} // namespace tether::modbus
