/**
 * @file Dispatcher.hpp
 * @brief Request → response core shared by RTU/TCP servers (plan §9.4).
 */
#pragma once

#include <functional>

#include "tether/modbus/link/Framer.hpp"
#include "tether/modbus/protocol/Codec.hpp"
#include "tether/modbus/server/DataStore.hpp"

namespace tether::modbus {

struct ServerOpts {
    /// Extra function access control; default allows everything the
    /// model covers. Return false → IllegalFunction.
    std::function<bool(uint8_t unit, FunctionCode)> allow;
    /// Called for diag FC8 subfunctions; default → IllegalFunction.
    std::function<Expected<DiagResponse>(uint8_t unit, uint16_t sub,
                                        const std::vector<uint8_t>&)>
        onDiagnostics;
    /// FC43 read-device-identification handler; default → IllegalFunction.
    std::function<Expected<Response>(uint8_t unit, const DevIdRequest&)>
        onDeviceId;
    /// FC17 report-server-id handler.
    std::function<std::vector<uint8_t>(uint8_t unit)> onServerId;
    /// Unit ids served by this dispatcher (empty = accept all).
    std::vector<uint8_t> units;
    /// Broadcast writes allowed (default true).
    bool allowBroadcastWrites = true;
};

/**
 * @brief Stateless PDU dispatcher: request Adu → response PDU.
 *
 * Handles: range checks, table mapping, bit pack/unpack, exception
 * translation, broadcast silence (unit 0 → `respond=false`).
 */
class RequestDispatcher {
public:
    RequestDispatcher(IDataModel& model, ServerOpts opts = {})
        : model_(model), opts_(std::move(opts)) {}

    struct Result {
        bool respond = true;
        std::vector<uint8_t> pdu;   ///< full PDU incl. fc byte
    };
    Result dispatch(uint8_t unit, std::span<const uint8_t> reqPdu);

private:
    Expected<Response> handle(uint8_t unit, FunctionCode fc,
                              const Request& req);
    Expected<Response> doRead(TableId t, FunctionCode fc,
                              const ReadRangeRequest& r);
    Expected<Response> doWrite(uint8_t unit, TableId t, FunctionCode fc,
                               const Request& req);
    bool unitOk(uint8_t u) const {
        return opts_.units.empty() ||
               std::find(opts_.units.begin(), opts_.units.end(), u) !=
                   opts_.units.end();
    }

    IDataModel& model_;
    ServerOpts  opts_;
};

} // namespace tether::modbus
