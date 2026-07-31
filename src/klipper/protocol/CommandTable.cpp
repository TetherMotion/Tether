/**
 * @file CommandTable.cpp
 * @brief CommandTable implementation: message encode/decode + dispatch.
 */

#include "tether/klipper/protocol/CommandTable.hpp"

namespace tether::klipper::protocol {

bool encodeMessage(const DataDictionary& dict, uint16_t msgid,
                   std::span<const ParamValue> params, std::vector<uint8_t>& out) {
    const MessageEntry* e = dict.lookupMsgid(msgid);
    if (!e) return false;
    if (params.size() != e->format.arity()) return false;
    // msgid VLQ
    uint8_t mid[2];
    size_t mn = encodeMsgId(msgid, mid);
    out.insert(out.end(), mid, mid + mn);
    // parameters
    uint8_t buf[kMaxBufferLength + 5];
    for (size_t i = 0; i < params.size(); ++i) {
        const auto& pv = params[i];
        size_t n = encodeParamValue(e->format.params[i].type, pv.integer, pv.bytes, buf);
        if (n == 0) return false;
        out.insert(out.end(), buf, buf + n);
    }
    return true;
}

std::vector<DecodedMessage> decodeMessages(const DataDictionary& dict,
                                            std::span<const uint8_t> content) {
    std::vector<DecodedMessage> result;
    const uint8_t* p = content.data();
    const uint8_t* end = p + content.size();
    while (p < end) {
        auto msgidOpt = decodeMsgId(p, end);
        if (!msgidOpt) break;
        const MessageEntry* e = dict.lookupMsgid(*msgidOpt);
        if (!e) break; // unknown msgid: stop
        DecodedMessage msg;
        msg.msgid = *msgidOpt;
        for (const auto& ps : e->format.params) {
            ParamValue pv;
            if (!decodeParamValue(ps.type, p, end, pv.integer, pv.bytes)) {
                return result; // truncated; return what we have
            }
            pv.isInteger = isIntegerType(ps.type);
            msg.params.push_back(std::move(pv));
        }
        result.push_back(std::move(msg));
    }
    return result;
}

void CommandTable::registerCommand(uint16_t msgid, CommandHandler handler) {
    commandHandlers_[msgid] = std::move(handler);
}
void CommandTable::registerResponse(uint16_t msgid, ResponseHandler handler) {
    responseHandlers_[msgid] = std::move(handler);
}
void CommandTable::dispatchCommand(const DecodedMessage& msg) const {
    auto it = commandHandlers_.find(msg.msgid);
    if (it != commandHandlers_.end()) it->second(msg.params);
}
void CommandTable::dispatchResponse(const DecodedMessage& msg) const {
    auto it = responseHandlers_.find(msg.msgid);
    if (it != responseHandlers_.end()) it->second(msg.params);
}

} // namespace tether::klipper::protocol
