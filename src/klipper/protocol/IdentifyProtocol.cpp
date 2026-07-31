/**
 * @file IdentifyProtocol.cpp
 * @brief IdentifyServer / IdentifyClient implementation.
 */

#include "tether/klipper/protocol/IdentifyProtocol.hpp"
#include "tether/klipper/protocol/CommandTable.hpp"

namespace tether::klipper::protocol {

namespace {

// Hard-coded format strings for the identify handshake.
//   identify_response offset=%u data=%.*s   (msgid 0)
//   identify offset=%u count=%c            (msgid 1)
// We encode/decode these directly with VLQ since the dictionary is not yet
// available when the handshake runs.

void encodeMsgIdAndParams(uint16_t msgid, const std::vector<int32_t>& intParams,
                          const std::vector<std::vector<uint8_t>>& strParams,
                          std::vector<uint8_t>& out) {
    uint8_t mid[2];
    size_t mn = encodeMsgId(msgid, mid);
    out.insert(out.end(), mid, mid + mn);
    size_t si = 0;
    for (size_t i = 0; i < intParams.size(); ++i) {
        uint8_t buf[5];
        size_t n = encodeParam(intParams[i], buf);
        out.insert(out.end(), buf, buf + n);
    }
    for (size_t i = 0; i < strParams.size(); ++i) {
        const auto& s = strParams[i];
        uint8_t buf[kMaxBufferLength + 5];
        size_t n = encodeParam(static_cast<int32_t>(s.size()), buf);
        out.insert(out.end(), buf, buf + n);
        out.insert(out.end(), s.begin(), s.end());
        (void)si;
    }
}

} // namespace

std::vector<uint8_t> IdentifyServer::buildResponseContent(uint32_t offset, uint8_t count) const {
    std::vector<uint8_t> out;
    std::vector<uint8_t> chunk;
    if (offset < blob_.size()) {
        uint32_t avail = static_cast<uint32_t>(blob_.size() - offset);
        uint32_t n = std::min<uint32_t>(count, avail);
        chunk.assign(blob_.begin() + offset, blob_.begin() + offset + n);
    }
    // Always emit offset + data (length-prefixed); empty data signals end.
    encodeMsgIdAndParams(kMsgIdIdentifyResponse, {static_cast<int32_t>(offset)}, {std::move(chunk)}, out);
    return out;
}

std::vector<uint8_t> IdentifyClient::buildRequestContent(uint8_t count) const {
    std::vector<uint8_t> out;
    encodeMsgIdAndParams(kMsgIdIdentify, {static_cast<int32_t>(nextOffset()), static_cast<int32_t>(count)}, {}, out);
    return out;
}

bool IdentifyClient::consumeResponseContent(std::span<const uint8_t> content) {
    const uint8_t* p = content.data();
    const uint8_t* end = p + content.size();
    auto msgid = decodeMsgId(p, end);
    if (!msgid || *msgid != kMsgIdIdentifyResponse) return false;
    // offset
    auto offsetOpt = decodeParam(p, end);
    if (!offsetOpt) return false;
    uint32_t offset = static_cast<uint32_t>(*offsetOpt);
    // data (length-prefixed)
    auto lenOpt = decodeParam(p, end);
    if (!lenOpt) return false;
    int32_t len = *lenOpt;
    if (len < 0) return false;
    if (p + len > end) return false;
    if (offset != received_.size()) return false; // out-of-order
    received_.insert(received_.end(), p, p + len);
    if (len == 0) complete_ = true;
    return true;
}

std::optional<DataDictionary> IdentifyClient::decodeDictionary() const {
    if (!complete_) return std::nullopt;
    std::string json = DataDictionary::fromWire(received_);
    DataDictionary dict;
    if (!dict.fromJson(json)) return std::nullopt;
    return dict;
}

} // namespace tether::klipper::protocol
