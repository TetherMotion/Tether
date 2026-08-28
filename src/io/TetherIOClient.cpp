/**
 * @file TetherIOClient.cpp
 * @brief C++ client implementation for the Tether IO binary protocol (Framing::None).
 */
#include "tether/io/TetherIOClient.hpp"

#include <algorithm>
#include <chrono>

namespace tether { namespace io {

namespace {

// ---- BufWriter helper (local, minimal) ----

class ClientBufWriter {
public:
    explicit ClientBufWriter(size_t cap) : buf_(cap) {}

    void putU8(uint8_t v) { ensure(1); buf_[pos_++] = v; }
    void putU16(uint16_t v) { ensure(2); buf_[pos_++] = v & 0xFF; buf_[pos_++] = (v >> 8) & 0xFF; }
    void putU32(uint32_t v) {
        ensure(4);
        buf_[pos_++] = v & 0xFF; buf_[pos_++] = (v >> 8) & 0xFF;
        buf_[pos_++] = (v >> 16) & 0xFF; buf_[pos_++] = (v >> 24) & 0xFF;
    }
    void putU64(uint64_t v) {
        ensure(8);
        for (int i = 0; i < 8; ++i) { buf_[pos_++] = (v >> (i * 8)) & 0xFF; }
    }
    void putF64(double v) { uint64_t bits; std::memcpy(&bits, &v, 8); putU64(bits); }
    void putBytes(const void* data, size_t len) {
        ensure(len); std::memcpy(buf_.data() + pos_, data, len); pos_ += len;
    }
    void putString16(std::string_view s) {
        putU16(static_cast<uint16_t>(s.size()));
        putBytes(s.data(), s.size());
    }
    void putVarint(uint32_t value) {
        while (value >= 0x80) { putU8((value & 0x7F) | 0x80); value >>= 7; }
        putU8(value & 0x7F);
    }

    std::vector<uint8_t> finish() { buf_.resize(pos_); return std::move(buf_); }

private:
    std::vector<uint8_t> buf_;
    size_t pos_ = 0;

    void ensure(size_t n) {
        if (pos_ + n > buf_.size()) buf_.resize(pos_ + n);
    }
};

// ---- BufReader helper (local, minimal) ----

class ClientBufReader {
public:
    ClientBufReader(const uint8_t* data, size_t len) : data_(data), len_(len) {}

    bool ok() const { return ok_ && pos_ <= len_; }
    size_t remaining() const { return ok_ ? (pos_ <= len_ ? len_ - pos_ : 0) : 0; }
    size_t pos() const { return pos_; }

    uint8_t getU8() {
        if (!ensure(1)) return 0;
        return data_[pos_++];
    }
    uint16_t getU16() {
        if (!ensure(2)) return 0;
        uint16_t v = data_[pos_] | (data_[pos_ + 1] << 8);
        pos_ += 2;
        return v;
    }
    uint32_t getU32() {
        if (!ensure(4)) return 0;
        uint32_t v = data_[pos_] | (data_[pos_ + 1] << 8) |
                     (data_[pos_ + 2] << 16) | (data_[pos_ + 3] << 24);
        pos_ += 4;
        return v;
    }
    uint64_t getU64() {
        if (!ensure(8)) return 0;
        uint64_t v = 0;
        for (int i = 0; i < 8; ++i) v |= static_cast<uint64_t>(data_[pos_ + i]) << (i * 8);
        pos_ += 8;
        return v;
    }
    double getF64() {
        uint64_t bits = getU64();
        double v;
        std::memcpy(&v, &bits, 8);
        return v;
    }
    uint32_t getVarint() {
        uint32_t value = 0;
        for (int shift = 0; shift < 35; shift += 7) {
            if (!ensure(1)) return 0;
            uint8_t byte = data_[pos_++];
            value |= (byte & 0x7F) << shift;
            if (!(byte & 0x80)) return value;
        }
        ok_ = false;
        return 0;
    }
    std::vector<uint8_t> getBytes(size_t n) {
        if (!ensure(n)) return {};
        std::vector<uint8_t> out(data_ + pos_, data_ + pos_ + n);
        pos_ += n;
        return out;
    }
    std::string getString16() {
        uint16_t len = getU16();
        if (!ok_ || !ensure(len)) return {};
        std::string s(reinterpret_cast<const char*>(data_ + pos_), len);
        pos_ += len;
        return s;
    }

private:
    const uint8_t* data_;
    size_t len_;
    size_t pos_ = 0;
    bool ok_ = true;

    bool ensure(size_t n) {
        if (pos_ + n > len_) { ok_ = false; return false; }
        return true;
    }
};

} // namespace

// ===========================================================================
// TetherIOClient implementation
// ===========================================================================

TetherIOClient::TetherIOClient(std::unique_ptr<ITransport> transport,
                               uint32_t defaultTimeoutMs)
    : transport_(std::move(transport))
    , defaultTimeoutMs_(defaultTimeoutMs) {}

TetherIOClient::~TetherIOClient() {
    close();
}

bool TetherIOClient::isConnected() const {
    return transport_ && transport_->isConnected();
}

void TetherIOClient::close() {
    if (transport_) transport_->close();
}

void TetherIOClient::setStreamDataCallback(StreamDataCallback cb) {
    streamCallback_ = std::move(cb);
}

void TetherIOClient::setLogDataCallback(LogDataCallback cb) {
    logCallback_ = std::move(cb);
}

// ---- Error helpers ----

ClientError TetherIOClient::makeError(ErrorCode code, std::string msg) {
    return {code, std::move(msg)};
}

ClientError TetherIOClient::parseError(const std::vector<uint8_t>& frame) {
    if (frame.size() < 2) return makeError(ErrorCode::InvalidMessage, "Malformed error frame");
    ClientBufReader r(frame.data(), frame.size());
    r.getU8(); // skip type byte
    uint32_t code = r.getU32();
    std::string msg = r.getString16();
    return makeError(static_cast<ErrorCode>(code), std::move(msg));
}

// ---- Low-level ----

bool TetherIOClient::sendRaw(const uint8_t* data, size_t len) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!transport_ || !transport_->isConnected()) return false;
    return transport_->send(data, len);
}

std::expected<std::vector<uint8_t>, ClientError>
TetherIOClient::receiveMessage(uint32_t timeoutMs) {
    if (!transport_ || !transport_->isConnected()) {
        return std::unexpected(makeError(ErrorCode::InternalError, "Not connected"));
    }
    uint32_t to = (timeoutMs == 0) ? defaultTimeoutMs_ : timeoutMs;
    std::vector<uint8_t> msg;
    if (!transport_->receiveMessage(msg, to)) {
        if (!transport_->isConnected()) {
            return std::unexpected(makeError(ErrorCode::InternalError, "Disconnected"));
        }
        return std::unexpected(makeError(ErrorCode::InternalError, "Receive timeout"));
    }
    if (msg.empty()) {
        return std::unexpected(makeError(ErrorCode::InvalidMessage, "Empty message"));
    }

    // Dispatch StreamData and LogData to callbacks, return everything else
    uint8_t type = msg[0];
    if (type == static_cast<uint8_t>(MessageType::StreamData)) {
        if (streamCallback_) {
            // Parse StreamData: type(1) + specId(u32) + count(u32) + rows
            ClientBufReader r(msg.data(), msg.size());
            r.getU8(); // type
            uint32_t specId = r.getU32();
            uint32_t count = r.getU32();
            for (uint32_t i = 0; i < count && r.ok(); ++i) {
                ClientStreamRow row;
                row.specId = specId;
                row.timestampUs = r.getU64();
                for (const auto& entry : streamLayout_) {
                    size_t sz = entry.valueSize > 0 ? entry.valueSize : r.getVarint();
                    row.values.push_back(r.getBytes(sz));
                }
                streamCallback_(row);
            }
        }
        // Recursively get the next message
        return receiveMessage(timeoutMs);
    }
    if (type == static_cast<uint8_t>(MessageType::LogData)) {
        if (logCallback_) {
            ClientBufReader r(msg.data(), msg.size());
            r.getU8(); // type
            r.getU64(); // timestampUs
            ClientLogRecord rec;
            rec.severity = r.getU8();
            rec.component = r.getString16();
            rec.message = r.getString16();
            rec.location = r.getString16();
            logCallback_(rec);
        }
        return receiveMessage(timeoutMs);
    }
    return msg;
}

std::expected<std::vector<uint8_t>, ClientError>
TetherIOClient::request(const std::vector<uint8_t>& payload,
                        MessageType expectedResponseType,
                        uint32_t timeoutMs) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!transport_ || !transport_->isConnected()) {
        return std::unexpected(makeError(ErrorCode::InternalError, "Not connected"));
    }
    if (!transport_->send(payload.data(), payload.size())) {
        return std::unexpected(makeError(ErrorCode::InternalError, "Send failed"));
    }

    uint32_t to = (timeoutMs == 0) ? defaultTimeoutMs_ : timeoutMs;
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(to);

    while (true) {
        uint32_t remain = static_cast<uint32_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now()).count());
        if (remain == 0) remain = 1;

        auto result = receiveMessage(remain);
        if (!result) return std::unexpected(result.error());

        const auto& frame = *result;
        if (frame.empty()) {
            return std::unexpected(makeError(ErrorCode::InvalidMessage, "Empty frame"));
        }

        uint8_t type = frame[0];
        if (type == static_cast<uint8_t>(expectedResponseType)) {
            return frame;
        }
        if (type == static_cast<uint8_t>(MessageType::Error)) {
            return std::unexpected(parseError(frame));
        }
        // Unexpected message type — keep waiting
    }
}

// ---- Ping ----

std::expected<uint32_t, ClientError> TetherIOClient::ping(uint32_t nonce) {
    ClientBufWriter w(6);
    w.putU8(static_cast<uint8_t>(MessageType::PingReq));
    w.putVarint(nonce);
    auto payload = w.finish();

    auto result = request(payload, MessageType::PongResp);
    if (!result) return std::unexpected(result.error());

    ClientBufReader r(result->data(), result->size());
    r.getU8(); // type
    uint32_t echoed = r.getVarint();
    if (!r.ok()) return std::unexpected(makeError(ErrorCode::InvalidMessage, "Malformed PongResp"));
    return echoed;
}

// ---- List Parameters ----

std::expected<std::vector<ClientCatalogEntry>, ClientError>
TetherIOClient::listParams(uint32_t offset, uint32_t maxCount) {
    ClientBufWriter w(9);
    w.putU8(static_cast<uint8_t>(MessageType::ListParamsReq));
    w.putU32(offset);
    w.putU32(maxCount);
    auto payload = w.finish();

    auto result = request(payload, MessageType::ListParamsResp);
    if (!result) return std::unexpected(result.error());

    ClientBufReader r(result->data(), result->size());
    r.getU8(); // type
    uint32_t total = r.getU32();
    uint32_t returnedOffset = r.getU32();
    uint32_t count = r.getU32();
    (void)total; (void)returnedOffset;

    std::vector<ClientCatalogEntry> entries;
    entries.reserve(count);
    for (uint32_t i = 0; i < count && r.ok(); ++i) {
        ClientCatalogEntry e;
        e.id = r.getU64();
        e.type = static_cast<ValueType>(r.getU8());
        e.valueSize = r.getU8();
        e.flags = r.getU8();
        e.name = r.getString16();
        e.description = r.getString16();
        e.group = r.getString16();
        entries.push_back(std::move(e));
    }
    if (!r.ok()) return std::unexpected(makeError(ErrorCode::InvalidMessage, "Malformed ListParamsResp"));
    return entries;
}

// ---- Get Parameter ----

std::expected<std::vector<uint8_t>, ClientError>
TetherIOClient::getParam(uint64_t id) {
    ClientBufWriter w(9);
    w.putU8(static_cast<uint8_t>(MessageType::GetParamReq));
    w.putU64(id);
    auto payload = w.finish();

    auto result = request(payload, MessageType::GetParamResp);
    if (!result) return std::unexpected(result.error());

    ClientBufReader r(result->data(), result->size());
    r.getU8(); // type
    uint64_t respId = r.getU64();
    (void)respId;
    uint8_t valueSize = r.getU8();
    size_t actualSize = valueSize;
    if (valueSize == 0) {
        actualSize = r.getVarint();
    }
    auto value = r.getBytes(actualSize);
    if (!r.ok()) return std::unexpected(makeError(ErrorCode::InvalidMessage, "Malformed GetParamResp"));
    return value;
}

// ---- Set Parameter (fixed) ----

std::expected<void, ClientError>
TetherIOClient::setParam(uint64_t id, const void* value, uint8_t valueSize) {
    ClientBufWriter w(10 + valueSize);
    w.putU8(static_cast<uint8_t>(MessageType::SetParamReq));
    w.putU64(id);
    w.putBytes(value, valueSize);
    auto payload = w.finish();

    auto result = request(payload, MessageType::SetParamResp);
    if (!result) return std::unexpected(result.error());
    return {};
}

// ---- Set Parameter (variable) ----

std::expected<void, ClientError>
TetherIOClient::setParamVar(uint64_t id, const void* value, size_t len) {
    ClientBufWriter w(15 + len);
    w.putU8(static_cast<uint8_t>(MessageType::SetParamReq));
    w.putU64(id);
    w.putVarint(static_cast<uint32_t>(len));
    w.putBytes(value, len);
    auto payload = w.finish();

    auto result = request(payload, MessageType::SetParamResp);
    if (!result) return std::unexpected(result.error());
    return {};
}

// ---- List Signals ----

std::expected<std::vector<ClientCatalogEntry>, ClientError>
TetherIOClient::listSignals(uint32_t offset, uint32_t maxCount) {
    ClientBufWriter w(9);
    w.putU8(static_cast<uint8_t>(MessageType::ListSignalsReq));
    w.putU32(offset);
    w.putU32(maxCount);
    auto payload = w.finish();

    auto result = request(payload, MessageType::ListSignalsResp);
    if (!result) return std::unexpected(result.error());

    ClientBufReader r(result->data(), result->size());
    r.getU8(); // type
    uint32_t total = r.getU32();
    uint32_t returnedOffset = r.getU32();
    uint32_t count = r.getU32();
    (void)total; (void)returnedOffset;

    std::vector<ClientCatalogEntry> entries;
    entries.reserve(count);
    for (uint32_t i = 0; i < count && r.ok(); ++i) {
        ClientCatalogEntry e;
        e.id = r.getU64();
        e.type = static_cast<ValueType>(r.getU8());
        e.valueSize = r.getU8();
        e.flags = r.getU8();
        e.name = r.getString16();
        e.description = r.getString16();
        e.group = r.getString16();
        entries.push_back(std::move(e));
    }
    if (!r.ok()) return std::unexpected(makeError(ErrorCode::InvalidMessage, "Malformed ListSignalsResp"));
    return entries;
}

// ---- Get Signal ----

std::expected<std::vector<uint8_t>, ClientError>
TetherIOClient::getSignal(uint64_t id) {
    ClientBufWriter w(9);
    w.putU8(static_cast<uint8_t>(MessageType::GetSignalReq));
    w.putU64(id);
    auto payload = w.finish();

    auto result = request(payload, MessageType::GetSignalResp);
    if (!result) return std::unexpected(result.error());

    ClientBufReader r(result->data(), result->size());
    r.getU8(); // type
    uint64_t respId = r.getU64();
    (void)respId;
    uint8_t valueSize = r.getU8();
    size_t actualSize = valueSize;
    if (valueSize == 0) {
        actualSize = r.getVarint();
    }
    auto value = r.getBytes(actualSize);
    if (!r.ok()) return std::unexpected(makeError(ErrorCode::InvalidMessage, "Malformed GetSignalResp"));
    return value;
}

// ---- Get Metadata ----

std::expected<std::vector<ClientMetadata>, ClientError>
TetherIOClient::getMetadata(uint64_t id) {
    ClientBufWriter w(9);
    w.putU8(static_cast<uint8_t>(MessageType::GetMetadataReq));
    w.putU64(id);
    auto payload = w.finish();

    auto result = request(payload, MessageType::GetMetadataResp);
    if (!result) return std::unexpected(result.error());

    ClientBufReader r(result->data(), result->size());
    r.getU8(); // type
    uint64_t respId = r.getU64();
    (void)respId;
    uint32_t count = r.getU32();

    std::vector<ClientMetadata> metadata;
    metadata.reserve(count);
    for (uint32_t i = 0; i < count && r.ok(); ++i) {
        ClientMetadata m;
        m.key = r.getString16();
        m.value = r.getString16();
        metadata.push_back(std::move(m));
    }
    if (!r.ok()) return std::unexpected(makeError(ErrorCode::InvalidMessage, "Malformed GetMetadataResp"));
    return metadata;
}

// ---- List Functions ----

std::expected<std::vector<ClientFunctionEntry>, ClientError>
TetherIOClient::listFunctions(uint32_t offset, uint32_t maxCount) {
    ClientBufWriter w(9);
    w.putU8(static_cast<uint8_t>(MessageType::ListFunctionsReq));
    w.putU32(offset);
    w.putU32(maxCount);
    auto payload = w.finish();

    auto result = request(payload, MessageType::ListFunctionsResp);
    if (!result) return std::unexpected(result.error());

    ClientBufReader r(result->data(), result->size());
    r.getU8(); // type
    uint32_t total = r.getU32();
    uint32_t returnedOffset = r.getU32();
    uint32_t count = r.getU32();
    (void)total; (void)returnedOffset;

    std::vector<ClientFunctionEntry> functions;
    functions.reserve(count);
    for (uint32_t i = 0; i < count && r.ok(); ++i) {
        ClientFunctionEntry f;
        f.id = r.getU64();
        f.name = r.getString16();
        f.description = r.getString16();
        f.group = r.getString16();
        uint32_t paramCount = r.getU32();
        for (uint32_t p = 0; p < paramCount && r.ok(); ++p) {
            ClientFunctionParam param;
            param.name = r.getString16();
            param.description = r.getString16();
            param.type = static_cast<ValueType>(r.getU8());
            param.flags = r.getU8();
            r.getU64(); // enumReference
            r.getU64(); // structReference
            param.maxValueSize = r.getU32();
            uint8_t hasDesc = r.getU8();
            if (hasDesc) {
                uint32_t descSize = r.getU32();
                r.getBytes(descSize); // skip value descriptor
            }
            if (param.flags & 2) { // hasDefault
                uint32_t defaultLen = r.getVarint();
                param.hasDefault = true;
                param.defaultValue = r.getBytes(defaultLen);
            }
            uint32_t metaCount = r.getU32();
            for (uint32_t m = 0; m < metaCount && r.ok(); ++m) {
                r.getString16(); // key
                r.getString16(); // value
            }
            f.parameters.push_back(std::move(param));
        }
        f.hasReturnValue = r.getU8() != 0;
        if (f.hasReturnValue) {
            r.getString16(); // return name
            r.getString16(); // return description
            f.returnType = static_cast<ValueType>(r.getU8());
            r.getU8();    // return flags
            r.getU64();   // enumReference
            r.getU64();   // structReference
            r.getU32();   // maxValueSize
            uint8_t hasDesc = r.getU8();
            if (hasDesc) {
                uint32_t descSize = r.getU32();
                r.getBytes(descSize);
            }
            uint32_t metaCount = r.getU32();
            for (uint32_t m = 0; m < metaCount && r.ok(); ++m) {
                r.getString16();
                r.getString16();
            }
        }
        uint32_t metaCount = r.getU32();
        for (uint32_t m = 0; m < metaCount && r.ok(); ++m) {
            r.getString16();
            r.getString16();
        }
        functions.push_back(std::move(f));
    }
    if (!r.ok()) return std::unexpected(makeError(ErrorCode::InvalidMessage, "Malformed ListFunctionsResp"));
    return functions;
}

// ---- Call Function ----

std::expected<ClientCallResult, ClientError>
TetherIOClient::callFunction(uint64_t functionId, const std::vector<FunctionArg>& args) {
    size_t argSize = 4;
    for (const auto& a : args) argSize += 4 + 1 + 4 + a.value.size();
    ClientBufWriter w(14 + argSize);
    w.putU8(static_cast<uint8_t>(MessageType::CallFunctionReq));
    w.putU64(functionId);
    w.putU32(static_cast<uint32_t>(args.size()));
    for (const auto& a : args) {
        w.putU32(a.position);
        w.putU8(static_cast<uint8_t>(a.type));
        w.putU32(static_cast<uint32_t>(a.value.size()));
        w.putBytes(a.value.data(), a.value.size());
    }
    auto payload = w.finish();

    auto result = request(payload, MessageType::CallFunctionResp);
    if (!result) return std::unexpected(result.error());

    ClientBufReader r(result->data(), result->size());
    r.getU8(); // type
    ClientCallResult cr;
    cr.functionId = r.getU64();
    // Server writes 0 for success, 1 for failure
    cr.success = r.getU8() == 0;
    if (!cr.success) {
        cr.errorCode = r.getU32();
        cr.errorMessage = r.getString16();
    } else {
        // Success path: putU32(0) + putU16(0) + optional TLV
        r.getU32(); // reserved (0)
        r.getU16(); // reserved (0)
        // Check for optional return value TLV: [position(4) + type(1) + length(4) + value]
        if (r.remaining() >= 9) {
            uint32_t pos = r.getU32();
            (void)pos;
            cr.hasReturnValue = true;
            cr.returnType = static_cast<ValueType>(r.getU8());
            uint32_t len = r.getU32();
            cr.returnValue = r.getBytes(len);
        }
    }
    if (!r.ok()) return std::unexpected(makeError(ErrorCode::InvalidMessage, "Malformed CallFunctionResp"));
    return cr;
}

// ---- Configure Stream ----

std::expected<TetherIOClient::ConfigureStreamResult, ClientError>
TetherIOClient::configureStream(const std::vector<uint64_t>& entryIds,
                                uint32_t intervalMs,
                                uint32_t chunkSize,
                                uint32_t skipCount,
                                uint8_t triggerMode,
                                uint64_t triggerEntryId) {
    size_t cap = 1 + 1 + 4 + 4 + 4 + 8 + 4 + entryIds.size() * 8 + 4;
    ClientBufWriter w(cap);
    w.putU8(static_cast<uint8_t>(MessageType::ConfigureStream));
    w.putU8(triggerMode);
    w.putU32(intervalMs);
    w.putU32(chunkSize);
    w.putU32(skipCount);
    w.putU64(triggerEntryId);
    w.putU32(static_cast<uint32_t>(entryIds.size()));
    for (uint64_t id : entryIds) w.putU64(id);
    w.putU32(0); // filterCount
    auto payload = w.finish();

    auto result = request(payload, MessageType::ConfigureStreamAck);
    if (!result) return std::unexpected(result.error());

    ClientBufReader r(result->data(), result->size());
    r.getU8(); // type
    ConfigureStreamResult cr;
    cr.specId = r.getU32();
    uint32_t count = r.getU32();
    uint32_t rowSize = r.getU32();
    (void)rowSize;
    for (uint32_t i = 0; i < count && r.ok(); ++i) {
        ClientStreamLayoutEntry e;
        e.id = r.getU64();
        e.type = static_cast<ValueType>(r.getU8());
        e.valueSize = r.getU8();
        cr.layout.push_back(std::move(e));
    }
    if (!r.ok()) return std::unexpected(makeError(ErrorCode::InvalidMessage, "Malformed ConfigureStreamAck"));
    streamLayout_ = cr.layout;
    return cr;
}

// ---- Start/Stop Stream ----

std::expected<void, ClientError> TetherIOClient::startStream() {
    std::vector<uint8_t> payload = {static_cast<uint8_t>(MessageType::StartStream)};
    // StartStream has no response — just send
    if (!sendRaw(payload.data(), payload.size())) {
        return std::unexpected(makeError(ErrorCode::InternalError, "Send failed"));
    }
    return {};
}

std::expected<void, ClientError> TetherIOClient::stopStream() {
    std::vector<uint8_t> payload = {static_cast<uint8_t>(MessageType::StopStream)};
    if (!sendRaw(payload.data(), payload.size())) {
        return std::unexpected(makeError(ErrorCode::InternalError, "Send failed"));
    }
    return {};
}

// ---- Snapshots ----

std::expected<std::pair<uint64_t, std::vector<ClientSnapshotValue>>, ClientError>
TetherIOClient::snapshotParams(const std::vector<uint64_t>& ids) {
    ClientBufWriter w(6 + ids.size() * 8);
    w.putU8(static_cast<uint8_t>(MessageType::SnapshotParamsReq));
    w.putU32(static_cast<uint32_t>(ids.size()));
    for (uint64_t id : ids) w.putU64(id);
    auto payload = w.finish();

    auto result = request(payload, MessageType::SnapshotParamsResp);
    if (!result) return std::unexpected(result.error());

    ClientBufReader r(result->data(), result->size());
    r.getU8(); // type
    uint64_t ts = r.getU64();
    uint32_t count = r.getU32();
    std::vector<ClientSnapshotValue> values;
    values.reserve(count);
    for (uint32_t i = 0; i < count && r.ok(); ++i) {
        ClientSnapshotValue v;
        v.id = r.getU64();
        v.valueSize = r.getU8();
        size_t actualSize = v.valueSize;
        if (v.valueSize == 0) actualSize = r.getVarint();
        v.value = r.getBytes(actualSize);
        values.push_back(std::move(v));
    }
    if (!r.ok()) return std::unexpected(makeError(ErrorCode::InvalidMessage, "Malformed SnapshotParamsResp"));
    return std::make_pair(ts, std::move(values));
}

std::expected<std::pair<uint64_t, std::vector<ClientSnapshotValue>>, ClientError>
TetherIOClient::snapshotSignals(const std::vector<uint64_t>& ids) {
    ClientBufWriter w(6 + ids.size() * 8);
    w.putU8(static_cast<uint8_t>(MessageType::SnapshotSignalsReq));
    w.putU32(static_cast<uint32_t>(ids.size()));
    for (uint64_t id : ids) w.putU64(id);
    auto payload = w.finish();

    auto result = request(payload, MessageType::SnapshotSignalsResp);
    if (!result) return std::unexpected(result.error());

    ClientBufReader r(result->data(), result->size());
    r.getU8(); // type
    uint64_t ts = r.getU64();
    uint32_t count = r.getU32();
    std::vector<ClientSnapshotValue> values;
    values.reserve(count);
    for (uint32_t i = 0; i < count && r.ok(); ++i) {
        ClientSnapshotValue v;
        v.id = r.getU64();
        v.valueSize = r.getU8();
        size_t actualSize = v.valueSize;
        if (v.valueSize == 0) actualSize = r.getVarint();
        v.value = r.getBytes(actualSize);
        values.push_back(std::move(v));
    }
    if (!r.ok()) return std::unexpected(makeError(ErrorCode::InvalidMessage, "Malformed SnapshotSignalsResp"));
    return std::make_pair(ts, std::move(values));
}

// ---- Feature Exchange ----

std::expected<std::vector<ClientFeature>, ClientError>
TetherIOClient::featureExchange(const std::vector<ClientFeature>& clientFeatures) {
    size_t cap = 6;
    for (const auto& f : clientFeatures) cap += 2 + f.name.size() + 1 + 4 + f.value.size();
    ClientBufWriter w(cap);
    w.putU8(static_cast<uint8_t>(MessageType::FeatureExchangeReq));
    w.putU32(static_cast<uint32_t>(clientFeatures.size()));
    for (const auto& f : clientFeatures) {
        w.putString16(f.name);
        w.putU8(f.type);
        w.putU32(static_cast<uint32_t>(f.value.size()));
        w.putBytes(f.value.data(), f.value.size());
    }
    auto payload = w.finish();

    auto result = request(payload, MessageType::FeatureExchangeResp);
    if (!result) return std::unexpected(result.error());

    ClientBufReader r(result->data(), result->size());
    r.getU8(); // type
    uint32_t count = r.getU32();
    std::vector<ClientFeature> features;
    features.reserve(count);
    for (uint32_t i = 0; i < count && r.ok(); ++i) {
        ClientFeature f;
        f.name = r.getString16();
        f.type = r.getU8();
        uint32_t valueLen = r.getU32();
        f.value = r.getBytes(valueLen);
        features.push_back(std::move(f));
    }
    if (!r.ok()) return std::unexpected(makeError(ErrorCode::InvalidMessage, "Malformed FeatureExchangeResp"));
    return features;
}

// ---- Log Subscription ----

std::expected<ClientLogSubscription, ClientError>
TetherIOClient::subscribeLog(uint8_t minSeverity,
                             const std::string& componentFilter,
                             const std::string& messageFilter,
                             const std::string& locationFilter) {
    ClientBufWriter w(16 + componentFilter.size() + messageFilter.size() + locationFilter.size());
    w.putU8(static_cast<uint8_t>(MessageType::SubscribeLogReq));
    w.putU8(minSeverity);
    w.putString16(componentFilter);
    w.putString16(messageFilter);
    w.putString16(locationFilter);
    auto payload = w.finish();

    auto result = request(payload, MessageType::SubscribeLogResp);
    if (!result) return std::unexpected(result.error());

    ClientBufReader r(result->data(), result->size());
    r.getU8(); // type
    ClientLogSubscription sub;
    sub.subscriptionId = r.getVarint();
    sub.failed = r.getU8() != 0;
    if (sub.failed) {
        sub.errorMessage = r.getString16();
    }
    if (!r.ok()) return std::unexpected(makeError(ErrorCode::InvalidMessage, "Malformed SubscribeLogResp"));
    return sub;
}

std::expected<uint32_t, ClientError>
TetherIOClient::unsubscribeLog(uint32_t subscriptionId) {
    ClientBufWriter w(6);
    w.putU8(static_cast<uint8_t>(MessageType::UnsubscribeLogReq));
    w.putVarint(subscriptionId);
    auto payload = w.finish();

    auto result = request(payload, MessageType::UnsubscribeLogResp);
    if (!result) return std::unexpected(result.error());

    ClientBufReader r(result->data(), result->size());
    r.getU8(); // type
    uint32_t respId = r.getVarint();
    uint8_t notFound = r.getU8();
    (void)notFound;
    if (!r.ok()) return std::unexpected(makeError(ErrorCode::InvalidMessage, "Malformed UnsubscribeLogResp"));
    return respId;
}

// ---- Datalog ----

std::expected<bool, ClientError>
TetherIOClient::configureDatalog(const std::string& logName,
                                 uint32_t sampleRateHz,
                                 bool enabled,
                                 const std::vector<uint64_t>& entryIds) {
    ClientBufWriter w(16 + logName.size() + entryIds.size() * 8);
    w.putU8(static_cast<uint8_t>(MessageType::ConfigureDatalogReq));
    w.putString16(logName);
    w.putU32(sampleRateHz);
    w.putU8(enabled ? 1 : 0);
    w.putU32(static_cast<uint32_t>(entryIds.size()));
    for (uint64_t id : entryIds) w.putU64(id);
    auto payload = w.finish();

    auto result = request(payload, MessageType::ConfigureDatalogResp);
    if (!result) return std::unexpected(result.error());

    ClientBufReader r(result->data(), result->size());
    r.getU8(); // type
    bool success = r.getU8() != 0;
    if (!r.ok()) return std::unexpected(makeError(ErrorCode::InvalidMessage, "Malformed ConfigureDatalogResp"));
    return success;
}

// ---- Threshold ----

std::expected<bool, ClientError>
TetherIOClient::configureThreshold(const std::string& name,
                                   bool isWhitelist,
                                   const std::vector<uint8_t>& encodedRules) {
    // The encoded rules are passed as-is to the server.
    ClientBufWriter w(8 + name.size() + encodedRules.size());
    w.putU8(static_cast<uint8_t>(MessageType::ConfigureThresholdReq));
    w.putString16(name);
    w.putU8(isWhitelist ? 1 : 0);
    w.putBytes(encodedRules.data(), encodedRules.size());
    auto payload = w.finish();

    auto result = request(payload, MessageType::ConfigureThresholdResp);
    if (!result) return std::unexpected(result.error());

    ClientBufReader r(result->data(), result->size());
    r.getU8(); // type
    bool success = r.getU8() != 0;
    if (!r.ok()) return std::unexpected(makeError(ErrorCode::InvalidMessage, "Malformed ConfigureThresholdResp"));
    return success;
}

// ---- Describe Struct ----

std::expected<TetherIOClient::StructDescriptor, ClientError>
TetherIOClient::describeStruct(uint64_t id) {
    ClientBufWriter w(9);
    w.putU8(static_cast<uint8_t>(MessageType::DescribeStructReq));
    w.putU64(id);
    auto payload = w.finish();

    auto result = request(payload, MessageType::DescribeStructResp);
    if (!result) return std::unexpected(result.error());

    ClientBufReader r(result->data(), result->size());
    r.getU8(); // type
    StructDescriptor sd;
    sd.entryId = r.getU64();
    sd.structName = r.getString16();
    sd.totalSize = r.getU32();
    uint32_t fieldCount = r.getU32();
    for (uint32_t i = 0; i < fieldCount && r.ok(); ++i) {
        StructField f;
        f.name = r.getString16();
        f.type = static_cast<ValueType>(r.getU8());
        f.offset = r.getU16();
        f.size = r.getU16();
        f.unit = r.getString16();
        sd.fields.push_back(std::move(f));
    }
    if (!r.ok()) return std::unexpected(makeError(ErrorCode::InvalidMessage, "Malformed DescribeStructResp"));
    return sd;
}

// ---- Input Streams ----

std::expected<TetherIOClient::CreateInputStreamResult, ClientError>
TetherIOClient::createInputStream(uint32_t maxValueSize,
                                  uint32_t maxBatchSize,
                                  const std::vector<uint8_t>& encodedValueDescriptor) {
    ClientBufWriter w(14 + encodedValueDescriptor.size());
    w.putU8(static_cast<uint8_t>(MessageType::CreateInputStreamReq));
    w.putU32(maxValueSize);
    w.putU32(maxBatchSize);
    w.putBytes(encodedValueDescriptor.data(), encodedValueDescriptor.size());
    auto payload = w.finish();

    auto result = request(payload, MessageType::CreateInputStreamResp);
    if (!result) return std::unexpected(result.error());

    ClientBufReader r(result->data(), result->size());
    r.getU8(); // type
    CreateInputStreamResult cr;
    cr.streamId = r.getU32();
    cr.success = r.getU8() != 0;
    if (!r.ok()) return std::unexpected(makeError(ErrorCode::InvalidMessage, "Malformed CreateInputStreamResp"));
    return cr;
}

std::expected<void, ClientError>
TetherIOClient::inputStreamData(uint32_t streamId,
                                const std::vector<std::vector<uint8_t>>& values) {
    size_t cap = 10;
    for (const auto& v : values) cap += 4 + v.size();
    ClientBufWriter w(cap);
    w.putU8(static_cast<uint8_t>(MessageType::InputStreamData));
    w.putU32(streamId);
    w.putU32(static_cast<uint32_t>(values.size()));
    for (const auto& v : values) {
        w.putU32(static_cast<uint32_t>(v.size()));
        w.putBytes(v.data(), v.size());
    }
    auto payload = w.finish();

    // InputStreamData may not get a response unless there's an error
    if (!sendRaw(payload.data(), payload.size())) {
        return std::unexpected(makeError(ErrorCode::InternalError, "Send failed"));
    }
    return {};
}

std::expected<std::pair<uint32_t, bool>, ClientError>
TetherIOClient::closeInputStream(uint32_t streamId) {
    ClientBufWriter w(6);
    w.putU8(static_cast<uint8_t>(MessageType::CloseInputStreamReq));
    w.putU32(streamId);
    auto payload = w.finish();

    auto result = request(payload, MessageType::CloseInputStreamResp);
    if (!result) return std::unexpected(result.error());

    ClientBufReader r(result->data(), result->size());
    r.getU8(); // type
    uint32_t respId = r.getU32();
    bool success = r.getU8() != 0;
    if (!r.ok()) return std::unexpected(makeError(ErrorCode::InvalidMessage, "Malformed CloseInputStreamResp"));
    return std::make_pair(respId, success);
}

}} // namespace tether::io
