/**
 * @file Session.cpp
 * @brief Per-client session: message deframing (SLIP or transport-native),
 *        protocol handling, streaming.
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#include "tether/io/Session.hpp"
#include "tether/io/SchemaValueCodec.hpp"
#include "SLIPStream/Buffer.hpp"

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <limits>

namespace tether { namespace io {

namespace {

bool validateCatalogValue(const SchemaCatalog* catalog, const EntryView& entry,
                 const uint8_t* data, size_t size) {
    if (!catalog || entry.schemaRef().key == SchemaKey{} || !catalog->graph()) return true;
    BufReader reader(data, size);
    return validateSchemaValue(*catalog->graph(), entry.schemaRef().key, reader) &&
        reader.remaining() == 0;
}

} // namespace

// --------------------------------------------------------------------------
// Construction / Destruction
// --------------------------------------------------------------------------

Session::Session(std::unique_ptr<ITransport> transport,
                 Registry& registry,
                 TimestampFn tsFn,
                 LogFn logFn,
                 const FeatureSet* serverFeatures,
                 DatalogRecorder* datalogRecorder,
                 InputStreamCreateFn inputStreamCreateFn,
                 InputStreamDataFn inputStreamDataFn,
                 ReceiveBufferFactory encodedBufferFactory,
                 ReceiveBufferFactory decodedBufferFactory,
                 Framing framing,
                 const std::vector<IRingStreamSource*>* ringSources,
                 const SchemaCatalog* schemaCatalog)
    : transport_(std::move(transport))
    , registry_(registry)
    , getTimestampUs_(tsFn)
    , logFn_(logFn)
    , serverFeatures_(serverFeatures)
    , datalogRecorder_(datalogRecorder)
    , inputStreamCreateFn_(std::move(inputStreamCreateFn))
    , inputStreamDataFn_(std::move(inputStreamDataFn))
    , framing_(framing)
    , ringSources_(ringSources)
    , schemaCatalog_(schemaCatalog)
    , slipRxBuf_(encodedBufferFactory ? encodedBufferFactory()
                                      : std::make_unique<DynamicReceiveBuffer>(
                                            DEFAULT_RECEIVE_BUFFER_CAPACITY,
                                            MAX_ENCODED_MESSAGE_SIZE))
    , decodeBuf_(decodedBufferFactory ? decodedBufferFactory()
                                      : std::make_unique<DynamicReceiveBuffer>(
                                            DEFAULT_RECEIVE_BUFFER_CAPACITY,
                                            MAX_MESSAGE_SIZE))
{}

Session::~Session() {
    if (catalogListenerHandle_ != 0) {
        registry_.removeChangeListener(catalogListenerHandle_);
    }
    if (transport_) transport_->close();
}

// --------------------------------------------------------------------------
// Logging helper
// --------------------------------------------------------------------------

void Session::log(const char* fmt, ...) {
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (logFn_) logFn_("TetherIOSession", "%s", buf);
    publishLog(LogSeverity::Info, "TetherIOSession", buf);
}

// --------------------------------------------------------------------------
// Main event loop
// --------------------------------------------------------------------------

void Session::run() {
    running_ = true;

    // Register for catalog change notifications
    catalogListenerHandle_ = registry_.addChangeListener([this]() {
        catalogDirty_.store(true, std::memory_order_relaxed);
    });

    log("Session started");

    // Deadline enforcement for outbound peer calls runs on its own
    // thread: expiry must not depend on wire traffic waking this loop.
    sweepStop_.store(false, std::memory_order_relaxed);
    invokeSweepThread_ = std::thread([this] { invokeSweepLoop(); });

    while (!stopRequested_.load(std::memory_order_relaxed)) {
        // Check for catalog changes
        if (catalogDirty_.exchange(false, std::memory_order_relaxed)) {
            sendCatalogChanged();
        }

        // Calculate timeout
        uint32_t timeoutMs;
        if (streaming_) {
            uint64_t now = getTimestampUs_();
            uint64_t elapsed = (now >= lastSampleTimeUs_) ? (now - lastSampleTimeUs_) : 0;
            uint64_t targetUs = static_cast<uint64_t>(intervalUs_);
            if (elapsed >= targetUs) {
                handleStreamingCycle();
                lastSampleTimeUs_ = now;
                timeoutMs = 0;
            } else {
                uint64_t waitUs = targetUs - elapsed;
                timeoutMs = static_cast<uint32_t>(waitUs / 1000);
                if (timeoutMs == 0) timeoutMs = 1;
            }
        } else {
            timeoutMs = 500;
        }

        if (framing_ == Framing::None) {
            // Message-oriented transport (e.g. WebSocket): each receiveMessage()
            // call returns one complete protocol message — no SLIP deframing.
            rxMsgBuf_.clear();
            if (transport_->receiveMessage(rxMsgBuf_, timeoutMs)) {
                if (!rxMsgBuf_.empty()) {
                    onMessage(rxMsgBuf_.data(), rxMsgBuf_.size());
                }
            } else if (!transport_->isConnected()) {
                log("Client disconnected");
                break;
            }
        } else {
            // Byte-stream transport: SLIP deframing.
            uint8_t rxBuf[1024];
            size_t r = transport_->receive(rxBuf, sizeof(rxBuf), timeoutMs);
            if (r == 0 && !transport_->isConnected()) {
                log("Client disconnected");
                break;
            }
            if (r > 0) {
                feedSlipData(rxBuf, r);
            }
        }
    }

    releaseRingSource();
    streaming_ = false;
    sweepStop_.store(true, std::memory_order_relaxed);
    invokeSweepCv_.notify_all();
    if (invokeSweepThread_.joinable()) invokeSweepThread_.join();
    finishPendingInvokes();
    running_ = false;
    log("Session ended");
}

void Session::requestStop() {
    stopRequested_ = true;
    if (transport_) transport_->close();
}

// --------------------------------------------------------------------------
// SLIP deframing
// --------------------------------------------------------------------------

void Session::feedSlipData(const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        if (slipDiscardUntilEnd_) {
            if (data[i] == 0xC0) {
                slipDiscardUntilEnd_ = false;
                slipRxPos_ = 0;
            }
            continue;
        }

        if (!slipRxBuf_->resize(slipRxPos_ + 1)) {
            slipRxPos_ = 0;
            slipDiscardUntilEnd_ = (data[i] != 0xC0);
            continue;
        }
        slipRxBuf_->data()[slipRxPos_++] = data[i];

        if (data[i] == 0xC0) {  // SLIP END
            size_t decLen = SLIPStream::decoded_length(slipRxBuf_->data(), slipRxPos_);
            if (decLen != SLIPStream::DECODE_ERROR && decLen > 0 &&
                decLen <= decodeBuf_->maxCapacity() && decodeBuf_->resize(decLen)) {
                size_t wrote = SLIPStream::decode_packet(
                    slipRxBuf_->data(), slipRxPos_, decodeBuf_->data(), decodeBuf_->capacity());
                if (wrote != SLIPStream::DECODE_ERROR && wrote > 0) {
                    onMessage(decodeBuf_->data(), wrote);
                }
            }
            slipRxPos_ = 0;
            slipRxBuf_->clear();
            decodeBuf_->clear();
        }
    }
}

// --------------------------------------------------------------------------
// Protocol dispatch
// --------------------------------------------------------------------------

void Session::onMessage(const uint8_t* data, size_t len) {
    if (len < 1) return;
    auto type = static_cast<MessageType>(data[0]);
    const uint8_t* body = data + 1;
    size_t bodyLen = len - 1;

    if (!schemaCommitted_ && type != MessageType::ClientHello &&
        type != MessageType::SchemaRequest && type != MessageType::SchemaDefinition &&
        type != MessageType::SchemaCommit && type != MessageType::SchemaReject) {
        sendError(ErrorCode::InvalidMessage, "Schema negotiation is not committed");
        return;
    }

    if (logFn_) {
        const char* name = "Unknown";
        switch (static_cast<uint8_t>(type)) {
            case 0x01: name = "ListParamsReq"; break;
            case 0x02: name = "ListParamsResp"; break;
            case 0x03: name = "ConfigureStreamReq"; break;
            case 0x04: name = "ConfigureStreamAck"; break;
            case 0x05: name = "StartStream"; break;
            case 0x06: name = "StopStream"; break;
            case 0x07: name = "StreamData"; break;
            case 0x08: name = "Error"; break;
            case 0x0B: name = "SetParamReq"; break;
            case 0x0C: name = "SetParamResp"; break;
            case 0x0D: name = "PingReq"; break;
            case 0x0E: name = "PongResp"; break;
            case 0x20: name = "ListSignalsReq"; break;
            case 0x21: name = "ListSignalsResp"; break;
            case 0x22: name = "GetParamReq"; break;
            case 0x23: name = "GetParamResp"; break;
            case 0x24: name = "GetSignalReq"; break;
            case 0x25: name = "GetSignalResp"; break;
            case 0x35: name = "ListFunctionsReq"; break;
            case 0x36: name = "ListFunctionsResp"; break;
            case 0x37: name = "CallFunctionReq"; break;
            case 0x3E: name = "InvokeExReq"; break;
            case 0x3F: name = "InvokeExResp"; break;
            case 0x40: name = "RegisterFunctionsReq"; break;
            case 0x50: name = "ClientHello"; break;
            case 0x51: name = "ServerHello"; break;
            case 0x52: name = "SchemaRequest"; break;
            case 0x53: name = "SchemaDefinition"; break;
            case 0x54: name = "SchemaCommit"; break;
            case 0x55: name = "SchemaReject"; break;
            case 0x56: name = "SchemaUpdate"; break;
            default: break;
        }
        logFn_("TetherIO", "dispatch %s (bodyLen=%zu)", name, bodyLen);
    }

    switch (type) {
        case MessageType::ListParamsReq:       handleListParamsReq(body, bodyLen); break;
        case MessageType::ListSignalsReq:      handleListSignalsReq(body, bodyLen); break;
        case MessageType::GetParamReq:         handleGetParamReq(body, bodyLen); break;
        case MessageType::SetParamReq:         handleSetParamReq(body, bodyLen); break;
        case MessageType::GetSignalReq:        handleGetSignalReq(body, bodyLen); break;
        case MessageType::ConfigureStreamReq:  handleConfigureStreamReq(body, bodyLen); break;
        case MessageType::StartStream:         handleStartStream(); break;
        case MessageType::StopStream:          handleStopStream(); break;
        case MessageType::GetMetadataReq:      handleGetMetadataReq(body, bodyLen); break;
        case MessageType::PingReq:              handlePingReq(body, bodyLen); break;
        case MessageType::SubscribeLogReq:      handleSubscribeLogReq(body, bodyLen); break;
        case MessageType::UnsubscribeLogReq:    handleUnsubscribeLogReq(body, bodyLen); break;
        case MessageType::SnapshotParamsReq:   handleSnapshotParamsReq(body, bodyLen); break;
        case MessageType::SnapshotSignalsReq:  handleSnapshotSignalsReq(body, bodyLen); break;
        case MessageType::FeatureExchangeReq:  handleFeatureExchangeReq(body, bodyLen); break;
        case MessageType::ClientHello:         handleClientHello(body, bodyLen); break;
        case MessageType::SchemaRequest:       handleSchemaRequest(body, bodyLen); break;
        case MessageType::SchemaCommit:        handleSchemaCommit(body, bodyLen); break;
        case MessageType::ConfigureDatalogReq: handleConfigureDatalogReq(body, bodyLen); break;
        case MessageType::DatalogStatusReq:    handleDatalogStatusReq(); break;
        case MessageType::ConfigureThresholdReq: handleConfigureThresholdReq(body, bodyLen); break;
        case MessageType::DescribeStructReq:   handleDescribeStructReq(body, bodyLen); break;
        case MessageType::ListFunctionsReq:    handleListFunctionsReq(body, bodyLen); break;
        case MessageType::CallFunctionReq:     handleCallFunctionReq(body, bodyLen); break;
        case MessageType::CreateInputStreamReq: handleCreateInputStreamReq(body, bodyLen); break;
        case MessageType::InputStreamData:      handleInputStreamData(body, bodyLen); break;
        case MessageType::CloseInputStreamReq:  handleCloseInputStreamReq(body, bodyLen); break;
        case MessageType::InvokeExReq:          handleInvokeExReq(body, bodyLen); break;
        case MessageType::InvokeExResp:         handleInvokeExResp(body, bodyLen); break;
        case MessageType::RegisterFunctionsReq: handleRegisterFunctionsReq(body, bodyLen); break;
        default:
            sendError(ErrorCode::UnknownMessageType, "Unknown message type");
            break;
    }
}

void Session::handleClientHello(const uint8_t* body, size_t len) {
    BufReader reader(body, len);
    ClientHelloV6 hello;
    if (!decodeClientHelloV6(reader, hello) || reader.remaining() != 0) {
        sendError(ErrorCode::InvalidMessage, "Invalid V6 ClientHello");
        return;
    }
    if (hello.minVersion > SCHEMA_PROTOCOL_VERSION || hello.maxVersion < SCHEMA_PROTOCOL_VERSION) {
        uint8_t buffer[128]{};
        BufWriter writer(buffer, sizeof(buffer));
        writer.putU8(static_cast<uint8_t>(MessageType::SchemaReject));
        encodeSchemaRejectV6(writer, {SchemaRejectCode::UnsupportedVersion,
                                      "V6 is not in the offered version range"});
        if (writer.ok()) sendRaw(buffer, writer.pos);
        return;
    }

    ServerHelloV6 response;
    response.selectedVersion = SCHEMA_PROTOCOL_VERSION;
    response.epoch = schemaCatalog_ && schemaCatalog_->epoch() != 0
        ? schemaCatalog_->epoch() : 1;
    if (schemaCatalog_) response.schemas = schemaCatalog_->manifest();
    schemaEpoch_ = response.epoch;
    schemaHelloReceived_ = true;
    schemaCommitted_ = false;

    if (txRawBuf_.size() < 256) txRawBuf_.resize(256);
    BufWriter writer(txRawBuf_.data(), txRawBuf_.size());
    writer.putU8(static_cast<uint8_t>(MessageType::ServerHello));
    encodeServerHelloV6(writer, response);
    if (writer.ok()) sendRaw(txRawBuf_.data(), writer.pos);
}

void Session::handleSchemaRequest(const uint8_t* body, size_t len) {
    if (!schemaHelloReceived_) {
        sendError(ErrorCode::InvalidMessage, "SchemaRequest before ClientHello");
        return;
    }
    BufReader reader(body, len);
    SchemaRequestV6 request;
    if (!decodeSchemaRequestV6(reader, request) || reader.remaining() != 0 ||
        request.epoch != schemaEpoch_) {
        sendError(ErrorCode::InvalidMessage, "Invalid V6 SchemaRequest");
        return;
    }
    if (!schemaCatalog_) {
        if (!request.definitions.empty()) {
            uint8_t buffer[128]{};
            BufWriter writer(buffer, sizeof(buffer));
            writer.putU8(static_cast<uint8_t>(MessageType::SchemaReject));
            encodeSchemaRejectV6(writer, {SchemaRejectCode::MissingDependency,
                                          "No schema catalog is available"});
            if (writer.ok()) sendRaw(buffer, writer.pos);
        }
        return;
    }

    for (const auto& requested : request.definitions) {
        const auto slot = schemaCatalog_->slotFor(requested);
        const auto* entry = slot ? schemaCatalog_->describe(*slot) : nullptr;
        const auto* node = slot ? schemaCatalog_->resolve(schemaEpoch_, *slot) : nullptr;
        if (!node && schemaCatalog_->graph()) {
            node = schemaCatalog_->graph()->find(requested.key);
            if (node && computeSchemaDigest(*node) != requested.digest) node = nullptr;
        }
        if ((!node) || (entry && entry->ref != requested)) {
            uint8_t buffer[128]{};
            BufWriter writer(buffer, sizeof(buffer));
            writer.putU8(static_cast<uint8_t>(MessageType::SchemaReject));
            encodeSchemaRejectV6(writer, {SchemaRejectCode::MissingDependency,
                                          "Requested schema is not in the catalog"});
            if (writer.ok()) sendRaw(buffer, writer.pos);
            continue;
        }

        if (txRawBuf_.size() < MAX_MESSAGE_SIZE) txRawBuf_.resize(MAX_MESSAGE_SIZE);
        BufWriter writer(txRawBuf_.data(), txRawBuf_.size());
        writer.putU8(static_cast<uint8_t>(MessageType::SchemaDefinition));
        encodeSchemaDefinitionV6(writer, {schemaEpoch_, *node});
        if (writer.ok()) sendRaw(txRawBuf_.data(), writer.pos);
    }
}

void Session::handleSchemaCommit(const uint8_t* body, size_t len) {
    if (!schemaHelloReceived_) {
        sendError(ErrorCode::InvalidMessage, "SchemaCommit before ClientHello");
        return;
    }
    BufReader reader(body, len);
    SchemaCommitV6 commit;
    if (!decodeSchemaCommitV6(reader, commit) || reader.remaining() != 0 ||
        commit.epoch != schemaEpoch_) {
        sendError(ErrorCode::InvalidMessage, "Invalid V6 SchemaCommit");
        return;
    }
    schemaCommitted_ = true;
}

// --------------------------------------------------------------------------
// Message handlers
// --------------------------------------------------------------------------

void Session::handleListParamsReq(const uint8_t* body, size_t len) {
    if (len < 8) { sendError(ErrorCode::InvalidMessage, "ListParamsReq too short"); return; }
    BufReader r(body, len);
    uint32_t offset   = r.getU32();
    uint32_t maxCount = r.getU32();
    if (!r.ok()) { sendError(ErrorCode::InvalidMessage, "parse error"); return; }

    auto entries = registry_.paramPage(offset, maxCount);
    uint32_t total = registry_.paramCount();

    size_t sz = 1 + 4 + 4 + 4 + 8;
    for (const auto& e : entries) {
        sz += 8 + 4 + 4 + 2 + e.name().size() + 2 + e.description().size() + 2 + e.group().size();
    }
    if (txRawBuf_.size() < sz) txRawBuf_.resize(sz);

    BufWriter w(txRawBuf_.data(), sz);
    w.putU8(static_cast<uint8_t>(MessageType::ListParamsResp));
    w.putU32(total);
    w.putU32(offset);
    w.putU32(static_cast<uint32_t>(entries.size()));
    w.putU64(schemaEpoch_);
    for (const auto& e : entries) {
        w.putU64(e.id());
        w.putU32(e.schemaSlot());
        w.putU32(e.flags());
        w.putStr16(e.name().data(), e.name().size());
        w.putStr16(e.description().data(), e.description().size());
        w.putStr16(e.group().data(), e.group().size());
    }
    if (w.ok()) sendRaw(txRawBuf_.data(), w.pos);
}

void Session::handleListSignalsReq(const uint8_t* body, size_t len) {
    if (len < 8) { sendError(ErrorCode::InvalidMessage, "ListSignalsReq too short"); return; }
    BufReader r(body, len);
    uint32_t offset   = r.getU32();
    uint32_t maxCount = r.getU32();
    if (!r.ok()) { sendError(ErrorCode::InvalidMessage, "parse error"); return; }

    auto entries = registry_.signalPage(offset, maxCount);
    uint32_t total = registry_.signalCount();

    size_t sz = 1 + 4 + 4 + 4 + 8;
    for (const auto& e : entries) {
        sz += 8 + 4 + 4 + 2 + e.name().size() + 2 + e.description().size() + 2 + e.group().size();
    }
    if (txRawBuf_.size() < sz) txRawBuf_.resize(sz);

    BufWriter w(txRawBuf_.data(), sz);
    w.putU8(static_cast<uint8_t>(MessageType::ListSignalsResp));
    w.putU32(total);
    w.putU32(offset);
    w.putU32(static_cast<uint32_t>(entries.size()));
    w.putU64(schemaEpoch_);
    for (const auto& e : entries) {
        w.putU64(e.id());
        w.putU32(e.schemaSlot());
        w.putU32(e.flags());
        w.putStr16(e.name().data(), e.name().size());
        w.putStr16(e.description().data(), e.description().size());
        w.putStr16(e.group().data(), e.group().size());
    }
    if (w.ok()) sendRaw(txRawBuf_.data(), w.pos);
}

void Session::handleGetParamReq(const uint8_t* body, size_t len) {
    if (len < 8) { sendError(ErrorCode::InvalidMessage, "GetParamReq too short"); return; }
    BufReader r(body, len);
    uint64_t id = r.getU64();
    if (!r.ok()) { sendError(ErrorCode::InvalidMessage, "parse error"); return; }

    EntryView entry = registry_.findParam(id);
    if (!entry) { sendError(ErrorCode::InvalidId, "Parameter not found"); return; }

    std::vector<uint8_t> value;
    uint8_t vs = entry.valueSize();
    size_t sz = 1 + 8 + MAX_VARINT_SIZE;
    if (entry.isVariableLength()) {
        value.resize(entry.maxValueSize());
        const size_t actual = entry.readVar(value.data(), value.size());
        value.resize(actual);
        sz += actual;
    } else {
        value.resize(vs);
        entry.read(value.data());
        sz += value.size();
    }
    if (!validateCatalogValue(schemaCatalog_, entry, value.data(), value.size())) {
        sendError(ErrorCode::InternalError, "Registry produced an invalid schema value");
        return;
    }
    if (txRawBuf_.size() < sz) txRawBuf_.resize(sz);

    BufWriter w(txRawBuf_.data(), sz);
    w.putU8(static_cast<uint8_t>(MessageType::GetParamResp));
    w.putU64(id);
    w.putVarint(static_cast<uint32_t>(value.size()));
    w.putBytes(value.data(), value.size());

    if (w.ok()) sendRaw(txRawBuf_.data(), w.pos);
}

void Session::handleSetParamReq(const uint8_t* body, size_t len) {
    if (len < 8) { sendError(ErrorCode::InvalidMessage, "SetParamReq too short"); return; }
    BufReader r(body, len);
    uint64_t id = r.getU64();
    if (!r.ok()) { sendError(ErrorCode::InvalidMessage, "parse error"); return; }

    EntryView entry = registry_.findParam(id);
    if (!entry) { sendError(ErrorCode::InvalidId, "Parameter not found"); return; }
    if (!entry.writable()) { sendError(ErrorCode::NotWritable, "Not writable"); return; }

    const uint32_t dataLen = r.getVarint();
    const uint32_t expectedSize = entry.valueSize();
    if (!r.ok() || dataLen > MAX_VARIABLE_VALUE_SIZE ||
        (entry.isVariableLength() && dataLen > entry.maxValueSize()) ||
        (!entry.isVariableLength() && dataLen != expectedSize)) {
        sendError(ErrorCode::InvalidMessage, "Invalid value length"); return;
    }
    const uint8_t* data = r.getBytes(dataLen);
    if (!r.ok() || r.remaining() != 0) { sendError(ErrorCode::InvalidMessage, "Value too short"); return; }
    if (!validateCatalogValue(schemaCatalog_, entry, data, dataLen)) {
        sendError(ErrorCode::InvalidMessage, "Value does not match schema");
        return;
    }
    if (entry.isVariableLength()) entry.writeVar(data, dataLen);
    else entry.write(data);

    uint8_t buf[9];
    BufWriter w(buf, sizeof(buf));
    w.putU8(static_cast<uint8_t>(MessageType::SetParamResp));
    w.putU64(id);
    if (w.ok()) sendRaw(buf, w.pos);
}

void Session::handleGetSignalReq(const uint8_t* body, size_t len) {
    if (len < 8) { sendError(ErrorCode::InvalidMessage, "GetSignalReq too short"); return; }
    BufReader r(body, len);
    uint64_t id = r.getU64();
    if (!r.ok()) { sendError(ErrorCode::InvalidMessage, "parse error"); return; }

    EntryView entry = registry_.findSignal(id);
    if (!entry) { sendError(ErrorCode::InvalidId, "Signal not found"); return; }

    std::vector<uint8_t> value;
    uint8_t vs = entry.valueSize();
    size_t sz = 1 + 8 + MAX_VARINT_SIZE;
    if (entry.isVariableLength()) {
        value.resize(entry.maxValueSize());
        const size_t actual = entry.readVar(value.data(), value.size());
        value.resize(actual);
        sz += actual;
    } else {
        value.resize(vs);
        entry.read(value.data());
        sz += value.size();
    }
    if (!validateCatalogValue(schemaCatalog_, entry, value.data(), value.size())) {
        sendError(ErrorCode::InternalError, "Registry produced an invalid schema value");
        return;
    }
    if (txRawBuf_.size() < sz) txRawBuf_.resize(sz);

    BufWriter w(txRawBuf_.data(), sz);
    w.putU8(static_cast<uint8_t>(MessageType::GetSignalResp));
    w.putU64(id);
    w.putVarint(static_cast<uint32_t>(value.size()));
    w.putBytes(value.data(), value.size());

    if (w.ok()) sendRaw(txRawBuf_.data(), w.pos);
}

void Session::handleConfigureStreamReq(const uint8_t* body, size_t len) {
    if (logFn_) logFn_("TetherIO", "handleConfigureStreamReq (bodyLen=%zu, streaming=%d)",
                        len, streaming_);
    if (streaming_) {
        streaming_ = false;
        rowsInChunk_ = 0;
    }

    // ParameterStream field order, with Tether's 32-bit count fields:
    // trigger, interval_ms, chunk_size, skip_count, trigger_id, entry_count,
    // entry IDs, filter_count, filters.
    if (len < 29) { sendError(ErrorCode::InvalidMessage, "ConfigureStream too short"); return; }

    BufReader r(body, len);
    uint8_t trigMode   = r.getU8();
    uint32_t intervalMs = r.getU32();
    uint32_t chunk     = r.getU32();
    uint32_t skip      = r.getU32();
    uint64_t triggerEntryId = r.getU64();
    uint32_t entryCount = r.getU32();
    if (!r.ok()) { sendError(ErrorCode::InvalidMessage, "parse error"); return; }

    if (trigMode > 1) { sendError(ErrorCode::InvalidMessage, "Invalid trigger mode"); return; }
    if (chunk == 0) chunk = 1;
    if (intervalMs == 0) intervalMs = 1;
    constexpr uint32_t MAX_STREAM_ENTRIES = 65536;
    if (entryCount > MAX_STREAM_ENTRIES) {
        sendError(ErrorCode::TooManyEntries, "Too many stream entries");
        return;
    }

    std::vector<uint64_t> entryIds;
    entryIds.reserve(entryCount);
    for (uint32_t i = 0; i < entryCount; ++i) {
        uint64_t eid = r.getU64();
        if (!r.ok()) { sendError(ErrorCode::InvalidMessage, "Truncated entry list"); return; }
        entryIds.push_back(eid);
    }

    std::vector<FilterProperty> streamFilters;
    uint32_t filterCount = r.getU32();
    if (!r.ok()) { sendError(ErrorCode::InvalidMessage, "Truncated filter count"); return; }
    if (filterCount > MAX_STREAM_ENTRIES) {
        sendError(ErrorCode::TooManyEntries, "Too many stream filters");
        return;
    }
    if (filterCount != 0 && !registry_.supportsStreamFilters()) {
        sendError(ErrorCode::FeatureNotSupported, "Stream filters are not supported");
        return;
    }
    streamFilters.reserve(filterCount);
    for (uint32_t i = 0; i < filterCount; ++i) {
        uint8_t nameLen = r.getU8();
        const uint8_t* name = r.getBytes(nameLen);
        if (!r.ok() || nameLen == 0) {
            sendError(ErrorCode::InvalidMessage, "Invalid stream filter name");
            return;
        }
        ValueType filterType = static_cast<ValueType>(r.getU8());
        if (!r.ok()) {
            sendError(ErrorCode::InvalidMessage, "Truncated stream filter type");
            return;
        }
        const uint8_t fixedSize = valueTypeSize(filterType);
        FilterProperty property;
        property.name.assign(reinterpret_cast<const char*>(name), nameLen);
        property.value.type = filterType;
        if (fixedSize != 0) {
            const uint8_t* value = r.getBytes(fixedSize);
            if (!r.ok()) {
                sendError(ErrorCode::InvalidMessage, "Truncated stream filter value");
                return;
            }
            property.value.data.assign(value, value + fixedSize);
        } else if (!isVariableLength(filterType)) {
            sendError(ErrorCode::InvalidMessage, "Unknown stream filter type");
            return;
        } else {
            const uint32_t valueLen = r.getVarint();
            if (!r.ok() || valueLen > MAX_VARIABLE_VALUE_SIZE) {
                sendError(ErrorCode::InvalidMessage, "Invalid stream filter value length");
                return;
            }
            const uint8_t* value = r.getBytes(valueLen);
            if (!r.ok()) {
                sendError(ErrorCode::InvalidMessage, "Truncated stream filter value");
                return;
            }
            property.value.data.assign(value, value + valueLen);
        }
        const auto validation = registry_.validateStreamFilter(property);
        if (!validation.ok) {
            sendError(ErrorCode::InvalidMessage, validation.message.c_str());
            return;
        }
        streamFilters.push_back(std::move(property));
    }
    if (!r.ok() || r.remaining() != 0) {
        sendError(ErrorCode::InvalidMessage, "Trailing ConfigureStream data");
        return;
    }

    triggerMode_      = static_cast<TriggerMode>(trigMode);
    const uint64_t intervalUs = static_cast<uint64_t>(intervalMs) * 1000ULL;
    intervalUs_       = static_cast<uint32_t>(std::min<uint64_t>(
        std::max<uint64_t>(1, intervalUs), std::numeric_limits<uint32_t>::max()));
    chunkSize_        = chunk;
    skipCount_        = skip;
    triggerEntryId_   = triggerEntryId;
    configuredEntryIds_ = std::move(entryIds);
    streamFilters_ = std::move(streamFilters);
    specId_++;

    releaseRingSource();
    buildCollectPlan();
    if (!bindRingSource()) {
        configured_ = false;
        sendError(ErrorCode::ResourceBusy,
                  "Stream source already bound to another session");
        return;
    }
    configured_ = true;
    skipCounter_ = 0;
    rowsInChunk_ = 0;
    chunkWritePos_ = 0;
    lastTriggerValue_.clear();

    // Send ParameterStream ConfigureAck. Tether deliberately retains 32-bit
    // resolved-count and row-size fields to support larger catalogs/chunks.
    size_t sz = 1 + 4 + 4 + 4 + 8 + collectPlan_.size() * 13;
    if (txRawBuf_.size() < sz) txRawBuf_.resize(sz);

    BufWriter w(txRawBuf_.data(), sz);
    w.putU8(static_cast<uint8_t>(MessageType::ConfigureStreamAck));
    w.putU32(specId_);
    w.putU32(static_cast<uint32_t>(collectPlan_.size()));
    w.putU32(rowSize_);
    w.putU64(schemaEpoch_);
    for (const auto& slot : collectPlan_) {
        w.putU64(slot.paramId);
        w.putU32(slot.entry.schemaSlot());
        w.putU8(slot.valueSize);
    }
    if (w.ok()) sendRaw(txRawBuf_.data(), w.pos);
    if (logFn_) logFn_("TetherIO", "ConfigureStream OK: specId=%u, %zu entries, rowSize=%u, intervalMs=%u, chunk=%u, skip=%u, trigger=%s",
                        specId_, collectPlan_.size(), rowSize_, intervalMs, chunk, skip,
                        trigMode == 0 ? "Time" : "OnChange");
}

void Session::handleStartStream() {
    if (!configured_) { sendError(ErrorCode::StreamNotConfigured, "Not configured"); return; }
    if (streaming_) { sendError(ErrorCode::AlreadyStreaming, "Already streaming"); return; }
    streaming_ = true;
    rowsInChunk_ = 0;
    chunkWritePos_ = 0;
    skipCounter_ = 0;
    lastSampleTimeUs_ = getTimestampUs_();
    lastTriggerValue_.clear();
    lastValues_.clear();
    lastValues_.resize(collectPlan_.size());
    if (ringSource_) {
        ringSource_->start();
        lastRingRowUs_ = lastSampleTimeUs_;
    }
    if (logFn_) logFn_("TetherIO", "StartStream: streaming started (interval=%uus, chunk=%u, %zu entries)",
                        intervalUs_, chunkSize_, collectPlan_.size());
}

void Session::handleStopStream() {
    if (!streaming_) { sendError(ErrorCode::NotStreaming, "Not streaming"); return; }
    if (rowsInChunk_ > 0) sendStreamData();
    streaming_ = false;
    releaseRingSource();
    if (logFn_) logFn_("TetherIO", "StopStream: streaming stopped");
}

void Session::handlePingReq(const uint8_t* body, size_t len) {
    BufReader r(body, len);
    const uint32_t nonce = r.getVarint();
    if (!r.ok() || r.remaining() != 0) {
        sendError(ErrorCode::InvalidMessage, "Invalid ping");
        return;
    }
    sendPongResp(nonce);
}

void Session::handleSubscribeLogReq(const uint8_t* body, size_t len) {
    BufReader r(body, len);
    const auto severity = static_cast<LogSeverity>(r.getU8());
    auto readFilter = [&r](std::string& out) {
        const uint16_t length = r.getU16();
        const uint8_t* bytes = r.getBytes(length);
        if (!r.ok()) return false;
        out.assign(reinterpret_cast<const char*>(bytes), length);
        return true;
    };

    LogSubscription subscription;
    subscription.minSeverity = severity;
    if (static_cast<uint8_t>(severity) > static_cast<uint8_t>(LogSeverity::Critical) ||
        !readFilter(subscription.componentFilter) ||
        !readFilter(subscription.messageFilter) ||
        !readFilter(subscription.locationFilter) || r.remaining() != 0) {
        sendError(ErrorCode::InvalidMessage, "Invalid log subscription");
        return;
    }

    uint32_t subscriptionId;
    bool capacityExceeded = false;
    {
        std::lock_guard<std::mutex> lock(sendMutex_);
        if (logSubscriptions_.size() >= 16) {
            capacityExceeded = true;
        } else {
            subscription.id = nextLogSubscriptionId_++;
            if (subscription.id == 0) subscription.id = nextLogSubscriptionId_++;
            subscriptionId = subscription.id;
            logSubscriptions_.push_back(std::move(subscription));
        }
    }
    if (capacityExceeded) {
        sendSubscribeLogResp(0, false, "Too many log subscriptions");
        return;
    }
    sendSubscribeLogResp(subscriptionId, true);
}

void Session::handleUnsubscribeLogReq(const uint8_t* body, size_t len) {
    BufReader r(body, len);
    const uint32_t id = r.getVarint();
    if (!r.ok() || r.remaining() != 0) {
        sendError(ErrorCode::InvalidMessage, "Invalid log subscription ID");
        return;
    }

    bool found = false;
    {
        std::lock_guard<std::mutex> lock(sendMutex_);
        const auto it = std::find_if(logSubscriptions_.begin(), logSubscriptions_.end(),
                                     [id](const LogSubscription& item) { return item.id == id; });
        found = it != logSubscriptions_.end();
        if (found) logSubscriptions_.erase(it);
    }
    sendUnsubscribeLogResp(id, found);
}

void Session::handleGetMetadataReq(const uint8_t* body, size_t len) {
    if (len < 8) { sendError(ErrorCode::InvalidMessage, "too short"); return; }
    BufReader r(body, len);
    uint64_t id = r.getU64();
    if (!r.ok()) { sendError(ErrorCode::InvalidMessage, "parse error"); return; }

    EntryView entry = registry_.find(id);
    if (!entry) { sendError(ErrorCode::InvalidId, "Not found"); return; }

    size_t sz = 1 + 8 + 4 + 8;
    entry.forEachMetadata([&sz](std::string_view k, std::string_view v) {
        sz += 2 + k.size() + 2 + v.size();
    });
    if (txRawBuf_.size() < sz) txRawBuf_.resize(sz);

    BufWriter w(txRawBuf_.data(), sz);
    w.putU8(static_cast<uint8_t>(MessageType::GetMetadataResp));
    w.putU64(id);
    w.putU32(static_cast<uint32_t>(entry.metadataCount()));
    entry.forEachMetadata([&w](std::string_view k, std::string_view v) {
        w.putStr16(k.data(), k.size());
        w.putStr16(v.data(), v.size());
    });
    if (w.ok()) sendRaw(txRawBuf_.data(), w.pos);
}

void Session::handleSnapshotParamsReq(const uint8_t* body, size_t len) {
    BufReader r(body, len);
    uint32_t count = r.getU32();
    if (!r.ok()) { sendError(ErrorCode::InvalidMessage, "parse error"); return; }

    std::vector<uint64_t> ids;
    if (count == 0) {
        // Snapshot all params
        auto page = registry_.paramPage(0, registry_.paramCount());
        for (const auto& e : page) ids.push_back(e.id());
    } else {
        ids.reserve(count);
        for (uint32_t i = 0; i < count; ++i) {
            ids.push_back(r.getU64());
            if (!r.ok()) { sendError(ErrorCode::InvalidMessage, "truncated"); return; }
        }
    }

    uint64_t ts = getTimestampUs_();

    struct SnapshotValue {
        uint64_t id;
        uint32_t schemaSlot;
        uint8_t valueSize;
        std::vector<uint8_t> bytes;
        bool variable = false;
    };
    std::vector<SnapshotValue> values;
    values.reserve(ids.size());

    size_t sz = 1 + 8 + 4 + 8;
    for (uint64_t id : ids) {
        EntryView e = registry_.findParam(id);
        if (!e) continue;
        SnapshotValue value{};
        value.id = id;
        value.schemaSlot = e.schemaSlot();
        value.valueSize = e.valueSize();
        value.variable = e.isVariableLength();
        if (value.variable) {
            value.bytes.resize(e.maxValueSize());
            const size_t actual = e.readVar(value.bytes.data(), value.bytes.size());
            value.bytes.resize(actual);
            sz += 8 + 4 + 1 + MAX_VARINT_SIZE + actual;
        } else {
            value.bytes.resize(value.valueSize);
            e.read(value.bytes.data());
            sz += 8 + 4 + 1 + value.valueSize;
        }
        values.push_back(std::move(value));
    }
    if (txRawBuf_.size() < sz) txRawBuf_.resize(sz);

    BufWriter w(txRawBuf_.data(), sz);
    w.putU8(static_cast<uint8_t>(MessageType::SnapshotParamsResp));
    w.putU64(ts);
    w.putU32(static_cast<uint32_t>(values.size()));
    w.putU64(schemaEpoch_);
    for (const auto& value : values) {
        w.putU64(value.id);
        w.putU32(value.schemaSlot);
        w.putU8(value.valueSize);
        if (value.variable) {
            w.putVarint(static_cast<uint32_t>(value.bytes.size()));
        }
        w.putBytes(value.bytes.data(), value.bytes.size());
    }
    if (w.ok()) sendRaw(txRawBuf_.data(), w.pos);
}

void Session::handleSnapshotSignalsReq(const uint8_t* body, size_t len) {
    BufReader r(body, len);
    uint32_t count = r.getU32();
    if (!r.ok()) { sendError(ErrorCode::InvalidMessage, "parse error"); return; }

    std::vector<uint64_t> ids;
    if (count == 0) {
        auto page = registry_.signalPage(0, registry_.signalCount());
        for (const auto& e : page) ids.push_back(e.id());
    } else {
        ids.reserve(count);
        for (uint32_t i = 0; i < count; ++i) {
            ids.push_back(r.getU64());
            if (!r.ok()) { sendError(ErrorCode::InvalidMessage, "truncated"); return; }
        }
    }

    uint64_t ts = getTimestampUs_();

    struct SnapshotValue {
        uint64_t id;
        uint32_t schemaSlot;
        uint8_t valueSize;
        std::vector<uint8_t> bytes;
        bool variable = false;
    };
    std::vector<SnapshotValue> values;
    values.reserve(ids.size());

    size_t sz = 1 + 8 + 4 + 8;
    for (uint64_t id : ids) {
        EntryView e = registry_.findSignal(id);
        if (!e) continue;
        SnapshotValue value{};
        value.id = id;
        value.valueSize = e.valueSize();
        value.schemaSlot = e.schemaSlot();
        value.variable = e.isVariableLength();
        if (value.variable) {
            value.bytes.resize(e.maxValueSize());
            const size_t actual = e.readVar(value.bytes.data(), value.bytes.size());
            value.bytes.resize(actual);
            sz += 8 + 4 + 1 + MAX_VARINT_SIZE + actual;
        } else {
            value.bytes.resize(value.valueSize);
            e.read(value.bytes.data());
            sz += 8 + 4 + 1 + value.valueSize;
        }
        values.push_back(std::move(value));
    }
    if (txRawBuf_.size() < sz) txRawBuf_.resize(sz);

    BufWriter w(txRawBuf_.data(), sz);
    w.putU8(static_cast<uint8_t>(MessageType::SnapshotSignalsResp));
    w.putU64(ts);
    w.putU32(static_cast<uint32_t>(values.size()));
    w.putU64(schemaEpoch_);
    for (const auto& value : values) {
        w.putU64(value.id);
        w.putU32(value.schemaSlot);
        w.putU8(value.valueSize);
        if (value.variable) {
            w.putVarint(static_cast<uint32_t>(value.bytes.size()));
        }
        w.putBytes(value.bytes.data(), value.bytes.size());
    }
    if (w.ok()) sendRaw(txRawBuf_.data(), w.pos);
}

void Session::handleFeatureExchangeReq(const uint8_t* body, size_t len) {
    BufReader r(body, len);
    if (!FeatureSet::decode(r, clientFeatures_) || r.remaining() != 0) {
        sendError(ErrorCode::InvalidMessage, "Invalid feature exchange");
        return;
    }

    // Build response with server features
    FeatureSet response;
    if (serverFeatures_) {
        response = *serverFeatures_;
    }
    // Always include protocol version
    bool hasVersion = false;
    for (const auto& f : response.features) {
        if (f.name == "protocol_version") { hasVersion = true; break; }
    }
    if (!hasVersion) {
        response.features.push_back(Feature::makeU32("protocol_version", PROTOCOL_VERSION));
    }
    const auto advertise = [&response](std::string_view name) {
        if (!response.find(std::string(name))) {
            response.features.push_back(Feature::makeBool(std::string(name), true));
        }
    };
    advertise("supports_ping");
    advertise("supports_log_subscriptions");
    advertise("supports_extended_value_types");
    advertise("supports_large_counts");
    advertise("supports_signals");
    advertise("supports_functions");
    if (registry_.supportsStreamFilters()) advertise("supports_stream_filters");

    size_t sz = 1 + 4 + response.features.size() * 64;
    if (txRawBuf_.size() < sz) txRawBuf_.resize(sz);

    BufWriter w(txRawBuf_.data(), sz);
    w.putU8(static_cast<uint8_t>(MessageType::FeatureExchangeResp));
    response.encode(w);
    if (w.ok()) sendRaw(txRawBuf_.data(), w.pos);
}

void Session::handleConfigureDatalogReq(const uint8_t* body, size_t len) {
    if (!datalogRecorder_) {
        sendError(ErrorCode::FeatureNotSupported, "Datalogging not available");
        return;
    }

    BufReader r(body, len);
    DatalogConfig config;
    if (!DatalogConfig::decode(r, config)) {
        sendError(ErrorCode::InvalidMessage, "Invalid datalog config");
        return;
    }

    // Configure the recorder
    auto readFn = [this](uint64_t entryId, void* dest, size_t maxSize) -> bool {
        EntryView e = registry_.find(entryId);
        if (!e || e.isVariableLength() || e.valueSize() > maxSize) return false;
        e.read(dest);
        return true;
    };

    datalogRecorder_->configure(config, readFn, getTimestampUs_, nullptr);
    if (config.enabled) {
        datalogRecorder_->start();
    } else {
        datalogRecorder_->stop();
    }

    uint8_t buf[2];
    BufWriter w(buf, sizeof(buf));
    w.putU8(static_cast<uint8_t>(MessageType::ConfigureDatalogResp));
    w.putU8(1);  // success
    if (w.ok()) sendRaw(buf, w.pos);
}

void Session::handleDatalogStatusReq() {
    DatalogStatus status;
    if (datalogRecorder_) {
        status = datalogRecorder_->status();
    }

    size_t sz = 1 + 1 + 8 + 8 + 256 + status.metadata.fields.size() * 64;
    if (txRawBuf_.size() < sz) txRawBuf_.resize(sz);

    BufWriter w(txRawBuf_.data(), sz);
    w.putU8(static_cast<uint8_t>(MessageType::DatalogStatusResp));
    status.encode(w);
    if (w.ok()) sendRaw(txRawBuf_.data(), w.pos);
}

void Session::handleConfigureThresholdReq(const uint8_t* body, size_t len) {
    BufReader r(body, len);
    ThresholdConfig config;
    if (!ThresholdConfig::decode(r, config)) {
        sendError(ErrorCode::ThresholdError, "Invalid threshold config");
        return;
    }

    thresholdFilter_.setConfig(config);

    uint8_t buf[2];
    BufWriter w(buf, sizeof(buf));
    w.putU8(static_cast<uint8_t>(MessageType::ConfigureThresholdResp));
    w.putU8(1);  // success
    if (w.ok()) sendRaw(buf, w.pos);
}

void Session::handleDescribeStructReq(const uint8_t* body, size_t len) {
    if (len < 8) { sendError(ErrorCode::InvalidMessage, "too short"); return; }
    BufReader r(body, len);
    uint64_t id = r.getU64();
    if (!r.ok()) { sendError(ErrorCode::InvalidMessage, "parse error"); return; }

    EntryView entry = registry_.find(id);
    if (!entry) { sendError(ErrorCode::InvalidId, "Not found"); return; }

    const StructDescriptor* sd = entry.structDesc();
    if (!sd) { sendError(ErrorCode::InvalidMessage, "No struct descriptor"); return; }

    size_t sz = 1 + 8 + 2 + sd->name.size() + 4 + 4 + sd->fields.size() * 64;
    if (txRawBuf_.size() < sz) txRawBuf_.resize(sz);

    BufWriter w(txRawBuf_.data(), sz);
    w.putU8(static_cast<uint8_t>(MessageType::DescribeStructResp));
    sd->encode(w);
    if (w.ok()) sendRaw(txRawBuf_.data(), w.pos);
}

void Session::handleListFunctionsReq(const uint8_t* body, size_t len) {
    if (len != 8) {
        sendError(ErrorCode::InvalidMessage, "Invalid ListFunctions request");
        return;
    }
    BufReader r(body, len);
    const uint32_t offset = r.getU32();
    const uint32_t maxCount = r.getU32();
    if (!r.ok()) {
        sendError(ErrorCode::InvalidMessage, "Invalid ListFunctions request");
        return;
    }

    const auto functions = registry_.functionPage(offset, maxCount);
    const uint32_t total = registry_.functionCount();
    size_t size = 1 + 4 + 4 + 4 + 8;
    const auto addStringSize = [&size](std::string_view value) {
        if (value.size() > MAX_STRING_SIZE || size > MAX_MESSAGE_SIZE - 2 - value.size()) {
            return false;
        }
        size += 2 + value.size();
        return true;
    };
    for (const auto& function : functions) {
        if (size > MAX_MESSAGE_SIZE - (8 + 4)) {
            sendError(ErrorCode::TooManyEntries, "Function catalog response too large");
            return;
        }
        size += 8;
        if (!addStringSize(function.name()) || !addStringSize(function.description()) ||
            !addStringSize(function.group()) || function.parameterCount() > MAX_COLLECTION_COUNT) {
            sendError(ErrorCode::TooManyEntries, "Function catalog response too large");
            return;
        }
        size += 4;
        for (const auto& parameter : function.parameters()) {
            if (!addStringSize(parameter.name) || !addStringSize(parameter.description) ||
                size > MAX_MESSAGE_SIZE - 1 - 1 - 8 - 8 - 4 - 4) {
                sendError(ErrorCode::TooManyEntries, "Function catalog response too large");
                return;
            }
            size += 4 + 4;
            if (parameter.hasDefault) {
                if (parameter.defaultValue.size() > MAX_VARIABLE_VALUE_SIZE ||
                    size > MAX_MESSAGE_SIZE - MAX_VARINT_SIZE - parameter.defaultValue.size()) {
                    sendError(ErrorCode::TooManyEntries, "Function catalog response too large");
                    return;
                }
                size += MAX_VARINT_SIZE + parameter.defaultValue.size();
            }
            if (parameter.metadata.size() > MAX_COLLECTION_COUNT) {
                sendError(ErrorCode::TooManyEntries, "Function metadata too large");
                return;
            }
            size += 4;
            for (const auto& [key, value] : parameter.metadata) {
                if (!addStringSize(key) || !addStringSize(value)) {
                    sendError(ErrorCode::TooManyEntries, "Function metadata too large");
                    return;
                }
            }
        }
        if (size > MAX_MESSAGE_SIZE - 1) {
            sendError(ErrorCode::TooManyEntries, "Function catalog response too large");
            return;
        }
        size += 1;
        if (function.returnValue().present) {
            const auto& result = function.returnValue();
            if (!addStringSize(result.name) || !addStringSize(result.description) ||
                size > MAX_MESSAGE_SIZE - 1 - 1 - 8 - 8 - 4) {
                sendError(ErrorCode::TooManyEntries, "Function catalog response too large");
                return;
            }
            size += 4 + 4;
            if (result.metadata.size() > MAX_COLLECTION_COUNT) {
                sendError(ErrorCode::TooManyEntries, "Function metadata too large");
                return;
            }
            size += 4;
            for (const auto& [key, value] : result.metadata) {
                if (!addStringSize(key) || !addStringSize(value)) {
                    sendError(ErrorCode::TooManyEntries, "Function metadata too large");
                    return;
                }
            }
        }
        if (function.metadata().size() > MAX_COLLECTION_COUNT) {
            sendError(ErrorCode::TooManyEntries, "Function metadata too large");
            return;
        }
        size += 4;
        for (const auto& [key, value] : function.metadata()) {
            if (!addStringSize(key) || !addStringSize(value)) {
                sendError(ErrorCode::TooManyEntries, "Function metadata too large");
                return;
            }
        }
    }

    txRawBuf_.resize(size);
    BufWriter w(txRawBuf_.data(), txRawBuf_.size());
    w.putU8(static_cast<uint8_t>(MessageType::ListFunctionsResp));
    w.putU32(total);
    w.putU32(offset);
    w.putU32(static_cast<uint32_t>(functions.size()));
    w.putU64(schemaEpoch_);
    for (const auto& function : functions) {
        const auto writeString = [&w](std::string_view value) {
            w.putStr16(value.data(), value.size());
        };
        w.putU64(function.id());
        writeString(function.name());
        writeString(function.description());
        writeString(function.group());
        w.putU32(static_cast<uint32_t>(function.parameterCount()));
        for (const auto& parameter : function.parameters()) {
            writeString(parameter.name);
            writeString(parameter.description);
            w.putU32(parameter.schemaSlot);
            w.putU32(parameter.flags());
            if (parameter.hasDefault) {
                w.putVarint(static_cast<uint32_t>(parameter.defaultValue.size()));
                w.putBytes(parameter.defaultValue.data(), parameter.defaultValue.size());
            }
            w.putU32(static_cast<uint32_t>(parameter.metadata.size()));
            for (const auto& [key, value] : parameter.metadata) {
                writeString(key);
                writeString(value);
            }
        }
        const auto& result = function.returnValue();
        w.putU8(result.present ? 1 : 0);
        if (result.present) {
            writeString(result.name);
            writeString(result.description);
            w.putU32(result.schemaSlot);
            w.putU32((result.enumReference != 0 ? FunctionParameterFlags::HasEnum : 0) |
                     (result.structReference != 0 ? FunctionParameterFlags::HasStruct : 0));
            w.putU32(static_cast<uint32_t>(result.metadata.size()));
            for (const auto& [key, value] : result.metadata) {
                writeString(key);
                writeString(value);
            }
        }
        w.putU32(static_cast<uint32_t>(function.metadata().size()));
        for (const auto& [key, value] : function.metadata()) {
            writeString(key);
            writeString(value);
        }
    }
    if (w.ok()) sendRaw(txRawBuf_.data(), w.pos);
}

void Session::handleCallFunctionReq(const uint8_t* body, size_t len) {
    if (len < 12) {
        sendError(ErrorCode::InvalidMessage, "Invalid CallFunction request");
        return;
    }
    BufReader r(body, len);
    const uint64_t functionId = r.getU64();
    const uint32_t argumentCount = r.getU32();
    if (!r.ok() || argumentCount > MAX_COLLECTION_COUNT) {
        sendError(ErrorCode::InvalidMessage, "Invalid function argument count");
        return;
    }
    const FunctionView function = registry_.findFunction(functionId);
    if (!function) {
        sendError(ErrorCode::InvalidId, "Function not found");
        return;
    }
    FunctionCallResult result = invokeFunctionChecked(function, argumentCount, r);
    sendFunctionCallResponse(functionId, function.returnValue(), result);
}

void Session::handleInvokeExReq(const uint8_t* body, size_t len) {
    BufReader r(body, len);
    const uint64_t requestId = r.getU64();
    const uint64_t functionId = r.getU64();
    r.getU64();  // deadline hint: enforced by the initiator
    const uint32_t argumentCount = r.getU32();
    const auto reject = [this, requestId](const char* message) {
        FunctionCallResult result;
        result.errorMessage = message;
        sendInvokeExResponse(requestId, FunctionReturn{}, result);
    };
    if (len < 28 || !r.ok() || argumentCount > MAX_COLLECTION_COUNT) {
        reject("Invalid InvokeEx request");
        return;
    }
    const FunctionView function = registry_.findFunction(functionId);
    if (!function) {
        FunctionCallResult result;
        result.error = ErrorCode::InvalidId;
        result.errorMessage = "Function not found";
        sendInvokeExResponse(requestId, FunctionReturn{}, result);
        return;
    }
    FunctionCallResult result = invokeFunctionChecked(function, argumentCount, r);
    sendInvokeExResponse(requestId, function.returnValue(), result);
}

void Session::handleInvokeExResp(const uint8_t* body, size_t len) {
    BufReader r(body, len);
    const uint64_t requestId = r.getU64();
    const uint8_t status = r.getU8();
    const uint32_t errorCode = r.getU32();
    const uint16_t errorLength = r.getU16();
    const uint8_t* errorBytes = r.getBytes(errorLength);
    if (!r.ok()) {
        log("Malformed InvokeExResp");
        return;
    }

    PendingInvoke pending;
    {
        std::lock_guard<std::mutex> lock(invokeMutex_);
        auto it = pendingInvokes_.find(requestId);
        if (it == pendingInvokes_.end()) {
            // Late response after deadline expiry, or foreign request id.
            return;
        }
        pending = std::move(it->second);
        pendingInvokes_.erase(it);
    }

    InvokeResult result;
    result.requestId = requestId;
    result.functionId = pending.functionId;
    const uint64_t now = getTimestampUs_();
    result.rttUs = now - pending.sentAtUs;
    if (pending.deadlineAtUs != 0 && now > pending.deadlineAtUs) {
        // The response crossed the wire after its deadline — same
        // outcome as a locally expired wait; the payload is dropped.
        result.timedOut = true;
        result.error = ErrorCode::Timeout;
        result.errorMessage = "InvokeEx deadline exceeded";
        pending.callback(result);
        return;
    }
    result.success = (status == 0);
    result.error = static_cast<ErrorCode>(errorCode);
    result.errorMessage.assign(reinterpret_cast<const char*>(errorBytes), errorLength);
    if (result.success && r.remaining() >= FUNCTION_TLV_HEADER_SIZE) {
        FunctionArgument returnTlv;
        if (decodeFunctionTlv(r, returnTlv)) {
            result.hasReturnValue = true;
            result.returnType = returnTlv.type;
            result.returnValue = std::move(returnTlv.value);
        } else {
            result.success = false;
            result.error = ErrorCode::FunctionInvocationError;
            result.errorMessage = "Malformed function return";
        }
    }
    if (!r.ok() || r.remaining() != 0) {
        result.success = false;
        result.error = ErrorCode::InvalidMessage;
        result.errorMessage = "Malformed InvokeEx response";
        result.hasReturnValue = false;
        result.returnValue.clear();
    }
    pending.callback(result);
}

void Session::handleRegisterFunctionsReq(const uint8_t* body, size_t len) {
    BufReader r(body, len);
    const uint32_t count = r.getU32();
    if (!r.ok() || count > MAX_COLLECTION_COUNT) {
        sendError(ErrorCode::InvalidMessage, "Invalid RegisterFunctions request");
        return;
    }
    std::vector<FunctionDescriptor> catalog;
    catalog.reserve(count);
    for (uint32_t i = 0; i < count; ++i) {
        FunctionDescriptor descriptor;
        if (!decodeFunctionDescriptor(r, descriptor) ||
            descriptor.id == 0 || descriptor.name.empty()) {
            sendError(ErrorCode::InvalidMessage, "Invalid function descriptor");
            return;
        }
        catalog.push_back(std::move(descriptor));
    }
    if (r.remaining() != 0) {
        sendError(ErrorCode::InvalidMessage, "Trailing RegisterFunctions data");
        return;
    }
    {
        std::lock_guard<std::mutex> lock(peerFunctionsMutex_);
        peerFunctions_ = std::move(catalog);
    }
    log("Client registered %u peer functions", count);
    sendRegisterFunctionsResp(count, true);
}

// --------------------------------------------------------------------------
// Peer (client-hosted) function invocation
// --------------------------------------------------------------------------

bool Session::callPeer(uint64_t functionId,
                       const std::vector<FunctionArgument>& arguments,
                       uint64_t deadlineUs,
                       InvokeResultFn callback) {
    if (!callback) return false;
    size_t size = 1 + 8 + 8 + 8 + 4;
    for (const auto& argument : arguments) {
        size += FUNCTION_TLV_HEADER_SIZE + argument.value.size();
        if (size > MAX_MESSAGE_SIZE) return false;
    }
    const uint64_t now = getTimestampUs_();
    const uint64_t deadlineAtUs =
        (deadlineUs == 0 || deadlineUs > UINT64_MAX - now) ? 0 : now + deadlineUs;

    std::vector<uint8_t> message(size);
    BufWriter w(message.data(), message.size());
    std::lock_guard<std::mutex> lock(invokeMutex_);
    if (pendingInvokes_.size() >= MAX_PENDING_INVOKES) return false;
    const uint64_t requestId = nextInvokeRequestId_++;
    w.putU8(static_cast<uint8_t>(MessageType::InvokeExReq));
    w.putU64(requestId);
    w.putU64(functionId);
    w.putU64(deadlineUs);
    w.putU32(static_cast<uint32_t>(arguments.size()));
    for (const auto& argument : arguments) {
        if (!encodeFunctionTlv(w, argument.position, argument.type,
                               argument.value.data(), argument.value.size())) {
            return false;
        }
    }
    if (!w.ok() || !sendRaw(message.data(), w.pos)) return false;
    pendingInvokes_.emplace(requestId,
                            PendingInvoke{functionId, now, deadlineAtUs, std::move(callback)});
    invokeSweepCv_.notify_one();
    return true;
}

std::vector<FunctionDescriptor> Session::peerFunctions() const {
    std::lock_guard<std::mutex> lock(peerFunctionsMutex_);
    return peerFunctions_;
}

std::optional<uint64_t> Session::findPeerFunctionId(std::string_view name) const {
    std::lock_guard<std::mutex> lock(peerFunctionsMutex_);
    for (const auto& descriptor : peerFunctions_) {
        if (descriptor.name == name) return descriptor.id;
    }
    return std::nullopt;
}

size_t Session::pendingInvokeCount() const {
    std::lock_guard<std::mutex> lock(invokeMutex_);
    return pendingInvokes_.size();
}

void Session::invokeSweepLoop() {
    while (!sweepStop_.load(std::memory_order_relaxed)) {
        uint64_t waitUs = UINT64_MAX;
        {
            std::unique_lock<std::mutex> lock(invokeMutex_);
            const uint64_t now = getTimestampUs_();
            for (const auto& [id, pending] : pendingInvokes_) {
                (void)id;
                if (pending.deadlineAtUs == 0) continue;
                if (pending.deadlineAtUs <= now) { waitUs = 0; break; }
                waitUs = std::min(waitUs, pending.deadlineAtUs - now);
            }
            if (sweepStop_.load(std::memory_order_relaxed)) return;
            if (waitUs == UINT64_MAX) {
                invokeSweepCv_.wait(lock, [this] {
                    return sweepStop_.load(std::memory_order_relaxed) ||
                           !pendingInvokes_.empty();
                });
            } else if (waitUs > 0) {
                invokeSweepCv_.wait_for(lock, std::chrono::microseconds(waitUs));
            }
        }
        if (sweepStop_.load(std::memory_order_relaxed)) return;
        sweepInvokeDeadlines();
    }
}

void Session::sweepInvokeDeadlines() {
    const uint64_t now = getTimestampUs_();
    std::vector<std::pair<InvokeResultFn, InvokeResult>> expired;
    {
        std::lock_guard<std::mutex> lock(invokeMutex_);
        for (auto it = pendingInvokes_.begin(); it != pendingInvokes_.end();) {
            const PendingInvoke& pending = it->second;
            if (pending.deadlineAtUs == 0 || now < pending.deadlineAtUs) {
                ++it;
                continue;
            }
            InvokeResult result;
            result.requestId = it->first;
            result.functionId = pending.functionId;
            result.timedOut = true;
            result.error = ErrorCode::Timeout;
            result.errorMessage = "InvokeEx deadline exceeded";
            result.rttUs = now - pending.sentAtUs;
            expired.emplace_back(std::move(it->second.callback), std::move(result));
            it = pendingInvokes_.erase(it);
        }
    }
    // Callbacks run on the sweep thread, outside the lock — a callback may
    // re-enter callPeer() without deadlocking.
    for (auto& [callback, result] : expired) callback(result);
}

void Session::finishPendingInvokes() {
    std::vector<std::pair<InvokeResultFn, InvokeResult>> drained;
    {
        std::lock_guard<std::mutex> lock(invokeMutex_);
        for (auto& [requestId, pending] : pendingInvokes_) {
            InvokeResult result;
            result.requestId = requestId;
            result.functionId = pending.functionId;
            result.error = ErrorCode::InternalError;
            result.errorMessage = "Session ended";
            drained.emplace_back(std::move(pending.callback), std::move(result));
        }
        pendingInvokes_.clear();
    }
    for (auto& [callback, result] : drained) callback(result);
}

void Session::handleCreateInputStreamReq(const uint8_t* body, size_t len) {
    if (!inputStreamCreateFn_) {
        sendError(ErrorCode::FeatureNotSupported, "Input streams not supported");
        return;
    }

    BufReader reader(body, len);
    ValueDescriptor descriptor;
    const uint32_t maxValueSize = reader.getU32();
    const uint32_t maxBatchSize = reader.getU32();
    if (!reader.ok() || !decodeValueDescriptor(reader, descriptor) ||
        reader.remaining() != 0 || maxValueSize == 0 || maxValueSize > MAX_VARIABLE_VALUE_SIZE ||
        maxBatchSize == 0 || maxBatchSize > MAX_AGGREGATE_ELEMENTS) {
        sendError(ErrorCode::InvalidMessage, "Invalid input stream descriptor");
        return;
    }

    if (nextInputStreamId_ == 0) nextInputStreamId_ = 1;
    const uint32_t streamId = nextInputStreamId_++;
    if (!inputStreamCreateFn_(streamId, descriptor, maxValueSize, maxBatchSize)) {
        sendError(ErrorCode::InternalError, "Input stream rejected");
        return;
    }
    inputStreams_.push_back({streamId, std::move(descriptor), maxValueSize, maxBatchSize});
    sendInputStreamResponse(MessageType::CreateInputStreamResp, streamId, true);
}

void Session::handleInputStreamData(const uint8_t* body, size_t len) {
    BufReader reader(body, len);
    const uint32_t streamId = reader.getU32();
    const uint32_t count = reader.getU32();
    auto stream = std::find_if(inputStreams_.begin(), inputStreams_.end(),
                               [streamId](const InputStreamState& state) {
                                   return state.id == streamId;
                               });
    if (!reader.ok()) {
        sendError(ErrorCode::InvalidMessage, "Invalid input stream data");
        return;
    }
    if (stream == inputStreams_.end() || count == 0 ||
        count > stream->maxBatchSize) {
        sendError(stream == inputStreams_.end() ? ErrorCode::InvalidId
                                                : ErrorCode::InvalidMessage,
                  "Invalid input stream data");
        return;
    }

    std::vector<std::vector<uint8_t>> values;
    values.reserve(count);
    for (uint32_t index = 0; index < count; ++index) {
        const uint32_t valueLength = reader.getU32();
        const uint8_t* value = reader.getBytes(valueLength);
        if (!reader.ok() || valueLength > stream->maxValueSize ||
            !validateValuePayload(stream->value, value, valueLength)) {
            sendError(ErrorCode::InvalidMessage, "Invalid input stream value");
            return;
        }
        values.emplace_back(value, value + valueLength);
    }
    if (reader.remaining() != 0) {
        sendError(ErrorCode::InvalidMessage, "Trailing input stream data");
        return;
    }
    if (inputStreamDataFn_) inputStreamDataFn_(streamId, values);
}

void Session::handleCloseInputStreamReq(const uint8_t* body, size_t len) {
    BufReader reader(body, len);
    const uint32_t streamId = reader.getU32();
    if (!reader.ok() || reader.remaining() != 0) {
        sendError(ErrorCode::InvalidMessage, "Invalid input stream close request");
        return;
    }
    const auto stream = std::find_if(inputStreams_.begin(), inputStreams_.end(),
                                     [streamId](const InputStreamState& state) {
                                         return state.id == streamId;
                                     });
    if (stream == inputStreams_.end()) {
        sendError(ErrorCode::InvalidId, "Input stream not found");
        return;
    }
    inputStreams_.erase(stream);
    sendInputStreamResponse(MessageType::CloseInputStreamResp, streamId, true);
}

// --------------------------------------------------------------------------
// Response senders
// --------------------------------------------------------------------------

bool Session::sendRaw(const uint8_t* data, size_t len) {
    std::lock_guard<std::mutex> lock(sendMutex_);

    if (framing_ == Framing::None) {
        // Message-oriented transport: send raw protocol message directly.
        return transport_->send(data, len);
    }

    // Byte-stream transport: SLIP-encode before sending.
    size_t encLen = SLIPStream::encoded_length(data, len);
    if (encLen == SLIPStream::ENCODE_ERROR) return false;

    if (txEncBuf_.size() < encLen) txEncBuf_.resize(encLen);

    size_t written = SLIPStream::encode_packet(data, len,
                                               txEncBuf_.data(), txEncBuf_.size());
    if (written == SLIPStream::ENCODE_ERROR) return false;

    return transport_->send(txEncBuf_.data(), written);
}

void Session::sendPongResp(uint32_t nonce) {
    uint8_t buf[1 + MAX_VARINT_SIZE];
    BufWriter w(buf, sizeof(buf));
    w.putU8(static_cast<uint8_t>(MessageType::PongResp));
    w.putVarint(nonce);
    if (w.ok()) sendRaw(buf, w.pos);
}

void Session::sendSubscribeLogResp(uint32_t subscriptionId, bool success,
                                   std::string_view error) {
    const size_t errorLength = std::min(error.size(), static_cast<size_t>(UINT16_MAX));
    const size_t totalLength = 1 + MAX_VARINT_SIZE + 1 + (success ? 0 : 2 + errorLength);
    if (txRawBuf_.size() < totalLength) txRawBuf_.resize(totalLength);

    BufWriter w(txRawBuf_.data(), totalLength);
    w.putU8(static_cast<uint8_t>(MessageType::SubscribeLogResp));
    w.putVarint(subscriptionId);
    w.putU8(success ? 0 : 1);
    if (!success) {
        w.putStr16(error.data(), errorLength);
    }
    if (w.ok()) sendRaw(txRawBuf_.data(), w.pos);
}

void Session::sendUnsubscribeLogResp(uint32_t subscriptionId, bool found) {
    uint8_t buf[1 + MAX_VARINT_SIZE + 1];
    BufWriter w(buf, sizeof(buf));
    w.putU8(static_cast<uint8_t>(MessageType::UnsubscribeLogResp));
    w.putVarint(subscriptionId);
    w.putU8(found ? 0 : 1);
    if (w.ok()) sendRaw(buf, w.pos);
}

bool Session::matchesLog(const LogSubscription& subscription,
                         const LogRecord& record) const {
    if (static_cast<uint8_t>(record.severity) < static_cast<uint8_t>(subscription.minSeverity)) {
        return false;
    }
    const auto contains = [](const std::string& filter, const std::string& value) {
        return filter.empty() || value.find(filter) != std::string::npos;
    };
    return contains(subscription.componentFilter, record.component) &&
           contains(subscription.messageFilter, record.message) &&
           contains(subscription.locationFilter, record.location);
}

void Session::sendLogData(const LogRecord& record) {
    const size_t componentLength = std::min(record.component.size(), static_cast<size_t>(UINT16_MAX));
    const size_t messageLength = std::min(record.message.size(), static_cast<size_t>(UINT16_MAX));
    const size_t locationLength = std::min(record.location.size(), static_cast<size_t>(UINT16_MAX));
    const size_t totalLength = 1 + 8 + 1 + 2 + componentLength +
                               2 + messageLength + 2 + locationLength;
    std::vector<uint8_t> raw(totalLength);
    BufWriter w(raw.data(), raw.size());
    w.putU8(static_cast<uint8_t>(MessageType::LogData));
    w.putU64(record.timestampUs);
    w.putU8(static_cast<uint8_t>(record.severity));
    w.putStr16(record.component.data(), componentLength);
    w.putStr16(record.message.data(), messageLength);
    w.putStr16(record.location.data(), locationLength);
    if (w.ok()) sendRaw(raw.data(), w.pos);
}

void Session::sendFunctionCallResponse(uint64_t functionId,
                                       const FunctionReturn& returnValue,
                                       const FunctionCallResult& result) {
    const size_t messageSize = std::min(result.errorMessage.size(), MAX_STRING_SIZE);
    const size_t valueSize = std::min(result.returnValue.size(), MAX_VARIABLE_VALUE_SIZE);
    const bool hasReturnValue = result.success && returnValue.present;
    const size_t totalSize = 1 + 8 + 1 + 4 + 2 + messageSize +
                             (hasReturnValue ? FUNCTION_TLV_HEADER_SIZE + valueSize : 0);
    if (totalSize > MAX_MESSAGE_SIZE) {
        sendError(ErrorCode::InternalError, "Function response too large");
        return;
    }
    std::vector<uint8_t> raw(totalSize);
    BufWriter writer(raw.data(), raw.size());
    writer.putU8(static_cast<uint8_t>(MessageType::CallFunctionResp));
    writer.putU64(functionId);
    writer.putU8(result.success ? 0 : 1);
    if (!result.success) {
        writer.putU32(static_cast<uint32_t>(result.error));
        writer.putStr16(result.errorMessage.data(), messageSize);
    } else {
        writer.putU32(0);
        writer.putU16(0);
        if (hasReturnValue) {
            encodeFunctionTlv(writer, 0, returnValue.type,
                              result.returnValue.data(), valueSize);
        }
    }
    if (writer.ok()) sendRaw(raw.data(), writer.pos);
}

void Session::sendInvokeExResponse(uint64_t requestId,
                                   const FunctionReturn& returnValue,
                                   const FunctionCallResult& result) {
    const size_t messageSize = std::min(result.errorMessage.size(), MAX_STRING_SIZE);
    const size_t valueSize = std::min(result.returnValue.size(), MAX_VARIABLE_VALUE_SIZE);
    const bool hasReturnValue = result.success && returnValue.present;
    const size_t totalSize = 1 + 8 + 1 + 4 + 2 + messageSize +
                             (hasReturnValue ? FUNCTION_TLV_HEADER_SIZE + valueSize : 0);
    if (totalSize > MAX_MESSAGE_SIZE) {
        sendError(ErrorCode::InternalError, "InvokeEx response too large");
        return;
    }
    std::vector<uint8_t> raw(totalSize);
    BufWriter writer(raw.data(), raw.size());
    writer.putU8(static_cast<uint8_t>(MessageType::InvokeExResp));
    writer.putU64(requestId);
    writer.putU8(result.success ? 0 : 1);
    writer.putU32(result.success ? 0 : static_cast<uint32_t>(result.error));
    writer.putStr16(result.errorMessage.data(), messageSize);
    if (hasReturnValue) {
        encodeFunctionTlv(writer, 0, returnValue.type,
                          result.returnValue.data(), valueSize);
    }
    if (writer.ok()) sendRaw(raw.data(), writer.pos);
}

void Session::sendRegisterFunctionsResp(uint32_t count, bool success) {
    uint8_t buffer[1 + 4 + 1];
    BufWriter writer(buffer, sizeof(buffer));
    writer.putU8(static_cast<uint8_t>(MessageType::RegisterFunctionsResp));
    writer.putU32(count);
    writer.putU8(success ? 0 : 1);
    if (writer.ok()) sendRaw(buffer, writer.pos);
}

void Session::sendInputStreamResponse(MessageType type, uint32_t streamId, bool success) {
    uint8_t buffer[1 + 4 + 1];
    BufWriter writer(buffer, sizeof(buffer));
    writer.putU8(static_cast<uint8_t>(type));
    writer.putU32(streamId);
    writer.putU8(success ? 0 : 1);
    if (writer.ok()) sendRaw(buffer, writer.pos);
}

void Session::publishLog(LogSeverity severity, std::string_view component,
                         std::string_view message, std::string_view location) {
    LogRecord record;
    record.timestampUs = getTimestampUs_ ? getTimestampUs_() : 0;
    record.severity = severity;
    record.component.assign(component);
    record.message.assign(message);
    record.location.assign(location);

    // The worker owns subscription mutation. Copy the matching decision before
    // sending so an asynchronous logger never races with subscribe/unsubscribe.
    bool matched = false;
    {
        std::lock_guard<std::mutex> lock(sendMutex_);
        for (const auto& subscription : logSubscriptions_) {
            if (matchesLog(subscription, record)) {
                matched = true;
                break;
            }
        }
    }
    if (matched) sendLogData(record);
}

void Session::sendError(ErrorCode code, const char* msg) {
    if (logFn_) logFn_("TetherIO", "← TX Error: code=%u msg=%s",
                        static_cast<unsigned>(code), msg ? msg : "(null)");
    size_t msgLen = std::min(msg ? std::strlen(msg) : 0,
                             static_cast<size_t>(UINT16_MAX));
    size_t totalLen = 1 + 4 + 2 + msgLen;
    if (txRawBuf_.size() < totalLen) txRawBuf_.resize(totalLen);

    BufWriter w(txRawBuf_.data(), totalLen);
    w.putU8(static_cast<uint8_t>(MessageType::Error));
    w.putU32(static_cast<uint32_t>(code));
    w.putU16(static_cast<uint16_t>(msgLen));
    if (msgLen > 0) w.putBytes(msg, msgLen);
    if (w.ok()) sendRaw(txRawBuf_.data(), w.pos);
}

void Session::sendCatalogChanged() {
    uint8_t buf[5];
    BufWriter w(buf, sizeof(buf));
    w.putU8(static_cast<uint8_t>(MessageType::CatalogChanged));
    w.putU32(registry_.revision());
    if (w.ok()) sendRaw(buf, w.pos);
}

// --------------------------------------------------------------------------
// Streaming internals
// --------------------------------------------------------------------------

void Session::buildCollectPlan() {
    collectPlan_.clear();
    rowSize_ = 0;
    hasVariableEntries_ = false;

    for (uint64_t eid : configuredEntryIds_) {
        EntryView entry = registry_.find(eid);
        if (!entry) continue;
        if (entry.flags() & EntryFlags::NoStream) continue;
        CollectSlot slot;
        slot.paramId    = eid;
        slot.entry      = entry;
        slot.isVariable = entry.isVariableLength();
        slot.maxValueSize = entry.maxValueSize();
        if (slot.isVariable) {
            hasVariableEntries_ = true;
            slot.valueSize = 0;
            rowSize_ += static_cast<uint32_t>(MAX_VARINT_SIZE + slot.maxValueSize);
        } else {
            slot.valueSize = entry.valueSize();
            rowSize_ += slot.valueSize;
        }
        collectPlan_.push_back(std::move(slot));
    }

    fullRowSize_ = 8 + rowSize_;

    size_t chunkBytes = static_cast<size_t>(chunkSize_) * fullRowSize_;
    chunkBuf_.resize(chunkBytes, 0);
    chunkWritePos_ = 0;

    size_t maxRaw = 1 + 4 + 4 + chunkBytes;
    txRawBuf_.reserve(maxRaw);
    txEncBuf_.reserve(maxRaw * 2 + 1);
}

void Session::collectOneRow() {
    std::vector<std::vector<uint8_t>> currentValues;
    if (!streamFilters_.empty()) currentValues.resize(collectPlan_.size());
    if (hasVariableEntries_) {
        // Ensure buffer has room
        if (chunkWritePos_ + fullRowSize_ > chunkBuf_.size()) {
            chunkBuf_.resize(chunkWritePos_ + fullRowSize_);
        }

        uint8_t* row = chunkBuf_.data() + chunkWritePos_;
        uint8_t* rowStart = row;

        uint64_t ts = getTimestampUs_();
        std::memcpy(row, &ts, 8);
        row += 8;

        for (size_t i = 0; i < collectPlan_.size(); ++i) {
            const auto& slot = collectPlan_[i];
            if (slot.isVariable) {
                std::vector<uint8_t> tmp(slot.maxValueSize);
                size_t vlen = slot.entry.readVar(tmp.data(), tmp.size());
                vlen = std::min(vlen, tmp.size());
                if (!currentValues.empty()) {
                    currentValues[i].assign(tmp.begin(), tmp.begin() + vlen);
                }
                size_t vBytes = encodeVarint(row, MAX_VARINT_SIZE, static_cast<uint32_t>(vlen));
                row += vBytes;
                std::memcpy(row, tmp.data(), vlen);
                row += vlen;
            } else {
                slot.entry.read(row);
                if (!currentValues.empty()) {
                    currentValues[i].assign(row, row + slot.valueSize);
                }
                row += slot.valueSize;
            }
        }

        if (!passesStreamFilters(currentValues)) {
            chunkWritePos_ = static_cast<size_t>(rowStart - chunkBuf_.data());
            return;
        }

        chunkWritePos_ = static_cast<size_t>(row - chunkBuf_.data());
        rowsInChunk_++;
        return;
    }

    size_t offset = static_cast<size_t>(rowsInChunk_) * fullRowSize_;
    uint8_t* row = chunkBuf_.data() + offset;
    uint8_t* rowStart = row;

    uint64_t ts = getTimestampUs_();
    std::memcpy(row, &ts, 8);
    row += 8;

    for (size_t i = 0; i < collectPlan_.size(); ++i) {
        const auto& slot = collectPlan_[i];
        slot.entry.read(row);
        if (!currentValues.empty()) {
            currentValues[i].assign(row, row + slot.valueSize);
        }

        // Threshold check
        if (!lastValues_[i].empty()) {
            if (!thresholdFilter_.passes(slot.paramId, lastValues_[i].data(), row, slot.valueSize)) {
                // Value didn't change enough — copy last value
                std::memcpy(row, lastValues_[i].data(), slot.valueSize);
            }
        }
        // Update last value
        if (lastValues_[i].size() != slot.valueSize) {
            lastValues_[i].resize(slot.valueSize);
        }
        std::memcpy(lastValues_[i].data(), row, slot.valueSize);

        row += slot.valueSize;
    }

    if (!passesStreamFilters(currentValues)) {
        std::memset(rowStart, 0, fullRowSize_);
        return;
    }

    rowsInChunk_++;
}

bool Session::passesStreamFilters(const std::vector<std::vector<uint8_t>>& values) const {
    if (streamFilters_.empty()) return true;
    for (const auto& filter : streamFilters_) {
        bool matched = false;
        for (size_t i = 0; i < collectPlan_.size(); ++i) {
            if (collectPlan_[i].entry.name() == filter.name &&
                values.size() > i && values[i] == filter.value.data) {
                matched = true;
                break;
            }
        }
        if (!matched) return false;
    }
    return true;
}

bool Session::shouldTrigger() {
    if (triggerMode_ == TriggerMode::Time) return true;

    // OnChange: check if any collected entry changed
    for (size_t i = 0; i < collectPlan_.size(); ++i) {
        const auto& slot = collectPlan_[i];
        std::vector<uint8_t> currentValue;
        uint8_t vs = slot.valueSize;
        if (slot.isVariable) {
            currentValue.resize(slot.maxValueSize);
            const size_t actual = slot.entry.readVar(currentValue.data(), currentValue.size());
            currentValue.resize(actual);
            vs = static_cast<uint8_t>(std::min<size_t>(actual, 255));
        } else {
            currentValue.resize(vs);
            slot.entry.read(currentValue.data());
        }

        if (lastValues_[i].empty()) {
            lastValues_[i] = currentValue;
            return true;
        }

        if (currentValue.size() != lastValues_[i].size() ||
            std::memcmp(currentValue.data(), lastValues_[i].data(), currentValue.size()) != 0) {
            return true;
        }
    }
    return false;
}

void Session::handleStreamingCycle() {
    if (ringSource_) {
        // Ring-backed stream: the producer timestamps and buffers rows; the
        // interval is the drain cadence.  Trigger/skip are evaluated per row
        // inside collectRingRows().
        collectRingRows();
        if (rowsInChunk_ >= chunkSize_) {
            sendStreamData();
        } else if (rowsInChunk_ > 0) {
            // A ring producer may pause mid-chunk; without a flush the pending
            // rows would stall indefinitely.  Flush once they've waited as
            // long as a polled chunk would take to fill (interval x chunk).
            const uint64_t now = getTimestampUs_();
            const uint64_t budget =
                static_cast<uint64_t>(intervalUs_) * chunkSize_;
            if (now - lastRingRowUs_ >= budget) sendStreamData();
        }
        return;
    }

    if (!shouldTrigger()) return;

    if (skipCounter_ > 0) {
        skipCounter_--;
        return;
    }
    skipCounter_ = skipCount_;

    collectOneRow();

    if (rowsInChunk_ >= chunkSize_) {
        sendStreamData();
    }
}

void Session::sendStreamData() {
    if (rowsInChunk_ == 0) return;

    size_t payloadLen = hasVariableEntries_ ? chunkWritePos_
                        : static_cast<size_t>(rowsInChunk_) * fullRowSize_;
    size_t headerLen = 1 + 4 + 4;
    size_t totalLen = headerLen + payloadLen;

    if (txRawBuf_.size() < totalLen) txRawBuf_.resize(totalLen);

    BufWriter w(txRawBuf_.data(), totalLen);
    w.putU8(static_cast<uint8_t>(MessageType::StreamData));
    w.putU32(specId_);
    w.putU32(rowsInChunk_);
    w.putBytes(chunkBuf_.data(), payloadLen);

    if (w.ok()) {
        if (logFn_) logFn_("TetherIO", "← TX StreamData: specId=%u, %u rows, %zu bytes",
                            specId_, rowsInChunk_, w.pos);
        if (!sendRaw(txRawBuf_.data(), w.pos)) {
            if (logFn_) logFn_("TetherIO", "sendStreamData: sendRaw failed, stopping stream");
            streaming_ = false;
        }
    } else {
        if (logFn_) logFn_("TetherIO", "sendStreamData: BufWriter failed (totalLen=%zu)", totalLen);
    }
    rowsInChunk_ = 0;
    chunkWritePos_ = 0;
}

// --------------------------------------------------------------------------
// Ring-buffered streaming
// --------------------------------------------------------------------------

bool Session::bindRingSource() {
    ringSource_ = nullptr;
    ringFieldOffsets_.clear();
    if (!ringSources_ || collectPlan_.empty() || hasVariableEntries_)
        return true;  // no sources registered / nothing streamable / var-len
                      // entries can't come from a fixed-size ring

    for (IRingStreamSource* src : *ringSources_) {
        if (!src) continue;
        auto ids   = src->schemaEntryIds();
        auto sizes = src->schemaFieldSizes();

        // The source must cover every configured entry.
        ringFieldOffsets_.assign(collectPlan_.size(), 0);
        bool all = true;
        for (size_t i = 0; i < collectPlan_.size() && all; ++i) {
            const auto& slot = collectPlan_[i];
            uint32_t offset = 8;  // leading u64 producer timestamp
            bool found = false;
            for (size_t j = 0; j < ids.size(); ++j) {
                if (ids[j] == slot.paramId) {
                    // Schema field size must match the registry value size.
                    if (sizes[j] != slot.valueSize) { all = false; break; }
                    ringFieldOffsets_[i] = offset;
                    found = true;
                    break;
                }
                offset += sizes[j];
            }
            if (!found) all = false;
        }
        if (!all) { ringFieldOffsets_.clear(); continue; }

        if (!src->tryAcquire()) return false;  // schema matches but in use
        ringSource_ = src;
        ringScratch_.assign(
            static_cast<size_t>(chunkSize_) * src->rowSize(), 0);
        if (logFn_)
            logFn_("TetherIO",
                   "Stream bound to ring source: %zu fields, rowSize=%zu, dropped=%llu",
                   collectPlan_.size(), src->rowSize(),
                   static_cast<unsigned long long>(src->dropped()));
        return true;
    }
    return true;  // no source covers this configuration: fall back to polling
}

void Session::collectRingRows() {
    const size_t srcRowSize = ringSource_->rowSize();
    const size_t space = chunkSize_ - rowsInChunk_;
    if (space == 0) return;
    if (ringScratch_.size() < space * srcRowSize)
        ringScratch_.resize(space * srcRowSize);

    std::vector<std::vector<uint8_t>> currentValues;
    if (!streamFilters_.empty()) currentValues.resize(collectPlan_.size());

    const size_t got = ringSource_->drainRows(ringScratch_.data(), space);
    if (got > 0) lastRingRowUs_ = getTimestampUs_();
    for (size_t n = 0; n < got; ++n) {
        const uint8_t* srow = ringScratch_.data() + n * srcRowSize;

        // OnChange trigger: emit a row only if a field differs from the
        // last emitted values.
        if (triggerMode_ == TriggerMode::OnChange) {
            bool changed = false;
            for (size_t i = 0; i < collectPlan_.size() && !changed; ++i) {
                const auto& slot = collectPlan_[i];
                const uint8_t* f = srow + ringFieldOffsets_[i];
                if (lastValues_[i].size() != slot.valueSize ||
                    std::memcmp(f, lastValues_[i].data(), slot.valueSize) != 0)
                    changed = true;
            }
            if (!changed) continue;
        }

        if (skipCounter_ > 0) { skipCounter_--; continue; }
        skipCounter_ = skipCount_;

        // Stream filters evaluate the producer's raw field values (same
        // inputs as the polled path's currentValues).
        if (!currentValues.empty()) {
            for (size_t i = 0; i < collectPlan_.size(); ++i) {
                const auto& slot = collectPlan_[i];
                const uint8_t* f = srow + ringFieldOffsets_[i];
                currentValues[i].assign(f, f + slot.valueSize);
            }
            if (!passesStreamFilters(currentValues)) continue;
        }

        uint8_t* row = chunkBuf_.data() +
                       static_cast<size_t>(rowsInChunk_) * fullRowSize_;
        std::memcpy(row, srow, 8);  // producer timestamp
        row += 8;

        for (size_t i = 0; i < collectPlan_.size(); ++i) {
            const auto& slot = collectPlan_[i];
            const uint8_t* f = srow + ringFieldOffsets_[i];
            // Threshold compression mirrors collectOneRow(): fields that
            // didn't pass the filter are sent as their last value.
            if (!lastValues_[i].empty() &&
                !thresholdFilter_.passes(slot.paramId, lastValues_[i].data(),
                                         f, slot.valueSize)) {
                std::memcpy(row, lastValues_[i].data(), slot.valueSize);
            } else {
                std::memcpy(row, f, slot.valueSize);
            }
            if (lastValues_[i].size() != slot.valueSize)
                lastValues_[i].resize(slot.valueSize);
            std::memcpy(lastValues_[i].data(), row, slot.valueSize);
            row += slot.valueSize;
        }
        rowsInChunk_++;
    }
}

void Session::releaseRingSource() {
    if (!ringSource_) return;
    ringSource_->stop();
    ringSource_->release();
    if (logFn_)
        logFn_("TetherIO", "Ring source released (dropped=%llu)",
               static_cast<unsigned long long>(ringSource_->dropped()));
    ringSource_ = nullptr;
    ringFieldOffsets_.clear();
}

}} // namespace tether::io
