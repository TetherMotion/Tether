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
#include <random>

namespace tether { namespace io {

namespace {

bool validateCatalogValue(const SchemaCatalog* catalog, const EntryView& entry,
                 const uint8_t* data, size_t size) {
    if (!catalog || entry.schemaRef().key == SchemaKey{} || !catalog->graph()) return true;
    BufReader reader(data, size);
    return validateSchemaValue(*catalog->graph(), entry.schemaRef().key, reader) &&
        reader.remaining() == 0;
}

void validateFunctionReturn(const SchemaCatalog* catalog, const FunctionReturn& signature,
                            FunctionCallResult& result) {
    if (!result.success || !signature.present || signature.schema.key == SchemaKey{}) return;
    const auto* graph = catalog ? catalog->graph() : nullptr;
    const auto* node = graph ? graph->find(signature.schema.key) : nullptr;
    const auto slot = catalog ? catalog->slotFor(signature.schema) : std::nullopt;
    if (!node || !slot || computeSchemaDigest(*node) != signature.schema.digest) {
        result.success = false;
        result.error = ErrorCode::FunctionInvocationError;
        result.errorMessage = "Function return schema is not installed";
        result.returnValue.clear();
        return;
    }
    BufReader reader(result.returnValue.data(), result.returnValue.size());
    if (!validateSchemaValue(*graph, signature.schema.key, reader) || reader.remaining() != 0) {
        result.success = false;
        result.error = ErrorCode::FunctionInvocationError;
        result.errorMessage = "Function return value does not match its schema";
        result.returnValue.clear();
    }
}

} // namespace

// --------------------------------------------------------------------------
// Construction / Destruction
// --------------------------------------------------------------------------

Session::Session(std::unique_ptr<ITransport> transport,
                 Registry& registry,
                 TimestampFn tsFn,
                 LogFn logFn,
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
{
    // Stable random session identifier used by authority leases, audit
    // records, and disconnect cleanup.
    std::random_device rd;
    std::uniform_int_distribution<uint64_t> dist;
    sessionId_ = "sess-" + [&] {
        char buffer[17];
        std::snprintf(buffer, sizeof(buffer), "%016llx",
                      static_cast<unsigned long long>(dist(rd)));
        return std::string(buffer);
    }();
    identity_.sessionId = sessionId_;
    identity_.role = machine::Role::Observer;
    identity_.authenticated = false;
}

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

void Session::setIdentity(machine::SessionIdentity identity) {
    std::lock_guard lock(identityMutex_);
    identity.sessionId = sessionId_;
    identity_ = std::move(identity);
}

machine::SessionIdentity Session::identity() const {
    std::lock_guard lock(identityMutex_);
    return identity_;
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
        case MessageType::ClientHello:         handleClientHello(body, bodyLen); break;
        case MessageType::SchemaRequest:       handleSchemaRequest(body, bodyLen); break;
        case MessageType::SchemaCommit:        handleSchemaCommit(body, bodyLen); break;
        case MessageType::ConfigureDatalogReq: handleConfigureDatalogReq(body, bodyLen); break;
        case MessageType::DatalogStatusReq:    handleDatalogStatusReq(); break;
        case MessageType::ConfigureThresholdReq: handleConfigureThresholdReq(body, bodyLen); break;
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

    const size_t needed = 1 + 1 + 4 + 24 + 4 + 4 +
        response.schemas.size() * (16 + 32 + 4);
    if (txRawBuf_.size() < needed) txRawBuf_.resize(needed);
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
}} // namespace tether::io
