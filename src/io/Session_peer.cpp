/**
 * @file Session_peer.cpp
 * @brief Session — peer function invocation and response senders.
 *
 * TU split out of Session.cpp.
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

// Peer (client-hosted) function invocation
// --------------------------------------------------------------------------

bool Session::callPeer(uint64_t functionId,
                       const std::vector<FunctionArgument>& arguments,
                       uint64_t deadlineUs,
                       InvokeResultFn callback) {
    if (!callback) return false;
    size_t size = 1 + 8 + 8 + 8 + 4;
    for (const auto& argument : arguments) {
        size += FUNCTION_VALUE_HEADER_MAX_SIZE + argument.value.size();
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
        const uint32_t key = argument.key != 0 ? argument.key : argument.position + 1;
        if (!encodeFunctionValue(w, key, argument.value.data(), argument.value.size())) {
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
    const SchemaEpoch epoch = reader.getU32();
    const SchemaSlot slot = reader.getU32();
    const uint32_t maxValueSize = reader.getU32();
    const uint32_t maxBatchSize = reader.getU32();
    const auto* node = schemaCatalog_ ? schemaCatalog_->resolve(epoch, slot) : nullptr;
    const auto* manifest = schemaCatalog_ ? schemaCatalog_->describe(slot) : nullptr;
    if (!reader.ok() || reader.remaining() != 0 || !node || !manifest ||
        maxValueSize == 0 || maxValueSize > MAX_VARIABLE_VALUE_SIZE ||
        maxBatchSize == 0 || maxBatchSize > MAX_AGGREGATE_ELEMENTS) {
        sendError(ErrorCode::InvalidMessage, "Invalid input stream descriptor");
        return;
    }

    if (nextInputStreamId_ == 0) nextInputStreamId_ = 1;
    const uint32_t streamId = nextInputStreamId_++;
    if (!inputStreamCreateFn_(streamId, manifest->ref, epoch, slot,
                              maxValueSize, maxBatchSize)) {
        sendError(ErrorCode::InternalError, "Input stream rejected");
        return;
    }
    inputStreams_.push_back({streamId, manifest->ref, epoch, slot,
                             maxValueSize, maxBatchSize});
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
            !schemaCatalog_ || !schemaCatalog_->graph()) {
            sendError(ErrorCode::InvalidMessage, "Invalid input stream value");
            return;
        }
        BufReader valueReader(value, valueLength);
        if (!validateSchemaValue(*schemaCatalog_->graph(), stream->schema.key, valueReader) ||
            valueReader.remaining() != 0) {
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
                             (result.success ? 1 : 0) +
                             (hasReturnValue ? FUNCTION_VALUE_HEADER_MAX_SIZE + valueSize : 0);
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
        writer.putU8(hasReturnValue ? 1 : 0);
        if (hasReturnValue) {
            encodeFunctionValue(writer, 1, result.returnValue.data(), valueSize);
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
                             1 + (hasReturnValue ? FUNCTION_VALUE_HEADER_MAX_SIZE + valueSize : 0);
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
    if (result.success) {
        writer.putU8(hasReturnValue ? 1 : 0);
        if (hasReturnValue) {
            encodeFunctionValue(writer, 1, result.returnValue.data(), valueSize);
        }
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
}} // namespace tether::io
