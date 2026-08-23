/**
 * @file Session.cpp
 * @brief Per-client session: SLIP deframing, protocol handling, streaming.
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#include "tether/io/Session.hpp"
#include "SLIPStream/Buffer.hpp"

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <limits>

namespace tether { namespace io {

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
                 ReceiveBufferFactory decodedBufferFactory)
    : transport_(std::move(transport))
    , registry_(registry)
    , getTimestampUs_(tsFn)
    , logFn_(logFn)
    , serverFeatures_(serverFeatures)
    , datalogRecorder_(datalogRecorder)
    , inputStreamCreateFn_(std::move(inputStreamCreateFn))
    , inputStreamDataFn_(std::move(inputStreamDataFn))
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

    streaming_ = false;
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
                    onSlipMessage(decodeBuf_->data(), wrote);
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

void Session::onSlipMessage(const uint8_t* data, size_t len) {
    if (len < 1) return;
    auto type = static_cast<MessageType>(data[0]);
    const uint8_t* body = data + 1;
    size_t bodyLen = len - 1;

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
        case MessageType::ConfigureDatalogReq: handleConfigureDatalogReq(body, bodyLen); break;
        case MessageType::DatalogStatusReq:    handleDatalogStatusReq(); break;
        case MessageType::ConfigureThresholdReq: handleConfigureThresholdReq(body, bodyLen); break;
        case MessageType::DescribeStructReq:   handleDescribeStructReq(body, bodyLen); break;
        case MessageType::ListFunctionsReq:    handleListFunctionsReq(body, bodyLen); break;
        case MessageType::CallFunctionReq:     handleCallFunctionReq(body, bodyLen); break;
        case MessageType::CreateInputStreamReq: handleCreateInputStreamReq(body, bodyLen); break;
        case MessageType::InputStreamData:      handleInputStreamData(body, bodyLen); break;
        case MessageType::CloseInputStreamReq:  handleCloseInputStreamReq(body, bodyLen); break;
        default:
            sendError(ErrorCode::UnknownMessageType, "Unknown message type");
            break;
    }
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

    size_t sz = 1 + 4 + 4 + 4;
    for (const auto& e : entries) {
        sz += 8 + 1 + 1 + 1 + 2 + e.name().size() + 2 + e.description().size() + 2 + e.group().size();
    }
    if (txRawBuf_.size() < sz) txRawBuf_.resize(sz);

    BufWriter w(txRawBuf_.data(), sz);
    w.putU8(static_cast<uint8_t>(MessageType::ListParamsResp));
    w.putU32(total);
    w.putU32(offset);
    w.putU32(static_cast<uint32_t>(entries.size()));
    for (const auto& e : entries) {
        w.putU64(e.id());
        w.putU8(static_cast<uint8_t>(e.valueType()));
        w.putU8(e.valueSize());
        w.putU8(e.flags());
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

    size_t sz = 1 + 4 + 4 + 4;
    for (const auto& e : entries) {
        sz += 8 + 1 + 1 + 1 + 2 + e.name().size() + 2 + e.description().size() + 2 + e.group().size();
    }
    if (txRawBuf_.size() < sz) txRawBuf_.resize(sz);

    BufWriter w(txRawBuf_.data(), sz);
    w.putU8(static_cast<uint8_t>(MessageType::ListSignalsResp));
    w.putU32(total);
    w.putU32(offset);
    w.putU32(static_cast<uint32_t>(entries.size()));
    for (const auto& e : entries) {
        w.putU64(e.id());
        w.putU8(static_cast<uint8_t>(e.valueType()));
        w.putU8(e.valueSize());
        w.putU8(e.flags());
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
    size_t sz = 1 + 8 + 1;
    if (entry.isVariableLength()) {
        value.resize(entry.maxValueSize());
        const size_t actual = entry.readVar(value.data(), value.size());
        value.resize(actual);
        sz += MAX_VARINT_SIZE + actual;
    } else {
        value.resize(vs);
        entry.read(value.data());
        sz += vs;
    }
    if (txRawBuf_.size() < sz) txRawBuf_.resize(sz);

    BufWriter w(txRawBuf_.data(), sz);
    w.putU8(static_cast<uint8_t>(MessageType::GetParamResp));
    w.putU64(id);
    w.putU8(vs);
    if (entry.isVariableLength()) {
        w.putVarint(static_cast<uint32_t>(value.size()));
    }
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

    if (entry.isVariableLength()) {
        uint32_t dataLen = r.getVarint();
        if (!r.ok() || dataLen > entry.maxValueSize()) {
            sendError(ErrorCode::InvalidMessage, "Invalid variable length"); return;
        }
        const uint8_t* data = r.getBytes(dataLen);
        if (!r.ok()) { sendError(ErrorCode::InvalidMessage, "Value too short"); return; }
        entry.writeVar(data, dataLen);
    } else {
        uint8_t vs = entry.valueSize();
        const uint8_t* data = r.getBytes(vs);
        if (!r.ok()) { sendError(ErrorCode::InvalidMessage, "Value too short"); return; }
        entry.write(data);
    }

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
    size_t sz = 1 + 8 + 1;
    if (entry.isVariableLength()) {
        value.resize(entry.maxValueSize());
        const size_t actual = entry.readVar(value.data(), value.size());
        value.resize(actual);
        sz += MAX_VARINT_SIZE + actual;
    } else {
        value.resize(vs);
        entry.read(value.data());
        sz += vs;
    }
    if (txRawBuf_.size() < sz) txRawBuf_.resize(sz);

    BufWriter w(txRawBuf_.data(), sz);
    w.putU8(static_cast<uint8_t>(MessageType::GetSignalResp));
    w.putU64(id);
    w.putU8(vs);
    if (entry.isVariableLength()) {
        w.putVarint(static_cast<uint32_t>(value.size()));
    }
    w.putBytes(value.data(), value.size());

    if (w.ok()) sendRaw(txRawBuf_.data(), w.pos);
}

void Session::handleConfigureStreamReq(const uint8_t* body, size_t len) {
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

    buildCollectPlan();
    configured_ = true;
    skipCounter_ = 0;
    rowsInChunk_ = 0;
    chunkWritePos_ = 0;
    lastTriggerValue_.clear();

    // Send ParameterStream ConfigureAck. Tether deliberately retains 32-bit
    // resolved-count and row-size fields to support larger catalogs/chunks.
    size_t sz = 1 + 4 + 4 + 4 + collectPlan_.size() * 10;
    if (txRawBuf_.size() < sz) txRawBuf_.resize(sz);

    BufWriter w(txRawBuf_.data(), sz);
    w.putU8(static_cast<uint8_t>(MessageType::ConfigureStreamAck));
    w.putU32(specId_);
    w.putU32(static_cast<uint32_t>(collectPlan_.size()));
    w.putU32(rowSize_);
    for (const auto& slot : collectPlan_) {
        w.putU64(slot.paramId);
        w.putU8(static_cast<uint8_t>(slot.entry.valueType()));
        w.putU8(slot.valueSize);
    }
    if (w.ok()) sendRaw(txRawBuf_.data(), w.pos);
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
}

void Session::handleStopStream() {
    if (!streaming_) { sendError(ErrorCode::NotStreaming, "Not streaming"); return; }
    if (rowsInChunk_ > 0) sendStreamData();
    streaming_ = false;
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

    size_t sz = 1 + 8 + 4;
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
        uint8_t valueSize;
        std::vector<uint8_t> bytes;
        bool variable = false;
    };
    std::vector<SnapshotValue> values;
    values.reserve(ids.size());

    size_t sz = 1 + 8 + 4;
    for (uint64_t id : ids) {
        EntryView e = registry_.findParam(id);
        if (!e) continue;
        SnapshotValue value{};
        value.id = id;
        value.valueSize = e.valueSize();
        value.variable = e.isVariableLength();
        if (value.variable) {
            value.bytes.resize(e.maxValueSize());
            const size_t actual = e.readVar(value.bytes.data(), value.bytes.size());
            value.bytes.resize(actual);
            sz += 8 + 1 + MAX_VARINT_SIZE + actual;
        } else {
            value.bytes.resize(value.valueSize);
            e.read(value.bytes.data());
            sz += 8 + 1 + value.valueSize;
        }
        values.push_back(std::move(value));
    }
    if (txRawBuf_.size() < sz) txRawBuf_.resize(sz);

    BufWriter w(txRawBuf_.data(), sz);
    w.putU8(static_cast<uint8_t>(MessageType::SnapshotParamsResp));
    w.putU64(ts);
    w.putU32(static_cast<uint32_t>(values.size()));
    for (const auto& value : values) {
        w.putU64(value.id);
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
        uint8_t valueSize;
        std::vector<uint8_t> bytes;
        bool variable = false;
    };
    std::vector<SnapshotValue> values;
    values.reserve(ids.size());

    size_t sz = 1 + 8 + 4;
    for (uint64_t id : ids) {
        EntryView e = registry_.findSignal(id);
        if (!e) continue;
        SnapshotValue value{};
        value.id = id;
        value.valueSize = e.valueSize();
        value.variable = e.isVariableLength();
        if (value.variable) {
            value.bytes.resize(e.maxValueSize());
            const size_t actual = e.readVar(value.bytes.data(), value.bytes.size());
            value.bytes.resize(actual);
            sz += 8 + 1 + MAX_VARINT_SIZE + actual;
        } else {
            value.bytes.resize(value.valueSize);
            e.read(value.bytes.data());
            sz += 8 + 1 + value.valueSize;
        }
        values.push_back(std::move(value));
    }
    if (txRawBuf_.size() < sz) txRawBuf_.resize(sz);

    BufWriter w(txRawBuf_.data(), sz);
    w.putU8(static_cast<uint8_t>(MessageType::SnapshotSignalsResp));
    w.putU64(ts);
    w.putU32(static_cast<uint32_t>(values.size()));
    for (const auto& value : values) {
        w.putU64(value.id);
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
    size_t size = 1 + 4 + 4 + 4;
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
            size += 1 + 1 + 8 + 8 + 4;
            if (parameter.valueDescriptor) {
                size += 1 + 4;
                size_t descriptorSize = 0;
                if (!valueDescriptorWireSize(*parameter.valueDescriptor, descriptorSize) ||
                    descriptorSize > UINT32_MAX || size > MAX_MESSAGE_SIZE - descriptorSize) {
                    sendError(ErrorCode::TooManyEntries, "Invalid function descriptor");
                    return;
                }
                size += descriptorSize;
            }
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
            size += 1 + 1 + 8 + 8 + 4;
            if (result.valueDescriptor) {
                size += 1 + 4;
                size_t descriptorSize = 0;
                if (!valueDescriptorWireSize(*result.valueDescriptor, descriptorSize) ||
                    descriptorSize > UINT32_MAX || size > MAX_MESSAGE_SIZE - descriptorSize) {
                    sendError(ErrorCode::TooManyEntries, "Invalid function descriptor");
                    return;
                }
                size += descriptorSize;
            }
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
            w.putU8(static_cast<uint8_t>(parameter.type));
            w.putU8(parameter.flags());
            w.putU64(parameter.enumReference);
            w.putU64(parameter.structReference);
            w.putU32(parameter.maxValueSize);
            w.putU8(parameter.valueDescriptor ? 1 : 0);
            if (parameter.valueDescriptor) {
                const size_t descriptorStart = w.pos;
                w.putU32(0);
                const size_t payloadStart = w.pos;
                encodeValueDescriptor(w, *parameter.valueDescriptor);
                const uint32_t descriptorSize = static_cast<uint32_t>(w.pos - payloadStart);
                if (w.ok()) {
                    txRawBuf_[descriptorStart] = static_cast<uint8_t>(descriptorSize);
                    txRawBuf_[descriptorStart + 1] = static_cast<uint8_t>(descriptorSize >> 8);
                    txRawBuf_[descriptorStart + 2] = static_cast<uint8_t>(descriptorSize >> 16);
                    txRawBuf_[descriptorStart + 3] = static_cast<uint8_t>(descriptorSize >> 24);
                }
            }
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
            w.putU8(static_cast<uint8_t>(result.type));
            w.putU8((result.enumReference != 0 ? FunctionParameterFlags::HasEnum : 0) |
                    (result.structReference != 0 ? FunctionParameterFlags::HasStruct : 0));
            w.putU64(result.enumReference);
            w.putU64(result.structReference);
            w.putU32(result.maxValueSize);
            w.putU8(result.valueDescriptor ? 1 : 0);
            if (result.valueDescriptor) {
                const size_t descriptorStart = w.pos;
                w.putU32(0);
                const size_t payloadStart = w.pos;
                encodeValueDescriptor(w, *result.valueDescriptor);
                const uint32_t descriptorSize = static_cast<uint32_t>(w.pos - payloadStart);
                if (w.ok()) {
                    txRawBuf_[descriptorStart] = static_cast<uint8_t>(descriptorSize);
                    txRawBuf_[descriptorStart + 1] = static_cast<uint8_t>(descriptorSize >> 8);
                    txRawBuf_[descriptorStart + 2] = static_cast<uint8_t>(descriptorSize >> 16);
                    txRawBuf_[descriptorStart + 3] = static_cast<uint8_t>(descriptorSize >> 24);
                }
            }
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
    if (argumentCount > function.parameterCount()) {
        sendError(ErrorCode::FunctionInvocationError, "Too many function arguments");
        return;
    }

    std::vector<FunctionArgument> supplied(function.parameterCount());
    std::vector<bool> seen(function.parameterCount(), false);
    for (uint32_t index = 0; index < argumentCount; ++index) {
        FunctionArgument argument;
        if (!decodeFunctionTlv(r, argument) || argument.position >= function.parameterCount()) {
            sendError(ErrorCode::FunctionInvocationError, "Invalid function argument TLV");
            return;
        }
        const auto position = static_cast<size_t>(argument.position);
        const auto& parameter = function.parameters()[position];
        if (seen[position] || argument.type != parameter.type) {
            sendError(ErrorCode::FunctionInvocationError, "Wrong or duplicate function argument");
            return;
        }
        if (parameter.valueDescriptor &&
            !validateValuePayload(*parameter.valueDescriptor, argument.value.data(),
                                  argument.value.size())) {
            sendError(ErrorCode::FunctionInvocationError, "Invalid aggregate function argument");
            return;
        }
        if (parameter.maxValueSize != 0 && argument.value.size() > parameter.maxValueSize) {
            sendError(ErrorCode::FunctionInvocationError, "Invalid function argument size");
            return;
        }
        const auto fixedSize = valueTypeSize(parameter.type);
        if ((fixedSize != 0 && argument.value.size() != fixedSize) ||
            argument.value.size() > parameter.maxValueSize && parameter.maxValueSize != 0) {
            sendError(ErrorCode::FunctionInvocationError, "Invalid function argument size");
            return;
        }
        supplied[position] = std::move(argument);
        seen[position] = true;
    }
    if (!r.ok() || r.remaining() != 0) {
        sendError(ErrorCode::FunctionInvocationError, "Trailing function call data");
        return;
    }

    for (size_t position = 0; position < function.parameterCount(); ++position) {
        const auto& parameter = function.parameters()[position];
        if (!seen[position]) {
            if (!parameter.optional || !parameter.hasDefault) {
                sendError(ErrorCode::FunctionInvocationError, "Missing required function argument");
                return;
            }
            supplied[position].position = static_cast<uint32_t>(position);
            supplied[position].type = parameter.type;
            supplied[position].value = parameter.defaultValue;
            supplied[position].provided = false;
            if (parameter.valueDescriptor &&
                !validateValuePayload(*parameter.valueDescriptor, supplied[position].value.data(),
                                      supplied[position].value.size())) {
                sendError(ErrorCode::FunctionInvocationError, "Invalid default function argument");
                return;
            }
            if (parameter.maxValueSize != 0 &&
                supplied[position].value.size() > parameter.maxValueSize) {
                sendError(ErrorCode::FunctionInvocationError, "Invalid default function argument");
                return;
            }
        }
    }

    FunctionCallResult result = function.invoke(supplied);
    const auto& returnValue = function.returnValue();
    if (result.success) {
        const size_t fixedSize = returnValue.present ? valueTypeSize(returnValue.type) : 0;
        const bool validReturn =
            (returnValue.present && fixedSize != 0 &&
             result.returnValue.size() == fixedSize) ||
            (returnValue.present && fixedSize == 0 &&
             result.returnValue.size() <=
                 (returnValue.maxValueSize != 0 ? returnValue.maxValueSize
                                                : MAX_VARIABLE_VALUE_SIZE)) ||
            (!returnValue.present && result.returnValue.empty());
        const bool validAggregateReturn =
            !returnValue.valueDescriptor ||
            validateValuePayload(*returnValue.valueDescriptor, result.returnValue.data(),
                                 result.returnValue.size());
        const bool withinReturnLimit =
            returnValue.maxValueSize == 0 || result.returnValue.size() <= returnValue.maxValueSize;
        if (!validReturn || !validAggregateReturn || !withinReturnLimit) {
            result.success = false;
            result.error = ErrorCode::FunctionInvocationError;
            result.errorMessage = "Invalid function return value";
            result.returnValue.clear();
        }
    }
    sendFunctionCallResponse(functionId, returnValue, result);
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
        if (!sendRaw(txRawBuf_.data(), w.pos)) {
            streaming_ = false;
        }
    }
    rowsInChunk_ = 0;
    chunkWritePos_ = 0;
}

}} // namespace tether::io
