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

namespace tether { namespace io {

// --------------------------------------------------------------------------
// Construction / Destruction
// --------------------------------------------------------------------------

Session::Session(std::unique_ptr<ITransport> transport,
                 Registry& registry,
                 TimestampFn tsFn,
                 LogFn logFn,
                 const FeatureSet* serverFeatures,
                 DatalogRecorder* datalogRecorder)
    : transport_(std::move(transport))
    , registry_(registry)
    , getTimestampUs_(tsFn)
    , logFn_(logFn)
    , serverFeatures_(serverFeatures)
    , datalogRecorder_(datalogRecorder)
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
    if (!logFn_) return;
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    logFn_("TetherIOSession", "%s", buf);
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

        if (slipRxPos_ >= SLIP_RX_BUF_SIZE) {
            slipRxPos_ = 0;
            slipDiscardUntilEnd_ = (data[i] != 0xC0);
            continue;
        }
        slipRxBuf_[slipRxPos_++] = data[i];

        if (data[i] == 0xC0) {  // SLIP END
            size_t decLen = SLIPStream::decoded_length(slipRxBuf_, slipRxPos_);
            if (decLen != SLIPStream::DECODE_ERROR && decLen > 0 &&
                decLen <= DECODE_BUF_SIZE) {
                size_t wrote = SLIPStream::decode_packet(
                    slipRxBuf_, slipRxPos_, decodeBuf_, DECODE_BUF_SIZE);
                if (wrote != SLIPStream::DECODE_ERROR && wrote > 0) {
                    onSlipMessage(decodeBuf_, wrote);
                }
            }
            slipRxPos_ = 0;
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
        case MessageType::SnapshotParamsReq:   handleSnapshotParamsReq(body, bodyLen); break;
        case MessageType::SnapshotSignalsReq:  handleSnapshotSignalsReq(body, bodyLen); break;
        case MessageType::FeatureExchangeReq:  handleFeatureExchangeReq(body, bodyLen); break;
        case MessageType::ConfigureDatalogReq: handleConfigureDatalogReq(body, bodyLen); break;
        case MessageType::DatalogStatusReq:    handleDatalogStatusReq(); break;
        case MessageType::ConfigureThresholdReq: handleConfigureThresholdReq(body, bodyLen); break;
        case MessageType::DescribeStructReq:   handleDescribeStructReq(body, bodyLen); break;
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

    if (len < 17) { sendError(ErrorCode::InvalidMessage, "ConfigureStreamReq too short"); return; }

    BufReader r(body, len);
    uint8_t trigMode   = r.getU8();
    uint32_t interval  = r.getU32();
    uint32_t chunk     = r.getU32();
    uint32_t skip      = r.getU32();
    uint32_t entryCount = r.getU32();
    if (!r.ok()) { sendError(ErrorCode::InvalidMessage, "parse error"); return; }

    if (trigMode > 1) { sendError(ErrorCode::InvalidMessage, "Invalid trigger mode"); return; }
    if (chunk == 0) chunk = 1;
    if (interval == 0) interval = 1;

    std::vector<uint64_t> entryIds;
    entryIds.reserve(entryCount);
    for (uint32_t i = 0; i < entryCount; ++i) {
        uint64_t eid = r.getU64();
        if (!r.ok()) { sendError(ErrorCode::InvalidMessage, "Truncated entry list"); return; }
        entryIds.push_back(eid);
    }

    triggerMode_      = static_cast<TriggerMode>(trigMode);
    intervalUs_       = interval;
    chunkSize_        = chunk;
    skipCount_        = skip;
    configuredEntryIds_ = std::move(entryIds);
    specId_++;

    buildCollectPlan();
    configured_ = true;
    skipCounter_ = 0;
    rowsInChunk_ = 0;
    chunkWritePos_ = 0;
    lastTriggerValue_.clear();

    // Send ConfigureStreamAck
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
    FeatureSet::decode(r, clientFeatures_);

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

// --------------------------------------------------------------------------
// Response senders
// --------------------------------------------------------------------------

bool Session::sendRaw(const uint8_t* data, size_t len) {
    size_t encLen = SLIPStream::encoded_length(data, len);
    if (encLen == SLIPStream::ENCODE_ERROR) return false;

    if (txEncBuf_.size() < encLen) txEncBuf_.resize(encLen);

    size_t written = SLIPStream::encode_packet(data, len,
                                               txEncBuf_.data(), txEncBuf_.size());
    if (written == SLIPStream::ENCODE_ERROR) return false;

    return transport_->send(txEncBuf_.data(), written);
}

void Session::sendError(ErrorCode code, const char* msg) {
    size_t msgLen = msg ? std::strlen(msg) : 0;
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
    if (hasVariableEntries_) {
        // Ensure buffer has room
        if (chunkWritePos_ + fullRowSize_ > chunkBuf_.size()) {
            chunkBuf_.resize(chunkWritePos_ + fullRowSize_);
        }

        uint8_t* row = chunkBuf_.data() + chunkWritePos_;

        uint64_t ts = getTimestampUs_();
        std::memcpy(row, &ts, 8);
        row += 8;

        for (size_t i = 0; i < collectPlan_.size(); ++i) {
            const auto& slot = collectPlan_[i];
            if (slot.isVariable) {
                uint8_t tmp[1024];
                size_t vlen = slot.entry.readVar(tmp, sizeof(tmp));
                size_t vBytes = encodeVarint(row, MAX_VARINT_SIZE, static_cast<uint32_t>(vlen));
                row += vBytes;
                std::memcpy(row, tmp, vlen);
                row += vlen;
            } else {
                slot.entry.read(row);
                row += slot.valueSize;
            }
        }

        chunkWritePos_ = static_cast<size_t>(row - chunkBuf_.data());
        rowsInChunk_++;
        return;
    }

    size_t offset = static_cast<size_t>(rowsInChunk_) * fullRowSize_;
    uint8_t* row = chunkBuf_.data() + offset;

    uint64_t ts = getTimestampUs_();
    std::memcpy(row, &ts, 8);
    row += 8;

    for (size_t i = 0; i < collectPlan_.size(); ++i) {
        const auto& slot = collectPlan_[i];
        slot.entry.read(row);

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

    rowsInChunk_++;
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
