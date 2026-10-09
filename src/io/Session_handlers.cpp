/**
 * @file Session_handlers.cpp
 * @brief Session — request message handlers.
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
    // Bound the chunk so a single StreamData frame and its backing buffer stay
    // reasonable regardless of what the client asked for (plan item 24).
    constexpr uint32_t MAX_STREAM_CHUNK = 4096;
    if (chunk > MAX_STREAM_CHUNK) chunk = MAX_STREAM_CHUNK;
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
        FilterProperty property;
        property.name.assign(reinterpret_cast<const char*>(name), nameLen);
        property.value.schemaEpoch = r.getU32();
        property.value.schemaSlot = r.getU32();
        const uint32_t valueLen = r.getVarint();
        const auto* node = schemaCatalog_ ? schemaCatalog_->resolve(
            property.value.schemaEpoch, property.value.schemaSlot) : nullptr;
        const auto* manifest = schemaCatalog_ ? schemaCatalog_->describe(
            property.value.schemaSlot) : nullptr;
        if (!r.ok() || !node || !manifest || valueLen > MAX_VARIABLE_VALUE_SIZE ||
            valueLen > r.remaining()) {
            sendError(ErrorCode::InvalidMessage, "Invalid stream filter schema");
            return;
        }
        const uint8_t* value = r.getBytes(valueLen);
        if (!r.ok()) {
            sendError(ErrorCode::InvalidMessage, "Truncated stream filter value");
            return;
        }
        property.value.schemaKey = manifest->ref.key;
        property.value.data.assign(value, value + valueLen);
        BufReader valueReader(property.value.data.data(), property.value.data.size());
        if (!validateSchemaValue(*schemaCatalog_->graph(), manifest->ref.key, valueReader) ||
            valueReader.remaining() != 0) {
            sendError(ErrorCode::InvalidMessage, "Invalid stream filter value");
            return;
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
            size += 4 + 4 + 4;
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
        for (size_t position = 0; position < function.parameterCount(); ++position) {
            const auto& parameter = function.parameters()[position];
            writeString(parameter.name);
            writeString(parameter.description);
            w.putU32(functionParameterKey(parameter, position));
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
            w.putU32(result.enumReference != 0 ? FunctionParameterFlags::HasEnum : 0);
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
    FunctionInvokeContext context;
    context.identity = identity();
    FunctionCallResult result = invokeFunctionChecked(function, argumentCount, r, &context);
    validateFunctionReturn(schemaCatalog_, function.returnValue(), result);
    sendFunctionCallResponse(functionId, function.returnValue(), result);
}

void Session::handleInvokeExReq(const uint8_t* body, size_t len) {
    BufReader r(body, len);
    const uint64_t requestId = r.getU64();
    const uint64_t functionId = r.getU64();
    const uint64_t deadlineUs = r.getU64();  // deadline hint: enforced by the initiator
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
    FunctionInvokeContext context;
    context.identity = identity();
    context.requestId = requestId;
    context.deadlineUs = deadlineUs;
    FunctionCallResult result = invokeFunctionChecked(function, argumentCount, r, &context);
    validateFunctionReturn(schemaCatalog_, function.returnValue(), result);
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
    const bool hasReturnValue = result.success && r.getU8() != 0;
    if (result.success && hasReturnValue) {
        FunctionArgument returnValue;
        if (decodeFunctionValue(r, returnValue) && returnValue.key == 1) {
            result.hasReturnValue = true;
            result.returnValue = std::move(returnValue.value);
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
}} // namespace tether::io
