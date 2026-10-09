/**
 * @file Session_streaming.cpp
 * @brief Session — streaming internals.
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
