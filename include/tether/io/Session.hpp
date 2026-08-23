/**
 * @file Session.hpp
 * @brief Per-client session for the Tether IO protocol.
 *
 * Each Session handles one transport connection.  It owns:
 *  - SLIP deframing/framing of transport bytes
 *  - Protocol message dispatch and response generation
 *  - Stream configuration, collection plan, and data transmission
 *  - Threshold filtering for change-based streaming
 *  - Snapshot support
 *
 * Sessions are independent: no shared mutable state between sessions
 * (only the read-only Registry is shared).
 *
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#pragma once

#include "tether/io/Protocol.hpp"
#include "tether/io/Registry.hpp"
#include "tether/io/Transport.hpp"
#include "tether/io/ThresholdFilter.hpp"
#include "tether/io/FeatureExchange.hpp"
#include "tether/io/Datalogging.hpp"
#include <cstdint>
#include <cstddef>
#include <functional>
#include <vector>
#include <atomic>
#include <memory>
#include <mutex>

namespace tether { namespace io {

/// Precomputed slot for efficient per-cycle data collection.
struct CollectSlot {
    uint64_t  paramId;
    uint8_t   valueSize;
    EntryView entry;
    bool      isVariable = false;
    uint16_t  maxValueSize = 0;
};

/// Callback returning current time in microseconds.
using TimestampFn = std::function<uint64_t()>;

/// Optional printf-style log callback.
using LogFn = void(*)(const char* tag, const char* fmt, ...);

/**
 * @class Session
 * @brief Manages one transport connection for parameter/signal streaming.
 *
 * Lifecycle:
 *  1. Construct with a transport, registry reference, and config.
 *  2. Call run() from the session's dedicated thread.
 *  3. run() blocks until the transport disconnects or requestStop() is called.
 */
class Session {
public:
    Session(std::unique_ptr<ITransport> transport,
            Registry& registry,
            TimestampFn tsFn,
            LogFn logFn = nullptr,
            const FeatureSet* serverFeatures = nullptr,
            DatalogRecorder* datalogRecorder = nullptr);
    ~Session();

    /// Run event loop (blocking).
    void run();

    /// Request graceful shutdown from another thread.
    void requestStop();

    /// Mark the session as live before its worker thread enters run().
    void markRunning() { running_.store(true, std::memory_order_relaxed); }

    /// True while run() is executing.
    bool isRunning() const { return running_.load(std::memory_order_relaxed); }

    /// Publish a log record to all matching subscriptions on this session.
    /// Safe to call from a thread other than the session worker.
    void publishLog(LogSeverity severity, std::string_view component,
                    std::string_view message, std::string_view location = {});

private:
    // ---- SLIP deframing ----
    void feedSlipData(const uint8_t* data, size_t len);
    void onSlipMessage(const uint8_t* data, size_t len);

    // ---- Protocol message handlers ----
    void handleListParamsReq(const uint8_t* body, size_t len);
    void handleListSignalsReq(const uint8_t* body, size_t len);
    void handleGetParamReq(const uint8_t* body, size_t len);
    void handleSetParamReq(const uint8_t* body, size_t len);
    void handleGetSignalReq(const uint8_t* body, size_t len);
    void handleConfigureStreamReq(const uint8_t* body, size_t len);
    void handleStartStream();
    void handleStopStream();
    void handlePingReq(const uint8_t* body, size_t len);
    void handleSubscribeLogReq(const uint8_t* body, size_t len);
    void handleUnsubscribeLogReq(const uint8_t* body, size_t len);
    void handleGetMetadataReq(const uint8_t* body, size_t len);
    void handleSnapshotParamsReq(const uint8_t* body, size_t len);
    void handleSnapshotSignalsReq(const uint8_t* body, size_t len);
    void handleFeatureExchangeReq(const uint8_t* body, size_t len);
    void handleConfigureDatalogReq(const uint8_t* body, size_t len);
    void handleDatalogStatusReq();
    void handleConfigureThresholdReq(const uint8_t* body, size_t len);
    void handleDescribeStructReq(const uint8_t* body, size_t len);
    void handleListFunctionsReq(const uint8_t* body, size_t len);
    void handleCallFunctionReq(const uint8_t* body, size_t len);

    // ---- Response senders ----
    bool sendRaw(const uint8_t* data, size_t len);
    void sendError(ErrorCode code, const char* msg);
    void sendCatalogChanged();
    void sendPongResp(uint32_t nonce);
    void sendSubscribeLogResp(uint32_t subscriptionId, bool success,
                              std::string_view error = {});
    void sendUnsubscribeLogResp(uint32_t subscriptionId, bool found);
    bool matchesLog(const LogSubscription& subscription,
                    const LogRecord& record) const;
    void sendLogData(const LogRecord& record);
    void sendFunctionCallResponse(uint64_t functionId, const FunctionReturn& returnValue,
                                  const FunctionCallResult& result);

    // ---- Streaming internals ----
    void buildCollectPlan();
    void collectOneRow();
    bool shouldTrigger();
    bool passesStreamFilters(const std::vector<std::vector<uint8_t>>& values) const;
    void handleStreamingCycle();
    void sendStreamData();

    // ---- Logging ----
    void log(const char* fmt, ...) __attribute__((format(printf, 2, 3)));

    // ==== Dependencies ====
    std::unique_ptr<ITransport> transport_;
    Registry&        registry_;
    TimestampFn      getTimestampUs_;
    LogFn            logFn_;
    const FeatureSet* serverFeatures_;
    DatalogRecorder* datalogRecorder_;

    // ==== Run state ====
    std::atomic<bool> running_{false};
    std::atomic<bool> stopRequested_{false};

    // ==== Stream configuration ====
    bool         configured_      = false;
    bool         streaming_       = false;
    TriggerMode  triggerMode_     = TriggerMode::Time;
    uint32_t     intervalUs_      = 100000;     ///< Internal microseconds between samples
    uint32_t     chunkSize_       = 1;
    uint32_t     skipCount_       = 0;
    uint64_t     triggerEntryId_  = 0;
    uint32_t     specId_          = 0;
    std::vector<uint64_t> configuredEntryIds_;
    std::vector<FilterProperty> streamFilters_;

    // ==== Collection plan ====
    std::vector<CollectSlot> collectPlan_;
    uint32_t rowSize_     = 0;
    uint32_t fullRowSize_ = 0;  ///< 8 (timestamp) + rowSize_

    // ==== Streaming runtime ====
    uint32_t skipCounter_     = 0;
    uint32_t rowsInChunk_     = 0;
    uint64_t lastSampleTimeUs_ = 0;
    std::vector<uint8_t> chunkBuf_;
    size_t   chunkWritePos_   = 0;
    bool     hasVariableEntries_ = false;

    // ==== Threshold filtering ====
    ThresholdFilter thresholdFilter_;
    std::vector<std::vector<uint8_t>> lastValues_;  ///< Per-slot last value

    // ==== OnChange trigger state ====
    std::vector<uint8_t> lastTriggerValue_;

    // ==== Catalog change listener ====
    size_t catalogListenerHandle_ = 0;
    std::atomic<bool> catalogDirty_{false};

    // ==== Client features (received via FeatureExchangeReq) ====
    FeatureSet clientFeatures_;

    // ==== Log subscriptions ====
    std::vector<LogSubscription> logSubscriptions_;
    uint32_t nextLogSubscriptionId_ = 1;
    mutable std::mutex sendMutex_;

    // ==== SLIP receive buffers ====
    static constexpr size_t SLIP_RX_BUF_SIZE = 8192;
    uint8_t slipRxBuf_[SLIP_RX_BUF_SIZE];
    size_t  slipRxPos_ = 0;
    bool    slipDiscardUntilEnd_ = false;

    static constexpr size_t DECODE_BUF_SIZE = 8192;
    uint8_t decodeBuf_[DECODE_BUF_SIZE];

    // ==== TX buffers ====
    std::vector<uint8_t> txRawBuf_;
    std::vector<uint8_t> txEncBuf_;
};

}} // namespace tether::io
