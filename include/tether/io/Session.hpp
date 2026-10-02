/**
 * @file Session.hpp
 * @brief Per-client session for the Tether IO protocol.
 *
 * Each Session handles one transport connection.  It owns:
 *  - Message deframing (SLIP for byte-stream transports, or transport-native
 *    framing for message-oriented transports like WebSocket)
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

#include "tether/io/MachineControl.hpp"
#include "tether/io/Protocol.hpp"
#include "tether/io/SchemaNegotiation.hpp"
#include "tether/io/Registry.hpp"
#include "tether/io/Transport.hpp"
#include "tether/io/ReceiveBuffer.hpp"
#include "tether/io/ThresholdFilter.hpp"
#include "tether/io/Datalogging.hpp"
#include "tether/io/RingStreamSource.hpp"
#include <cstdint>
#include <cstddef>
#include <condition_variable>
#include <functional>
#include <vector>
#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <thread>

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

/// Called when a client creates an input stream. The schema reference identifies
/// each value in the committed session schema epoch; the callback may reject it.
using InputStreamCreateFn = std::function<bool(
    uint32_t streamId, const SchemaRef& schema, SchemaEpoch epoch,
    SchemaSlot slot, uint32_t maxValueSize, uint32_t maxBatchSize)>;

/// Called on the session worker after a validated input batch is received.
using InputStreamDataFn = std::function<void(
    uint32_t streamId, const std::vector<std::vector<uint8_t>>& values)>;

/**
 * @class Session
 * @brief Manages one transport connection for parameter/signal streaming.
 *
 * Lifecycle:
 *  1. Construct with a transport, registry reference, and config.
 *  2. Call run() from the session's dedicated thread.
 *  3. run() blocks until the transport disconnects or requestStop() is called.
 *
 * The `framing` parameter controls how message boundaries are established:
 *  - `Framing::Slip` (default): SLIP deframing is applied on top of the
 *    byte-stream transport.  Used for serial/TCP transports.
 *  - `Framing::None`: The transport provides native message boundaries
 *    (e.g. WebSocket).  No SLIP framing is applied — each transport message
 *    is treated as one protocol message.
 */
class Session {
public:
    Session(std::unique_ptr<ITransport> transport,
            Registry& registry,
            TimestampFn tsFn,
            LogFn logFn = nullptr,
            DatalogRecorder* datalogRecorder = nullptr,
            InputStreamCreateFn inputStreamCreateFn = nullptr,
            InputStreamDataFn inputStreamDataFn = nullptr,
            ReceiveBufferFactory encodedBufferFactory = nullptr,
            ReceiveBufferFactory decodedBufferFactory = nullptr,
            Framing framing = Framing::Slip,
            const std::vector<IRingStreamSource*>* ringSources = nullptr,
            const SchemaCatalog* schemaCatalog = nullptr);
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

    // ---- Peer (client-hosted) function invocation ----

    /// Outcome of a callPeer() invocation.  Wire results are delivered on
    /// the session worker thread; deadline expiries on the sweep thread.
    /// `timedOut` distinguishes a locally expired deadline from a
    /// peer-reported failure; a late InvokeExResp arriving after expiry
    /// is dropped without invoking the callback twice.
    struct InvokeResult {
        uint64_t   requestId  = 0;
        uint64_t   functionId = 0;
        bool       success    = false;
        bool       timedOut   = false;
        ErrorCode  error      = ErrorCode::None;
        std::string errorMessage;
        bool       hasReturnValue = false;
        ValueType  returnType = ValueType::Binary;
        std::vector<uint8_t> returnValue;
        /// Send-to-completion time; includes timeout value for timedOut.
        uint64_t   rttUs      = 0;
    };
    using InvokeResultFn = std::function<void(const InvokeResult&)>;

    /// Invoke a function the client published via RegisterFunctionsReq.
    /// `deadlineUs` bounds the wait in getTimestampUs_() units; 0 waits
    /// forever.  Returns false when the pending table is full
    /// (MAX_PENDING_INVOKES) or the transport send fails.  Safe to call
    /// from any thread; multiple calls may be outstanding concurrently
    /// (pipelining).
    bool callPeer(uint64_t functionId,
                  const std::vector<FunctionArgument>& arguments,
                  uint64_t deadlineUs,
                  InvokeResultFn callback);

    /// Copy of the catalog the client published via RegisterFunctionsReq.
    /// Safe to call from any thread.
    std::vector<FunctionDescriptor> peerFunctions() const;

    /// Registered id for `name`, or nullopt when no peer function matches.
    std::optional<uint64_t> findPeerFunctionId(std::string_view name) const;

    /// Number of InvokeEx calls currently awaiting a response.
    size_t pendingInvokeCount() const;

    // ---- Session identity (transport-authenticated caller) ----

    /// Stable identifier for this session, generated at construction.
    /// Authority leases and audit records reference it.
    const std::string& sessionId() const { return sessionId_; }

    /// Assign the transport-authenticated identity (e.g. from a WebSocket
    /// upgrade's TLS/mTLS credential or an authenticated session cookie).
    /// Until called, the session is an unauthenticated observer.
    void setIdentity(machine::SessionIdentity identity);

    /// Current session identity as seen by function invocation contexts.
    machine::SessionIdentity identity() const;

private:
    // ---- Message deframing ----
    void feedSlipData(const uint8_t* data, size_t len);
    void onMessage(const uint8_t* data, size_t len);

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
    void handleClientHello(const uint8_t* body, size_t len);
    void handleSchemaRequest(const uint8_t* body, size_t len);
    void handleSchemaCommit(const uint8_t* body, size_t len);
    void handleConfigureDatalogReq(const uint8_t* body, size_t len);
    void handleDatalogStatusReq();
    void handleConfigureThresholdReq(const uint8_t* body, size_t len);
    void handleListFunctionsReq(const uint8_t* body, size_t len);
    void handleCallFunctionReq(const uint8_t* body, size_t len);
    void handleCreateInputStreamReq(const uint8_t* body, size_t len);
    void handleInputStreamData(const uint8_t* body, size_t len);
    void handleCloseInputStreamReq(const uint8_t* body, size_t len);
    void handleInvokeExReq(const uint8_t* body, size_t len);
    void handleInvokeExResp(const uint8_t* body, size_t len);
    void handleRegisterFunctionsReq(const uint8_t* body, size_t len);

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
    void sendInvokeExResponse(uint64_t requestId, const FunctionReturn& returnValue,
                              const FunctionCallResult& result);
    void sendInputStreamResponse(MessageType type, uint32_t streamId, bool success);
    void sendRegisterFunctionsResp(uint32_t count, bool success);

    // ---- Peer invocation internals ----
    /// Deadline-enforcement thread: sleeps until the earliest pending
    /// deadline (or a catalog/pending change), then sweeps.  Required
    /// because expiry must not depend on wire traffic waking run().
    void invokeSweepLoop();
    /// Fail and remove all pending InvokeEx calls whose deadline passed.
    /// Callbacks fire outside invokeMutex_.
    void sweepInvokeDeadlines();
    /// Complete every pending call with an error (session teardown).
    void finishPendingInvokes();

    // ---- Streaming internals ----
    void buildCollectPlan();
    void collectOneRow();
    bool shouldTrigger();
    bool passesStreamFilters(const std::vector<std::vector<uint8_t>>& values) const;
    void handleStreamingCycle();
    void sendStreamData();

    // ---- Ring-buffered streaming ----
    /// Try to bind a ring stream source whose schema covers all configured
    /// entries.  Returns true if a source was bound (ringSource_ set) or no
    /// source matched (ringSource_ null) — false only when a matching source
    /// was found but is already bound to another session.
    bool bindRingSource();
    /// Drain pending rows from ringSource_ into chunkBuf_, projecting each
    /// schema row onto the configured field subset.
    void collectRingRows();
    /// Stop production and release the bound source (stream stop, reconfigure,
    /// session end).
    void releaseRingSource();

    // ---- Logging ----
    void log(const char* fmt, ...) __attribute__((format(printf, 2, 3)));

    // ==== Dependencies ====
    std::unique_ptr<ITransport> transport_;
    Registry&        registry_;
    TimestampFn      getTimestampUs_;
    LogFn            logFn_;
    DatalogRecorder* datalogRecorder_;
    InputStreamCreateFn inputStreamCreateFn_;
    InputStreamDataFn inputStreamDataFn_;
    Framing          framing_ = Framing::Slip;

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

    struct InputStreamState {
        uint32_t id = 0;
        SchemaRef schema;
        SchemaEpoch epoch = 0;
        SchemaSlot slot = 0;
        uint32_t maxValueSize = 0;
        uint32_t maxBatchSize = 0;
    };
    std::vector<InputStreamState> inputStreams_;
    uint32_t nextInputStreamId_ = 1;

    // ==== Peer (client-registered) function catalog ====
    /// Written by the session worker on RegisterFunctionsReq; read by
    /// applications through peerFunctions()/findPeerFunctionId() from
    /// other threads.
    std::vector<FunctionDescriptor> peerFunctions_;
    mutable std::mutex peerFunctionsMutex_;

    // ==== Outbound peer invocation ====
    struct PendingInvoke {
        uint64_t functionId   = 0;
        uint64_t sentAtUs     = 0;
        uint64_t deadlineAtUs = 0;    ///< 0 = no deadline
        InvokeResultFn callback;
    };
    /// request_id → pending call.  Written by callPeer() on application
    /// threads and by the worker in handleInvokeExResp/sweeps.
    std::map<uint64_t, PendingInvoke> pendingInvokes_;
    mutable std::mutex invokeMutex_;
    uint64_t nextInvokeRequestId_ = 1;
    /// Deadline enforcement thread + wakeup for newly inserted pendings.
    std::thread invokeSweepThread_;
    std::condition_variable invokeSweepCv_;
    std::atomic<bool> sweepStop_{false};

    // ==== Ring-buffered streaming ====
    /// Sources registered by the server; consulted in bindRingSource().
    const std::vector<IRingStreamSource*>* ringSources_ = nullptr;
    const SchemaCatalog* schemaCatalog_ = nullptr;
    /// Timestamp (getTimestampUs_) of the last ring drain that produced rows.
    /// Used to flush a partial chunk when the producer goes quiet.
    uint64_t lastRingRowUs_ = 0;
    /// Bound source (owned elsewhere) while a ring-backed stream is active.
    IRingStreamSource* ringSource_ = nullptr;
    /// Per collectPlan_ slot: byte offset of the field inside a schema row.
    std::vector<uint32_t> ringFieldOffsets_;
    /// Scratch for drained schema rows (sized to chunkSize_ * src rowSize).
    std::vector<uint8_t> ringScratch_;

    // ==== OnChange trigger state ====
    std::vector<uint8_t> lastTriggerValue_;

    // ==== Catalog change listener ====
    size_t catalogListenerHandle_ = 0;
    std::atomic<bool> catalogDirty_{false};

    // ==== Session identity ====
    std::string sessionId_;
    machine::SessionIdentity identity_;
    mutable std::mutex identityMutex_;

    // ==== V6 schema negotiation state ====
    bool schemaHelloReceived_ = false;
    bool schemaCommitted_ = false;
    SchemaEpoch schemaEpoch_ = 0;

    // ==== Log subscriptions ====
    std::vector<LogSubscription> logSubscriptions_;
    uint32_t nextLogSubscriptionId_ = 1;
    mutable std::mutex sendMutex_;

    // ==== SLIP receive buffers ====
    std::unique_ptr<IReceiveBuffer> slipRxBuf_;
    size_t  slipRxPos_ = 0;
    bool    slipDiscardUntilEnd_ = false;

    std::unique_ptr<IReceiveBuffer> decodeBuf_;

    // ==== Message-mode receive buffer (Framing::None) ====
    std::vector<uint8_t> rxMsgBuf_;

    // ==== TX buffers ====
    std::vector<uint8_t> txRawBuf_;
    std::vector<uint8_t> txEncBuf_;
};

}} // namespace tether::io
