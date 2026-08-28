/**
 * @file TetherIOClient.hpp
 * @brief C++ client for the Tether IO binary protocol (Framing::None).
 *
 * Provides typed request/response methods for all Tether IO protocol
 * functions. Works with any ITransport that supports receiveMessage()
 * (e.g. WebSocket, MessagePipeTransport).
 *
 * Usage:
 *   auto [client, server] = MessagePipeTransport::create();
 *   TetherIOClient ioClient(std::move(client));
 *   auto params = ioClient.listParams(0, 1000);
 *   ioClient.setParam(1, someValue);
 *   ioClient.configureStream({10, 11}, 10, 1);
 *   ioClient.startStream();
 *   // ... receive StreamData via stream callback ...
 *   ioClient.stopStream();
 */
#pragma once

#include "tether/io/Protocol.hpp"
#include "tether/io/Transport.hpp"
#include "tether/io/Registry.hpp"
#include "tether/io/Function.hpp"

#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <expected>
#include <variant>

namespace tether { namespace io {

/// Catalog entry returned by listParams / listSignals.
struct ClientCatalogEntry {
    uint64_t id = 0;
    ValueType type = ValueType::F64;
    uint8_t valueSize = 0;
    uint8_t flags = 0;
    std::string name;
    std::string description;
    std::string group;
};

/// Function parameter descriptor.
struct ClientFunctionParam {
    std::string name;
    std::string description;
    ValueType type = ValueType::F64;
    uint8_t flags = 0;
    uint32_t maxValueSize = 0;
    bool hasDefault = false;
    std::vector<uint8_t> defaultValue;
};

/// Function entry returned by listFunctions.
struct ClientFunctionEntry {
    uint64_t id = 0;
    std::string name;
    std::string description;
    std::string group;
    std::vector<ClientFunctionParam> parameters;
    bool hasReturnValue = false;
    ValueType returnType = ValueType::F64;
};

/// Stream layout entry from ConfigureStreamAck.
struct ClientStreamLayoutEntry {
    uint64_t id = 0;
    ValueType type = ValueType::F64;
    uint8_t valueSize = 0;
};

/// A single stream data row.
struct ClientStreamRow {
    uint32_t specId = 0;
    uint64_t timestampUs = 0;
    std::vector<std::vector<uint8_t>> values;
};

/// Metadata key-value pair.
struct ClientMetadata {
    std::string key;
    std::string value;
};

/// Snapshot value entry.
struct ClientSnapshotValue {
    uint64_t id = 0;
    uint8_t valueSize = 0;
    std::vector<uint8_t> value;
};

/// Feature entry from feature exchange.
struct ClientFeature {
    std::string name;
    uint8_t type = 0;
    std::vector<uint8_t> value;
};

/// Error from the server or client.
struct ClientError {
    ErrorCode code = ErrorCode::None;
    std::string message;
};

/// Result of a CallFunction request.
struct ClientCallResult {
    uint64_t functionId = 0;
    bool success = false;
    uint32_t errorCode = 0;
    std::string errorMessage;
    bool hasReturnValue = false;
    ValueType returnType = ValueType::F64;
    std::vector<uint8_t> returnValue;
};

/// Result of a SubscribeLog request.
struct ClientLogSubscription {
    uint32_t subscriptionId = 0;
    bool failed = false;
    std::string errorMessage;
};

/// Log record received via LogData.
struct ClientLogRecord {
    uint8_t severity = 0;
    std::string component;
    std::string message;
    std::string location;
};

/// Stream data callback.
using StreamDataCallback = std::function<void(const ClientStreamRow&)>;
/// Log data callback.
using LogDataCallback = std::function<void(const ClientLogRecord&)>;

/**
 * @brief Synchronous C++ client for the Tether IO binary protocol.
 *
 * Each request method sends a message and waits for the response with
 * a configurable timeout. StreamData and LogData messages are dispatched
 * to registered callbacks asynchronously (during request waits).
 */
class TetherIOClient {
public:
    /// Construct with a transport (takes ownership).
    explicit TetherIOClient(std::unique_ptr<ITransport> transport,
                            uint32_t defaultTimeoutMs = 5000);
    ~TetherIOClient();

    TetherIOClient(const TetherIOClient&) = delete;
    TetherIOClient& operator=(const TetherIOClient&) = delete;

    // ---- Connection management ----

    bool isConnected() const;
    void close();

    // ---- Callbacks ----

    void setStreamDataCallback(StreamDataCallback cb);
    void setLogDataCallback(LogDataCallback cb);

    // ---- Ping ----

    /// Send PingReq with a nonce, expect PongResp. Returns the echoed nonce.
    std::expected<uint32_t, ClientError> ping(uint32_t nonce = 0);

    // ---- Catalog: Parameters ----

    /// List parameters starting at offset, up to maxCount entries.
    std::expected<std::vector<ClientCatalogEntry>, ClientError>
    listParams(uint32_t offset = 0, uint32_t maxCount = 1000);

    /// Get a parameter's current value.
    std::expected<std::vector<uint8_t>, ClientError>
    getParam(uint64_t id);

    /// Set a parameter's value (fixed-length).
    std::expected<void, ClientError>
    setParam(uint64_t id, const void* value, uint8_t valueSize);

    /// Set a parameter's value (variable-length).
    std::expected<void, ClientError>
    setParamVar(uint64_t id, const void* value, size_t len);

    // ---- Catalog: Signals ----

    /// List signals starting at offset, up to maxCount entries.
    std::expected<std::vector<ClientCatalogEntry>, ClientError>
    listSignals(uint32_t offset = 0, uint32_t maxCount = 1000);

    /// Get a signal's current value.
    std::expected<std::vector<uint8_t>, ClientError>
    getSignal(uint64_t id);

    // ---- Metadata ----

    /// Get metadata key-value pairs for an entry.
    std::expected<std::vector<ClientMetadata>, ClientError>
    getMetadata(uint64_t id);

    // ---- Functions ----

    /// List functions starting at offset, up to maxCount entries.
    std::expected<std::vector<ClientFunctionEntry>, ClientError>
    listFunctions(uint32_t offset = 0, uint32_t maxCount = 1000);

    /// Call a function by ID with typed arguments.
    /// Each argument is {position, type, valueBytes}.
    struct FunctionArg {
        uint32_t position = 0;
        ValueType type = ValueType::F64;
        std::vector<uint8_t> value;
    };
    std::expected<ClientCallResult, ClientError>
    callFunction(uint64_t functionId, const std::vector<FunctionArg>& args = {});

    // ---- Streaming ----

    /// Configure a stream. Returns {specId, layout}.
    struct ConfigureStreamResult {
        uint32_t specId = 0;
        std::vector<ClientStreamLayoutEntry> layout;
    };
    std::expected<ConfigureStreamResult, ClientError>
    configureStream(const std::vector<uint64_t>& entryIds,
                    uint32_t intervalMs = 100,
                    uint32_t chunkSize = 1,
                    uint32_t skipCount = 0,
                    uint8_t triggerMode = 0,
                    uint64_t triggerEntryId = 0);

    /// Start streaming (no response expected).
    std::expected<void, ClientError> startStream();

    /// Stop streaming.
    std::expected<void, ClientError> stopStream();

    // ---- Snapshots ----

    /// Snapshot all or specific parameters.
    /// Empty ids = snapshot all.
    std::expected<std::pair<uint64_t, std::vector<ClientSnapshotValue>>, ClientError>
    snapshotParams(const std::vector<uint64_t>& ids = {});

    /// Snapshot all or specific signals.
    std::expected<std::pair<uint64_t, std::vector<ClientSnapshotValue>>, ClientError>
    snapshotSignals(const std::vector<uint64_t>& ids = {});

    // ---- Feature Exchange ----

    std::expected<std::vector<ClientFeature>, ClientError>
    featureExchange(const std::vector<ClientFeature>& clientFeatures = {});

    // ---- Log Subscription ----

    std::expected<ClientLogSubscription, ClientError>
    subscribeLog(uint8_t minSeverity,
                 const std::string& componentFilter = {},
                 const std::string& messageFilter = {},
                 const std::string& locationFilter = {});

    std::expected<uint32_t, ClientError>
    unsubscribeLog(uint32_t subscriptionId);

    // ---- Datalog ----

    std::expected<bool, ClientError>
    configureDatalog(const std::string& logName,
                     uint32_t sampleRateHz,
                     bool enabled,
                     const std::vector<uint64_t>& entryIds = {});

    // ---- Threshold ----

    std::expected<bool, ClientError>
    configureThreshold(const std::string& name,
                       bool isWhitelist,
                       const std::vector<uint8_t>& encodedRules);

    // ---- Describe Struct ----

    struct StructField {
        std::string name;
        ValueType type = ValueType::F64;
        uint16_t offset = 0;
        uint16_t size = 0;
        std::string unit;
    };
    struct StructDescriptor {
        uint64_t entryId = 0;
        std::string structName;
        uint32_t totalSize = 0;
        std::vector<StructField> fields;
    };
    std::expected<StructDescriptor, ClientError>
    describeStruct(uint64_t id);

    // ---- Input Streams ----

    struct CreateInputStreamResult {
        uint32_t streamId = 0;
        bool success = false;
    };
    std::expected<CreateInputStreamResult, ClientError>
    createInputStream(uint32_t maxValueSize,
                      uint32_t maxBatchSize,
                      const std::vector<uint8_t>& encodedValueDescriptor);

    std::expected<void, ClientError>
    inputStreamData(uint32_t streamId,
                    const std::vector<std::vector<uint8_t>>& values);

    std::expected<std::pair<uint32_t, bool>, ClientError>
    closeInputStream(uint32_t streamId);

    // ---- Low-level access ----

    /// Send a raw protocol message (no framing).
    bool sendRaw(const uint8_t* data, size_t len);

    /// Receive the next message, dispatching StreamData/LogData to callbacks.
    /// Returns the message type and payload, or an error.
    std::expected<std::vector<uint8_t>, ClientError>
    receiveMessage(uint32_t timeoutMs = 0);

    /// Send a request and wait for a specific response type.
    /// StreamData and LogData messages are dispatched to callbacks while waiting.
    std::expected<std::vector<uint8_t>, ClientError>
    request(const std::vector<uint8_t>& payload,
            MessageType expectedResponseType,
            uint32_t timeoutMs = 0);

private:
    std::unique_ptr<ITransport> transport_;
    uint32_t defaultTimeoutMs_;
    StreamDataCallback streamCallback_;
    LogDataCallback logCallback_;
    std::vector<ClientStreamLayoutEntry> streamLayout_;
    std::mutex mutex_;

    // Internal helpers
    ClientError makeError(ErrorCode code, std::string msg);
    ClientError parseError(const std::vector<uint8_t>& frame);
    std::vector<uint8_t> encodeVarint(uint32_t value);
    uint32_t decodeVarint(const uint8_t* data, size_t len, size_t& consumed);
    std::string readString16(const uint8_t* data, size_t len, size_t& offset);
};

}} // namespace tether::io
