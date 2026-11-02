#pragma once

/**
 * @file IoConfig.hpp
 * @brief Compile-time feature enables for the Tether IO protocol.
 *
 * The IO protocol is sliced into independent capabilities that are enabled
 * by default and opted out of at compile time. A minimal build — e.g. an MCU
 * peer that only implements parameters and signals — turns off the flags it
 * does not need and the corresponding code is compiled out of the library.
 * There is no negotiated "profile": a peer's wire behaviour is simply the
 * subset its build implements, and requests for disabled features are
 * answered with Error(FeatureNotSupported).
 *
 * Every flag is wrapped in #ifndef so a compile definition
 * (-DTETHER_IO_ENABLE_X=0) or a CMake/Kconfig-generated TetherConfig.hpp
 * always wins over the default here. Flags are always defined to 0 or 1 —
 * test them with '#if TETHER_IO_ENABLE_X', never '#ifdef'.
 *
 * Dependency rules (enforced by the TETHER_IO_*_ENABLED effective macros
 * below — the raw TETHER_IO_ENABLE_* user flags are never tested directly):
 *  - DYNAMIC_SCHEMA requires SCHEMA (it restricts the schema codec).
 *  - FUNCTIONS and INPUT_STREAMS require SCHEMA (function parameters and
 *    input-stream values are schema-typed on the wire).
 *  - STREAM_FILTERS requires SCHEMA and STREAMING.
 *  - THRESHOLDS requires STREAMING (it only affects stream rows).
 */

// Pull in the generated top-level feature flags when the build produced one.
#if __has_include("tether/TetherConfig.hpp")
#include "tether/TetherConfig.hpp"
#endif

// ============================================================================
// Value / schema model
// ============================================================================

/**
 * Schema machinery: catalog, manifest, slots, schema definitions, digest
 * computation (BLAKE3) and schema-value validation.
 *
 * Disable for peers whose entries are all simple flat ValueType values —
 * the ServerHello then advertises an empty manifest and no schema
 * definitions are ever served or requested.
 */
#ifndef TETHER_IO_ENABLE_SCHEMA
#define TETHER_IO_ENABLE_SCHEMA 1
#endif

/**
 * Variable-layout schema kinds in the value codec (String, Bytes,
 * DynamicArray, Map, OneOf, tagged structs, varint scalars).
 *
 * Disable to keep only fixed-layout ("flat") schemata — the kinds
 * SchemaLayout reports as fixedLayout/memcpyImage: fixed scalars,
 * FixedString, FixedArray<fixed>, Optional<fixed> and packed structs of
 * those. Their wire image is a packed little-endian C struct, so a reduced
 * peer can encode/decode by memcpy. Requires TETHER_IO_ENABLE_SCHEMA.
 */
#ifndef TETHER_IO_ENABLE_DYNAMIC_SCHEMA
#define TETHER_IO_ENABLE_DYNAMIC_SCHEMA 1
#endif

// ============================================================================
// Catalog / value features
// ============================================================================

/**
 * Periodic and change-triggered streaming: ConfigureStream, StartStream,
 * StopStream, StreamData and the collect plan / ring source machinery.
 */
#ifndef TETHER_IO_ENABLE_STREAMING
#define TETHER_IO_ENABLE_STREAMING 1
#endif

/**
 * Schema-typed stream filter entries inside ConfigureStreamReq.
 * Requires TETHER_IO_ENABLE_SCHEMA and TETHER_IO_ENABLE_STREAMING.
 */
#ifndef TETHER_IO_ENABLE_STREAM_FILTERS
#define TETHER_IO_ENABLE_STREAM_FILTERS 1
#endif

/**
 * Per-entry change thresholds (ConfigureThresholdReq) used to suppress
 * insignificant changes in streamed rows. Requires TETHER_IO_ENABLE_STREAMING.
 */
#ifndef TETHER_IO_ENABLE_THRESHOLDS
#define TETHER_IO_ENABLE_THRESHOLDS 1
#endif

/**
 * Atomic multi-entry reads: SnapshotParamsReq / SnapshotSignalsReq.
 */
#ifndef TETHER_IO_ENABLE_SNAPSHOTS
#define TETHER_IO_ENABLE_SNAPSHOTS 1
#endif

/**
 * Typed function invocation: function registry, ListFunctions,
 * CallFunction, InvokeEx (both directions), RegisterFunctions peer catalog,
 * and the per-session invoke-deadline sweep thread.
 * Requires TETHER_IO_ENABLE_SCHEMA (parameters are schema-typed).
 */
#ifndef TETHER_IO_ENABLE_FUNCTIONS
#define TETHER_IO_ENABLE_FUNCTIONS 1
#endif

/**
 * Client-to-server input streams: CreateInputStream / InputStreamData /
 * CloseInputStream. Requires TETHER_IO_ENABLE_SCHEMA.
 */
#ifndef TETHER_IO_ENABLE_INPUT_STREAMS
#define TETHER_IO_ENABLE_INPUT_STREAMS 1
#endif

/**
 * Binary datalogging: ConfigureDatalog / DatalogStatus and DatalogRecorder.
 */
#ifndef TETHER_IO_ENABLE_DATALOG
#define TETHER_IO_ENABLE_DATALOG 1
#endif

/**
 * Server-side log record subscriptions: SubscribeLog / UnsubscribeLog /
 * LogData and Session::publishLog / Server::publishLog.
 */
#ifndef TETHER_IO_ENABLE_LOG_SUBSCRIPTIONS
#define TETHER_IO_ENABLE_LOG_SUBSCRIPTIONS 1
#endif

/**
 * Entry metadata: GetMetadataReq/Resp and the per-entry metadata map.
 * Disabling removes the std::map member from every ParamEntry/SignalEntry.
 */
#ifndef TETHER_IO_ENABLE_METADATA
#define TETHER_IO_ENABLE_METADATA 1
#endif

/**
 * Catalog-change push: the registry listener list and CatalogChanged.
 */
#ifndef TETHER_IO_ENABLE_CATALOG_NOTIFY
#define TETHER_IO_ENABLE_CATALOG_NOTIFY 1
#endif

// ============================================================================
// Roles and transports
// ============================================================================

/** Server role: Session and Server (the accepting, per-client machinery). */
#ifndef TETHER_IO_ENABLE_SERVER
#define TETHER_IO_ENABLE_SERVER 1
#endif

/** Client role: TetherIOClient. */
#ifndef TETHER_IO_ENABLE_CLIENT
#define TETHER_IO_ENABLE_CLIENT 1
#endif

/** POSIX TCP transport. */
#ifndef TETHER_IO_TRANSPORT_TCP
#define TETHER_IO_TRANSPORT_TCP 1
#endif

/** Serial transport with pluggable ISerialDriver. */
#ifndef TETHER_IO_TRANSPORT_SERIAL
#define TETHER_IO_TRANSPORT_SERIAL 1
#endif

/** WebSocket transport. */
#ifndef TETHER_IO_TRANSPORT_WS
#define TETHER_IO_TRANSPORT_WS 1
#endif

// ============================================================================
// Sizing knobs (values, not booleans)
// ============================================================================

/** Maximum decoded protocol message size in bytes. */
#ifndef TETHER_IO_MAX_MESSAGE_SIZE
#define TETHER_IO_MAX_MESSAGE_SIZE (1024 * 1024)
#endif

/** Maximum entries/filters accepted in a single ConfigureStream request. */
#ifndef TETHER_IO_MAX_STREAM_ENTRIES
#define TETHER_IO_MAX_STREAM_ENTRIES 65536
#endif

/** Maximum outstanding InvokeEx calls per session. */
#ifndef TETHER_IO_MAX_PENDING_INVOKES
#define TETHER_IO_MAX_PENDING_INVOKES 256
#endif

/** Maximum log subscriptions per session. */
#ifndef TETHER_IO_MAX_LOG_SUBSCRIPTIONS
#define TETHER_IO_MAX_LOG_SUBSCRIPTIONS 16
#endif

// ============================================================================
// Effective capabilities (test these, never the raw ENABLE_* flags)
// ============================================================================

#define TETHER_IO_SCHEMA_ENABLED          (TETHER_IO_ENABLE_SCHEMA)
#define TETHER_IO_DYNAMIC_SCHEMA_ENABLED  (TETHER_IO_ENABLE_SCHEMA && \
                                           TETHER_IO_ENABLE_DYNAMIC_SCHEMA)
#define TETHER_IO_FUNCTIONS_ENABLED       (TETHER_IO_ENABLE_SCHEMA && \
                                           TETHER_IO_ENABLE_FUNCTIONS)
#define TETHER_IO_INPUT_STREAMS_ENABLED   (TETHER_IO_ENABLE_SCHEMA && \
                                           TETHER_IO_ENABLE_INPUT_STREAMS)
#define TETHER_IO_STREAMING_ENABLED       (TETHER_IO_ENABLE_STREAMING)
#define TETHER_IO_STREAM_FILTERS_ENABLED  (TETHER_IO_ENABLE_SCHEMA && \
                                           TETHER_IO_ENABLE_STREAMING && \
                                           TETHER_IO_ENABLE_STREAM_FILTERS)
#define TETHER_IO_THRESHOLDS_ENABLED      (TETHER_IO_ENABLE_STREAMING && \
                                           TETHER_IO_ENABLE_THRESHOLDS)
#define TETHER_IO_SNAPSHOTS_ENABLED       (TETHER_IO_ENABLE_SNAPSHOTS)
#define TETHER_IO_DATALOG_ENABLED         (TETHER_IO_ENABLE_DATALOG)
#define TETHER_IO_LOG_SUBSCRIPTIONS_ENABLED (TETHER_IO_ENABLE_LOG_SUBSCRIPTIONS)
#define TETHER_IO_METADATA_ENABLED        (TETHER_IO_ENABLE_METADATA)
#define TETHER_IO_CATALOG_NOTIFY_ENABLED  (TETHER_IO_ENABLE_CATALOG_NOTIFY)
#define TETHER_IO_SERVER_ENABLED          (TETHER_IO_ENABLE_SERVER)
#define TETHER_IO_CLIENT_ENABLED          (TETHER_IO_ENABLE_CLIENT)
