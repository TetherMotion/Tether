/**
 * @file EtherCATSdoConfiguration.hpp
 * @brief EtherCAT config: SDO CONFIGURATION
 *
 * Split out of EtherCATConfig.hpp.
 */

#pragma once

// Pull in the generated top-level feature flags (TETHER_ENABLE_*).
#if __has_include("tether/TetherConfig.hpp")
#include "tether/TetherConfig.hpp"
#endif

// ============================================================================
// SDO CONFIGURATION
// ============================================================================

/**
 * @brief Raw SDO mailbox buffer safety ceiling (bytes)
 *
 * Safety ceiling for the dynamically-allocated buffer used by the raw SDO
 * upload/download layer for mailbox communication. The actual buffer is
 * allocated per-call to match the slave's reported mailbox size
 * (max of SM0/SM1 length); this constant caps that allocation to guard
 * against corrupted or malicious slaves reporting absurd mailbox sizes.
 *
 * The ESC211 FNI objects are 256-byte OctetString sections; a 256-byte
 * SDO download needs 272 bytes on the wire (6 mbx + 2 CoE + 8 SDO +
 * 256 data), so a 256-byte mailbox forces segmented transfers that the
 * ESC211 rejects. The manufacturer ENI uses 512-byte mailboxes. Some
 * slaves use 1024-byte mailboxes.
 *
 * @note If a slave's mailbox is larger than this ceiling, Tether will
 *       reject the transfer with an error that explicitly names Tether's
 *       internal ceiling as the limiting factor (not the slave).
 *
 * Memory impact: up to this value per in-flight SDO call (heap-allocated)
 * Default: 16384
 */
#ifndef ECAT_RAW_SDO_MBX_BUFFER_SIZE
#define ECAT_RAW_SDO_MBX_BUFFER_SIZE   16384
#endif

/**
 * @brief SDO request queue depth
 * 
 * Number of SDO requests that can be queued for background processing.
 * 
 * @note Recommendation: 8-16 for typical applications, higher for scripted config
 * 
 * Memory impact: ~64 bytes per queue slot
 * Default: 16
 */
#ifndef ECAT_SDO_QUEUE_DEPTH
#define ECAT_SDO_QUEUE_DEPTH            16
#endif

/**
 * @brief Maximum SDO data size (bytes)
 * 
 * Maximum size for a single SDO transfer. Segmented transfers handle larger data.
 * 
 * @note Recommendation: 256 covers most objects. Increase for large string objects.
 * 
 * Default: 256
 */
#ifndef ECAT_SDO_MAX_DATA_SIZE
#define ECAT_SDO_MAX_DATA_SIZE          256
#endif

/**
 * @brief SDO timeout (milliseconds)
 * 
 * Timeout for waiting for SDO response from slave.
 * 
 * @note Recommendation: 1000ms for reliable operation, reduce for faster error detection
 * 
 * Default: 1000
 */
#ifndef ECAT_SDO_TIMEOUT_MS
#define ECAT_SDO_TIMEOUT_MS             1000
#endif

/**
 * @brief SDO retry count
 * 
 * Number of retries on SDO communication failure before giving up.
 * 
 * Default: 3
 */
#ifndef ECAT_SDO_RETRY_COUNT
#define ECAT_SDO_RETRY_COUNT            3
#endif

/**
 * @brief Maximum number of segments for a segmented SDO upload
 * 
 * Limits how many segment requests are issued for a single SDO upload.
 * Acts as a safety bound against a slave that never marks the last segment.
 * 
 * Default: 200
 */
#ifndef ECAT_SDO_UPLOAD_MAX_SEGMENTS
#define ECAT_SDO_UPLOAD_MAX_SEGMENTS    200
#endif

/**
 * @brief Maximum number of segments for a segmented SDO download
 * 
 * Limits how many segment requests are issued for a single SDO download.
 * Acts as a safety bound against a slave that never marks the last segment.
 * 
 * Default: 200
 */
#ifndef ECAT_SDO_DOWNLOAD_MAX_SEGMENTS
#define ECAT_SDO_DOWNLOAD_MAX_SEGMENTS  200
#endif

/**
 * @brief Maximum number of stale mailbox response retries
 *
 * When a slave returns a stale mailbox response (wrong counter, wrong
 * index, or wrong subindex), Tether clears the mailbox and re-sends the
 * request. This limits the number of such retries before giving up.
 * This is a Tether-internal limit.
 *
 * Default: 8
 */
#ifndef ECAT_SDO_MAX_STALE_RETRIES
#define ECAT_SDO_MAX_STALE_RETRIES      8
#endif

/**
 * @brief Maximum number of mailbox poll attempts per SDO phase
 *
 * Each SDO phase (init upload, segment request, etc.) polls the SM1
 * status register this many times before timing out. Combined with
 * the poll interval, this determines the per-phase timeout.
 * This is a Tether-internal limit.
 *
 * Default: 50
 */
#ifndef ECAT_SDO_MAX_POLL_ATTEMPTS
#define ECAT_SDO_MAX_POLL_ATTEMPTS      50
#endif

/**
 * @brief CoE manager queue depth (per slave)
 *
 * Maximum number of pending CoE read/write requests per slave in the
 * CoEManager's async queue. This is a Tether-internal limit — if the
 * queue is full, new requests are rejected with CoEErrorCode::QueueFull.
 * Note: this is separate from ECAT_SDO_QUEUE_DEPTH which controls the
 * SDOManager queue.
 *
 * Default: 32
 */
#ifndef ECAT_COE_QUEUE_DEPTH
#define ECAT_COE_QUEUE_DEPTH            32
#endif

/**
 * @brief CoE default timeout (milliseconds)
 *
 * Default timeout for CoE read/write operations when not explicitly
 * specified by the caller.
 *
 * Default: 1000
 */
#ifndef ECAT_COE_DEFAULT_TIMEOUT_MS
#define ECAT_COE_DEFAULT_TIMEOUT_MS     1000
#endif

/**
 * @brief CoE default poll interval (milliseconds)
 *
 * Default interval for polling the slave mailbox status register
 * during CoE operations.
 *
 * Default: 5
 */
#ifndef ECAT_COE_DEFAULT_POLL_INTERVAL_MS
#define ECAT_COE_DEFAULT_POLL_INTERVAL_MS 5
#endif

/**
 * @brief SDO manager maximum data size (bytes)
 *
 * Maximum data size for a single SDO transfer in the SDOManager.
 * Larger transfers use segmented transfer automatically.
 * Note: this is the SDOManager-level limit, separate from
 * ECAT_SDO_MAX_DATA_SIZE which is the raw SDO layer limit.
 *
 * Default: 256
 */
#ifndef ECAT_SDO_MANAGER_MAX_DATA_SIZE
#define ECAT_SDO_MANAGER_MAX_DATA_SIZE  256
#endif

/**
 * @brief SDO manager queue depth
 *
 * Maximum number of pending SDO requests in the SDOManager queue.
 * Note: this is the SDOManager-level limit, separate from
 * ECAT_SDO_QUEUE_DEPTH which is the general SDO queue limit.
 *
 * Default: 16
 */
#ifndef ECAT_SDO_MANAGER_QUEUE_DEPTH
#define ECAT_SDO_MANAGER_QUEUE_DEPTH    16
#endif

/**
 * @brief SDO manager default timeout (milliseconds)
 *
 * Default timeout for SDO operations in the SDOManager.
 *
 * Default: 1000
 */
#ifndef ECAT_SDO_MANAGER_DEFAULT_TIMEOUT_MS
#define ECAT_SDO_MANAGER_DEFAULT_TIMEOUT_MS 1000
#endif

/**
 * @brief Write-verify maximum data length (bytes)
 *
 * Maximum data length for a single write-verify operation in the
 * WriteVerifier class. This is a Tether-internal stack buffer limit.
 * If a write exceeds this size, Tether will reject it with an error
 * that explicitly names Tether's internal buffer as the limiting factor.
 *
 * @note Increase if you need to verify writes larger than the default.
 *
 * Memory impact: 1x this value per WriteVerifier instance (stack)
 * Default: 256
 */
#ifndef ECAT_WRITE_VERIFY_MAX_DATA_LEN
#define ECAT_WRITE_VERIFY_MAX_DATA_LEN  256
#endif

/**
 * @brief Retry verify buffer size (bytes)
 *
 * Size of the stack-allocated read-back buffer in the retry layer's
 * APWR/FPWR verify path. This is a Tether-internal limit. If a write
 * exceeds this size, Tether will reject the verify with an error that
 * explicitly names Tether's internal buffer as the limiting factor.
 *
 * @note Most register writes are small (< 64 bytes). Increase only if
 *       you need to verify larger register writes.
 *
 * Memory impact: 1x this value per retry-verify call (stack)
 * Default: 64
 */
#ifndef ECAT_RETRY_VERIFY_BUFFER_SIZE
#define ECAT_RETRY_VERIFY_BUFFER_SIZE   64
#endif
