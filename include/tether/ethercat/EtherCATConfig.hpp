/**
 * @file EtherCATConfig.hpp
 * @brief Centralized configuration for EtherCAT stack features
 * 
 * @details
 * This file contains all user-configurable options for the EtherCAT stack.
 * Modify these settings to enable/disable features and tune performance
 * for your specific application requirements.
 * 
 * ## Configuration Categories
 * 
 * 1. **Feature Enables** - Turn protocols on/off to save code space
 * 2. **Buffer Sizes** - Tune memory usage vs. capability trade-offs
 * 3. **Timing Parameters** - Adjust timeouts and retry behavior
 * 4. **Platform Selection** - Choose platform-specific implementations
 * 
 * ## Memory Impact Summary
 * 
 * | Feature | Code Size | RAM Usage | Recommendation |
 * |---------|-----------|-----------|----------------|
 * | PDO     | ~4 KB     | ~2 KB     | Always enable for realtime |
 * | SDO     | ~6 KB     | ~1 KB     | Enable for configuration |
 * | FoE     | ~8 KB     | ~4 KB     | Enable for firmware updates |
 * | VoE     | ~3 KB     | ~1 KB     | Enable for vendor features |
 * | EoE     | ~10 KB    | ~8 KB     | Enable for IP networking |
 * | DC      | ~5 KB     | ~0.5 KB   | Enable for synchronized motion |
 * 
 * ## Quick Start Profiles
 * 
 * ### Minimal (Motion Control Only)
 * @code
 * #define ECAT_FEATURE_PDO_ENABLED    1
 * #define ECAT_FEATURE_SDO_ENABLED    1
 * #define ECAT_FEATURE_DC_ENABLED     1
 * #define ECAT_FEATURE_FOE_ENABLED    0
 * #define ECAT_FEATURE_VOE_ENABLED    0
 * #define ECAT_FEATURE_EOE_ENABLED    0
 * @endcode
 * 
 * ### Full Featured
 * @code
 * #define ECAT_FEATURE_PDO_ENABLED    1
 * #define ECAT_FEATURE_SDO_ENABLED    1
 * #define ECAT_FEATURE_DC_ENABLED     1
 * #define ECAT_FEATURE_FOE_ENABLED    1
 * #define ECAT_FEATURE_VOE_ENABLED    1
 * #define ECAT_FEATURE_EOE_ENABLED    1
 * @endcode
 * 
 * ### Firmware Update Station
 * @code
 * #define ECAT_FEATURE_PDO_ENABLED    0
 * #define ECAT_FEATURE_SDO_ENABLED    1
 * #define ECAT_FEATURE_DC_ENABLED     0
 * #define ECAT_FEATURE_FOE_ENABLED    1
 * #define ECAT_FEATURE_VOE_ENABLED    0
 * #define ECAT_FEATURE_EOE_ENABLED    0
 * @endcode
 */

#pragma once

// ============================================================================
// STATISTICS (compile-time switch — controllable via CMake)
// ============================================================================
// Set via CMake option TETHER_ENABLE_ETHERCAT_STATS (ON/OFF).
// When disabled, all statistics collection is compiled out.

#ifndef TETHER_ENABLE_ETHERCAT_STATS
#define TETHER_ENABLE_ETHERCAT_STATS 1
#endif

// ============================================================================
// FEATURE ENABLES
// ============================================================================
// Set to 1 to enable, 0 to disable. Disabled features are compiled out
// completely, saving both code space and RAM.

/**
 * @brief Enable Process Data Object (PDO) support
 * 
 * PDOs are the primary mechanism for realtime cyclic data exchange.
 * Required for most motion control and I/O applications.
 * 
 * @note Recommendation: Enable unless you only need configuration/diagnostics
 * 
 * Dependencies: None
 * Code size: ~4 KB
 * RAM usage: ~2 KB (scales with number of slaves and PDO sizes)
 */
#ifndef ECAT_FEATURE_PDO_ENABLED
#define ECAT_FEATURE_PDO_ENABLED    1
#endif

/**
 * @brief Enable Service Data Object (SDO) support
 * 
 * SDOs provide non-realtime access to the slave's object dictionary.
 * Used for configuration, diagnostics, and parameter access.
 * 
 * @note Recommendation: Enable for most applications
 * 
 * Dependencies: Mailbox protocol (SM0/SM1)
 * Code size: ~6 KB
 * RAM usage: ~1 KB (queue + buffers)
 */
#ifndef ECAT_FEATURE_SDO_ENABLED
#define ECAT_FEATURE_SDO_ENABLED    1
#endif

/**
 * @brief Enable Distributed Clocks (DC) support
 * 
 * DC provides synchronized timing across all slaves in the network.
 * Essential for coordinated multi-axis motion control.
 * 
 * @note Recommendation: Enable for motion control, disable for simple I/O
 * 
 * Dependencies: Hardware timer (GPTimer on ESP32)
 * Code size: ~5 KB
 * RAM usage: ~0.5 KB
 */
#ifndef ECAT_FEATURE_DC_ENABLED
#define ECAT_FEATURE_DC_ENABLED     1
#endif

/**
 * @brief Enable File over EtherCAT (FoE) support
 * 
 * FoE allows file transfers to/from slaves. Primary uses:
 * - Firmware updates (bootloader mode)
 * - Configuration file upload/download
 * - Log file retrieval
 * 
 * @note Recommendation: Enable if slaves support firmware updates
 * 
 * Dependencies: SDO (mailbox), Platform filesystem abstraction
 * Code size: ~8 KB
 * RAM usage: ~4 KB (file buffer + state)
 */
#ifndef ECAT_FEATURE_FOE_ENABLED
#define ECAT_FEATURE_FOE_ENABLED    1
#endif

/**
 * @brief Enable Vendor-specific over EtherCAT (VoE) support
 * 
 * VoE provides a vendor-specific mailbox channel for proprietary
 * features not covered by standard protocols. Uses include:
 * - Proprietary diagnostic protocols
 * - Custom firmware features
 * - Vendor-specific configuration
 * 
 * @note Recommendation: Enable only if your slaves use VoE
 * 
 * Dependencies: SDO (mailbox)
 * Code size: ~3 KB
 * RAM usage: ~1 KB
 */
#ifndef ECAT_FEATURE_VOE_ENABLED
#define ECAT_FEATURE_VOE_ENABLED    1
#endif

/**
 * @brief Enable Ethernet over EtherCAT (EoE) support
 * 
 * EoE tunnels standard Ethernet/IP traffic through the EtherCAT network.
 * This allows slaves to have virtual Ethernet ports for:
 * - Remote configuration via web interface
 * - TCP/IP diagnostics
 * - Integration with IT networks
 * 
 * @warning EoE adds significant overhead. Only enable if required.
 * 
 * @note Recommendation: Disable for pure motion control applications
 * 
 * Dependencies: SDO (mailbox), Network stack (lwIP on ESP32)
 * Code size: ~10 KB
 * RAM usage: ~8 KB (frame buffers + state)
 */
#ifndef ECAT_FEATURE_EOE_ENABLED
#define ECAT_FEATURE_EOE_ENABLED    1
#endif

// ============================================================================
// PLATFORM SELECTION
// ============================================================================

/**
 * @brief Select the platform for filesystem operations (FoE)
 * 
 * Options:
 * - ECAT_PLATFORM_ESP32_LITTLEFS: ESP32 with LittleFS (recommended for ESP32)
 * - ECAT_PLATFORM_ESP32_SPIFFS: ESP32 with SPIFFS (legacy)
 * - ECAT_PLATFORM_POSIX: Standard POSIX file I/O (Linux testing)
 * - ECAT_PLATFORM_NONE: No filesystem (FoE disabled or custom implementation)
 * 
 * @note Auto-detected based on ESP_PLATFORM if not explicitly set
 */
#define ECAT_PLATFORM_ESP32_LITTLEFS    1
#define ECAT_PLATFORM_ESP32_SPIFFS      2
#define ECAT_PLATFORM_POSIX             3
#define ECAT_PLATFORM_NONE              0

#ifndef ECAT_PLATFORM_FILESYSTEM
#if defined(ESP_PLATFORM)
    #define ECAT_PLATFORM_FILESYSTEM    ECAT_PLATFORM_ESP32_LITTLEFS
#elif defined(__linux__) || defined(__unix__)
    #define ECAT_PLATFORM_FILESYSTEM    ECAT_PLATFORM_POSIX
#else
    #define ECAT_PLATFORM_FILESYSTEM    ECAT_PLATFORM_NONE
#endif
#endif

/**
 * @brief LittleFS partition label (ESP32 only)
 * 
 * The partition label in partitions.csv where LittleFS is mounted.
 * Default: "storage"
 */
#ifndef ECAT_LITTLEFS_PARTITION_LABEL
#define ECAT_LITTLEFS_PARTITION_LABEL   "storage"
#endif

/**
 * @brief LittleFS mount point (ESP32 only)
 * 
 * The VFS mount path for LittleFS.
 * Default: "/littlefs"
 */
#ifndef ECAT_LITTLEFS_MOUNT_POINT
#define ECAT_LITTLEFS_MOUNT_POINT       "/littlefs"
#endif

// ============================================================================
// PDO CONFIGURATION
// ============================================================================

/**
 * @brief Maximum number of slaves for PDO mapping
 * 
 * Limits memory allocation for slave configuration arrays.
 * 
 * @note Recommendation: Set to actual expected slave count + small margin
 * 
 * Memory impact: ~64 bytes per slot
 * Range: 1-247 (EtherCAT limit)
 * Default: 16
 */
#ifndef ECAT_PDO_MAX_SLAVES
#define ECAT_PDO_MAX_SLAVES             16
#endif

/**
 * @brief Maximum PDO entries per slave
 * 
 * Limits the number of PDO objects that can be mapped per slave.
 * 
 * @note Recommendation: Most slaves have 2-8 PDOs. Set higher for complex slaves.
 * 
 * Memory impact: ~32 bytes per entry per slave
 * Default: 16
 */
#ifndef ECAT_PDO_MAX_ENTRIES_PER_SLAVE
#define ECAT_PDO_MAX_ENTRIES_PER_SLAVE  16
#endif

/**
 * @brief Maximum total PDO data size per slave (bytes)
 * 
 * Buffer size for PDO data exchange. Must accommodate largest PDO mapping.
 * 
 * @note Recommendation: Sum of all PDO sizes for your largest slave + padding
 * 
 * Memory impact: 2x this value per slave (double buffering)
 * Default: 256
 */
#ifndef ECAT_PDO_MAX_DATA_SIZE
#define ECAT_PDO_MAX_DATA_SIZE          256
#endif

// ============================================================================
// SDO CONFIGURATION
// ============================================================================

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

// ============================================================================
// FOE CONFIGURATION
// ============================================================================

/**
 * @brief FoE transfer buffer size (bytes)
 * 
 * Size of the buffer used for FoE file transfers. Larger buffers improve
 * throughput but use more RAM.
 * 
 * @note Recommendation: 512-1024 for good balance. Match slave's mailbox size if known.
 * 
 * Memory impact: 2x this value (read + write buffers)
 * Default: 512
 */
#ifndef ECAT_FOE_BUFFER_SIZE
#define ECAT_FOE_BUFFER_SIZE            512
#endif

/**
 * @brief FoE maximum filename length
 * 
 * Maximum length for FoE filenames including null terminator.
 * 
 * Default: 64
 */
#ifndef ECAT_FOE_MAX_FILENAME
#define ECAT_FOE_MAX_FILENAME           64
#endif

/**
 * @brief FoE transfer timeout (milliseconds)
 * 
 * Timeout for each FoE packet exchange. Total transfer timeout is
 * this value multiplied by number of packets.
 * 
 * @note Recommendation: 3000-5000ms for firmware updates (flash write is slow)
 * 
 * Default: 5000
 */
#ifndef ECAT_FOE_TIMEOUT_MS
#define ECAT_FOE_TIMEOUT_MS             5000
#endif

/**
 * @brief FoE password for protected transfers
 * 
 * Default password sent with FoE requests. Many slaves ignore this.
 * 
 * Default: 0 (no password)
 */
#ifndef ECAT_FOE_DEFAULT_PASSWORD
#define ECAT_FOE_DEFAULT_PASSWORD       0
#endif

/**
 * @brief Maximum concurrent FoE transfers
 * 
 * Number of simultaneous FoE transfers supported.
 * 
 * @note Recommendation: 1 is usually sufficient. Increase for parallel updates.
 * 
 * Memory impact: ~1KB per transfer slot
 * Default: 1
 */
#ifndef ECAT_FOE_MAX_TRANSFERS
#define ECAT_FOE_MAX_TRANSFERS          1
#endif

// ============================================================================
// VOE CONFIGURATION
// ============================================================================

/**
 * @brief VoE maximum data size (bytes)
 * 
 * Maximum payload size for vendor-specific mailbox messages.
 * 
 * @note Recommendation: Match your vendor's protocol requirements
 * 
 * Default: 256
 */
#ifndef ECAT_VOE_MAX_DATA_SIZE
#define ECAT_VOE_MAX_DATA_SIZE          256
#endif

/**
 * @brief VoE request queue depth
 * 
 * Number of VoE requests that can be queued.
 * 
 * Default: 8
 */
#ifndef ECAT_VOE_QUEUE_DEPTH
#define ECAT_VOE_QUEUE_DEPTH            8
#endif

/**
 * @brief VoE timeout (milliseconds)
 * 
 * Default: 1000
 */
#ifndef ECAT_VOE_TIMEOUT_MS
#define ECAT_VOE_TIMEOUT_MS             1000
#endif

// ============================================================================
// EOE CONFIGURATION
// ============================================================================

/**
 * @brief EoE frame buffer count
 * 
 * Number of Ethernet frame buffers for EoE. More buffers improve throughput
 * but use more RAM.
 * 
 * @note Recommendation: 4-8 for typical use, increase for high-throughput
 * 
 * Memory impact: 1518 bytes per buffer (max Ethernet frame)
 * Default: 4
 */
#ifndef ECAT_EOE_FRAME_BUFFER_COUNT
#define ECAT_EOE_FRAME_BUFFER_COUNT     4
#endif

/**
 * @brief EoE maximum frame size (bytes)
 * 
 * Maximum Ethernet frame size to handle. Standard is 1518 bytes.
 * 
 * Default: 1518
 */
#ifndef ECAT_EOE_MAX_FRAME_SIZE
#define ECAT_EOE_MAX_FRAME_SIZE         1518
#endif

/**
 * @brief EoE fragment size (bytes)
 * 
 * Size of each EoE fragment in mailbox. Must fit in mailbox with headers.
 * 
 * @note Recommendation: Match slave's mailbox size minus ~32 bytes for headers
 * 
 * Default: 256
 */
#ifndef ECAT_EOE_FRAGMENT_SIZE
#define ECAT_EOE_FRAGMENT_SIZE          256
#endif

/**
 * @brief EoE fragment reassembly timeout (milliseconds)
 * 
 * Timeout for receiving all fragments of an Ethernet frame.
 * 
 * Default: 1000
 */
#ifndef ECAT_EOE_FRAGMENT_TIMEOUT_MS
#define ECAT_EOE_FRAGMENT_TIMEOUT_MS    1000
#endif

/**
 * @brief Enable EoE IP address assignment
 * 
 * When enabled, the master can assign IP addresses to slaves.
 * 
 * Default: 1
 */
#ifndef ECAT_EOE_IP_ASSIGNMENT_ENABLED
#define ECAT_EOE_IP_ASSIGNMENT_ENABLED  1
#endif

/**
 * @brief EoE virtual network interface name
 * 
 * Name of the virtual network interface created for EoE (where supported).
 * 
 * Default: "eoe0"
 */
#ifndef ECAT_EOE_INTERFACE_NAME
#define ECAT_EOE_INTERFACE_NAME         "eoe0"
#endif

// ============================================================================
// DC CONFIGURATION
// ============================================================================

/**
 * @brief DC sync cycle time (nanoseconds)
 * 
 * Target cycle time for DC synchronization. Common values:
 * - 1000000 (1ms) - Standard industrial
 * - 500000 (500µs) - Fast I/O
 * - 250000 (250µs) - High-performance motion
 * 
 * @warning Shorter cycles require faster hardware and optimized code
 * 
 * Default: 1000000 (1ms)
 */
#ifndef ECAT_DC_CYCLE_TIME_NS
#define ECAT_DC_CYCLE_TIME_NS           1000000
#endif

/**
 * @brief DC drift compensation enabled
 * 
 * Enable automatic compensation for clock drift between master and slaves.
 * 
 * Default: 1
 */
#ifndef ECAT_DC_DRIFT_COMPENSATION
#define ECAT_DC_DRIFT_COMPENSATION      1
#endif

/**
 * @brief DC maximum allowed drift (nanoseconds)
 * 
 * Maximum acceptable clock drift before triggering resynchronization.
 * 
 * Default: 10000 (10µs)
 */
#ifndef ECAT_DC_MAX_DRIFT_NS
#define ECAT_DC_MAX_DRIFT_NS            10000
#endif

// ============================================================================
// TIMING AND PERFORMANCE
// ============================================================================

/**
 * @brief Mailbox polling interval (milliseconds)
 * 
 * How often to poll slave mailboxes for responses when not using interrupts.
 * 
 * @note Recommendation: 1-5ms. Lower = faster response, higher CPU usage
 * 
 * Default: 1
 */
#ifndef ECAT_MAILBOX_POLL_INTERVAL_MS
#define ECAT_MAILBOX_POLL_INTERVAL_MS   1
#endif

/**
 * @brief Ethernet frame timeout (milliseconds)
 * 
 * Timeout waiting for EtherCAT frame to return from the network.
 * 
 * @note Recommendation: 10-50ms. Longer for noisy/long networks.
 * 
 * Default: 20
 */
#ifndef ECAT_FRAME_TIMEOUT_MS
#define ECAT_FRAME_TIMEOUT_MS           20
#endif

/**
 * @brief State change timeout (milliseconds)
 * 
 * Timeout waiting for slave state machine transitions.
 * 
 * Default: 5000
 */
#ifndef ECAT_STATE_CHANGE_TIMEOUT_MS
#define ECAT_STATE_CHANGE_TIMEOUT_MS    5000
#endif

// ============================================================================
// DEBUG AND LOGGING
// ============================================================================

/**
 * @brief Enable verbose EtherCAT logging
 * 
 * When enabled, logs detailed information about EtherCAT operations.
 * Useful for debugging but adds overhead.
 * 
 * Default: 0 (disabled in production)
 */
#ifndef ECAT_DEBUG_LOGGING
#define ECAT_DEBUG_LOGGING              0
#endif

/**
 * @brief Enable frame dump logging
 * 
 * When enabled, logs hex dumps of EtherCAT frames.
 * Very verbose - use only for protocol debugging.
 * 
 * Default: 0
 */
#ifndef ECAT_DEBUG_FRAME_DUMP
#define ECAT_DEBUG_FRAME_DUMP           0
#endif

/**
 * @brief Log tag for EtherCAT modules
 * 
 * Used with TETHER_LOG* macros for platform-independent logging.
 */
#ifndef ECAT_LOG_TAG
#define ECAT_LOG_TAG                    "ECAT"
#endif

// ============================================================================
// TASK CONFIGURATION
// ============================================================================

/**
 * @brief SDO processing task priority
 * 
 * FreeRTOS priority for the background SDO task.
 * Should be lower than realtime task but higher than idle.
 * 
 * Default: 5
 */
#ifndef ECAT_SDO_TASK_PRIORITY
#define ECAT_SDO_TASK_PRIORITY          5
#endif

/**
 * @brief SDO task stack size (bytes)
 * 
 * Default: 4096
 */
#ifndef ECAT_SDO_TASK_STACK_SIZE
#define ECAT_SDO_TASK_STACK_SIZE        4096
#endif

/**
 * @brief SDO task CPU core (ESP32)
 * 
 * Which core to pin the SDO task to. Use tskNO_AFFINITY for no pinning.
 * 
 * Default: 0
 */
#ifndef ECAT_SDO_TASK_CORE
#define ECAT_SDO_TASK_CORE              0
#endif

/**
 * @brief FoE processing task priority
 * 
 * Default: 4 (lower than SDO)
 */
#ifndef ECAT_FOE_TASK_PRIORITY
#define ECAT_FOE_TASK_PRIORITY          4
#endif

/**
 * @brief FoE task stack size (bytes)
 * 
 * Default: 8192 (needs more for file operations)
 */
#ifndef ECAT_FOE_TASK_STACK_SIZE
#define ECAT_FOE_TASK_STACK_SIZE        8192
#endif

/**
 * @brief EoE processing task priority
 * 
 * Default: 6 (higher than SDO for network responsiveness)
 */
#ifndef ECAT_EOE_TASK_PRIORITY
#define ECAT_EOE_TASK_PRIORITY          6
#endif

/**
 * @brief EoE task stack size (bytes)
 * 
 * Default: 4096
 */
#ifndef ECAT_EOE_TASK_STACK_SIZE
#define ECAT_EOE_TASK_STACK_SIZE        4096
#endif

/**
 * @brief Realtime (DC) task priority
 * 
 * Should be the highest priority task.
 * 
 * Default: configMAX_PRIORITIES - 1
 */
#ifndef ECAT_DC_TASK_PRIORITY
#define ECAT_DC_TASK_PRIORITY           (configMAX_PRIORITIES - 1)
#endif

/**
 * @brief Realtime task stack size (bytes)
 * 
 * Default: 4096
 */
#ifndef ECAT_DC_TASK_STACK_SIZE
#define ECAT_DC_TASK_STACK_SIZE         4096
#endif

/**
 * @brief Realtime task CPU core (ESP32)
 * 
 * @note Recommendation: Pin to core 1, keep core 0 for WiFi/BT
 * 
 * Default: 1
 */
#ifndef ECAT_DC_TASK_CORE
#define ECAT_DC_TASK_CORE               1
#endif

// ============================================================================
// VALIDATION
// ============================================================================

// Validate feature dependencies
#if ECAT_FEATURE_FOE_ENABLED && !ECAT_FEATURE_SDO_ENABLED
    #warning "FoE requires SDO support. Enabling SDO."
    #undef ECAT_FEATURE_SDO_ENABLED
    #define ECAT_FEATURE_SDO_ENABLED 1
#endif

#if ECAT_FEATURE_VOE_ENABLED && !ECAT_FEATURE_SDO_ENABLED
    #warning "VoE requires SDO support. Enabling SDO."
    #undef ECAT_FEATURE_SDO_ENABLED
    #define ECAT_FEATURE_SDO_ENABLED 1
#endif

#if ECAT_FEATURE_EOE_ENABLED && !ECAT_FEATURE_SDO_ENABLED
    #warning "EoE requires SDO support. Enabling SDO."
    #undef ECAT_FEATURE_SDO_ENABLED
    #define ECAT_FEATURE_SDO_ENABLED 1
#endif

// Validate buffer sizes
#if ECAT_PDO_MAX_SLAVES > 247
    #error "ECAT_PDO_MAX_SLAVES cannot exceed 247 (EtherCAT limit)"
#endif

#if ECAT_FOE_BUFFER_SIZE < 128
    #error "ECAT_FOE_BUFFER_SIZE must be at least 128 bytes"
#endif

#if ECAT_EOE_FRAGMENT_SIZE < 64
    #error "ECAT_EOE_FRAGMENT_SIZE must be at least 64 bytes"
#endif
