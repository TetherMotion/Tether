/**
 * @file Platform.hpp
 * @brief Platform abstraction layer for EtherCAT stack
 * 
 * @details
 * This header provides platform-independent interfaces for:
 * - File system operations (for FoE)
 * - Network interface creation (for EoE)
 * - Timer/timing utilities
 * 
 * ## Platform Selection
 * 
 * The platform is selected automatically based on ECAT_PLATFORM_FILESYSTEM
 * in TetherConfig.hpp. Supported platforms:
 * 
 * - **ESP32 + LittleFS** (ECAT_PLATFORM_ESP32_LITTLEFS): Default for ESP32
 * - **ESP32 + SPIFFS** (ECAT_PLATFORM_ESP32_SPIFFS): Legacy ESP32
 * - **POSIX** (ECAT_PLATFORM_POSIX): Linux/Unix testing
 * - **None** (ECAT_PLATFORM_NONE): No filesystem (memory transfers only)
 * 
 * ## Implementing Custom Platforms
 * 
 * To add a new platform:
 * 1. Add a new ECAT_PLATFORM_xxx define
 * 2. Implement the platform interface in a new .cpp file
 * 3. Link your implementation when ECAT_PLATFORM_FILESYSTEM matches
 * 
 * @code
 * // Example custom platform implementation
 * namespace EtherCAT {
 * namespace platform {
 * 
 * class MyPlatformFile : public PlatformFile {
 * public:
 *     bool open(const char* path, FileMode mode) override {
 *         // Your implementation
 *     }
 *     // ... implement other methods
 * };
 * 
 * PlatformFile* create_file() {
 *     return new MyPlatformFile();
 * }
 * 
 * } // namespace platform
 * } // namespace EtherCAT
 * @endcode
 */

#pragma once

#include "TetherConfig.hpp"

#include <cstdint>
#include <cstddef>
#include <functional>

namespace EtherCAT {
namespace Platform {

// ============================================================================
// File System Abstraction
// ============================================================================

/**
 * @brief File open mode
 */
enum class FileMode {
    READ,           ///< Open for reading (file must exist)
    WRITE,          ///< Open for writing (truncate if exists)
    WRITE_CREATE,   ///< Open for writing (create if not exists)
    APPEND,         ///< Open for appending
    READ_WRITE,     ///< Open for read and write
};

/**
 * @brief File seek origin
 */
enum class SeekOrigin {
    BEGIN,      ///< Seek from beginning
    CURRENT,    ///< Seek from current position
    END,        ///< Seek from end
};

/**
 * @brief File information
 */
struct FileInfo {
    char name[64];      ///< File name
    uint32_t size;      ///< File size in bytes
    bool is_directory;  ///< true if directory
    uint32_t modified;  ///< Modification time (unix timestamp, 0 if unavailable)
};

/**
 * @brief Abstract file interface
 * 
 * Platform-specific implementations inherit from this.
 */
class PlatformFile {
public:
    virtual ~PlatformFile() = default;
    
    /**
     * @brief Open file
     * @param path File path
     * @param mode Open mode
     * @return true on success
     */
    virtual bool open(const char* path, FileMode mode) = 0;
    
    /**
     * @brief Close file
     */
    virtual void close() = 0;
    
    /**
     * @brief Check if file is open
     */
    virtual bool is_open() const = 0;
    
    /**
     * @brief Read from file
     * @param buffer Destination buffer
     * @param size Bytes to read
     * @return Bytes actually read, or -1 on error
     */
    virtual int32_t read(void* buffer, size_t size) = 0;
    
    /**
     * @brief Write to file
     * @param buffer Source buffer
     * @param size Bytes to write
     * @return Bytes actually written, or -1 on error
     */
    virtual int32_t write(const void* buffer, size_t size) = 0;
    
    /**
     * @brief Seek in file
     * @param offset Offset in bytes
     * @param origin Seek origin
     * @return New position, or -1 on error
     */
    virtual int32_t seek(int32_t offset, SeekOrigin origin) = 0;
    
    /**
     * @brief Get current position
     * @return Current position, or -1 on error
     */
    virtual int32_t tell() = 0;
    
    /**
     * @brief Get file size
     * @return File size, or -1 on error
     */
    virtual int32_t size() = 0;
    
    /**
     * @brief Flush buffers to storage
     * @return true on success
     */
    virtual bool flush() = 0;
    
    /**
     * @brief Check for end of file
     */
    virtual bool eof() = 0;
    
    /**
     * @brief Get last error code
     * @return Platform-specific error code
     */
    virtual int get_error() = 0;
};

/**
 * @brief Create platform-specific file object
 * 
 * Caller owns returned object and must delete it.
 * 
 * @return New file object, nullptr if filesystem not available
 */
PlatformFile* create_file();

/**
 * @brief Check if file exists
 * @param path File path
 * @return true if file exists
 */
bool file_exists(const char* path);

/**
 * @brief Get file info
 * @param path File path
 * @param[out] info File information
 * @return true on success
 */
bool file_stat(const char* path, FileInfo* info);

/**
 * @brief Delete file
 * @param path File path
 * @return true on success
 */
bool file_delete(const char* path);

/**
 * @brief Rename/move file
 * @param old_path Current path
 * @param new_path New path
 * @return true on success
 */
bool file_rename(const char* old_path, const char* new_path);

/**
 * @brief Create directory
 * @param path Directory path
 * @return true on success (or already exists)
 */
bool dir_create(const char* path);

/**
 * @brief Get available space on filesystem
 * @param path Path to check (for mounted filesystems)
 * @return Available bytes, or 0 on error
 */
uint32_t fs_available_space(const char* path);

/**
 * @brief Get total filesystem size
 * @param path Path to check
 * @return Total bytes, or 0 on error
 */
uint32_t fs_total_space(const char* path);

/**
 * @brief Initialize filesystem
 * 
 * Called during EtherCAT init. May mount filesystem if needed.
 * 
 * @return true on success
 */
bool fs_init();

/**
 * @brief Deinitialize filesystem
 */
void fs_deinit();

/**
 * @brief Check if filesystem is available
 */
bool fs_is_available();

// ============================================================================
// Directory Iteration
// ============================================================================

/**
 * @brief Directory iterator callback
 * @param info File/directory information
 * @param user_data User-provided context
 * @return true to continue, false to stop iteration
 */
using DirIterCallback = std::function<bool(const FileInfo& info, void* user_data)>;

/**
 * @brief Iterate directory contents
 * @param path Directory path
 * @param callback Called for each entry
 * @param user_data Passed to callback
 * @return Number of entries processed, or -1 on error
 */
int32_t dir_iterate(const char* path, DirIterCallback callback, void* user_data);

// ============================================================================
// Network Interface Abstraction (for EoE)
// ============================================================================

/**
 * @brief Virtual network interface for EoE
 * 
 * Platform-specific implementations create TAP-like interfaces
 * that integrate with the system's network stack.
 */
class PlatformNetif {
public:
    virtual ~PlatformNetif() = default;
    
    /**
     * @brief Initialize the interface
     * @param mac_address MAC address (6 bytes)
     * @return true on success
     */
    virtual bool init(const uint8_t mac_address[6]) = 0;
    
    /**
     * @brief Deinitialize the interface
     */
    virtual void deinit() = 0;
    
    /**
     * @brief Set interface up
     */
    virtual bool up() = 0;
    
    /**
     * @brief Set interface down
     */
    virtual bool down() = 0;
    
    /**
     * @brief Check if interface is up
     */
    virtual bool is_up() const = 0;
    
    /**
     * @brief Inject received frame into network stack
     * @param frame Frame data (with Ethernet header)
     * @param len Frame length
     * @return true on success
     */
    virtual bool inject_frame(const uint8_t* frame, size_t len) = 0;
    
    /**
     * @brief Callback for frames to transmit
     * 
     * @param frame Frame data (with Ethernet header)
     * @param len Frame length
     * @param user_data User context
     */
    using TxCallback = std::function<void(const uint8_t* frame, size_t len, void* user_data)>;
    
    /**
     * @brief Set transmit callback
     * 
     * Called when network stack wants to send a frame.
     */
    virtual void set_tx_callback(TxCallback callback, void* user_data) = 0;
    
    /**
     * @brief Set IP configuration
     */
    virtual bool set_ip(uint32_t ip, uint32_t netmask, uint32_t gateway) = 0;
    
    /**
     * @brief Get platform-specific handle
     * 
     * Returns platform-specific network interface handle.
     */
    virtual void* get_handle() = 0;
};

/**
 * @brief Create platform-specific network interface
 * 
 * @param name Interface name (e.g., "eoe0")
 * @return New interface, nullptr on failure
 */
PlatformNetif* create_netif(const char* name);

/**
 * @brief Check if network interfaces are supported
 */
bool netif_is_supported();

// ============================================================================
// Timing Utilities
// ============================================================================

/**
 * @brief Get current time in milliseconds
 * 
 * Monotonic clock, suitable for measuring elapsed time.
 */
uint32_t get_time_ms();

/**
 * @brief Get current time in microseconds
 */
uint64_t get_time_us();

/**
 * @brief Delay for specified milliseconds
 * 
 * Yields to other tasks if possible.
 */
void delay_ms(uint32_t ms);

/**
 * @brief Delay for specified microseconds
 * 
 * May be busy-wait for short delays.
 */
void delay_us(uint32_t us);

// ============================================================================
// Critical Section
// ============================================================================

/**
 * @brief Enter critical section (disable interrupts)
 * @return State to pass to exit_critical
 */
uint32_t enter_critical();

/**
 * @brief Exit critical section
 * @param state State from enter_critical
 */
void exit_critical(uint32_t state);

/**
 * @brief RAII critical section guard
 */
class CriticalSection {
public:
    CriticalSection() : state_(enter_critical()) {}
    ~CriticalSection() { exit_critical(state_); }
    
    // Non-copyable
    CriticalSection(const CriticalSection&) = delete;
    CriticalSection& operator=(const CriticalSection&) = delete;
    
private:
    uint32_t state_;
};

// ============================================================================
// Memory Utilities
// ============================================================================

/**
 * @brief Allocate DMA-capable memory
 * 
 * For Ethernet buffers that may be used with DMA.
 * 
 * @param size Size in bytes
 * @return Allocated memory, nullptr on failure
 */
void* alloc_dma_buffer(size_t size);

/**
 * @brief Free DMA-capable memory
 */
void free_dma_buffer(void* ptr);

// ============================================================================
// Platform Information
// ============================================================================

/**
 * @brief Get platform name
 */
const char* get_platform_name();

/**
 * @brief Get filesystem type name
 */
const char* get_filesystem_name();

} // namespace platform
} // namespace EtherCAT
