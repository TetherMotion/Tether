/**
 * @file platform_esp32.cpp
 * @brief ESP32 platform implementation for EtherCAT stack
 * 
 * Provides filesystem (LittleFS/SPIFFS) and network interface implementations
 * for ESP32 platform.
 */

#include "EtherCATPlatform.hpp"
#include "EtherCATConfig.hpp"

#if defined(ESP_PLATFORM)

#include <cstring>
#include <climits>
#include <sys/stat.h>
#include <dirent.h>
#include <unistd.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Filesystem-specific includes
#if ECAT_PLATFORM_FILESYSTEM == ECAT_PLATFORM_ESP32_LITTLEFS
#include "esp_littlefs.h"
#elif ECAT_PLATFORM_FILESYSTEM == ECAT_PLATFORM_ESP32_SPIFFS
#include "esp_spiffs.h"
#endif

// Network interface includes (for EoE)
#if ECAT_FEATURE_EOE_ENABLED
#include "esp_netif.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/etharp.h"
#endif

static const char* TAG = ECAT_LOG_TAG "_PLAT";

namespace EtherCAT {
namespace Platform {

// ============================================================================
// Filesystem State
//
// Filesystem initialization state is tracked internally without global state.
// ESP32-only (compiled conditionally).
// ============================================================================

// ============================================================================
// ESP32 File Implementation
// ============================================================================

#if ECAT_PLATFORM_FILESYSTEM != ECAT_PLATFORM_NONE

class ESP32File : public PlatformFile {
public:
    ESP32File() : file_(nullptr), last_error_(0) {}
    
    ~ESP32File() override {
        close();
    }
    
    bool open(const char* path, FileMode mode) override {
        if (file_) {
            close();
        }
        
        const char* mode_str;
        switch (mode) {
            case FileMode::READ:
                mode_str = "rb";
                break;
            case FileMode::WRITE:
            case FileMode::WRITE_CREATE:
                mode_str = "wb";
                break;
            case FileMode::APPEND:
                mode_str = "ab";
                break;
            case FileMode::READ_WRITE:
                mode_str = "r+b";
                break;
            default:
                return false;
        }
        
        file_ = fopen(path, mode_str);
        if (!file_) {
            last_error_ = errno;
            ESP_LOGD(TAG, "Failed to open %s: %d", path, last_error_);
            return false;
        }
        
        return true;
    }
    
    void close() override {
        if (file_) {
            fclose(file_);
            file_ = nullptr;
        }
    }
    
    bool is_open() const override {
        return file_ != nullptr;
    }
    
    int32_t read(void* buffer, size_t size) override {
        if (!file_) return -1;
        
        size_t read = fread(buffer, 1, size, file_);
        if (read == 0 && ferror(file_)) {
            last_error_ = errno;
            return -1;
        }
        return static_cast<int32_t>(read);
    }
    
    int32_t write(const void* buffer, size_t size) override {
        if (!file_) return -1;
        
        size_t written = fwrite(buffer, 1, size, file_);
        if (written != size && ferror(file_)) {
            last_error_ = errno;
            return -1;
        }
        return static_cast<int32_t>(written);
    }
    
    int32_t seek(int32_t offset, SeekOrigin origin) override {
        if (!file_) return -1;
        
        int whence;
        switch (origin) {
            case SeekOrigin::BEGIN:   whence = SEEK_SET; break;
            case SeekOrigin::CURRENT: whence = SEEK_CUR; break;
            case SeekOrigin::END:     whence = SEEK_END; break;
            default: return -1;
        }
        
        if (fseek(file_, offset, whence) != 0) {
            last_error_ = errno;
            return -1;
        }
        
        return ftell(file_);
    }
    
    int32_t tell() override {
        if (!file_) return -1;
        return ftell(file_);
    }
    
    int32_t size() override {
        if (!file_) return -1;
        
        long cur = ftell(file_);
        fseek(file_, 0, SEEK_END);
        long sz = ftell(file_);
        fseek(file_, cur, SEEK_SET);
        
        return static_cast<int32_t>(sz);
    }
    
    bool flush() override {
        if (!file_) return false;
        return fflush(file_) == 0;
    }
    
    bool eof() override {
        if (!file_) return true;
        return feof(file_) != 0;
    }
    
    int get_error() override {
        return last_error_;
    }
    
private:
    FILE* file_;
    int last_error_;
};

#endif // ECAT_PLATFORM_FILESYSTEM != ECAT_PLATFORM_NONE

// ============================================================================
// Filesystem Functions
// ============================================================================

PlatformFile* create_file() {
#if ECAT_PLATFORM_FILESYSTEM != ECAT_PLATFORM_NONE
    return new ESP32File();
#else
    return nullptr;
#endif
}

bool fs_init() {
#if ECAT_PLATFORM_FILESYSTEM == ECAT_PLATFORM_ESP32_LITTLEFS
    esp_vfs_littlefs_conf_t conf = {
        .base_path = ECAT_LITTLEFS_MOUNT_POINT,
        .partition_label = ECAT_LITTLEFS_PARTITION_LABEL,
        .format_if_mount_failed = true,
        .dont_mount = false,
    };
    
    esp_err_t ret = esp_vfs_littlefs_register(&conf);
    if (ret != ESP_OK) {
        if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "LittleFS partition '%s' not found", ECAT_LITTLEFS_PARTITION_LABEL);
        } else {
            ESP_LOGE(TAG, "Failed to mount LittleFS: %s", esp_err_to_name(ret));
        }
        return false;
    }
    
    size_t total = 0, used = 0;
    esp_littlefs_info(ECAT_LITTLEFS_PARTITION_LABEL, &total, &used);
    ESP_LOGI(TAG, "LittleFS mounted: %zu/%zu bytes used", used, total);
    
#elif ECAT_PLATFORM_FILESYSTEM == ECAT_PLATFORM_ESP32_SPIFFS
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };
    
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SPIFFS: %s", esp_err_to_name(ret));
        return false;
    }
    
    size_t total = 0, used = 0;
    esp_spiffs_info(NULL, &total, &used);
    ESP_LOGI(TAG, "SPIFFS mounted: %zu/%zu bytes used", used, total);
    
#else
    // No filesystem
    return true;
#endif
    
    return true;
}

void fs_deinit() {
#if ECAT_PLATFORM_FILESYSTEM == ECAT_PLATFORM_ESP32_LITTLEFS
    esp_vfs_littlefs_unregister(ECAT_LITTLEFS_PARTITION_LABEL);
#elif ECAT_PLATFORM_FILESYSTEM == ECAT_PLATFORM_ESP32_SPIFFS
    esp_vfs_spiffs_unregister(NULL);
#endif
}

bool fs_is_available() {
    // Check if filesystem is mounted by attempting to access the mount point
#if ECAT_PLATFORM_FILESYSTEM == ECAT_PLATFORM_ESP32_LITTLEFS
    struct stat st;
    return (stat(ECAT_LITTLEFS_MOUNT_POINT, &st) == 0);
#elif ECAT_PLATFORM_FILESYSTEM == ECAT_PLATFORM_ESP32_SPIFFS
    struct stat st;
    return (stat("/spiffs", &st) == 0);
#else
    return true; // No filesystem means "available" in the sense that it's not needed
#endif
}

bool file_exists(const char* path) {
    struct stat st;
    return (stat(path, &st) == 0);
}

bool file_stat(const char* path, FileInfo* info) {
    struct stat st;
    if (stat(path, &st) != 0) {
        return false;
    }
    
    // Extract filename from path
    const char* name = strrchr(path, '/');
    name = name ? name + 1 : path;
    strncpy(info->name, name, sizeof(info->name) - 1);
    info->name[sizeof(info->name) - 1] = '\0';
    
    info->size = static_cast<uint32_t>(st.st_size);
    info->is_directory = S_ISDIR(st.st_mode);
    info->modified = static_cast<uint32_t>(st.st_mtime);
    
    return true;
}

bool file_delete(const char* path) {
    return (unlink(path) == 0);
}

bool file_rename(const char* old_path, const char* new_path) {
    return (rename(old_path, new_path) == 0);
}

bool dir_create(const char* path) {
    struct stat st;
    if (stat(path, &st) == 0) {
        return S_ISDIR(st.st_mode);
    }
    return (mkdir(path, 0755) == 0);
}

uint32_t fs_available_space(const char* path) {
    (void)path;
#if ECAT_PLATFORM_FILESYSTEM == ECAT_PLATFORM_ESP32_LITTLEFS
    size_t total = 0, used = 0;
    if (esp_littlefs_info(ECAT_LITTLEFS_PARTITION_LABEL, &total, &used) == ESP_OK) {
        return static_cast<uint32_t>(total - used);
    }
#elif ECAT_PLATFORM_FILESYSTEM == ECAT_PLATFORM_ESP32_SPIFFS
    size_t total = 0, used = 0;
    if (esp_spiffs_info(NULL, &total, &used) == ESP_OK) {
        return static_cast<uint32_t>(total - used);
    }
#endif
    return 0;
}

uint32_t fs_total_space(const char* path) {
    (void)path;
#if ECAT_PLATFORM_FILESYSTEM == ECAT_PLATFORM_ESP32_LITTLEFS
    size_t total = 0, used = 0;
    if (esp_littlefs_info(ECAT_LITTLEFS_PARTITION_LABEL, &total, &used) == ESP_OK) {
        return static_cast<uint32_t>(total);
    }
#elif ECAT_PLATFORM_FILESYSTEM == ECAT_PLATFORM_ESP32_SPIFFS
    size_t total = 0, used = 0;
    if (esp_spiffs_info(NULL, &total, &used) == ESP_OK) {
        return static_cast<uint32_t>(total);
    }
#endif
    return 0;
}

int32_t dir_iterate(const char* path, DirIterCallback callback, void* user_data) {
    DIR* dir = opendir(path);
    if (!dir) {
        return -1;
    }
    
    int32_t count = 0;
    struct dirent* entry;
    
    while ((entry = readdir(dir)) != nullptr) {
        // Skip . and ..
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
            continue;
        }
        
        // Build full path for stat (PATH_MAX is typically 256)
        char full_path[PATH_MAX];
        int n = snprintf(full_path, sizeof(full_path), "%s/%s", path, entry->d_name);
        if (n < 0 || static_cast<size_t>(n) >= sizeof(full_path)) {
            continue;  // Path too long
        }
        
        FileInfo info;
        if (file_stat(full_path, &info)) {
            count++;
            if (!callback(info, user_data)) {
                break;
            }
        }
    }
    
    closedir(dir);
    return count;
}

// ============================================================================
// Network Interface Implementation (for EoE)
// ============================================================================

#if ECAT_FEATURE_EOE_ENABLED

/**
 * @brief ESP32 EoE network interface implementation
 * 
 * Creates a custom esp_netif that routes frames through EoE.
 */
class ESP32EoENetif : public PlatformNetif {
public:
    ESP32EoENetif(const char* name) 
        : netif_(nullptr)
        , tx_callback_(nullptr)
        , tx_user_data_(nullptr)
        , is_up_(false)
    {
        strncpy(name_, name, sizeof(name_) - 1);
        name_[sizeof(name_) - 1] = '\0';
    }
    
    ~ESP32EoENetif() override {
        deinit();
    }
    
    bool init(const uint8_t mac_address[6]) override {
        if (netif_) {
            return false;  // Already initialized
        }
        
        memcpy(mac_, mac_address, 6);
        
        // Create custom netif configuration
        esp_netif_inherent_config_t base_cfg = {
            .flags = (esp_netif_flags_t)(ESP_NETIF_FLAG_AUTOUP),
            .ip_info = nullptr,
            .get_ip_event = 0,
            .lost_ip_event = 0,
            .if_key = name_,
            .if_desc = name_,
            .route_prio = 50,
        };
        
        // Custom driver for EoE
        esp_netif_driver_ifconfig_t driver_cfg = {
            .handle = this,
            .transmit = eoe_transmit_wrapper,
            .driver_free_rx_buffer = nullptr,
        };
        
        const esp_netif_config_t cfg = {
            .base = &base_cfg,
            .driver = &driver_cfg,
            .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH,
        };
        
        netif_ = esp_netif_new(&cfg);
        if (!netif_) {
            ESP_LOGE(TAG, "Failed to create netif");
            return false;
        }
        
        // Set MAC address
        esp_netif_set_mac(netif_, mac_);
        
        ESP_LOGI(TAG, "EoE netif '%s' created", name_);
        return true;
    }
    
    void deinit() override {
        if (netif_) {
            esp_netif_destroy(netif_);
            netif_ = nullptr;
        }
        is_up_ = false;
    }
    
    bool up() override {
        if (!netif_) return false;
        esp_netif_action_start(netif_, nullptr, 0, nullptr);
        is_up_ = true;
        return true;
    }
    
    bool down() override {
        if (!netif_) return false;
        esp_netif_action_stop(netif_, nullptr, 0, nullptr);
        is_up_ = false;
        return true;
    }
    
    bool is_up() const override {
        return is_up_;
    }
    
    bool inject_frame(const uint8_t* frame, size_t len) override {
        if (!netif_ || !is_up_) return false;
        
        // Pass frame to lwIP
        esp_netif_receive(netif_, const_cast<uint8_t*>(frame), len, nullptr);
        return true;
    }
    
    void set_tx_callback(TxCallback callback, void* user_data) override {
        tx_callback_ = callback;
        tx_user_data_ = user_data;
    }
    
    bool set_ip(uint32_t ip, uint32_t netmask, uint32_t gateway) override {
        if (!netif_) return false;
        
        esp_netif_ip_info_t ip_info;
        ip_info.ip.addr = ip;
        ip_info.netmask.addr = netmask;
        ip_info.gw.addr = gateway;
        
        esp_netif_dhcpc_stop(netif_);
        return esp_netif_set_ip_info(netif_, &ip_info) == ESP_OK;
    }
    
    void* get_handle() override {
        return netif_;
    }
    
    // Called by lwIP when it wants to transmit
    esp_err_t transmit(void* buffer, size_t len) {
        if (tx_callback_) {
            tx_callback_(static_cast<const uint8_t*>(buffer), len, tx_user_data_);
        }
        return ESP_OK;
    }
    
private:
    static esp_err_t eoe_transmit_wrapper(void* h, void* buffer, size_t len) {
        auto* self = static_cast<ESP32EoENetif*>(h);
        return self->transmit(buffer, len);
    }
    
    esp_netif_t* netif_;
    TxCallback tx_callback_;
    void* tx_user_data_;
    bool is_up_;
    char name_[16];
    uint8_t mac_[6];
};

#endif // ECAT_FEATURE_EOE_ENABLED

PlatformNetif* create_netif(const char* name) {
#if ECAT_FEATURE_EOE_ENABLED
    return new ESP32EoENetif(name);
#else
    (void)name;
    return nullptr;
#endif
}

bool netif_is_supported() {
#if ECAT_FEATURE_EOE_ENABLED
    return true;
#else
    return false;
#endif
}

// ============================================================================
// Timing Functions
// ============================================================================

uint32_t get_time_ms() {
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

uint64_t get_time_us() {
    return static_cast<uint64_t>(esp_timer_get_time());
}

void delay_ms(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void delay_us(uint32_t us) {
    if (us < 1000) {
        // Busy wait for short delays
        uint64_t start = esp_timer_get_time();
        while ((esp_timer_get_time() - start) < us) {
            // Spin
        }
    } else {
        // Use FreeRTOS for longer delays
        vTaskDelay(pdMS_TO_TICKS((us + 999) / 1000));
    }
}

// ============================================================================
// Critical Section
//
// Uses FreeRTOS port-level critical section macros which don't require
// global state. The taskENTER_CRITICAL/taskEXIT_CRITICAL macros manage
// interrupt state internally.
// ============================================================================

uint32_t enter_critical() {
    taskENTER_CRITICAL();
    return 0;
}

void exit_critical(uint32_t state) {
    (void)state;
    taskEXIT_CRITICAL();
}

// ============================================================================
// Memory Utilities
// ============================================================================

void* alloc_dma_buffer(size_t size) {
    return heap_caps_malloc(size, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
}

void free_dma_buffer(void* ptr) {
    heap_caps_free(ptr);
}

// ============================================================================
// Platform Information
// ============================================================================

const char* get_platform_name() {
    return "ESP32";
}

const char* get_filesystem_name() {
#if ECAT_PLATFORM_FILESYSTEM == ECAT_PLATFORM_ESP32_LITTLEFS
    return "LittleFS";
#elif ECAT_PLATFORM_FILESYSTEM == ECAT_PLATFORM_ESP32_SPIFFS
    return "SPIFFS";
#else
    return "None";
#endif
}

} // namespace Platform
} // namespace EtherCAT

#endif // ESP_PLATFORM
