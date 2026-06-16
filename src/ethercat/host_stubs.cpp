#include "tether/ethercat/Raw.hpp"
#include "tether/ethercat/SDOManager.hpp"
#include <chrono>
#include <thread>

// Forward declarations for minimal host stubs when EtherCAT types are not available
namespace EtherCAT {
namespace Platform {
    class PlatformFile;
    struct FileInfo;
    class PlatformNetif;
}
}

#if !defined(TETHER_ENABLE_ETHERCAT) && !defined(TETHER_COMPILE_MASTER)

// Host-only minimal stubs. These are excluded when building the full EtherCAT
// master (TETHER_COMPILE_MASTER=1) to avoid duplicate symbol definitions.
namespace EtherCAT {

bool readSlaveApplicationLayerState(uint16_t slave_index, uint8_t& state_code) {
    (void)slave_index; (void)state_code; return false;
}

bool requestSlaveApplicationLayerState(uint16_t slave_index, uint8_t state_code) {
    (void)slave_index; (void)state_code; return false;
}

bool transitionSlaveToPreOperational(uint16_t slave_index) {
    (void)slave_index; return false;
}

namespace Raw {
uint16_t get_discovered_slave_count() { return 0; }
} // namespace Raw

bool ConfigureWatchdogs(uint16_t slave_index, uint16_t pdi_timeout_100us, uint16_t pdata_timeout_100us) {
    (void)slave_index; (void)pdi_timeout_100us; (void)pdata_timeout_100us; return false;
}

bool DisableWatchdogs(uint16_t slave_index) {
    (void)slave_index; return false;
}

bool ReadWatchdogStatus(uint16_t slave_index, uint8_t& wd_status, uint8_t& pdi_cnt, uint8_t& pdata_cnt) {
    (void)slave_index; (void)wd_status; (void)pdi_cnt; (void)pdata_cnt; return false;
}

} // namespace EtherCAT



#endif // !TETHER_ENABLE_ETHERCAT && !TETHER_COMPILE_MASTER

// SDO helpers often implemented on the raw layer - provide minimal stubs (always present)
extern "C" bool ecm_sdo_read(uint16_t slave_addr, uint16_t index, uint8_t subindex, void* data, size_t len, bool use_configured_addr) {
    (void)slave_addr; (void)index; (void)subindex; (void)data; (void)len; (void)use_configured_addr; return false;
}

extern "C" bool ecm_sdo_write(uint16_t slave_addr, uint16_t index, uint8_t subindex, const void* data, size_t len, bool use_configured_addr) {
    (void)slave_addr; (void)index; (void)subindex; (void)data; (void)len; (void)use_configured_addr; return false;
}

// Minimal platform stubs
namespace EtherCAT {
namespace Platform {

PlatformFile* create_file() { return nullptr; }

bool file_exists(const char* path) { (void)path; return false; }
bool file_stat(const char* path, FileInfo* info) { (void)path; (void)info; return false; }
bool file_delete(const char* path) { (void)path; return false; }
bool file_rename(const char* old_path, const char* new_path) { (void)old_path; (void)new_path; return false; }
bool dir_create(const char* path) { (void)path; return false; }
uint32_t fs_available_space(const char* path) { (void)path; return 0; }
uint32_t fs_total_space(const char* path) { (void)path; return 0; }

bool fs_init() { return false; }
void fs_deinit() {}
bool fs_is_available() { return false; }

PlatformNetif* create_netif(const char* name) { (void)name; return nullptr; }
bool netif_is_supported() { return false; }

uint32_t get_time_ms() { using namespace std::chrono; return static_cast<uint32_t>(duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count()); }
uint64_t get_time_us() { using namespace std::chrono; return static_cast<uint64_t>(duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count()); }

void delay_ms(uint32_t ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }
void delay_us(uint32_t us) { std::this_thread::sleep_for(std::chrono::microseconds(us)); }

uint32_t enter_critical() { return 0; }
void exit_critical(uint32_t) { }

} // namespace Platform
// Note: dc_set_pdo_enabled is now implemented in dc_init.cpp, no stub needed

} // namespace EtherCAT
