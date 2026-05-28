#include "tether/ethercat/EtherCATRaw.hpp"
#include "tether/ethercat/EtherCATPlatform.hpp"
#include <chrono>
#include <thread>

// These stubs are only needed when NOT linking against tether_ethercat_master
// which already provides host_stubs.cpp with these symbols.
#if 0

namespace Platform {

Platform::PlatformFile* create_file() { return nullptr; }

bool file_exists(const char* /*path*/) { return false; }
bool file_stat(const char* /*path*/, Platform::FileInfo* /*info*/) { return false; }
bool file_delete(const char* /*path*/) { return false; }
bool file_rename(const char* /*old_path*/, const char* /*new_path*/) { return false; }
bool dir_create(const char* /*path*/) { return false; }
uint32_t fs_available_space(const char* /*path*/) { return 0; }
uint32_t fs_total_space(const char* /*path*/) { return 0; }

bool fs_init() { return false; }
void fs_deinit() {}
bool fs_is_available() { return false; }

Platform::PlatformNetif* create_netif(const char* /*name*/) { return nullptr; }
bool netif_is_supported() { return false; }

uint32_t get_time_ms() { using namespace std::chrono; return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()); }
uint64_t get_time_us() { using namespace std::chrono; return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()); }

void delay_ms(uint32_t ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }
void delay_us(uint32_t us) { std::this_thread::sleep_for(std::chrono::microseconds(us)); }

uint32_t enter_critical() { return 0; }
void exit_critical(uint32_t) { }

} // namespace Platform

#endif // 0 - symbols provided by host_stubs.cpp in tether_ethercat_master

namespace EtherCAT {

class EtherCATMaster;

#if !defined(TETHER_COMPILE_MASTER)
namespace Raw {
    uint16_t adp_for_slave_index(uint16_t index) { (void)index; return 0; }

    bool configure_mailbox_from_sii(EtherCATMaster& /*master*/, uint16_t /*adp*/,
                               uint16_t* /*wr_addr*/, uint16_t* /*wr_len*/, uint16_t* /*rd_addr*/, uint16_t* /*rd_len*/, uint16_t* /*proto*/) {
        return false;
    }
}
#endif

} // namespace EtherCAT

#if 0
extern "C" bool ecm_sdo_read(uint16_t /*slave_addr*/, uint16_t /*index*/, uint8_t /*subindex*/, void* /*data*/, size_t /*len*/, bool /*use_configured_addr*/) {
    return false;
}

extern "C" bool ecm_sdo_write(uint16_t /*slave_addr*/, uint16_t /*index*/, uint8_t /*subindex*/, const void* /*data*/, size_t /*len*/, bool /*use_configured_addr*/) {
    return false;
}
#endif // 0 - provided by host_stubs.cpp
