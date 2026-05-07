#include <gtest/gtest.h>

#include "tether/ethercat/EtherCATPlatform.hpp"

#include <cstdint>
#include <limits>

extern "C" bool ecm_sdo_read(uint16_t slave_addr, uint16_t index, uint8_t subindex,
                             void* data, size_t len, bool use_configured_addr);
extern "C" bool ecm_sdo_write(uint16_t slave_addr, uint16_t index, uint8_t subindex,
                              const void* data, size_t len, bool use_configured_addr);

TEST(EtherCATHostStubs, PlatformFilesystemAPIsReturnSafeDefaults) {
    using namespace EtherCAT::Platform;

    EXPECT_EQ(create_file(), nullptr);
    EXPECT_FALSE(file_exists(nullptr));
    EXPECT_FALSE(file_stat(nullptr, nullptr));
    EXPECT_FALSE(file_delete(nullptr));
    EXPECT_FALSE(file_rename(nullptr, nullptr));
    EXPECT_FALSE(dir_create(nullptr));
    EXPECT_EQ(fs_available_space(nullptr), 0u);
    EXPECT_EQ(fs_total_space(nullptr), 0u);
    EXPECT_FALSE(fs_init());
    fs_deinit();
    EXPECT_FALSE(fs_is_available());
}

TEST(EtherCATHostStubs, PlatformNetifAndTimingAPIsAreCallable) {
    using namespace EtherCAT::Platform;

    EXPECT_EQ(create_netif(nullptr), nullptr);
    EXPECT_FALSE(netif_is_supported());

    const auto t1ms = get_time_ms();
    const auto t1us = get_time_us();
    EXPECT_GE(t1ms, 0u);
    EXPECT_GE(t1us, 0u);

    // 0-duration delays should return quickly; primary intent is to execute the code.
    delay_ms(0);
    delay_us(0);

    EXPECT_EQ(enter_critical(), 0u);
    exit_critical(0u);
}

TEST(EtherCATHostStubs, SDOSHelpersReturnFalseForAllInputs) {
    uint8_t buffer[8] = {0};
    EXPECT_FALSE(ecm_sdo_read(0, 0, 0, buffer, sizeof(buffer), false));
    EXPECT_FALSE(ecm_sdo_read(std::numeric_limits<uint16_t>::max(), 0xFFFF, 0xFF,
                              nullptr, 0, true));

    EXPECT_FALSE(ecm_sdo_write(0, 0, 0, buffer, sizeof(buffer), false));
    EXPECT_FALSE(ecm_sdo_write(std::numeric_limits<uint16_t>::max(), 0xFFFF, 0xFF,
                               nullptr, 0, true));
}
