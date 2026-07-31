/**
 * @file test_klipper_config_ext.cpp
 * @brief Config tests: KlipperConfig chaining, StandardCommands completeness,
 *        build dictionary validity.
 */

#include <gtest/gtest.h>
#include "tether/klipper/config/KlipperConfig.hpp"
#include "tether/klipper/config/StandardCommands.hpp"
#include "tether/klipper/protocol/DataDictionary.hpp"

using namespace tether::klipper::config;
using namespace tether::klipper::protocol;

// ============================================================================
// KlipperConfig tests
// ============================================================================

TEST(KlipperConfigExt, DefaultValues) {
    KlipperConfig cfg;
    const auto& dict = cfg.dictionary();
    EXPECT_EQ(dict.app(), "tether_klipper");
    EXPECT_EQ(dict.version(), "1.0.0");
    EXPECT_EQ(dict.buildVersions(), "tether_klipper");
}

TEST(KlipperConfigExt, ChainingApp) {
    KlipperConfig cfg;
    cfg.app("my_app").version("2.0.0").buildVersions("build_v2");
    const auto& dict = cfg.dictionary();
    EXPECT_EQ(dict.app(), "my_app");
    EXPECT_EQ(dict.version(), "2.0.0");
    EXPECT_EQ(dict.buildVersions(), "build_v2");
}

TEST(KlipperConfigExt, AddCommand) {
    KlipperConfig cfg;
    cfg.addCommand("test_cmd val=%u");
    auto dict = cfg.build();
    auto found = dict.lookupCommand("test_cmd val=%u");
    ASSERT_TRUE(found.has_value());
}

TEST(KlipperConfigExt, AddResponse) {
    KlipperConfig cfg;
    cfg.addResponse("test_resp result=%u");
    auto dict = cfg.build();
    auto found = dict.lookupResponse("test_resp result=%u");
    ASSERT_TRUE(found.has_value());
}

TEST(KlipperConfigExt, AddOutput) {
    KlipperConfig cfg;
    cfg.addOutput("debug_output msg=%s");
    auto dict = cfg.build();
    auto found = dict.lookupOutput("debug_output msg=%s");
    ASSERT_TRUE(found.has_value());
}

TEST(KlipperConfigExt, AddConstant) {
    KlipperConfig cfg;
    cfg.addConstant("MY_CONST", 42);
    auto dict = cfg.build();
    auto val = dict.lookupConstant("MY_CONST");
    ASSERT_TRUE(val.has_value());
    auto* intPtr = std::get_if<int64_t>(&*val);
    ASSERT_NE(intPtr, nullptr);
    EXPECT_EQ(*intPtr, 42);
}

TEST(KlipperConfigExt, AddConstantString) {
    KlipperConfig cfg;
    cfg.addConstantString("MY_STR", "hello_world");
    auto dict = cfg.build();
    auto val = dict.lookupConstant("MY_STR");
    ASSERT_TRUE(val.has_value());
    auto* strPtr = std::get_if<std::string>(&*val);
    ASSERT_NE(strPtr, nullptr);
    EXPECT_EQ(*strPtr, "hello_world");
}

TEST(KlipperConfigExt, AddEnumValue) {
    KlipperConfig cfg;
    cfg.addEnumValue("pins", "PA0", 0);
    cfg.addEnumValue("pins", "PA1", 1);
    auto dict = cfg.build();
    auto val = dict.resolveEnum("pins", "PA0");
    ASSERT_TRUE(val.has_value());
    EXPECT_EQ(*val, 0u);
    auto val1 = dict.resolveEnum("pins", "PA1");
    ASSERT_TRUE(val1.has_value());
    EXPECT_EQ(*val1, 1u);
}

TEST(KlipperConfigExt, AddEnumRange) {
    KlipperConfig cfg;
    cfg.addEnumRange("pins", "PA", 0, 16);
    auto dict = cfg.build();
    auto val0 = dict.resolveEnum("pins", "PA0");
    ASSERT_TRUE(val0.has_value());
    EXPECT_EQ(*val0, 0u);
    auto val15 = dict.resolveEnum("pins", "PA15");
    ASSERT_TRUE(val15.has_value());
    EXPECT_EQ(*val15, 15u);
}

TEST(KlipperConfigExt, BuildReturnsDictionary) {
    KlipperConfig cfg;
    cfg.addCommand("cmd1");
    cfg.addCommand("cmd2");
    DataDictionary dict = cfg.build();
    EXPECT_TRUE(dict.lookupCommand("cmd1").has_value());
    EXPECT_TRUE(dict.lookupCommand("cmd2").has_value());
}

TEST(KlipperConfigExt, DictionaryAccessor) {
    KlipperConfig cfg;
    cfg.addCommand("test_cmd");
    const auto& dict = cfg.dictionary();
    EXPECT_TRUE(dict.lookupCommand("test_cmd").has_value());
}

TEST(KlipperConfigExt, MultipleCommandsSequential) {
    KlipperConfig cfg;
    cfg.addCommand("cmd1");
    uint16_t id1 = cfg.dictionary().lookupCommand("cmd1").value_or(0);
    cfg.addCommand("cmd2");
    uint16_t id2 = cfg.dictionary().lookupCommand("cmd2").value_or(0);
    EXPECT_NE(id1, id2);
    EXPECT_GT(id2, id1);
}

// ============================================================================
// StandardCommands tests - using actual format strings from StandardCommands.hpp
// ============================================================================

TEST(KlipperStandardCommandsExt, RegistersCommands) {
    KlipperConfig cfg;
    withStandardCommands(cfg, 180000000);
    auto dict = cfg.build();

    // Check for core commands (exact format strings from StandardCommands.hpp)
    EXPECT_TRUE(dict.lookupCommand("get_clock").has_value());
    EXPECT_TRUE(dict.lookupCommand("allocate_oids oid=%c").has_value());
    EXPECT_TRUE(dict.lookupCommand("get_config").has_value());
    EXPECT_TRUE(dict.lookupCommand("get_status").has_value());
    EXPECT_TRUE(dict.lookupCommand("shutdown").has_value());
    EXPECT_TRUE(dict.lookupCommand("finalize_config crc=%u").has_value());
}

TEST(KlipperStandardCommandsExt, RegistersDigitalOut) {
    KlipperConfig cfg;
    withStandardCommands(cfg, 180000000);
    auto dict = cfg.build();

    EXPECT_TRUE(dict.lookupCommand("update_digital_out oid=%c value=%c").has_value());
    EXPECT_TRUE(dict.lookupCommand("queue_digital_out oid=%c clock=%u value=%c").has_value());
    EXPECT_TRUE(dict.lookupCommand("config_digital_out oid=%c pin=%u value=%c").has_value());
}

TEST(KlipperStandardCommandsExt, RegistersPwmOut) {
    KlipperConfig cfg;
    withStandardCommands(cfg, 180000000);
    auto dict = cfg.build();

    EXPECT_TRUE(dict.lookupCommand("update_pwm_out oid=%c value=%hu").has_value());
    EXPECT_TRUE(dict.lookupCommand("queue_pwm_out oid=%c clock=%u value=%hu").has_value());
    EXPECT_TRUE(dict.lookupCommand("config_pwm_out oid=%c pin=%u cycle_ticks=%u value=%hu").has_value());
}

TEST(KlipperStandardCommandsExt, RegistersStepper) {
    KlipperConfig cfg;
    withStandardCommands(cfg, 180000000);
    auto dict = cfg.build();

    EXPECT_TRUE(dict.lookupCommand("queue_step oid=%c interval=%u count=%hu add=%hi").has_value());
    EXPECT_TRUE(dict.lookupCommand("set_next_step_dir oid=%c dir=%c").has_value());
    EXPECT_TRUE(dict.lookupCommand("config_stepper oid=%c step_pin=%u dir_pin=%u invert_step=%c").has_value());
    EXPECT_TRUE(dict.lookupCommand("reset_step_clock oid=%c clock=%u").has_value());
}

TEST(KlipperStandardCommandsExt, RegistersEndstop) {
    KlipperConfig cfg;
    withStandardCommands(cfg, 180000000);
    auto dict = cfg.build();

    EXPECT_TRUE(dict.lookupCommand("endstop_query oid=%c").has_value());
    EXPECT_TRUE(dict.lookupCommand("endstop_home oid=%c clock=%u sample_ticks=%u").has_value());
    EXPECT_TRUE(dict.lookupCommand("config_endstop oid=%c pin=%u flags=%c").has_value());
    EXPECT_TRUE(dict.lookupResponse("endstop_state oid=%c homed=%c").has_value());
}

TEST(KlipperStandardCommandsExt, RegistersTrsync) {
    KlipperConfig cfg;
    withStandardCommands(cfg, 180000000);
    auto dict = cfg.build();

    EXPECT_TRUE(dict.lookupCommand("trsync_start oid=%c clock=%u duration=%u").has_value());
    EXPECT_TRUE(dict.lookupCommand("trsync_set_timeout oid=%c clock=%u").has_value());
    EXPECT_TRUE(dict.lookupCommand("config_trsync oid=%c").has_value());
    EXPECT_TRUE(dict.lookupResponse("trsync_state oid=%c can_trigger=%c trigger_clock=%u").has_value());
}

TEST(KlipperStandardCommandsExt, RegistersSpi) {
    KlipperConfig cfg;
    withStandardCommands(cfg, 180000000);
    auto dict = cfg.build();

    EXPECT_TRUE(dict.lookupCommand("spi_transfer oid=%c data=%*s").has_value());
    EXPECT_TRUE(dict.lookupCommand("config_spi oid=%c pin=%u spi_bus=%u mode=%c rate=%u").has_value());
    EXPECT_TRUE(dict.lookupResponse("spi_response oid=%c response=%*s").has_value());
}

TEST(KlipperStandardCommandsExt, RegistersI2c) {
    KlipperConfig cfg;
    withStandardCommands(cfg, 180000000);
    auto dict = cfg.build();

    EXPECT_TRUE(dict.lookupCommand("i2c_write oid=%c addr=%c write=%*s").has_value());
    EXPECT_TRUE(dict.lookupCommand("i2c_read oid=%c addr=%c read_len=%u").has_value());
    EXPECT_TRUE(dict.lookupCommand("config_i2c oid=%c sda_pin=%u scl_pin=%u rate=%u").has_value());
    EXPECT_TRUE(dict.lookupResponse("i2c_response oid=%c response=%*s").has_value());
}

TEST(KlipperStandardCommandsExt, RegistersAnalogIn) {
    KlipperConfig cfg;
    withStandardCommands(cfg, 180000000);
    auto dict = cfg.build();

    EXPECT_TRUE(dict.lookupCommand("config_analog_in oid=%c pin=%u").has_value());
    EXPECT_TRUE(dict.lookupCommand("query_analog_in oid=%c").has_value());
    EXPECT_TRUE(dict.lookupResponse("analog_in_result oid=%c value=%hu").has_value());
}

TEST(KlipperStandardCommandsExt, ClockFreqConstant) {
    KlipperConfig cfg;
    withStandardCommands(cfg, 180000000);
    auto dict = cfg.build();

    auto val = dict.lookupConstant("CLOCK_FREQ");
    ASSERT_TRUE(val.has_value());
    auto* intPtr = std::get_if<int64_t>(&*val);
    ASSERT_NE(intPtr, nullptr);
    EXPECT_EQ(*intPtr, 180000000);
}

TEST(KlipperStandardCommandsExt, CustomClockFreq) {
    KlipperConfig cfg;
    withStandardCommands(cfg, 1000000); // 1 MHz
    auto dict = cfg.build();

    auto val = dict.lookupConstant("CLOCK_FREQ");
    ASSERT_TRUE(val.has_value());
    auto* intPtr = std::get_if<int64_t>(&*val);
    ASSERT_NE(intPtr, nullptr);
    EXPECT_EQ(*intPtr, 1000000);
}

TEST(KlipperStandardCommandsExt, McuConstant) {
    KlipperConfig cfg;
    withStandardCommands(cfg, 180000000);
    auto dict = cfg.build();

    auto val = dict.lookupConstant("MCU");
    ASSERT_TRUE(val.has_value());
    auto* strPtr = std::get_if<std::string>(&*val);
    ASSERT_NE(strPtr, nullptr);
    EXPECT_EQ(*strPtr, "tether_klipper");
}

TEST(KlipperStandardCommandsExt, PinEnums) {
    KlipperConfig cfg;
    withStandardCommands(cfg, 180000000);
    auto dict = cfg.build();

    // StandardCommands uses "pin" (singular) with range keys "PA0", "PB0", "PC0"
    auto pa0 = dict.resolveEnum("pin", "PA0");
    ASSERT_TRUE(pa0.has_value());
    EXPECT_EQ(*pa0, 0u);

    auto pb0 = dict.resolveEnum("pin", "PB0");
    ASSERT_TRUE(pb0.has_value());
    EXPECT_EQ(*pb0, 16u); // PA0-15 = 0-15, PB0-15 = 16-31
}

TEST(KlipperStandardCommandsExt, SpiBusEnums) {
    KlipperConfig cfg;
    withStandardCommands(cfg, 180000000);
    auto dict = cfg.build();

    auto spi1 = dict.resolveEnum("spi_bus", "spi1");
    ASSERT_TRUE(spi1.has_value());
    EXPECT_EQ(*spi1, 0u);

    auto spi2 = dict.resolveEnum("spi_bus", "spi2");
    ASSERT_TRUE(spi2.has_value());
    EXPECT_EQ(*spi2, 1u);

    auto spi3 = dict.resolveEnum("spi_bus", "spi3");
    ASSERT_TRUE(spi3.has_value());
    EXPECT_EQ(*spi3, 2u);
}

TEST(KlipperStandardCommandsExt, DictionaryWireBlobValid) {
    KlipperConfig cfg;
    withStandardCommands(cfg, 180000000);
    auto dict = cfg.build();

    auto wire = dict.toWire();
    ASSERT_FALSE(wire.empty());

    auto json = DataDictionary::fromWire(wire);
    ASSERT_FALSE(json.empty());

    DataDictionary dict2;
    ASSERT_TRUE(dict2.fromJson(json));
    EXPECT_TRUE(dict2.lookupCommand("get_clock").has_value());
}

TEST(KlipperStandardCommandsExt, ResponseFormats) {
    KlipperConfig cfg;
    withStandardCommands(cfg, 180000000);
    auto dict = cfg.build();

    // Check for response formats (exact strings from StandardCommands.hpp)
    EXPECT_TRUE(dict.lookupResponse("clock clock=%u").has_value());
    EXPECT_TRUE(dict.lookupResponse("identify_response offset=%u data=%.*s").has_value());
    EXPECT_TRUE(dict.lookupResponse("config_result oid_count=%c config_crc=%u").has_value());
    EXPECT_TRUE(dict.lookupResponse("status clock=%u status=%c").has_value());
}

TEST(KlipperStandardCommandsExt, TotalCommandCount) {
    KlipperConfig cfg;
    withStandardCommands(cfg, 180000000);
    auto dict = cfg.build();

    // Count all messages
    const auto& msgs = dict.messages();
    EXPECT_GT(msgs.size(), 20u); // Should have many commands + responses
}
