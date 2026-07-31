/**
 * @file StandardCommands.hpp
 * @brief Standard Klipper command/response set for a tether_klipper device.
 *
 * @details
 * This header provides a helper that registers the standard set of Klipper
 * commands, responses, enumerations, and constants into a KlipperConfig.
 * Devices can use this as a starting point and add custom commands as needed.
 */

#pragma once

#include "tether/klipper/config/KlipperConfig.hpp"

namespace tether::klipper::config {

/**
 * @brief Register the standard Klipper command/response set.
 *
 * Includes: get_clock, allocate_oids, get_config, get_status, shutdown,
 * digital_out, pwm_out, stepper, endstop, trsync, spi, i2c, analog_in, and
 * queue_step / queue_digital_out / queue_pwm_out scheduling commands.
 *
 * @param config The config builder to populate.
 * @param clockFreqHz The MCU clock frequency (added as CLOCK_FREQ constant).
 * @return The config builder (for chaining).
 */
inline KlipperConfig& withStandardCommands(KlipperConfig& config, uint32_t clockFreqHz) {
    // Core commands
    config.addCommand("get_clock")
          .addCommand("allocate_oids oid=%c")
          .addCommand("get_config")
          .addCommand("get_status")
          .addCommand("shutdown")
          .addCommand("finalize_config crc=%u");

    // Responses
    config.addResponse("clock clock=%u")
          .addResponse("identify_response offset=%u data=%.*s")
          .addResponse("config_result oid_count=%c config_crc=%u")
          .addResponse("status clock=%u status=%c");

    // Digital out
    config.addCommand("update_digital_out oid=%c value=%c")
          .addCommand("queue_digital_out oid=%c clock=%u value=%c")
          .addCommand("config_digital_out oid=%c pin=%u value=%c");

    // PWM out
    config.addCommand("update_pwm_out oid=%c value=%hu")
          .addCommand("queue_pwm_out oid=%c clock=%u value=%hu")
          .addCommand("config_pwm_out oid=%c pin=%u cycle_ticks=%u value=%hu");

    // Stepper
    config.addCommand("queue_step oid=%c interval=%u count=%hu add=%hi")
          .addCommand("set_next_step_dir oid=%c dir=%c")
          .addCommand("config_stepper oid=%c step_pin=%u dir_pin=%u invert_step=%c")
          .addCommand("reset_step_clock oid=%c clock=%u");

    // Endstop
    config.addCommand("endstop_query oid=%c")
          .addCommand("endstop_home oid=%c clock=%u sample_ticks=%u")
          .addResponse("endstop_state oid=%c homed=%c")
          .addCommand("config_endstop oid=%c pin=%u flags=%c");

    // Trsync
    config.addCommand("trsync_start oid=%c clock=%u duration=%u")
          .addCommand("trsync_set_timeout oid=%c clock=%u")
          .addResponse("trsync_state oid=%c can_trigger=%c trigger_clock=%u")
          .addCommand("config_trsync oid=%c");

    // SPI
    config.addCommand("spi_transfer oid=%c data=%*s")
          .addResponse("spi_response oid=%c response=%*s")
          .addCommand("config_spi oid=%c pin=%u spi_bus=%u mode=%c rate=%u");

    // I2C
    config.addCommand("i2c_write oid=%c addr=%c write=%*s")
          .addCommand("i2c_read oid=%c addr=%c read_len=%u")
          .addResponse("i2c_response oid=%c response=%*s")
          .addCommand("config_i2c oid=%c sda_pin=%u scl_pin=%u rate=%u");

    // Analog in
    config.addCommand("config_analog_in oid=%c pin=%u")
          .addCommand("query_analog_in oid=%c")
          .addResponse("analog_in_result oid=%c value=%hu");

    // Additional query commands (matching Klipper's command set)
    config.addCommand("query_endstop oid=%c")
          .addResponse("endstop_state oid=%c state=%c")
          .addCommand("query_adc oid=%c")
          .addResponse("adc_result oid=%c value=%hu")
          .addCommand("query_status")
          .addResponse("status_result state=%c uptime=%u");

    // LED / Neopixel
    config.addCommand("update_neopixel oid=%c data=%*s")
          .addCommand("config_neopixel oid=%c pin=%u chain_count=%hu");

    // Fan
    config.addCommand("set_fan_speed oid=%c speed=%hu")
          .addCommand("config_fan oid=%c pin=%u");

    // Sensor
    config.addCommand("query_sensor oid=%c")
          .addResponse("sensor_result oid=%c value=%hu");

    // Pulse counter
    config.addCommand("pulse_counter_query oid=%c")
          .addResponse("pulse_counter_result oid=%c count=%i")
          .addCommand("config_pulse_counter oid=%c pin=%u");

    // ADXL345
    config.addCommand("adxl345_query oid=%c")
          .addResponse("adxl345_result oid=%c x=%hi y=%hi z=%hi")
          .addCommand("config_adxl345 oid=%c spi_bus=%u");

    // TMC UART
    config.addCommand("tmc_uart_read oid=%c reg=%c")
          .addResponse("tmc_uart_result oid=%c reg=%c value=%u")
          .addCommand("tmc_uart_write oid=%c reg=%c value=%u")
          .addCommand("config_tmc_uart oid=%c addr=%c");

    // Constants
    config.addConstant("CLOCK_FREQ", static_cast<int64_t>(clockFreqHz));
    config.addConstantString("MCU", "tether_klipper");

    // Standard enumerations
    config.addEnumRange("pin", "PA0", 0, 16);
    config.addEnumRange("pin", "PB0", 16, 16);
    config.addEnumRange("pin", "PC0", 32, 16);
    config.addEnumValue("spi_bus", "spi1", 0);
    config.addEnumValue("spi_bus", "spi2", 1);
    config.addEnumValue("spi_bus", "spi3", 2);

    return config;
}

} // namespace tether::klipper::config
