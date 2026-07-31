#pragma once

/// @file PrinterObjects.hpp
/// @brief Umbrella header for all printer object definitions.
///
/// This file includes all topic-based printer object headers. Individual
/// components can also be included directly for faster compilation.

// Core objects (extruder, heater_bed, fan, heaters, mcu, stepper_enable, etc.)
#include "tether/klipper/klippy/PrinterObjectsCore.hpp"
// Heater objects (heater_generic, temperature_probe)
#include "tether/klipper/klippy/PrinterObjectsHeaters.hpp"
// Fan objects (temperature_fan, controller_fan, heater_fan, fan_generic)
#include "tether/klipper/klippy/PrinterObjectsFans.hpp"
// Motion objects (skew_correction, input_shaper, pressure_advance, etc.)
#include "tether/klipper/klippy/PrinterObjectsMotion.hpp"
// Probe and bed leveling objects (probe, bltouch, z_tilt, bed_mesh, etc.)
#include "tether/klipper/klippy/PrinterObjectsProbes.hpp"
// Sensor objects (temperature_sensor, filament sensors, load_cell, etc.)
#include "tether/klipper/klippy/PrinterObjectsSensors.hpp"
// LED objects (led, dotstar, neopixel)
#include "tether/klipper/klippy/PrinterObjectsLeds.hpp"
// TMC driver objects (tmc_uart, tmc_driver)
#include "tether/klipper/klippy/PrinterObjectsTmc.hpp"
// Peripheral objects (digital_out, pwm_out, spi, i2c, endstop, trsync)
#include "tether/klipper/klippy/PrinterObjectsPeripherals.hpp"
// Advanced objects (firmware_retraction, exclude_object, z_thermal_adjust)
#include "tether/klipper/klippy/PrinterObjectsAdvanced.hpp"
// Miscellaneous objects (gcode_macro, output_pin, servo, menu, etc.)
#include "tether/klipper/klippy/PrinterObjectsMisc.hpp"
// E2 additional objects (adxl345, delayed_gcode, save_variables, board_pins)
#include "tether/klipper/klippy/PrinterObjectsE2.hpp"
