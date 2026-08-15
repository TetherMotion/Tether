/**
 * @file Peripherals.hpp
 * @brief Umbrella header for all peripheral objects (kept for backward compatibility).
 *
 * This header was previously a single god-object file containing all peripheral
 * definitions. It has been split into focused sub-headers. This umbrella include
 * is retained so that existing consumers that include "Peripherals.hpp" continue
 * to work without modification.
 *
 * Split sub-headers:
 *   - Gpio.hpp       : DigitalOut, PWMOut, DigitalOutProxy, PWMOutProxy, AnalogIn, AnalogInProxy
 *   - Endstop.hpp    : Endstop, EndstopProxy
 *   - Sync.hpp       : TrsyncState, Trsync, TrsyncProxy
 *   - Bus.hpp        : Spi, SpiProxy, I2c, I2cProxy
 *   - Actuators.hpp  : LedColor, Fan, Neopixel
 *   - Sensors.hpp    : FilamentSensor, HallFilamentSensor, PulseCounter, Adxl345, Tsl1401clFilamentSensor
 */

#pragma once

#include "tether/klipper/objects/Actuators.hpp"
#include "tether/klipper/objects/Bus.hpp"
#include "tether/klipper/objects/Endstop.hpp"
#include "tether/klipper/objects/Gpio.hpp"
#include "tether/klipper/objects/Sensors.hpp"
#include "tether/klipper/objects/Sync.hpp"
