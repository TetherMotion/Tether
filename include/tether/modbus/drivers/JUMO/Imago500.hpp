/**
 * @file Imago500.hpp
 * @brief JUMO Imago 500 Modbus/RTU driver — umbrella include.
 *
 *  - Imago500Map.hpp: complete grouped register description (all
 *    datasheet §5 address tables) + `imago500Profile()` DeviceProfile
 *    for name-based exposure through BoundDevice / IoBridge.
 *  - Imago500Device.hpp: `JumoImago500` typed driver with semantic
 *    methods for every group.
 */
#pragma once

#include "tether/modbus/drivers/JUMO/Imago500Map.hpp"
#include "tether/modbus/drivers/JUMO/Imago500Device.hpp"
