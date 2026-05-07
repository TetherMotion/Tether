/**
 * @file SLIP.hpp
 * @author Uli Köhler <github@techoverflow.net>
 * @version 1.0
 * @date 2025-08-19
 * 
 * Common definitions and functions for working with SLIP data
 * 
 * @copyright Copyright (C) 2022..2025 Uli Köhler
 */
#pragma once
#include <cstdint>

namespace SLIPStream {

// SLIP special character definitions (namespaced, without SLIP_ prefix)
inline constexpr uint8_t END    = 0xC0; // End of packet character
inline constexpr uint8_t ESC    = 0xDB; // Escape character
inline constexpr uint8_t ESCEND = 0xDC; // Escaped END data byte
inline constexpr uint8_t ESCESC = 0xDD; // Escaped ESC data byte

} // namespace SLIPStream
