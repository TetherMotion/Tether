/**
 * @file SerialConfig.hpp
 * @brief Serial port configuration + "dev:baud-bitsParityStop" parsing.
 */
#pragma once

#include <cstdint>
#include <string>
#include <string_view>

#include "tether/modbus/ModbusError.hpp"

namespace tether::modbus {

enum class Parity   : uint8_t { None, Even, Odd };
enum class StopBits : uint8_t { One = 1, Two = 2 };
enum class FlowControl : uint8_t { None, RtsCts, XonXoff };

struct SerialConfig {
    std::string  device = "/dev/ttyUSB0";
    uint32_t     baud = 19200;
    uint8_t      dataBits = 8;
    Parity       parity = Parity::Even;
    StopBits     stopBits = StopBits::One;
    FlowControl  flow = FlowControl::None;

    /// Parse "PATH:BAUD[-BITS<P|parity><S|stop>]" e.g.
    /// "/dev/ttyUSB0:19200-8E1", "/dev/ttyS1:9600-8N2", "ttyUSB1:9600".
    /// Also accepts bare "PATH" (defaults applied).
    static Expected<SerialConfig> parse(std::string_view text);

    /// "9600-8E1" style suffix only.
    static Expected<SerialConfig> parseSettings(std::string_view dev,
                                                std::string_view settings);

    std::string describe() const;  // "/dev/ttyUSB0@19200-8E1"
    char parityChar() const {
        return parity == Parity::Even ? 'E' : parity == Parity::Odd ? 'O' : 'N';
    }
    /// Bits per character on the wire (start+data+parity+stop).
    uint8_t bitsPerChar() const {
        return uint8_t(1 + dataBits + (parity == Parity::None ? 0 : 1) +
                       (stopBits == StopBits::Two ? 2 : 1));
    }
};

Expected<Parity>   parseParity(std::string_view);
Expected<StopBits> parseStopBits(std::string_view);

} // namespace tether::modbus
