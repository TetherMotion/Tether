#include "tether/modbus/link/SerialConfig.hpp"

#include <charconv>

namespace tether::modbus {

Expected<Parity> parseParity(std::string_view s) {
    if (s == "none" || s == "n" || s == "N") return Parity::None;
    if (s == "even" || s == "e" || s == "E") return Parity::Even;
    if (s == "odd"  || s == "o" || s == "O") return Parity::Odd;
    return err<Parity>(ModbusError::config("bad parity: " + std::string(s)));
}
Expected<StopBits> parseStopBits(std::string_view s) {
    if (s == "1") return StopBits::One;
    if (s == "2") return StopBits::Two;
    return err<StopBits>(ModbusError::config("bad stop bits: " + std::string(s)));
}

static Expected<uint32_t> toUint(std::string_view s, const char* what) {
    uint32_t v = 0;
    const auto* end = s.data() + s.size();
    auto [ptr, ec] = std::from_chars(s.data(), end, v);
    if (ec != std::errc{} || ptr != end)
        return err<uint32_t>(ModbusError::config(
            std::string("bad ") + what + ": " + std::string(s)));
    return v;
}

Expected<SerialConfig> SerialConfig::parse(std::string_view text) {
    const auto colon = text.find(':');
    if (colon == std::string_view::npos) {
        SerialConfig c;
        c.device = std::string(text);
        return c;
    }
    return parseSettings(text.substr(0, colon), text.substr(colon + 1));
}

Expected<SerialConfig> SerialConfig::parseSettings(std::string_view dev,
                                                   std::string_view s) {
    SerialConfig c;
    c.device = std::string(dev);
    // BAUD[-<bits><parity><stop>]
    const auto dash = s.find('-');
    const auto baudStr = s.substr(0, dash);
    auto baud = toUint(baudStr, "baud");
    if (!baud) return err<SerialConfig>(baud.error());
    c.baud = *baud;
    if (dash != std::string_view::npos) {
        const auto rest = s.substr(dash + 1);
        // e.g. "8E1", "7E1", "8N2", "8O1"
        if (rest.size() != 3)
            return err<SerialConfig>(ModbusError::config(
                "bad serial format: " + std::string(rest)));
        auto bits = toUint(rest.substr(0, 1), "data bits");
        if (!bits || *bits < 5 || *bits > 8)
            return err<SerialConfig>(ModbusError::config("data bits 5..8"));
        c.dataBits = uint8_t(*bits);
        auto p = parseParity(rest.substr(1, 1));
        if (!p) return err<SerialConfig>(p.error());
        c.parity = *p;
        auto sb = parseStopBits(rest.substr(2, 1));
        if (!sb) return err<SerialConfig>(sb.error());
        c.stopBits = *sb;
    }
    return c;
}

std::string SerialConfig::describe() const {
    return device + "@" + std::to_string(baud) + "-" +
           std::to_string(dataBits) + parityChar() +
           (stopBits == StopBits::Two ? "2" : "1");
}

} // namespace tether::modbus
