#include "tether/modbus/typed/AddressNotation.hpp"

#include <charconv>

namespace tether::modbus {
namespace {

Expected<TableId> tableByName(std::string_view s) {
    if (s == "hr" || s == "holding" || s == "4x") return TableId::HoldingRegister;
    if (s == "ir" || s == "input"   || s == "3x") return TableId::InputRegister;
    if (s == "di" || s == "discrete"|| s == "1x") return TableId::DiscreteInput;
    if (s == "co" || s == "coil") return TableId::Coil;
    return err<TableId>(ModbusError::config("bad table: " + std::string(s)));
}
Expected<uint32_t> toU32(std::string_view s, int base = 10) {
    uint32_t v = 0;
    auto [p, ec] = std::from_chars(s.data(), s.data() + s.size(), v, base);
    if (ec != std::errc{} || p != s.data() + s.size())
        return err<uint32_t>(ModbusError::config("bad number: " + std::string(s)));
    return v;
}
Expected<Address> finish(TableId t, uint32_t off, uint8_t bit) {
    if (off > 0xFFFF)
        return err<Address>(ModbusError::config("offset out of range"));
    return Address{t, uint16_t(off), bit};
}
} // namespace

Expected<Address> parseAddress(std::string_view text) {
    // split optional ".bit" suffix
    uint8_t bit = 0xFF;
    if (const auto dot = text.find('.'); dot != std::string_view::npos) {
        auto b = toU32(text.substr(dot + 1));
        if (!b || *b > 15)
            return err<Address>(ModbusError::config("bad bit index"));
        bit = uint8_t(*b);
        text = text.substr(0, dot);
    }
    if (text.empty())
        return err<Address>(ModbusError::config("empty address"));

    // table:name form
    if (const auto colon = text.find(':'); colon != std::string_view::npos) {
        auto t = tableByName(text.substr(0, colon));
        if (!t) return err<Address>(t.error());
        auto n = toU32(text.substr(colon + 1));
        if (!n) return err<Address>(n.error());
        return finish(*t, *n, bit);
    }
    // hex "0x1F" (checked before Nx so "0x17" is hex, not a table form)
    if (text.size() > 2 && (text.substr(0, 2) == "0x" ||
                            text.substr(0, 2) == "0X")) {
        auto n = toU32(text.substr(2), 16);
        if (!n) return err<Address>(n.error());
        return finish(TableId::HoldingRegister, *n, bit);
    }
    // Nx<offset> form (4x17 → hr:17)
    if (text.size() >= 3 && (text[1] == 'x' || text[1] == 'X')) {
        auto t = tableByName(text.substr(0, 2));
        if (!t) return err<Address>(t.error());
        auto n = toU32(text.substr(2));
        if (!n) return err<Address>(n.error());
        return finish(*t, *n, bit);
    }
    // PLC 5/6-digit: first digit selects table
    const bool allDigits = text.find_first_not_of("0123456789") ==
                           std::string_view::npos;
    if (allDigits && text.size() >= 5) {
        const char lead = text[0];
        TableId t;
        switch (lead) {
            case '0': t = TableId::Coil; break;
            case '1': t = TableId::DiscreteInput; break;
            case '3': t = TableId::InputRegister; break;
            case '4': t = TableId::HoldingRegister; break;
            default:
                return err<Address>(
                    ModbusError::config("bad PLC prefix"));
        }
        auto n = toU32(text.substr(1));
        if (!n) return err<Address>(n.error());
        // PLC numbers are 1-based → offset = n - 1
        return *n == 0 ? err<Address>(ModbusError::config("PLC addr 0"))
                       : finish(t, *n - 1, bit);
    }
    if (allDigits) {
        auto n = toU32(text);
        if (!n) return err<Address>(n.error());
        return finish(TableId::HoldingRegister, *n, bit);
    }
    return err<Address>(ModbusError::config("bad address: " + std::string(text)));
}

std::string formatAddress(const Address& a) {
    std::string s = std::string(tableName(a.table)) + ":" +
                    std::to_string(a.offset);
    if (a.hasBit()) s += "." + std::to_string(a.bit);
    return s;
}
std::string formatPlcAddress(const Address& a) {
    const uint32_t plc = tablePlcBase(a.table) + a.offset;
    std::string s = std::to_string(plc);
    if (a.hasBit()) s += "." + std::to_string(a.bit);
    return s;
}

} // namespace tether::modbus
