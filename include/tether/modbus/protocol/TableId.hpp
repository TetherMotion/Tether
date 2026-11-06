/**
 * @file TableId.hpp
 * @brief The four Modbus data tables (plan §3.1).
 */
#pragma once

#include <cstdint>
#include <string_view>

namespace tether::modbus {

/**@brief The four primary Modbus tables. */
enum class TableId : uint8_t {
    Coil             = 0,  ///< read-write bits   (0xxxx)
    DiscreteInput    = 1,  ///< read-only bits    (1xxxx)
    InputRegister    = 3,  ///< read-only regs    (3xxxx)
    HoldingRegister  = 4,  ///< read-write regs   (4xxxx)
};

constexpr bool isBitTable(TableId t) {
    return t == TableId::Coil || t == TableId::DiscreteInput;
}
constexpr bool isRegisterTable(TableId t) { return !isBitTable(t); }
constexpr bool isWritable(TableId t) {
    return t == TableId::Coil || t == TableId::HoldingRegister;
}
constexpr bool isReadOnly(TableId t) { return !isWritable(t); }

constexpr std::string_view tableName(TableId t) {
    switch (t) {
        case TableId::Coil:            return "coil";
        case TableId::DiscreteInput:   return "di";
        case TableId::InputRegister:   return "in";
        case TableId::HoldingRegister: return "hr";
    }
    return "?";
}

/**@brief PLC-style display base for a table (coil=0xxxx, di=1xxxx…). */
constexpr uint32_t tablePlcBase(TableId t) {
    switch (t) {
        case TableId::Coil:            return 00001;
        case TableId::DiscreteInput:   return 10001;
        case TableId::InputRegister:   return 30001;
        case TableId::HoldingRegister: return 40001;
    }
    return 0;
}

} // namespace tether::modbus
