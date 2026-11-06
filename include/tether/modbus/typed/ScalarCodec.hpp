/**
 * @file ScalarCodec.hpp
 * @brief Register ↔ typed value codec: ValueType, WordOrder, scale.
 *        (plan §10.2-10.4)
 */
#pragma once

#include <cstring>
#include <span>
#include <vector>

#include "tether/modbus/ModbusError.hpp"

namespace tether::modbus {

enum class ValueType : uint8_t {
    U16, I16, U32, I32, U64, I64, F32, F64,
    Bit,            ///< single coil/di element
};

enum class WordOrder : uint8_t {
    BigEndian,      ///< AB CD (default; most common)
    LittleEndian,   ///< DC BA (byte+word swap)
    WordSwap,       ///< CD AB
    ByteSwap,       ///< BA DC
};

constexpr size_t valueSize(ValueType t) {
    switch (t) {
        case ValueType::Bit:
        case ValueType::U16:
        case ValueType::I16: return 1;
        case ValueType::U32:
        case ValueType::I32:
        case ValueType::F32: return 2;
        case ValueType::U64:
        case ValueType::I64:
        case ValueType::F64: return 4;
    }
    return 1;
}

/// Reorder `n` 16-bit registers' bytes per `ord` → big-endian u64 image.
inline uint64_t orderToBe(std::span<const uint16_t> regs, WordOrder ord) {
    uint8_t b[8] = {0};
    for (size_t i = 0; i < regs.size() && i < 4; ++i) {
        b[i * 2]     = uint8_t(regs[i] >> 8);
        b[i * 2 + 1] = uint8_t(regs[i]);
    }
    auto swap2 = [&](int i, int j) {
        std::swap(b[i], b[j]); std::swap(b[i + 1], b[j + 1]);
    };
    const size_t n = std::min<size_t>(regs.size() * 2, 8);
    switch (ord) {
        case WordOrder::BigEndian: break;
        case WordOrder::WordSwap:
            for (size_t i = 0; i + 3 < n; i += 4) swap2(i, i + 2);
            break;
        case WordOrder::ByteSwap:
            for (size_t i = 0; i + 1 < n; i += 2) std::swap(b[i], b[i + 1]);
            break;
        case WordOrder::LittleEndian:
            for (size_t i = 0; i + 3 < n; i += 4) {
                std::swap(b[i], b[i + 3]); std::swap(b[i + 1], b[i + 2]);
            }
            break;
    }
    uint64_t v = 0;
    for (size_t i = 0; i < n; ++i) v = (v << 8) | b[i];
    return v;
}

inline std::vector<uint16_t> orderFromBe(uint64_t v, size_t nregs,
                                         WordOrder ord) {
    uint8_t b[8] = {0};
    const size_t n = nregs * 2;
    for (size_t i = 0; i < n; ++i)
        b[n - 1 - i] = uint8_t(v >> (i * 8));
    auto swap2 = [&](int i, int j) {
        std::swap(b[i], b[j]); std::swap(b[i + 1], b[j + 1]);
    };
    switch (ord) {
        case WordOrder::BigEndian: break;
        case WordOrder::WordSwap:
            for (size_t i = 0; i + 3 < n; i += 4) swap2(i, i + 2);
            break;
        case WordOrder::ByteSwap:
            for (size_t i = 0; i + 1 < n; i += 2) std::swap(b[i], b[i + 1]);
            break;
        case WordOrder::LittleEndian:
            for (size_t i = 0; i + 3 < n; i += 4) {
                std::swap(b[i], b[i + 3]); std::swap(b[i + 1], b[i + 2]);
            }
            break;
    }
    std::vector<uint16_t> out(nregs);
    for (size_t i = 0; i < nregs; ++i)
        out[i] = uint16_t(b[i * 2] << 8 | b[i * 2 + 1]);
    return out;
}

struct ScalarSpec {
    ValueType type = ValueType::U16;
    WordOrder order = WordOrder::BigEndian;
    double    scale = 1.0;       ///< engineering = raw * scale + offset
    double    offset = 0.0;
};

/// Decode regs → engineering double (or raw integer when scale==1).
inline double decodeScalar(std::span<const uint16_t> regs,
                           const ScalarSpec& s) {
    const uint64_t v = orderToBe(regs, s.order);
    double d = 0;
    switch (s.type) {
        case ValueType::U16: d = double(uint16_t(v)); break;
        case ValueType::I16: d = double(int16_t(v)); break;
        case ValueType::U32: d = double(uint32_t(v)); break;
        case ValueType::I32: d = double(int32_t(v)); break;
        case ValueType::U64: d = double(v); break;
        case ValueType::I64: d = double(int64_t(v)); break;
        case ValueType::F32: { float f; uint32_t u = uint32_t(v);
            std::memcpy(&f, &u, 4); d = f; break; }
        case ValueType::F64: { double f; std::memcpy(&f, &v, 8); d = f; break; }
        case ValueType::Bit: d = v ? 1.0 : 0.0; break;
    }
    return d * s.scale + s.offset;
}

inline Expected<std::vector<uint16_t>>
encodeScalar(double eng, const ScalarSpec& s) {
    const double raw = (eng - s.offset) / s.scale;
    uint64_t v = 0;
    switch (s.type) {
        case ValueType::U16:
        case ValueType::Bit:
            v = uint64_t(int64_t(raw)) & 0xFFFF; break;
        case ValueType::I16: v = uint64_t(int64_t(raw)) & 0xFFFF; break;
        case ValueType::U32:
        case ValueType::I32: v = uint64_t(int64_t(raw)) & 0xFFFFFFFF; break;
        case ValueType::U64:
        case ValueType::I64: v = uint64_t(int64_t(raw)); break;
        case ValueType::F32: { float f = float(raw);
            uint32_t u; std::memcpy(&u, &f, 4); v = u; break; }
        case ValueType::F64: { std::memcpy(&v, &raw, 8); break; }
    }
    return orderFromBe(v, valueSize(s.type), s.order);
}

} // namespace tether::modbus
