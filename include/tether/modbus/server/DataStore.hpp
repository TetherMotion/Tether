/**
 * @file DataStore.hpp
 * @brief Server-side data model: declared extents, non-overlapping
 *        (plan §9.2-9.3).
 */
#pragma once

#include <array>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <span>
#include <vector>

#include "tether/modbus/ModbusError.hpp"
#include "tether/modbus/protocol/ExceptionCode.hpp"
#include "tether/modbus/protocol/TableId.hpp"

namespace tether::modbus {

/// Read/write access against a single table region.
struct RegionAccess {
    TableId  table;
    uint16_t start = 0;
    uint16_t count = 0;
    std::span<uint16_t>       regs;   ///< reg tables
    std::span<uint8_t>        bits;   ///< bit tables (1 byte per bit)
};

class IDataModel {
public:
    virtual ~IDataModel() = default;
    /// Read `count` elements at `addr` → out (regs) or bits.
    virtual Expected<void> read(TableId, uint16_t addr, uint16_t count,
                                std::span<uint16_t> regs,
                                std::span<uint8_t> bits) = 0;
    virtual Expected<void> write(TableId, uint16_t addr, uint16_t count,
                                 std::span<const uint16_t> regs,
                                 std::span<const uint8_t> bits) = 0;
    virtual bool covers(TableId, uint16_t addr, uint16_t count) const = 0;
};

/**
 * @brief Range-map store: per-table declared extents backed by vectors.
 *
 * Extents are declared up-front and non-overlapping; every access is
 * bounds-checked against them (undisclosed addresses → IllegalAddress).
 */
class DataStore final : public IDataModel {
public:
    /// Declare an extent; fails on overlap.
    Expected<void> declare(TableId t, uint16_t start, uint16_t count);
    bool covers(TableId t, uint16_t addr, uint16_t count) const override;
    Expected<void> read(TableId t, uint16_t addr, uint16_t count,
                        std::span<uint16_t> regs,
                        std::span<uint8_t> bits) override;
    Expected<void> write(TableId t, uint16_t addr, uint16_t count,
                         std::span<const uint16_t> regs,
                         std::span<const uint8_t> bits) override;

    /// Convenience: direct element access for app/sim code.
    Expected<uint16_t> getReg(TableId t, uint16_t addr);
    Expected<void>     setReg(TableId t, uint16_t addr, uint16_t v);
    Expected<bool>     getBit(TableId t, uint16_t addr);
    Expected<void>     setBit(TableId t, uint16_t addr, bool v);

    /// Optional write hook (audit / gate). Return false → IllegalDataValue.
    using WriteHook = std::function<bool(TableId, uint16_t, uint16_t,
                                         std::span<const uint16_t>,
                                         std::span<const uint8_t>)>;
    void setWriteHook(WriteHook h) { hook_ = std::move(h); }

    /// All declared extents (diagnostics/UI).
    std::vector<std::pair<TableId, std::pair<uint16_t, uint16_t>>>
    extents() const;

private:
    struct Extent {
        uint16_t start = 0, count = 0;
        std::vector<uint16_t> regs;
        std::vector<uint8_t>  bits;
    };
    /// Find extent covering [addr, addr+count) → index or -1.
    int findExtent(TableId t, uint16_t addr, uint16_t count) const;

    std::array<std::vector<Extent>, 4> tables_{};
    mutable std::mutex m_;
    WriteHook hook_;
};

/**
 * @brief Callback-backed model (device firmware style).
 * Each access is delegated to user functions.
 */
class CallbackStore final : public IDataModel {
public:
    using ReadFn  = std::function<Expected<void>(
        uint16_t addr, uint16_t count,
        std::span<uint16_t>, std::span<uint8_t>)>;
    using WriteFn = std::function<Expected<void>(
        uint16_t addr, uint16_t count,
        std::span<const uint16_t>, std::span<const uint8_t>)>;
    using CoverFn = std::function<bool(uint16_t, uint16_t)>;

    void attach(TableId t, ReadFn r, WriteFn w, CoverFn c) {
        fns_[t] = {std::move(r), std::move(w), std::move(c)};
    }
    Expected<void> read(TableId t, uint16_t a, uint16_t n,
                        std::span<uint16_t> r,
                        std::span<uint8_t> b) override {
        auto it = fns_.find(t);
        if (it == fns_.end() || !it->second.r)
            return err<void>(exceptionToError(uint8_t(ExceptionCode::IllegalDataAddress)));
        return it->second.r(a, n, r, b);
    }
    Expected<void> write(TableId t, uint16_t a, uint16_t n,
                         std::span<const uint16_t> r,
                         std::span<const uint8_t> b) override {
        auto it = fns_.find(t);
        if (it == fns_.end() || !it->second.w)
            return err<void>(exceptionToError(uint8_t(ExceptionCode::IllegalDataAddress)));
        return it->second.w(a, n, r, b);
    }
    bool covers(TableId t, uint16_t a, uint16_t n) const override {
        auto it = fns_.find(t);
        return it != fns_.end() && it->second.c && it->second.c(a, n);
    }
private:
    struct Fns { ReadFn r; WriteFn w; CoverFn c; };
    std::map<TableId, Fns> fns_;
};

} // namespace tether::modbus
