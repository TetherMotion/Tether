/**
 * @file TypedRef.hpp
 * @brief Typed endpoint handle: Address + ScalarSpec → read()/write()
 *        engineering values through a ModbusClient (plan §10.5).
 */
#pragma once

#include <future>

#include "tether/modbus/client/ModbusClient.hpp"
#include "tether/modbus/typed/AddressNotation.hpp"
#include "tether/modbus/typed/ScalarCodec.hpp"

namespace tether::modbus {

class TypedRef {
public:
    TypedRef(ModbusClient& client, Address addr, ScalarSpec spec)
        : client_(client), addr_(addr), spec_(spec) {}

    Expected<double> read(const TransactionOptions& o = {}) const {
        const size_t n = valueSize(spec_.type);
        if (spec_.type == ValueType::Bit) {
            auto bits = addr_.table == TableId::Coil
                ? client_.readCoils(addr_.offset, 1, o)
                : client_.readDiscreteInputs(addr_.offset, 1, o);
            if (!bits) return err<double>(bits.error());
            return bits->front() ? 1.0 : 0.0;
        }
        auto regs = addr_.table == TableId::InputRegister
            ? client_.readInput(addr_.offset, uint16_t(n), o)
            : client_.readHolding(addr_.offset, uint16_t(n), o);
        if (!regs) return err<double>(regs.error());
        return decodeScalar(*regs, spec_);
    }

    Expected<void> write(double eng, const TransactionOptions& o = {}) const {
        if (spec_.type == ValueType::Bit)
            return client_.writeCoil(addr_.offset, eng != 0.0, o);
        auto regs = encodeScalar(eng, spec_);
        if (!regs) return err<void>(regs.error());
        if (regs->size() == 1)
            return client_.writeReg(addr_.offset, regs->front(), o);
        return client_.writeRegs(addr_.offset, *regs, o);
    }

    /// Read the bit sub-field when addr has a bit index ("40001.3").
    Expected<bool> readBit(const TransactionOptions& o = {}) const {
        if (!addr_.hasBit())
            return err<bool>(ModbusError::config("no bit index"));
        auto v = read(o);
        if (!v) return err<bool>(v.error());
        const auto regs = encodeScalar(*v, ScalarSpec{spec_.type,
                                     spec_.order, 1.0, 0.0});
        if (!regs || regs->empty())
            return err<bool>(ModbusError::codec("bit decode"));
        return ((*regs)[0] >> addr_.bit) & 1;
    }

    // ---- std::future async twins -----------------------------------------
    std::future<Expected<double>>
    readAsync(TransactionOptions o = {}) const {
        return std::async(std::launch::async,
                          [*this, o = std::move(o)] { return read(o); });
    }
    std::future<Expected<void>>
    writeAsync(double eng, TransactionOptions o = {}) const {
        return std::async(std::launch::async,
                          [*this, eng, o = std::move(o)] {
                              return write(eng, o);
                          });
    }
    std::future<Expected<bool>>
    readBitAsync(TransactionOptions o = {}) const {
        return std::async(std::launch::async,
                          [*this, o = std::move(o)] { return readBit(o); });
    }

    const Address&    address() const { return addr_; }
    const ScalarSpec& spec() const { return spec_; }

private:
    ModbusClient& client_;
    Address       addr_;
    ScalarSpec    spec_;
};

/**
 * @brief Coalesce typed register reads into minimal Modbus blocks.
 *
 * Insert addresses, then `blocks()` returns merged read ranges (gap ≤
 * `maxGap`, qty ≤ `maxQty`) — plan §10.7.
 */
class BlockPlan {
public:
    explicit BlockPlan(uint16_t maxGap = 4, uint16_t maxQty = 125)
        : maxGap_(maxGap), maxQty_(maxQty) {}

    void add(TableId t, uint16_t off, uint16_t n = 1) {
        items_.push_back({t, off, n});
    }
    void add(const Address& a, const ScalarSpec& s) {
        add(a.table, a.offset, uint16_t(valueSize(s.type)));
    }

    /// Merged blocks sorted by (table, offset).
    std::vector<Range> blocks() const {
        auto items = items_;
        std::sort(items.begin(), items.end(), [](const Item& a,
                                                 const Item& b) {
            if (a.table != b.table)
                return int(a.table) < int(b.table);
            return a.off < b.off;
        });
        std::vector<Range> out;
        for (const auto& it : items) {
            if (!out.empty() && out.back().table == it.table &&
                uint32_t(it.off) <= out.back().end() + maxGap_ &&
                (uint32_t(it.off) + it.n) - out.back().offset <= maxQty_) {
                const uint32_t end = std::max<uint32_t>(out.back().end(),
                                                        it.off + it.n);
                out.back().qty = uint16_t(end - out.back().offset);
            } else {
                out.push_back(Range{it.table, it.off, it.n});
            }
        }
        return out;
    }

private:
    struct Item { TableId table; uint16_t off, n; };
    uint16_t maxGap_, maxQty_;
    std::vector<Item> items_;
};

} // namespace tether::modbus
