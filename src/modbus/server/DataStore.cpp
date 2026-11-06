#include "tether/modbus/server/DataStore.hpp"

#include "tether/modbus/protocol/Constants.hpp"

namespace tether::modbus {
namespace {
/// TableId values are 0,1,3,4 → dense index 0..3.
constexpr size_t ti(TableId t) {
    switch (t) {
        case TableId::Coil:            return 0;
        case TableId::DiscreteInput:   return 1;
        case TableId::InputRegister:   return 2;
        case TableId::HoldingRegister: return 3;
    }
    return 0;
}
Expected<void> illegalAddr() {
    return err<void>(exceptionToError(
        uint8_t(ExceptionCode::IllegalDataAddress)));
}
} // namespace

Expected<void> DataStore::declare(TableId t, uint16_t start, uint16_t count) {
    if (count == 0 || uint32_t(start) + count > kAddressSpace)
        return err<void>(ModbusError::config("extent out of range"));
    std::lock_guard g(m_);
    auto& v = tables_[ti(t)];
    for (const auto& e : v)
        if (start < e.start + e.count && e.start < start + count)
            return err<void>(ModbusError::config("extent overlap"));
    Extent e;
    e.start = start; e.count = count;
    if (isBitTable(t)) e.bits.assign(count, 0);
    else               e.regs.assign(count, 0);
    v.push_back(std::move(e));
    return {};
}

int DataStore::findExtent(TableId t, uint16_t addr, uint16_t count) const {
    const auto& v = tables_[ti(t)];
    for (size_t i = 0; i < v.size(); ++i)
        if (addr >= v[i].start && uint32_t(addr) + count <=
            uint32_t(v[i].start) + v[i].count)
            return int(i);
    return -1;
}

bool DataStore::covers(TableId t, uint16_t a, uint16_t n) const {
    std::lock_guard g(m_);
    return findExtent(t, a, n) >= 0;
}

Expected<void> DataStore::read(TableId t, uint16_t a, uint16_t n,
                               std::span<uint16_t> regs,
                               std::span<uint8_t> bits) {
    std::lock_guard g(m_);
    const int i = findExtent(t, a, n);
    if (i < 0) return illegalAddr();
    const auto& e = tables_[ti(t)][size_t(i)];
    const size_t off = a - e.start;
    if (isBitTable(t)) {
        if (bits.size() < n)
            return err<void>(ModbusError::codec("bits span too small"));
        for (uint16_t k = 0; k < n; ++k) bits[k] = e.bits[off + k];
    } else {
        if (regs.size() < n)
            return err<void>(ModbusError::codec("regs span too small"));
        for (uint16_t k = 0; k < n; ++k) regs[k] = e.regs[off + k];
    }
    return {};
}

Expected<void> DataStore::write(TableId t, uint16_t a, uint16_t n,
                                std::span<const uint16_t> regs,
                                std::span<const uint8_t> bits) {
    std::lock_guard g(m_);
    const int i = findExtent(t, a, n);
    if (i < 0) return illegalAddr();
    if (hook_ && !hook_(t, a, n, regs, bits))
        return err<void>(exceptionToError(uint8_t(ExceptionCode::IllegalDataValue)));
    auto& e = tables_[ti(t)][size_t(i)];
    const size_t off = a - e.start;
    if (isBitTable(t)) {
        if (bits.size() < n)
            return err<void>(ModbusError::codec("bits span too small"));
        for (uint16_t k = 0; k < n; ++k)
            e.bits[off + k] = bits[k] ? 1 : 0;
    } else {
        if (regs.size() < n)
            return err<void>(ModbusError::codec("regs span too small"));
        for (uint16_t k = 0; k < n; ++k) e.regs[off + k] = regs[k];
    }
    return {};
}

Expected<uint16_t> DataStore::getReg(TableId t, uint16_t a) {
    uint16_t v = 0;
    if (auto e = read(t, a, 1, std::span(&v, 1), {}); !e)
        return err<uint16_t>(e.error());
    return v;
}
Expected<void> DataStore::setReg(TableId t, uint16_t a, uint16_t v) {
    return write(t, a, 1, std::span(&v, 1), {});
}
Expected<bool> DataStore::getBit(TableId t, uint16_t a) {
    uint8_t v = 0;
    if (auto e = read(t, a, 1, {}, std::span(&v, 1)); !e)
        return err<bool>(e.error());
    return v != 0;
}
Expected<void> DataStore::setBit(TableId t, uint16_t a, bool v) {
    const uint8_t b = v ? 1 : 0;
    return write(t, a, 1, {}, std::span(&b, 1));
}

std::vector<std::pair<TableId, std::pair<uint16_t, uint16_t>>>
DataStore::extents() const {
    std::lock_guard g(m_);
    std::vector<std::pair<TableId, std::pair<uint16_t, uint16_t>>> out;
    for (size_t t = 0; t < 4; ++t)
        for (const auto& e : tables_[t])
            out.push_back({TableId(t), {e.start, e.count}});
    return out;
}

} // namespace tether::modbus
