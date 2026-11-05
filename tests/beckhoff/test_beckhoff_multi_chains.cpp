/**
 * @file test_beckhoff_multi_chains.cpp
 * @brief Coverage for the Multi*Terminal chain classes — flat channel
 *        indexing, module sorting, capacity limits, attach/detect paths
 *        and typed module access — using fake terminals and fabricated
 *        discovery results (no bus I/O).
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <vector>

#include "tether/Beckhoff/EL1014.hpp"
#include "tether/Beckhoff/EL2004.hpp"
#include "tether/Beckhoff/MultiAnalogInputTerminal.hpp"
#include "tether/Beckhoff/MultiAnalogOutputTerminal.hpp"
#include "tether/Beckhoff/MultiCombinedIoTerminal.hpp"
#include "tether/Beckhoff/MultiEL1014.hpp"
#include "tether/Beckhoff/MultiEL2004.hpp"
#include "tether/Beckhoff/MultiInputTerminal.hpp"
#include "tether/Beckhoff/MultiOutputTerminal.hpp"
#include "tether/ethercat/Master.hpp"

using namespace EtherCAT;
using namespace EtherCAT::Beckhoff;

namespace {

// ---------------------------------------------------------------------------
// Fake chained devices
// ---------------------------------------------------------------------------

class FakeAnalogIn : public IAnalogInputTerminal {
public:
    FakeAnalogIn(uint16_t idx, size_t channels) : idx_(idx), ch_(channels) {}
    uint16_t slaveIndex() const override { return idx_; }
    size_t channelCount() const override { return ch_; }
    const char* deviceName() const override { return "FakeAin"; }
    int32_t value(size_t ch) const override {
        return static_cast<int32_t>(idx_) * 1000 + static_cast<int32_t>(ch);
    }
    uint16_t status(size_t) const override { return 0xCAFE; }
    bool hasStatus(size_t) const override { return true; }
    Result<> prepareForLogicalExchange() override { return {}; }
    Result<> mapLogicalAndEnterSafeOp() override { return {}; }
    Result<> requestOp(int) override { return {}; }
private:
    uint16_t idx_;
    size_t ch_;
};

class FakeAnalogOut : public IAnalogOutputTerminal {
public:
    FakeAnalogOut(uint16_t idx, size_t channels)
        : idx_(idx), ch_(channels), vals_(channels, 0) {}
    uint16_t slaveIndex() const override { return idx_; }
    size_t channelCount() const override { return ch_; }
    const char* deviceName() const override { return "FakeAout"; }
    void setValue(size_t ch, int32_t v) override {
        if (ch < vals_.size()) vals_[ch] = v;
    }
    int32_t value(size_t ch) const override {
        return ch < vals_.size() ? vals_[ch] : 0;
    }
    void allZero() override { std::fill(vals_.begin(), vals_.end(), 0); }
    Result<> prepareForLogicalExchange() override { return {}; }
    Result<> mapLogicalAndEnterSafeOp() override { return {}; }
    Result<> requestOp(int) override { return {}; }
private:
    uint16_t idx_;
    size_t ch_;
    std::vector<int32_t> vals_;
};

class FakeDigitalIn : public IInputTerminal {
public:
    FakeDigitalIn(uint16_t idx, size_t bits, uint64_t pattern = 0)
        : idx_(idx), bits_(bits), pattern_(pattern) {}
    uint16_t slaveIndex() const override { return idx_; }
    size_t bitCount() const override { return bits_; }
    const char* deviceName() const override { return "FakeDin"; }
    bool bit(size_t b) const override {
        return b < 64 && ((pattern_ >> b) & 1u);
    }
    uint64_t bits() const override { return pattern_; }
    Result<> prepareForLogicalExchange() override { return {}; }
    Result<> mapLogicalAndEnterSafeOp() override { return {}; }
    Result<> requestOp(int) override { return {}; }
private:
    uint16_t idx_;
    size_t bits_;
    uint64_t pattern_;
};

class FakeDigitalOut : public IOutputTerminal {
public:
    FakeDigitalOut(uint16_t idx, size_t bits) : idx_(idx), bits_(bits) {}
    uint16_t slaveIndex() const override { return idx_; }
    size_t bitCount() const override { return bits_; }
    const char* deviceName() const override { return "FakeDout"; }
    void setBit(size_t b, bool on) override {
        if (b >= 64) return;
        if (on) state_ |= (uint64_t{1} << b);
        else state_ &= ~(uint64_t{1} << b);
    }
    bool bit(size_t b) const override { return b < 64 && ((state_ >> b) & 1u); }
    void setBits(uint64_t v) override { state_ = v; }
    uint64_t bits() const override { return state_; }
    void allOff() override { state_ = 0; }
    Result<> prepareForLogicalExchange() override { return {}; }
    Result<> mapLogicalAndEnterSafeOp() override { return {}; }
    Result<> requestOp(int) override { return {}; }
private:
    uint16_t idx_;
    size_t bits_;
    uint64_t state_ = 0;
};

class FakeCombinedIo : public ICombinedIoTerminal {
public:
    FakeCombinedIo(uint16_t idx, size_t ins, size_t outs, uint64_t in_pat = 0)
        : idx_(idx), ins_(ins), outs_(outs), in_pat_(in_pat) {}
    uint16_t slaveIndex() const override { return idx_; }
    size_t inputChannelCount() const override { return ins_; }
    size_t outputChannelCount() const override { return outs_; }
    const char* deviceName() const override { return "FakeIo"; }
    bool input(size_t i) const override {
        return i < 64 && ((in_pat_ >> i) & 1u);
    }
    void setOutput(size_t i, bool on) override {
        if (i >= 64) return;
        if (on) out_state_ |= (uint64_t{1} << i);
        else out_state_ &= ~(uint64_t{1} << i);
    }
    bool output(size_t i) const override {
        return i < 64 && ((out_state_ >> i) & 1u);
    }
    void allOutputsOff() override { out_state_ = 0; }
    Result<> prepareForLogicalExchange() override { return {}; }
    Result<> mapLogicalAndEnterSafeOp() override { return {}; }
    Result<> requestOp(int) override { return {}; }
private:
    uint16_t idx_;
    size_t ins_, outs_;
    uint64_t in_pat_;
    uint64_t out_state_ = 0;
};

DiscoveredSlave fakeSlave(uint16_t index, uint32_t vendor, uint32_t product) {
    DiscoveredSlave s;
    s.index = index;
    s.vendor_id = vendor;
    s.product_code = product;
    return s;
}

} // anonymous namespace

// ============================================================================
// MultiAnalogInputTerminal
// ============================================================================

TEST(MultiAnalogInput, FlatChannelIndexingAndSorting) {
    Master master;
    MultiAnalogInputTerminal<> chain(master);
    ASSERT_TRUE(chain.attach(std::make_unique<FakeAnalogIn>(7, 1)));
    ASSERT_TRUE(chain.attach(std::make_unique<FakeAnalogIn>(3, 2)));
    ASSERT_TRUE(chain.attach(std::make_unique<FakeAnalogIn>(5, 1)));

    EXPECT_EQ(chain.moduleCount(), 3u);
    EXPECT_EQ(chain.channelCount(), 4u);
    // Sorted by slave index: 3 (2ch), 5 (1ch), 7 (1ch).
    EXPECT_EQ(chain.slaveIndex(0), 3);
    EXPECT_EQ(chain.slaveIndex(1), 5);
    EXPECT_EQ(chain.slaveIndex(2), 7);
    EXPECT_EQ(chain.channelOffset(0), 0u);
    EXPECT_EQ(chain.channelOffset(1), 2u);
    EXPECT_EQ(chain.channelOffset(2), 3u);

    EXPECT_EQ(chain.value(0), 3000);
    EXPECT_EQ(chain.value(2), 5000);
    EXPECT_EQ(chain.value(3), 7000);
    EXPECT_EQ(chain.status(1), 0xCAFE);
    EXPECT_TRUE(chain.hasStatus(0));
    EXPECT_EQ(&chain.module(1), &chain.module(1));
}

TEST(MultiAnalogInput, OutOfRangeAndEdgeCases) {
    Master master;
    MultiAnalogInputTerminal<> chain(master);
    EXPECT_FALSE(chain.attach(nullptr).has_value());
    ASSERT_TRUE(chain.attach(std::make_unique<FakeAnalogIn>(1, 1)));

    EXPECT_THROW((void)chain.value(1), std::out_of_range);
    EXPECT_THROW((void)chain.status(5), std::out_of_range);
    EXPECT_THROW((void)chain.hasStatus(5), std::out_of_range);
    EXPECT_THROW((void)chain.module(2), std::out_of_range);
    EXPECT_EQ(chain.lastErrorModule(), SIZE_MAX);
    EXPECT_EQ(chain.baseLogicalAddress(), 0u);
    EXPECT_EQ(chain.pdoManager(), nullptr);
    EXPECT_EQ(chain.logicalAddressManager(), nullptr);
    EXPECT_GT(chain.knownDevices().size(), 0u);
}

TEST(MultiAnalogInput, CapacityLimitAndEmptyConfigure) {
    Master master;
    MultiAnalogInputTerminal<3> chain(master);
    ASSERT_TRUE(chain.attach(std::make_unique<FakeAnalogIn>(1, 2)));
    ASSERT_TRUE(chain.attach(std::make_unique<FakeAnalogIn>(2, 2)));
    EXPECT_EQ(chain.channelCount(), 2u);
    EXPECT_EQ(chain.moduleCount(), 1u);

    Master master2;
    MultiAnalogInputTerminal<> empty(master2);
    auto r = empty.configure();
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), Error::NoDeviceFound);
}

TEST(MultiAnalogInput, DetectWithFabricatedScan) {
    Master master;
    MultiAnalogInputTerminal<> chain(master);
    const auto& devs = chain.knownDevices();
    std::vector<DiscoveredSlave> scan{
        fakeSlave(0, devs[0].vendor_id, devs[0].product_code),
        fakeSlave(1, 0xDEAD, 0xBEEF),  // unknown device — skipped
    };
    auto r = chain.detect(std::span<const DiscoveredSlave>(scan));
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(*r, 1u);
    EXPECT_EQ(chain.moduleCount(), 1u);

    // Also exercise the DeviceIdentity -> matcher conversion path.
    Master master2;
    MultiAnalogInputTerminal<> chain2(master2);
    auto r2 = chain2.detect(std::span<const DeviceIdentity>(devs.data(), 1),
                            std::span<const DiscoveredSlave>(scan));
    ASSERT_TRUE(r2.has_value());
    EXPECT_EQ(*r2, 1u);
}

// ============================================================================
// MultiAnalogOutputTerminal
// ============================================================================

TEST(MultiAnalogOutput, FlatSetAndReadback) {
    Master master;
    MultiAnalogOutputTerminal<> chain(master);
    ASSERT_TRUE(chain.attach(std::make_unique<FakeAnalogOut>(4, 2)));
    ASSERT_TRUE(chain.attach(std::make_unique<FakeAnalogOut>(1, 2)));

    // Sorted: slave1 -> channels 0-1, slave4 -> 2-3.
    chain.setValue(0, 111);
    chain.setValue(3, 444);
    EXPECT_EQ(chain.value(0), 111);
    EXPECT_EQ(chain.value(3), 444);
    EXPECT_EQ(chain.value(1), 0);

    chain.allZero();
    EXPECT_EQ(chain.value(0), 0);
    EXPECT_EQ(chain.value(3), 0);

    EXPECT_THROW(chain.setValue(9, 1), std::out_of_range);
    EXPECT_THROW((void)chain.value(9), std::out_of_range);
    EXPECT_THROW((void)chain.module(5), std::out_of_range);
    EXPECT_EQ(chain.channelOffset(1), 2u);
    EXPECT_GT(chain.knownDevices().size(), 0u);
}

TEST(MultiAnalogOutput, DetectWithCustomMatcher) {
    Master master;
    MultiAnalogOutputTerminal<> chain(master);
    DeviceIdentity fake_id{0x1234, 0x5678, 16, "Fake"};
    typename MultiAnalogOutputTerminal<>::DeviceMatcher m{
        fake_id,
        [](Master&, const DiscoveredSlave& s)
            -> std::unique_ptr<IAnalogOutputTerminal> {
            return std::make_unique<FakeAnalogOut>(s.index, 2);
        }};
    std::vector<DiscoveredSlave> scan{
        fakeSlave(0, 0x1234, 0x5678),
        fakeSlave(1, 0x00000002, 0x07D43052),  // EL2004 — not matched
    };
    const typename MultiAnalogOutputTerminal<>::DeviceMatcher ms[]{m};
    auto r = chain.detect(std::span<const decltype(m)>(ms, 1),
                          std::span<const DiscoveredSlave>(scan));
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(*r, 1u);
    EXPECT_EQ(chain.channelCount(), 2u);
}

// ============================================================================
// MultiInputTerminal / MultiOutputTerminal
// ============================================================================

TEST(MultiDigitalInput, FlatBitsAcrossModules) {
    Master master;
    MultiInputTerminal<> chain(master);
    ASSERT_TRUE(chain.attach(std::make_unique<FakeDigitalIn>(5, 4, 0b1010)));
    ASSERT_TRUE(chain.attach(std::make_unique<FakeDigitalIn>(1, 4, 0b0011)));

    EXPECT_EQ(chain.moduleCount(), 2u);
    EXPECT_EQ(chain.channelCount(), 8u);
    // Sorted: slave1 bits 0-3 (0b0011), slave5 bits 4-7 (0b1010).
    EXPECT_TRUE(chain.bit(0));
    EXPECT_TRUE(chain.bit(1));
    EXPECT_FALSE(chain.bit(2));
    EXPECT_TRUE(chain.bit(5));
    EXPECT_TRUE(chain.bit(7));
    EXPECT_FALSE(chain.bit(4));

    auto b = chain.bits();
    EXPECT_TRUE(b[0] && b[1]);
    EXPECT_FALSE(b[2]);
    EXPECT_TRUE(b[5] && b[7]);
    EXPECT_THROW((void)chain.bit(8), std::out_of_range);
    EXPECT_EQ(chain.slaveIndex(0), 1);
    EXPECT_GT(chain.knownDevices().size(), 0u);
}

TEST(MultiDigitalOutput, SetBitsAndAllOnOff) {
    Master master;
    MultiOutputTerminal<> chain(master);
    ASSERT_TRUE(chain.attach(std::make_unique<FakeDigitalOut>(9, 4)));
    ASSERT_TRUE(chain.attach(std::make_unique<FakeDigitalOut>(2, 4)));

    // Sorted: slave2 bits 0-3, slave9 bits 4-7.
    chain.setBit(6, true);
    EXPECT_TRUE(chain.bit(6));
    EXPECT_FALSE(chain.bit(0));

    typename MultiOutputTerminal<>::Bitset bs;
    bs[0] = 1;
    bs[7] = 1;
    chain.setBits(bs);
    EXPECT_TRUE(chain.bit(0));
    EXPECT_TRUE(chain.bit(7));
    EXPECT_FALSE(chain.bit(6));

    chain.allOn();
    for (size_t i = 0; i < chain.channelCount(); ++i)
        EXPECT_TRUE(chain.bit(i)) << i;

    chain.allOff();
    for (size_t i = 0; i < chain.channelCount(); ++i)
        EXPECT_FALSE(chain.bit(i)) << i;

    EXPECT_THROW(chain.setBit(99, true), std::out_of_range);
    EXPECT_THROW((void)chain.bit(99), std::out_of_range);
    EXPECT_GT(chain.knownDevices().size(), 0u);
}

// ============================================================================
// MultiCombinedIoTerminal
// ============================================================================

TEST(MultiCombinedIo, FlatInOutIndexing) {
    Master master;
    MultiCombinedIoTerminal<> chain(master);
    ASSERT_TRUE(chain.attach(
        std::make_unique<FakeCombinedIo>(6, 2, 2, 0b10)));
    ASSERT_TRUE(chain.attach(
        std::make_unique<FakeCombinedIo>(1, 2, 2, 0b01)));

    // Sorted: slave1 ins 0-1 / outs 0-1, slave6 ins 2-3 / outs 2-3.
    EXPECT_TRUE(chain.input(0));
    EXPECT_FALSE(chain.input(1));
    EXPECT_TRUE(chain.input(3));

    chain.setOutput(2, true);
    EXPECT_TRUE(chain.output(2));
    EXPECT_FALSE(chain.output(0));
    chain.allOutputsOff();
    EXPECT_FALSE(chain.output(2));

    EXPECT_FALSE(chain.attach(nullptr).has_value());
    EXPECT_THROW((void)chain.input(9), std::out_of_range);
    EXPECT_THROW(chain.setOutput(9, true), std::out_of_range);
    EXPECT_THROW((void)chain.module(4), std::out_of_range);
    EXPECT_GT(chain.knownDevices().size(), 0u);
}

// ============================================================================
// MultiEL1014 / MultiEL2004 — typed detect + module access
// ============================================================================

TEST(MultiEL1014, DetectBuildsTypedModules) {
    Master master;
    MultiEL1014<> chain(master);
    std::vector<DiscoveredSlave> scan{
        fakeSlave(0, Devices::EL1014.vendor_id, Devices::EL1014.product_code),
        fakeSlave(1, Devices::EL2004.vendor_id, Devices::EL2004.product_code),
    };
    auto r = chain.detect(std::span<const DiscoveredSlave>(scan));
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(*r, 1u);
    EXPECT_EQ(chain.moduleCount(), 1u);
    EXPECT_EQ(chain.channelCount(), EL1014::kNumChannels);

    // Typed module access (static_cast to EL1014).
    EL1014& mod = chain.module(0);
    EXPECT_EQ(mod.slaveIndex(), 0);

    // Flat bit reads reach the typed module (no PDO data — all false).
    EXPECT_FALSE(chain.bit(0));
    EXPECT_EQ(chain.bits().count(), 0u);
    EXPECT_EQ(MultiEL1014<>::kChannelsPerModule, EL1014::kNumChannels);
    EXPECT_GT((MultiEL1014<>::kMaxModules), 0u);
}

TEST(MultiEL2004, DetectBuildsTypedModules) {
    Master master;
    MultiEL2004<> chain(master);
    std::vector<DiscoveredSlave> scan{
        fakeSlave(0, Devices::EL2004.vendor_id, Devices::EL2004.product_code),
        fakeSlave(1, Devices::EL1014.vendor_id, Devices::EL1014.product_code),
    };
    auto r = chain.detect(std::span<const DiscoveredSlave>(scan));
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(*r, 1u);
    EXPECT_EQ(chain.channelCount(), EL2004::kNumChannels);

    EL2004& mod = chain.module(0);
    EXPECT_EQ(mod.slaveIndex(), 0);
    EXPECT_FALSE(chain.bit(0));
    EXPECT_EQ(MultiEL2004<>::kChannelsPerModule, EL2004::kNumChannels);
}
