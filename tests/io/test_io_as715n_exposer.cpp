/**
 * @file test_io_as715n_exposer.cpp
 * @brief Unit tests for AS715NExposer: PDO signal catalog, full SDO
 *        register exposure through a fake ISDOTransport, and the
 *        ring-buffered PDO stream source.
 */

#include <gtest/gtest.h>

#include "tether/io/Registry.hpp"
#include "tether/io/exposers/AS715NExposer.hpp"

#include <chrono>
#include <cstring>
#include <map>
#include <vector>

using namespace tether::io;
using namespace tether::io::exposers;
namespace AS715N_pdo = ::EtherCAT::Drives::AS715N_pdo;
namespace RegAS715N  = ::EtherCAT::Drives::Registers::AS715N;
namespace OD         = ::EtherCAT::ObjectDictionary;

// ---------------------------------------------------------------------------
// Fakes
// ---------------------------------------------------------------------------

/// SDO transport that answers uploads with a deterministic byte pattern and
/// records downloads.  Runs on the CoEManager worker thread.
class FakeSdoTransport : public ::EtherCAT::SDO::ISDOTransport {
public:
    static constexpr size_t kObjectLen = 8;

    static uint8_t patternByte(uint16_t index, uint8_t sub, size_t i) {
        return static_cast<uint8_t>((index >> 8) ^ (index & 0xFF) ^ sub ^
                                    static_cast<uint8_t>(i * 31));
    }

    bool sdoUpload(uint16_t, uint8_t* mbx_counter,
                   uint16_t, uint16_t, uint16_t, uint16_t,
                   uint16_t index, uint8_t sub,
                   uint8_t* out, size_t out_cap, size_t* out_len,
                   bool, unsigned int, unsigned int) override {
        const size_t n = std::min(out_cap, kObjectLen);
        for (size_t i = 0; i < n; ++i) out[i] = patternByte(index, sub, i);
        if (out_len) *out_len = n;
        if (mbx_counter) ++*mbx_counter;
        return true;
    }

    bool sdoDownload(uint16_t, uint8_t* mbx_counter,
                     uint16_t, uint16_t, uint16_t, uint16_t,
                     uint16_t index, uint8_t sub,
                     const uint8_t* data, size_t data_len,
                     bool, unsigned int, unsigned int) override {
        lastWriteIndex_ = index;
        lastWriteSub_   = sub;
        lastWriteData_.assign(data, data + data_len);
        ++writeCount_;
        if (mbx_counter) ++*mbx_counter;
        return true;
    }

    uint64_t getMicroseconds() override {
        return static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());
    }

    uint16_t lastWriteIndex_ = 0;
    uint8_t  lastWriteSub_   = 0;
    std::vector<uint8_t> lastWriteData_;
    uint32_t writeCount_ = 0;
};

/// PDO access returning canned images; available=false simulates a drive
/// that is absent (recovery) — reads must then report zeros, not crash.
class FakePdoAccess : public IAS715NPdoAccess {
public:
    FakePdoAccess() {
        auto* rb = reinterpret_cast<uint8_t*>(&rx_);
        for (size_t i = 0; i < sizeof(rx_); ++i) rb[i] = static_cast<uint8_t>(i);
        auto* tb = reinterpret_cast<uint8_t*>(&tx_);
        for (size_t i = 0; i < sizeof(tx_); ++i)
            tb[i] = static_cast<uint8_t>(0x80 + i);
    }

    bool readRxPDO1704(AS715N_pdo::AS715N_RxPDO_1704& out) override {
        if (!available_) return false;
        out = rx_;
        return true;
    }
    bool readTxPDO1B04(AS715N_pdo::AS715N_TxPDO_1B04& out) override {
        if (!available_) return false;
        out = tx_;
        return true;
    }

    AS715N_pdo::AS715N_RxPDO_1704 rx_{};
    AS715N_pdo::AS715N_TxPDO_1B04 tx_{};
    bool available_ = true;
};

// ---------------------------------------------------------------------------
// Fixture
// ---------------------------------------------------------------------------

class AS715NExposerTest : public ::testing::Test {
protected:
    static constexpr uint64_t kBase =
        (ModuleId::AS715NDrive << 8) | 0;  // slave 0

    void SetUp() override {
        coe_.configureMailbox(0x1800, 128, 0x1C00, 128);
        ASSERT_TRUE(coe_.init());
        exposer_ = std::make_unique<AS715NExposer<8>>(pdo_, coe_, 0);
        exposer_->expose(registry_, kBase);
    }

    void TearDown() override {
        exposer_.reset();   // unregister callbacks before coe dies
        coe_.deinit();
    }

    static const ::EtherCAT::Drives::Registers::RegisterListOfLists&
    allGroups() {
        static const ::EtherCAT::Drives::Registers::RegisterListOfLists g = {
            &RegAS715N::C00::kRegisterList, &RegAS715N::C01::kRegisterList,
            &RegAS715N::C02::kRegisterList, &RegAS715N::C03::kRegisterList,
            &RegAS715N::C04::kRegisterList, &RegAS715N::C05::kRegisterList,
            &RegAS715N::C06::kRegisterList, &RegAS715N::C07::kRegisterList,
            &RegAS715N::C0A::kRegisterList, &RegAS715N::C10::kRegisterList,
            &RegAS715N::C13::kRegisterList, &RegAS715N::F30::kRegisterList,
            &RegAS715N::F31::kRegisterList, &RegAS715N::R20::kRegisterList,
            &RegAS715N::R22::kRegisterList, &RegAS715N::U40::kRegisterList,
            &RegAS715N::U41::kRegisterList, &RegAS715N::U42::kRegisterList,
        };
        return g;
    }

    static uint64_t sdoId(uint16_t index, uint8_t sub) {
        return makeId(kBase,
                      0x01000000u | (static_cast<uint32_t>(index) << 8) | sub);
    }

    FakeSdoTransport transport_;
    FakePdoAccess    pdo_;
    Registry         registry_;
    ::EtherCAT::CoE::CoEManager coe_{0, transport_};
    std::unique_ptr<AS715NExposer<8>> exposer_;
};

// ---------------------------------------------------------------------------
// PDO signal catalog
// ---------------------------------------------------------------------------

TEST_F(AS715NExposerTest, AllPdoFieldsRegisteredAsSignals) {
    // 19 fields + 2 raw images.
    EXPECT_EQ(registry_.signalCount(), detail::kPdoFieldCount + 2);

    for (size_t i = 0; i < detail::kPdoFieldCount; ++i) {
        const auto& f = detail::kPdoFields[i];
        EntryView v = registry_.findSignal(makeId(kBase, i + 1));
        ASSERT_TRUE(static_cast<bool>(v)) << "missing signal for field " << f.name;
        const std::string expectName = std::string("drive0.pdo.") + f.name;
        EXPECT_EQ(v.name(), expectName);
        EXPECT_EQ(v.valueType(), f.type);
        EXPECT_EQ(v.valueSize(), f.size);
        EXPECT_EQ(v.group(), "as715n.drive0.pdo");
    }

    EntryView rxImg = registry_.findSignal(makeId(kBase, 0x0100));
    EntryView txImg = registry_.findSignal(makeId(kBase, 0x0101));
    ASSERT_TRUE(static_cast<bool>(rxImg));
    ASSERT_TRUE(static_cast<bool>(txImg));
    EXPECT_EQ(rxImg.valueType(), ValueType::Binary);
    EXPECT_TRUE(rxImg.isVariableLength());
    EXPECT_TRUE(txImg.isVariableLength());
}

TEST_F(AS715NExposerTest, PdoFieldReadReturnsImageBytes) {
    pdo_.tx_.statusword = 0xBEEF;
    pdo_.rx_.target_position = -123456;

    uint16_t sw = 0;
    registry_.findSignal(makeId(kBase, 11))   // tx_statusword
        .read(&sw);
    EXPECT_EQ(sw, 0xBEEF);

    int32_t tp = 0;
    registry_.findSignal(makeId(kBase, 2))    // rx_target_position
        .read(&tp);
    EXPECT_EQ(tp, -123456);
}

TEST_F(AS715NExposerTest, PdoAbsentDriveReadsZeros) {
    pdo_.available_ = false;
    uint16_t sw = 0xFFFF;
    registry_.findSignal(makeId(kBase, 11)).read(&sw);
    EXPECT_EQ(sw, 0);

    uint8_t img[64];
    std::memset(img, 0xAA, sizeof(img));
    EXPECT_EQ(registry_.findSignal(makeId(kBase, 0x0101))
                  .readVar(img, sizeof(img)), 0u);
}

TEST_F(AS715NExposerTest, RawPdoImagesReadable) {
    uint8_t buf[64];
    size_t n = registry_.findSignal(makeId(kBase, 0x0100))
                   .readVar(buf, sizeof(buf));
    ASSERT_EQ(n, sizeof(AS715N_pdo::AS715N_RxPDO_1704));
    EXPECT_EQ(std::memcmp(buf, &pdo_.rx_, n), 0);

    // Too-small buffer reports nothing rather than truncating silently.
    EXPECT_EQ(registry_.findSignal(makeId(kBase, 0x0100)).readVar(buf, 4), 0u);
}

// ---------------------------------------------------------------------------
// SDO catalog
// ---------------------------------------------------------------------------

TEST_F(AS715NExposerTest, SdoCatalogCoversAllRegisterLists) {
    size_t expected = 0;
    for (const auto* list : allGroups())
        for (const auto* e : *list)
            if (e) ++expected;
    ASSERT_GT(expected, 100u);              // sanity: catalog is large
    EXPECT_EQ(registry_.paramCount(), expected);

    // Every register entry resolves by its computed id and carries
    // NoStream + OD metadata.
    size_t checked = 0;
    for (const auto* list : allGroups()) {
        for (const auto* e : *list) {
            if (!e) continue;
            EntryView v = registry_.findParam(sdoId(e->index, e->subindex));
            ASSERT_TRUE(static_cast<bool>(v)) << "missing SDO param 0x" << std::hex << e->index
                           << ":" << (int)e->subindex;
            EXPECT_TRUE(v.flags() & EntryFlags::NoStream);
            EXPECT_TRUE(v.flags() & EntryFlags::Readable);
            char expectName[64];
            std::snprintf(expectName, sizeof(expectName),
                          "drive0.sdo.%04X_%02X", e->index, e->subindex);
            EXPECT_EQ(v.name(), std::string_view(expectName));
            bool sawIndex = false;
            v.forEachMetadata([&](std::string_view k, std::string_view val) {
                if (k == "index") {
                    char hex[8];
                    std::snprintf(hex, sizeof(hex), "0x%04X", e->index);
                    EXPECT_EQ(val, std::string_view(hex));
                    sawIndex = true;
                }
            });
            EXPECT_TRUE(sawIndex);
            ++checked;
        }
    }
    EXPECT_EQ(checked, expected);
}

TEST_F(AS715NExposerTest, SdoReadGoesThroughCoE) {
    // Pick a fixed-size scalar entry; readFn must return the pattern the
    // fake transport serves.
    const OD::ObjectDictionaryEntry* target = nullptr;
    for (const auto* list : allGroups()) {
        for (const auto* e : *list) {
            if (e && detail::odFixedSize(e->data_type) > 0) {
                target = e;
                break;
            }
        }
        if (target) break;
    }
    ASSERT_NE(target, nullptr);

    EntryView v = registry_.findParam(sdoId(target->index, target->subindex));
    ASSERT_TRUE(static_cast<bool>(v));
    const uint8_t sz = detail::odFixedSize(target->data_type);
    uint8_t buf[8] = {};
    v.read(buf);
    for (uint8_t i = 0; i < sz; ++i)
        EXPECT_EQ(buf[i],
                  FakeSdoTransport::patternByte(target->index, target->subindex, i));
}

TEST_F(AS715NExposerTest, WritableSdoWritesThroughCoE) {
    const OD::ObjectDictionaryEntry* target = nullptr;
    for (const auto* list : allGroups()) {
        for (const auto* e : *list) {
            if (e && e->modification_mode != OD::ModificationMode::ReadOnly &&
                detail::odFixedSize(e->data_type) > 0) {
                target = e;
                break;
            }
        }
        if (target) break;
    }
    ASSERT_NE(target, nullptr) << "no writable fixed-size SDO in catalog";

    EntryView v = registry_.findParam(sdoId(target->index, target->subindex));
    ASSERT_TRUE(static_cast<bool>(v));
    EXPECT_TRUE(v.flags() & EntryFlags::Writable);

    const uint8_t payload[8] = {0xDE, 0xAD, 0xBE, 0xEF, 1, 2, 3, 4};
    v.write(payload);
    EXPECT_EQ(transport_.writeCount_, 1u);
    EXPECT_EQ(transport_.lastWriteIndex_, target->index);
    EXPECT_EQ(transport_.lastWriteSub_, target->subindex);
    EXPECT_EQ(transport_.lastWriteData_.size(),
              detail::odFixedSize(target->data_type));
    EXPECT_EQ(std::memcmp(transport_.lastWriteData_.data(), payload,
                          transport_.lastWriteData_.size()), 0);
}

TEST_F(AS715NExposerTest, ReadOnlySdoHasNoWriteCallback) {
    const OD::ObjectDictionaryEntry* target = nullptr;
    for (const auto* list : allGroups()) {
        for (const auto* e : *list) {
            if (e && e->modification_mode == OD::ModificationMode::ReadOnly) {
                target = e;
                break;
            }
        }
        if (target) break;
    }
    ASSERT_NE(target, nullptr) << "no read-only SDO in catalog";

    EntryView v = registry_.findParam(sdoId(target->index, target->subindex));
    ASSERT_TRUE(static_cast<bool>(v));
    EXPECT_FALSE(v.flags() & EntryFlags::Writable);
}

// ---------------------------------------------------------------------------
// Ring stream source
// ---------------------------------------------------------------------------

TEST_F(AS715NExposerTest, RingSchemaCoversAllPdoFields) {
    auto& src = exposer_->ringSource();
    ASSERT_EQ(src.schemaEntryIds().size(), detail::kPdoFieldCount);
    EXPECT_EQ(src.rowSize(), sizeof(AS715NPdoRow));
    for (size_t i = 0; i < detail::kPdoFieldCount; ++i) {
        EXPECT_EQ(src.schemaEntryIds()[i], makeId(kBase, i + 1));
        EXPECT_EQ(src.schemaFieldSizes()[i], detail::kPdoFields[i].size);
    }
}

TEST_F(AS715NExposerTest, RingProduceDrainRoundtrip) {
    auto& src = exposer_->ringSource();
    ASSERT_TRUE(src.tryAcquire());
    src.start();

    AS715NPdoRow sent{};
    sent.ts_us = 4242;
    sent.rx_controlword = 0x000F;
    sent.tx_position_actual = 777;
    exposer_->produceRow([&](AS715NPdoRow& r) { r = sent; });

    std::vector<uint8_t> rows(src.rowSize());
    ASSERT_EQ(src.drainRows(rows.data(), 1), 1u);
    const auto* got = reinterpret_cast<const AS715NPdoRow*>(rows.data());
    EXPECT_EQ(got->ts_us, 4242u);
    EXPECT_EQ(got->rx_controlword, 0x000F);
    EXPECT_EQ(got->tx_position_actual, 777);

    src.stop();
    src.release();
}

TEST_F(AS715NExposerTest, RingProduceGatedWhileInactive) {
    // No acquire/start: produce must be a no-op.
    EXPECT_FALSE(exposer_->produceRow([](AS715NPdoRow&) {}));
}

TEST_F(AS715NExposerTest, SecondDriveGetsUniqueIds) {
    Registry reg2;
    constexpr uint64_t kBase1 = (ModuleId::AS715NDrive << 8) | 1;
    AS715NExposer<8> exp1(pdo_, coe_, 1);
    exp1.expose(reg2, kBase1);

    // Same counts, distinct ids: drive1's tx_statusword is the same local
    // offset under a different module base.
    EXPECT_EQ(reg2.signalCount(), registry_.signalCount());
    EXPECT_EQ(reg2.paramCount(), registry_.paramCount());
    EntryView v1 = reg2.findSignal(makeId(kBase1, 11));
    ASSERT_TRUE(static_cast<bool>(v1));
    EXPECT_EQ(v1.name(), "drive1.pdo.tx_statusword");
    EXPECT_NE(makeId(kBase1, 11), makeId(kBase, 11));
    EXPECT_FALSE(static_cast<bool>(reg2.findSignal(makeId(kBase, 11))));
}
