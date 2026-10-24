/**
 * @file test_io_named_sdo_exposer.cpp
 * @brief Unit tests for NamedSdoExposer: slug generation, named-alias
 *        registration over a fake ISDOTransport, collision dedup and
 *        coexistence with the index-addressed AS715NExposer catalog.
 */

#include <gtest/gtest.h>

#include "tether/io/Registry.hpp"
#include "tether/io/exposers/AS715NExposer.hpp"
#include "tether/io/exposers/NamedSdoExposer.hpp"

#include <chrono>
#include <cstring>
#include <map>
#include <vector>

using namespace tether::io;
using namespace tether::io::exposers;
namespace RegAS715N  = ::EtherCAT::Drives::Registers::AS715N;
namespace OD         = ::EtherCAT::ObjectDictionary;
namespace Regs       = ::EtherCAT::Drives::Registers;
namespace exdet      = tether::io::exposers::detail;

// ---------------------------------------------------------------------------
// Fakes — same pattern as test_io_as715n_exposer.cpp
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// Slugification
// ---------------------------------------------------------------------------

TEST(NamedSdoSlugify, BasicCases) {
    using exdet::slugifyName;
    EXPECT_EQ(slugifyName("1st speed loop gain"), "1st_speed_loop_gain");
    EXPECT_EQ(slugifyName("Identity: vendor ID"), "identity_vendor_id");
    EXPECT_EQ(slugifyName("SM2 sync: cycle time ns"),
              "sm2_sync_cycle_time_ns");
    EXPECT_EQ(slugifyName("RxPDO assign: count"), "rxpdo_assign_count");
    EXPECT_EQ(slugifyName("PDFF control coefficient"),
              "pdff_control_coefficient");
}

TEST(NamedSdoSlugify, SeparatorsCollapseAndTrim) {
    using exdet::slugifyName;
    EXPECT_EQ(slugifyName("  --weird__name--  "), "weird_name");
    EXPECT_EQ(slugifyName(""), "");
    EXPECT_EQ(slugifyName("!!!"), "");
    EXPECT_EQ(slugifyName("a"), "a");
    EXPECT_EQ(slugifyName("A.B_C"), "a_b_c");
}

// ---------------------------------------------------------------------------
// Fixture
// ---------------------------------------------------------------------------

class NamedSdoExposerTest : public ::testing::Test {
protected:
    static constexpr uint64_t kBase =
        (ModuleId::AS715NDrive << 8) | 0;  // slave 0 — same idBase as the
                                           // hex exposer would use

    void SetUp() override {
        coe_.configureMailbox(0x1800, 128, 0x1C00, 128);
        ASSERT_TRUE(coe_.init());
        exposer_ = std::make_unique<NamedSdoExposer>(
            coe_, Regs::RegisterListOfLists{
                      &RegAS715N::C01::kRegisterList,
                      &RegAS715N::C02::kRegisterList,
                      &tether::io::exposers::detail::kCoeObjectList(),
                  },
            "drive0", "as715n");
        exposer_->expose(registry_, kBase);
    }

    void TearDown() override {
        exposer_.reset();   // unregister callbacks before coe dies
        coe_.deinit();
    }

    const ParamEntry* findByName(const std::string& name) {
        for (uint32_t off = 0;;) {
            auto page = registry_.paramPage(off, 200);
            if (page.empty()) break;
            for (const EntryView& v : page)
                if (v.name() == name) return v.asParam();
            off += static_cast<uint32_t>(page.size());
        }
        return nullptr;
    }

    FakeSdoTransport transport_;
    Registry         registry_;
    ::EtherCAT::CoE::CoEManager coe_{0, transport_};
    std::unique_ptr<NamedSdoExposer> exposer_;
};

// ---------------------------------------------------------------------------
// Catalog
// ---------------------------------------------------------------------------

TEST_F(NamedSdoExposerTest, EveryEntryGetsASlugName) {
    size_t expected = 0;
    for (const auto* list : Regs::RegisterListOfLists{
             &RegAS715N::C01::kRegisterList,
             &RegAS715N::C02::kRegisterList,
             &tether::io::exposers::detail::kCoeObjectList()}) {
        for (const auto* e : *list)
            if (e) ++expected;
    }
    ASSERT_GT(expected, 60u);   // sanity
    EXPECT_EQ(registry_.paramCount(), expected);

    for (const auto* list : Regs::RegisterListOfLists{
             &RegAS715N::C01::kRegisterList,
             &RegAS715N::C02::kRegisterList,
             &tether::io::exposers::detail::kCoeObjectList()}) {
        for (const auto* e : *list) {
            if (!e) continue;
            const std::string want =
                "drive0.sdo." +
                exdet::slugifyName(e->name ? e->name : "");
            EntryView v = registry_.findParam(
                NamedSdoExposer::namedId(kBase, e->index, e->subindex));
            ASSERT_TRUE(static_cast<bool>(v))
                << "missing named param for " << e->name;
            EXPECT_EQ(v.name(), want);
            EXPECT_EQ(v.group(), "as715n.drive0.sdo");
            EXPECT_TRUE(v.flags() & EntryFlags::NoStream);
            EXPECT_TRUE(v.flags() & EntryFlags::Readable);
        }
    }
}

TEST_F(NamedSdoExposerTest, KnownRegisterResolvesBySlug) {
    // C01.02 "1st speed loop gain" — the canonical PID-ish parameter.
    EntryView v = registry_.findParam(
        NamedSdoExposer::namedId(kBase, 0x2001, 0x02));
    ASSERT_TRUE(static_cast<bool>(v));
    EXPECT_EQ(v.name(), "drive0.sdo.1st_speed_loop_gain");
    EXPECT_EQ(v.valueType(), ValueType::U16);

    // ...and the name resolves the other way, too.
    const ParamEntry* p = findByName("drive0.sdo.1st_speed_loop_gain");
    ASSERT_NE(p, nullptr);
    EXPECT_EQ(p->id, v.id());
}

TEST_F(NamedSdoExposerTest, MetadataCarriesIndexAndCanonicalName) {
    EntryView v = registry_.findParam(
        NamedSdoExposer::namedId(kBase, 0x2001, 0x02));
    ASSERT_TRUE(static_cast<bool>(v));

    std::map<std::string, std::string> md;
    v.forEachMetadata([&](std::string_view k, std::string_view val) {
        md.emplace(k, val);
    });
    EXPECT_EQ(md["index"], "0x2001");
    EXPECT_EQ(md["subindex"], "0x02");
    EXPECT_EQ(md["sdo_name"], "drive0.sdo.2001_02");
    EXPECT_EQ(md["modification"], "during_operation");
    EXPECT_EQ(md["min"], "1");
    EXPECT_EQ(md["max"], "20000");
}

// ---------------------------------------------------------------------------
// Read / write through the CoE manager
// ---------------------------------------------------------------------------

TEST_F(NamedSdoExposerTest, NamedParamReadsThroughCoE) {
    EntryView v = registry_.findParam(
        NamedSdoExposer::namedId(kBase, 0x2001, 0x02));
    ASSERT_TRUE(static_cast<bool>(v));

    uint8_t buf[8] = {};
    v.read(buf);
    for (uint8_t i = 0; i < 2; ++i)   // U16 → 2 bytes
        EXPECT_EQ(buf[i],
                  FakeSdoTransport::patternByte(0x2001, 0x02, i));
}

TEST_F(NamedSdoExposerTest, WritableNamedParamWritesThroughCoE) {
    EntryView v = registry_.findParam(
        NamedSdoExposer::namedId(kBase, 0x2001, 0x02));
    ASSERT_TRUE(static_cast<bool>(v));
    ASSERT_TRUE(v.flags() & EntryFlags::Writable);

    const uint8_t payload[2] = {0x34, 0x12};
    v.write(payload);
    EXPECT_EQ(transport_.writeCount_, 1u);
    EXPECT_EQ(transport_.lastWriteIndex_, 0x2001);
    EXPECT_EQ(transport_.lastWriteSub_, 0x02);
    ASSERT_EQ(transport_.lastWriteData_.size(), 2u);
    EXPECT_EQ(transport_.lastWriteData_[0], 0x34);
    EXPECT_EQ(transport_.lastWriteData_[1], 0x12);
}

TEST_F(NamedSdoExposerTest, ReadOnlyNamedParamHasNoWriteCallback) {
    // 0x1000 device type — ReadOnly in kCoeObjects.
    EntryView v = registry_.findParam(
        NamedSdoExposer::namedId(kBase, 0x1000, 0x00));
    ASSERT_TRUE(static_cast<bool>(v));
    EXPECT_EQ(v.name(), "drive0.sdo.device_type");
    EXPECT_FALSE(v.flags() & EntryFlags::Writable);
}

TEST_F(NamedSdoExposerTest, VariableLengthEntryUsesVarCallbacks) {
    // 0x1008 manufacturer device name — VisibleString → variable-length.
    EntryView v = registry_.findParam(
        NamedSdoExposer::namedId(kBase, 0x1008, 0x00));
    ASSERT_TRUE(static_cast<bool>(v));
    EXPECT_EQ(v.name(), "drive0.sdo.manufacturer_device_name");
    EXPECT_TRUE(v.isVariableLength());

    uint8_t buf[64];
    const size_t n = v.readVar(buf, sizeof(buf));
    ASSERT_EQ(n, FakeSdoTransport::kObjectLen);
    for (size_t i = 0; i < n; ++i)
        EXPECT_EQ(buf[i],
                  FakeSdoTransport::patternByte(0x1008, 0x00, i));
}

// ---------------------------------------------------------------------------
// Naming edge cases
// ---------------------------------------------------------------------------

TEST_F(NamedSdoExposerTest, SlugCollisionsGetIndexSuffix) {
    static const OD::ObjectDictionaryEntry kDupA = {
        .index = 0x2030, .subindex = 0x00,
        .name = "Control in progress",
        .data_type = OD::ObjectDictionaryDataType::Unsigned16,
        .modification_mode = OD::ModificationMode::ReadOnly,
        .effective_time = OD::EffectiveTime::Immediately};
    static const OD::ObjectDictionaryEntry kDupB = {
        .index = 0x2031, .subindex = 0x00,
        .name = "Control in progress",
        .data_type = OD::ObjectDictionaryDataType::Unsigned16,
        .modification_mode = OD::ModificationMode::ReadOnly,
        .effective_time = OD::EffectiveTime::Immediately};
    static const Regs::RegisterList kDups = {&kDupA, &kDupB};

    Registry reg;
    NamedSdoExposer exp(coe_, Regs::RegisterListOfLists{&kDups},
                      "drive0", "as715n");
    exp.expose(reg, kBase);

    EXPECT_EQ(reg.paramCount(), 2u);
    const ParamEntry* first = nullptr;
    const ParamEntry* second = nullptr;
    auto page = reg.paramPage(0, 10);
    ASSERT_EQ(page.size(), 2u);
    first = page[0].asParam();
    second = page[1].asParam();
    EXPECT_EQ(first->name, "drive0.sdo.control_in_progress");
    EXPECT_EQ(second->name, "drive0.sdo.control_in_progress_2031_00");
}

TEST_F(NamedSdoExposerTest, UnnamedEntryFallsBackToIndexName) {
    static const OD::ObjectDictionaryEntry kNameless = {
        .index = 0x2FFF, .subindex = 0x07,
        .name = nullptr,
        .data_type = OD::ObjectDictionaryDataType::Unsigned8,
        .modification_mode = OD::ModificationMode::ReadOnly,
        .effective_time = OD::EffectiveTime::Immediately};
    static const Regs::RegisterList kList = {&kNameless};

    Registry reg;
    NamedSdoExposer exp(coe_, Regs::RegisterListOfLists{&kList},
                      "drive0", "as715n");
    exp.expose(reg, kBase);

    ASSERT_EQ(reg.paramCount(), 1u);
    EntryView v = reg.paramPage(0, 1)[0];
    EXPECT_EQ(v.name(), "drive0.sdo.obj_2FFF_07");
}

TEST_F(NamedSdoExposerTest, NamedIdsDoNotCollideWithIndexScheme) {
    // Named entries use local tag 0x02; AS715NExposer's sdo.* entries use
    // 0x01 — the same object must land on a different id.
    const uint64_t hexId = makeId(
        kBase, 0x01000000u | (static_cast<uint32_t>(0x2001) << 8) | 0x02);
    const uint64_t namedId =
        NamedSdoExposer::namedId(kBase, 0x2001, 0x02);
    EXPECT_NE(hexId, namedId);
}

TEST_F(NamedSdoExposerTest, CoexistsWithIndexExposerOnSameIdBase) {
    // Register the real AS715NExposer on the same registry+idBase: named
    // aliases and `sdo.XXXX_XX` entries must not collide.
    class NullPdoAccess : public IAS715NPdoAccess {
    public:
        bool readRxPDO1704(::EtherCAT::Drives::AS715N_pdo::
                               AS715N_RxPDO_1704&) override { return false; }
        bool readTxPDO1B04(::EtherCAT::Drives::AS715N_pdo::
                               AS715N_TxPDO_1B04&) override { return false; }
    } pdo;

    Registry reg;
    AS715NExposer<4> hexExp(pdo, coe_, 0);
    hexExp.expose(reg, kBase);
    const uint32_t hexCount = reg.paramCount();

    NamedSdoExposer namedExp(coe_, AS715NExposer<>::sdoCatalog(),
                             "drive0", "as715n");
    namedExp.expose(reg, kBase);

    // Every SDO object now exists twice — once per naming scheme.
    EXPECT_GT(reg.paramCount(), hexCount);

    EntryView hex = reg.findParam(
        makeId(kBase, 0x01000000u | (0x2001u << 8) | 0x02));
    EntryView named = reg.findParam(
        NamedSdoExposer::namedId(kBase, 0x2001, 0x02));
    ASSERT_TRUE(static_cast<bool>(hex));
    ASSERT_TRUE(static_cast<bool>(named));
    EXPECT_EQ(hex.name(), "drive0.sdo.2001_02");
    EXPECT_EQ(named.name(), "drive0.sdo.1st_speed_loop_gain");
}
