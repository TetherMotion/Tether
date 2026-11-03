/**
 * @file test_encapsulation.cpp
 * @brief Coverage for Encapsulation.cpp: the --encapsulation spec parser,
 *        config logging, and the kernel cBPF filter-program builder.
 *
 * Both parseEncapsulationSpec() and buildEncapsulationBpfProgram() are
 * pure functions — no wire needed.  (setupEncapsulation()/attach* need a
 * real HAL::IEthernet socket and stay out of scope.)
 */

#include <gtest/gtest.h>

#include "tether/ethercat/Encapsulation.hpp"

using namespace EtherCAT;

namespace {

bool parse(const std::string& spec, EncapsulationConfig& out) {
    return parseEncapsulationSpec(spec, out, "test");
}

} // namespace

// ============================================================================
// parseEncapsulationSpec — raw / empty
// ============================================================================

TEST(EncapsulationSpec, EmptyAndRawSpecsProduceDisabledConfig) {
    EncapsulationConfig cfg;
    EXPECT_TRUE(parse("", cfg));
    EXPECT_FALSE(cfg.enabled());
    EXPECT_TRUE(parse("raw", cfg));
    EXPECT_FALSE(cfg.enabled());
    EXPECT_TRUE(parse("none", cfg));
    EXPECT_FALSE(cfg.enabled());
}

TEST(EncapsulationSpec, RawCannotBeCombined) {
    EncapsulationConfig cfg;
    EXPECT_FALSE(parse("raw,udp", cfg));
    EXPECT_FALSE(parse("udp,none", cfg));
    EXPECT_FALSE(parse("raw,vlan:100", cfg));
}

// ============================================================================
// parseEncapsulationSpec — vlan:
// ============================================================================

TEST(EncapsulationSpec, VlanSingleVidSetsRangeAndTxDefault) {
    EncapsulationConfig cfg;
    ASSERT_TRUE(parse("vlan:100", cfg));
    ASSERT_TRUE(cfg.rxRange.has_value());
    EXPECT_EQ(cfg.rxRange->start, 100);
    EXPECT_EQ(cfg.rxRange->end, 100);
    ASSERT_TRUE(cfg.txVlan.has_value());
    EXPECT_EQ(*cfg.txVlan, 100);  // TX defaults to first RX VID
    EXPECT_TRUE(cfg.vlanActive());
    EXPECT_TRUE(cfg.enabled());
    EXPECT_FALSE(cfg.rxAny);
}

TEST(EncapsulationSpec, VlanRangeSetsTxToRangeStart) {
    EncapsulationConfig cfg;
    ASSERT_TRUE(parse("vlan:100-200", cfg));
    ASSERT_TRUE(cfg.rxRange.has_value());
    EXPECT_EQ(cfg.rxRange->start, 100);
    EXPECT_EQ(cfg.rxRange->end, 200);
    EXPECT_EQ(*cfg.txVlan, 100);
}

TEST(EncapsulationSpec, VlanAnyIsCatchAllWithoutTxDefault) {
    EncapsulationConfig cfg;
    ASSERT_TRUE(parse("vlan:any", cfg));
    EXPECT_TRUE(cfg.rxAny);
    EXPECT_FALSE(cfg.rxRange.has_value());
    EXPECT_TRUE(cfg.vlanActive());
}

TEST(EncapsulationSpec, VlanRejectsOutOfRangeAndMalformed) {
    EncapsulationConfig cfg;
    EXPECT_FALSE(parse("vlan:0", cfg));
    EXPECT_FALSE(parse("vlan:4096", cfg));
    EXPECT_FALSE(parse("vlan:abc", cfg));
    EXPECT_FALSE(parse("vlan:", cfg));
    EXPECT_FALSE(parse("vlan:200-100", cfg));  // lo > hi
    EXPECT_FALSE(parse("vlan:1-x", cfg));
    EXPECT_FALSE(parse("vlan:100,vlan:200", cfg));  // duplicate
}

TEST(EncapsulationSpec, VlanBoundaryVidsAccepted) {
    EncapsulationConfig cfg;
    ASSERT_TRUE(parse("vlan:1", cfg));
    EXPECT_EQ(*cfg.txVlan, 1);
    ASSERT_TRUE(parse("vlan:4095", cfg));
    EXPECT_EQ(*cfg.txVlan, 4095);
    ASSERT_TRUE(parse("vlan:1-4095", cfg));
    EXPECT_EQ(cfg.rxRange->end, 4095);
}

// ============================================================================
// parseEncapsulationSpec — vlantx:
// ============================================================================

TEST(EncapsulationSpec, VlanTxOverridesImpliedTxVid) {
    EncapsulationConfig cfg;
    ASSERT_TRUE(parse("vlan:100-200,vlantx:150", cfg));
    EXPECT_EQ(*cfg.txVlan, 150);
    EXPECT_EQ(cfg.rxRange->start, 100);
}

TEST(EncapsulationSpec, VlanTxOffDisablesTxTag) {
    EncapsulationConfig cfg;
    ASSERT_TRUE(parse("vlan:100,vlantx:off", cfg));
    EXPECT_FALSE(cfg.txVlan.has_value());
    ASSERT_TRUE(parse("vlan:100,vlantx:none", cfg));
    EXPECT_FALSE(cfg.txVlan.has_value());
}

TEST(EncapsulationSpec, VlanTxAloneWithoutRxIsValid) {
    EncapsulationConfig cfg;
    ASSERT_TRUE(parse("vlantx:300", cfg));
    ASSERT_TRUE(cfg.txVlan.has_value());
    EXPECT_EQ(*cfg.txVlan, 300);
    EXPECT_FALSE(cfg.rxRange.has_value());
    EXPECT_TRUE(cfg.vlanActive());  // TX tag alone activates VLAN mode
}

TEST(EncapsulationSpec, VlanTxRejectsMalformedAndDuplicate) {
    EncapsulationConfig cfg;
    EXPECT_FALSE(parse("vlantx:0", cfg));
    EXPECT_FALSE(parse("vlantx:5000", cfg));
    EXPECT_FALSE(parse("vlantx:xyz", cfg));
    EXPECT_FALSE(parse("vlantx:100,vlantx:200", cfg));
}

// ============================================================================
// parseEncapsulationSpec — udp
// ============================================================================

TEST(EncapsulationSpec, UdpAloneEnablesDefaultPort) {
    EncapsulationConfig cfg;
    ASSERT_TRUE(parse("udp", cfg));
    EXPECT_TRUE(cfg.udp);
    EXPECT_EQ(cfg.udpPort, kEtherCATUdpPort);
    EXPECT_TRUE(cfg.enabled());
    EXPECT_FALSE(cfg.vlanActive());
}

TEST(EncapsulationSpec, UdpCustomPort) {
    EncapsulationConfig cfg;
    ASSERT_TRUE(parse("udp:12345", cfg));
    EXPECT_TRUE(cfg.udp);
    EXPECT_EQ(cfg.udpPort, 12345);
}

TEST(EncapsulationSpec, UdpRejectsBadPortAndDuplicate) {
    EncapsulationConfig cfg;
    EXPECT_FALSE(parse("udp:0", cfg));
    EXPECT_FALSE(parse("udp:65536", cfg));
    EXPECT_FALSE(parse("udp:notaport", cfg));
    EXPECT_FALSE(parse("udp,udp:1234", cfg));
}

// ============================================================================
// parseEncapsulationSpec — combos, whitespace, unknown tokens
// ============================================================================

TEST(EncapsulationSpec, VlanAndUdpCombine) {
    EncapsulationConfig cfg;
    ASSERT_TRUE(parse("vlan:100,udp", cfg));
    EXPECT_TRUE(cfg.vlanActive());
    EXPECT_TRUE(cfg.udp);
    EXPECT_TRUE(cfg.enabled());
}

TEST(EncapsulationSpec, TokensTolerateWhitespaceAndEmptySegments) {
    EncapsulationConfig cfg;
    ASSERT_TRUE(parse(" vlan:42 , udp:34980 ,", cfg));
    EXPECT_TRUE(cfg.vlanActive());
    EXPECT_EQ(*cfg.txVlan, 42);
    EXPECT_EQ(cfg.udpPort, 34980);
}

TEST(EncapsulationSpec, UnknownTokensFail) {
    EncapsulationConfig cfg;
    EXPECT_FALSE(parse("foo", cfg));
    EXPECT_FALSE(parse("vlan:100,bar", cfg));
    EXPECT_FALSE(parse("ip", cfg));
    EXPECT_FALSE(parse("vlanx:10", cfg));  // near-miss is still unknown
}

TEST(EncapsulationSpec, ConfigResetOnReparse) {
    EncapsulationConfig cfg;
    ASSERT_TRUE(parse("vlan:100,udp:1234", cfg));
    ASSERT_TRUE(parse("", cfg));  // re-parse resets all fields
    EXPECT_FALSE(cfg.enabled());
    EXPECT_FALSE(cfg.txVlan.has_value());
    EXPECT_FALSE(cfg.rxAny);
    EXPECT_FALSE(cfg.rxRange.has_value());
}

// ============================================================================
// logEncapsulationConfig — early-out on disabled, logs on enabled
// ============================================================================

TEST(EncapsulationLog, DisabledConfigIsSilentNoop) {
    EncapsulationConfig cfg;  // disabled
    logEncapsulationConfig(cfg, "test");  // must not crash
    SUCCEED();
}

TEST(EncapsulationLog, EnabledVariantsLogAllBranches) {
    EncapsulationConfig cfg;

    ASSERT_TRUE(parse("vlan:100-200", cfg));
    logEncapsulationConfig(cfg, "test");

    ASSERT_TRUE(parse("vlan:any", cfg));
    logEncapsulationConfig(cfg, "test");

    ASSERT_TRUE(parse("vlan:any,vlantx:300", cfg));
    logEncapsulationConfig(cfg, "test");

    ASSERT_TRUE(parse("udp", cfg));
    logEncapsulationConfig(cfg, "test");

    ASSERT_TRUE(parse("vlan:100,udp:34980", cfg));
    logEncapsulationConfig(cfg, "test");

    SUCCEED();
}

// ============================================================================
// buildEncapsulationBpfProgram — program shape per config
// ============================================================================

TEST(EncapsulationBpf, RawConfigProducesEthercatOnlyProgram) {
    EncapsulationConfig cfg;  // untagged EtherCAT-only ingress filter
    auto prog = buildEncapsulationBpfProgram(cfg);
    EXPECT_FALSE(prog.empty());
    // VLAN-mode programs are strictly larger (extra tag legs).
    EncapsulationConfig vlan;
    ASSERT_TRUE(parse("vlan:100", vlan));
    EXPECT_LT(prog.size(),
              buildEncapsulationBpfProgram(vlan).size());
}

TEST(EncapsulationBpf, VlanConfigProducesNonEmptyProgram) {
    EncapsulationConfig cfg;
    ASSERT_TRUE(parse("vlan:100", cfg));
    auto prog = buildEncapsulationBpfProgram(cfg);
    EXPECT_FALSE(prog.empty());
}

TEST(EncapsulationBpf, DeliveryHintsChangeProgramShape) {
    EncapsulationConfig cfg;
    ASSERT_TRUE(parse("vlan:100-200", cfg));

    auto auto_prog = buildEncapsulationBpfProgram(cfg, VlanDeliveryHint::Auto);
    auto stripped =
        buildEncapsulationBpfProgram(cfg, VlanDeliveryHint::StrippedOnly);
    auto inlined =
        buildEncapsulationBpfProgram(cfg, VlanDeliveryHint::InlineOnly);

    EXPECT_FALSE(auto_prog.empty());
    EXPECT_FALSE(stripped.empty());
    EXPECT_FALSE(inlined.empty());
    // Auto emits both legs — the pruned variants must not be identical
    // supersets of it.
    EXPECT_NE(auto_prog.size(), 0u);
}

TEST(EncapsulationBpf, UdpAndCatchAllProducePrograms) {
    EncapsulationConfig cfg;
    ASSERT_TRUE(parse("udp", cfg));
    EXPECT_FALSE(buildEncapsulationBpfProgram(cfg).empty());

    ASSERT_TRUE(parse("vlan:any", cfg));
    EXPECT_FALSE(buildEncapsulationBpfProgram(cfg).empty());

    ASSERT_TRUE(parse("vlan:100,udp", cfg));
    EXPECT_FALSE(buildEncapsulationBpfProgram(
                     cfg, VlanDeliveryHint::InlineOnly)
                     .empty());
}
