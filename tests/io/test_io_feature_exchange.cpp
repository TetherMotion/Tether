/**
 * @file test_io_feature_exchange.cpp
 * @brief Unit tests for Feature, FeatureSet.
 */
#include <gtest/gtest.h>
#include "tether/io/FeatureExchange.hpp"

using namespace tether::io;

TEST(IOFeature, MakeBool) {
    Feature f = Feature::makeBool("supports_datalogging", true);
    EXPECT_EQ(f.name, "supports_datalogging");
    EXPECT_EQ(f.type, ValueType::Bool);
    EXPECT_TRUE(f.getBool());

    Feature f2 = Feature::makeBool("test", false);
    EXPECT_FALSE(f2.getBool());
}

TEST(IOFeature, MakeU32) {
    Feature f = Feature::makeU32("protocol_version", 1);
    EXPECT_EQ(f.name, "protocol_version");
    EXPECT_EQ(f.type, ValueType::U32);
    EXPECT_EQ(f.getU32(), 1u);
}

TEST(IOFeature, MakeString) {
    Feature f = Feature::makeString("server_name", "Tether");
    EXPECT_EQ(f.name, "server_name");
    EXPECT_EQ(f.type, ValueType::String);
    EXPECT_EQ(f.getString(), "Tether");
}

TEST(IOFeature, FeatureSetFindAndSupports) {
    FeatureSet fs;
    fs.features.push_back(Feature::makeBool("supports_datalogging", true));
    fs.features.push_back(Feature::makeBool("supports_thresholds", false));
    fs.features.push_back(Feature::makeU32("protocol_version", 1));

    const Feature* f = fs.find("supports_datalogging");
    ASSERT_NE(f, nullptr);
    EXPECT_TRUE(f->getBool());

    EXPECT_TRUE(fs.supports("supports_datalogging"));
    EXPECT_FALSE(fs.supports("supports_thresholds"));
    EXPECT_FALSE(fs.supports("nonexistent"));
}

TEST(IOFeature, FeatureSetEncodeDecodeRoundtrip) {
    FeatureSet original;
    original.features.push_back(Feature::makeBool("flag_a", true));
    original.features.push_back(Feature::makeU32("max_chunk", 64));
    original.features.push_back(Feature::makeString("name", "TestServer"));

    uint8_t buf[1024];
    BufWriter w(buf, sizeof(buf));
    original.encode(w);
    ASSERT_TRUE(w.ok());

    BufReader r(buf, w.pos);
    FeatureSet decoded;
    EXPECT_TRUE(FeatureSet::decode(r, decoded));

    ASSERT_EQ(decoded.features.size(), 3u);
    EXPECT_EQ(decoded.features[0].name, "flag_a");
    EXPECT_TRUE(decoded.features[0].getBool());
    EXPECT_EQ(decoded.features[1].name, "max_chunk");
    EXPECT_EQ(decoded.features[1].getU32(), 64u);
    EXPECT_EQ(decoded.features[2].name, "name");
    EXPECT_EQ(decoded.features[2].getString(), "TestServer");
}

TEST(IOFeature, EmptyFeatureSet) {
    FeatureSet fs;
    uint8_t buf[32];
    BufWriter w(buf, sizeof(buf));
    fs.encode(w);
    ASSERT_TRUE(w.ok());

    BufReader r(buf, w.pos);
    FeatureSet decoded;
    EXPECT_TRUE(FeatureSet::decode(r, decoded));
    EXPECT_EQ(decoded.features.size(), 0u);
}
