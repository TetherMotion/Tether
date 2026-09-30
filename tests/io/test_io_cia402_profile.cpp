#include "tether/io/CiA402Profile.hpp"
#include "tether/io/SchemaValueCodec.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>

namespace tether::io::cia402 {
namespace {

TEST(CiA402ProfileTest, DriveSnapshotRoundTripsWithExplicitWireLayout) {
    DriveSnapshotV1 source;
    source.timestampUs = 123456789;
    source.stateGeneration = 42;
    source.slaveIndex = 3;
    source.alStatusCode = 0x2310;
    source.statusWord = 0x1637;
    source.controlWord = 0x000F;
    source.faultCode = 0x8611;
    source.qualityFlags = DriveQuality::Simulated | DriveQuality::Estimated;
    source.alState = 8;
    source.ds402State = 9;
    source.targetMode = 8;
    source.displayMode = 8;
    source.targetPosition = 100000;
    source.demandPosition = 99990;
    source.actualPosition = 99980;
    source.followingError = 20;
    source.targetVelocity = -1200;
    source.actualVelocity = -1198;
    source.targetTorque = -200;
    source.actualTorque = -198;
    source.homingState = 3;
    source.reserved = 0;

    std::array<uint8_t, DriveSnapshotV1::kEncodedSize> bytes{};
    BufWriter writer(bytes.data(), bytes.size());
    source.encode(writer);

    ASSERT_TRUE(writer.ok());
    ASSERT_EQ(writer.pos, bytes.size());

    BufReader reader(bytes.data(), bytes.size());
    DriveSnapshotV1 decoded;
    ASSERT_TRUE(DriveSnapshotV1::decode(reader, decoded));
    EXPECT_EQ(reader.remaining(), 0U);
    EXPECT_EQ(decoded.timestampUs, source.timestampUs);
    EXPECT_EQ(decoded.stateGeneration, source.stateGeneration);
    EXPECT_EQ(decoded.slaveIndex, source.slaveIndex);
    EXPECT_EQ(decoded.alStatusCode, source.alStatusCode);
    EXPECT_EQ(decoded.statusWord, source.statusWord);
    EXPECT_EQ(decoded.controlWord, source.controlWord);
    EXPECT_EQ(decoded.faultCode, source.faultCode);
    EXPECT_EQ(decoded.qualityFlags, source.qualityFlags);
    EXPECT_EQ(decoded.targetPosition, source.targetPosition);
    EXPECT_EQ(decoded.demandPosition, source.demandPosition);
    EXPECT_EQ(decoded.actualPosition, source.actualPosition);
    EXPECT_EQ(decoded.followingError, source.followingError);
    EXPECT_EQ(decoded.targetVelocity, source.targetVelocity);
    EXPECT_EQ(decoded.actualVelocity, source.actualVelocity);
    EXPECT_EQ(decoded.targetTorque, source.targetTorque);
    EXPECT_EQ(decoded.actualTorque, source.actualTorque);
}

TEST(CiA402ProfileTest, SchemaMatchesPackedSnapshotPayload) {
    const auto graph = driveSnapshotSchemaGraph();
    ASSERT_TRUE(validateSchemaGraph(graph));
    ASSERT_EQ(graph.nodes.size(), 8U);
    const auto& snapshot = graph.nodes.back();
    EXPECT_EQ(snapshot.name, kDriveSnapshotSchemaName);
    ASSERT_EQ(snapshot.fields.size(), 22U);
    EXPECT_EQ(snapshot.fields.front().name, "timestamp_us");
    EXPECT_EQ(snapshot.fields.back().name, "reserved");
    ASSERT_EQ(fixedSchemaSize(graph, snapshot.key), DriveSnapshotV1::kEncodedSize);

    std::array<uint8_t, DriveSnapshotV1::kEncodedSize> bytes{};
    BufReader reader(bytes.data(), bytes.size());
    EXPECT_TRUE(validateSchemaValue(graph, snapshot.key, reader));
    EXPECT_EQ(reader.remaining(), 0U);
}

TEST(CiA402ProfileTest, TruncatedSnapshotIsRejected) {
    std::array<uint8_t, DriveSnapshotV1::kEncodedSize - 1> bytes{};
    BufReader reader(bytes.data(), bytes.size());
    DriveSnapshotV1 decoded;

    EXPECT_FALSE(DriveSnapshotV1::decode(reader, decoded));
    EXPECT_TRUE(reader.ok());
}

} // namespace
} // namespace tether::io::cia402
