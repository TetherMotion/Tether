#include "tether/io/CiA402Profile.hpp"

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

TEST(CiA402ProfileTest, DescriptorMatchesEncodedOffsetsAndSchema) {
    const auto descriptor = driveSnapshotDescriptor(0x1234);

    EXPECT_EQ(descriptor.entryId, 0x1234U);
    EXPECT_EQ(descriptor.name, kDriveSnapshotSchema);
    EXPECT_EQ(descriptor.totalSize, DriveSnapshotV1::kEncodedSize);
    ASSERT_EQ(descriptor.fields.size(), 22U);
    EXPECT_EQ(descriptor.fields.front().name, "timestamp_us");
    EXPECT_EQ(descriptor.fields.front().offset, 0U);
    EXPECT_EQ(descriptor.fields.back().name, "reserved");
    EXPECT_EQ(descriptor.fields.back().offset, 63U);
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
