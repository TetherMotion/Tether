#pragma once

/**
 * @file CiA402Profile.hpp
 * @brief Fixed wire types for the machine.cia402.v1 IO profile.
 */

#include "tether/io/BinaryStruct.hpp"
#include "tether/io/Protocol.hpp"

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace tether::io::cia402 {

inline constexpr std::string_view kProfileName = "machine.cia402.v1";
inline constexpr uint32_t kProfileVersion = 1;
inline constexpr std::string_view kDriveSnapshotSchema =
    "tether.machine.cia402.DriveSnapshotV1";

/// Quality flags carried by DriveSnapshotV1::qualityFlags.
enum class DriveQuality : uint32_t {
    None = 0,
    Stale = 1u << 0,
    Simulated = 1u << 1,
    Estimated = 1u << 2,
    EthercatDegraded = 1u << 3,
    UnitConversionFailed = 1u << 4,
};

constexpr uint32_t operator|(DriveQuality lhs, DriveQuality rhs) {
    return static_cast<uint32_t>(lhs) | static_cast<uint32_t>(rhs);
}

/**
 * Fixed-layout drive state shared by the generic IO client and CiA 402 UI.
 *
 * Values are encoded explicitly in little-endian order; C++ object layout is
 * not part of the wire contract. Positions and velocities are drive-native
 * integer units. The application descriptor supplies engineering conversion.
 */
struct DriveSnapshotV1 {
    static constexpr size_t kEncodedSize = 64;

    uint64_t timestampUs = 0;
    uint64_t stateGeneration = 0;
    uint16_t slaveIndex = 0;
    uint16_t alStatusCode = 0;
    uint16_t statusWord = 0;
    uint16_t controlWord = 0;
    uint16_t faultCode = 0;
    uint32_t qualityFlags = 0;
    uint8_t alState = 0;
    uint8_t ds402State = 0;
    int8_t targetMode = 0;
    int8_t displayMode = 0;
    int32_t targetPosition = 0;
    int32_t demandPosition = 0;
    int32_t actualPosition = 0;
    int32_t followingError = 0;
    int32_t targetVelocity = 0;
    int32_t actualVelocity = 0;
    int16_t targetTorque = 0;
    int16_t actualTorque = 0;
    uint8_t homingState = 0;
    uint8_t reserved = 0;

    void encode(BufWriter& writer) const {
        writer.putU64(timestampUs);
        writer.putU64(stateGeneration);
        writer.putU16(slaveIndex);
        writer.putU16(alStatusCode);
        writer.putU16(statusWord);
        writer.putU16(controlWord);
        writer.putU16(faultCode);
        writer.putU32(qualityFlags);
        writer.putU8(alState);
        writer.putU8(ds402State);
        writer.putU8(static_cast<uint8_t>(targetMode));
        writer.putU8(static_cast<uint8_t>(displayMode));
        writer.putI32(targetPosition);
        writer.putI32(demandPosition);
        writer.putI32(actualPosition);
        writer.putI32(followingError);
        writer.putI32(targetVelocity);
        writer.putI32(actualVelocity);
        writer.putU16(static_cast<uint16_t>(targetTorque));
        writer.putU16(static_cast<uint16_t>(actualTorque));
        writer.putU8(homingState);
        writer.putU8(reserved);
    }

    static bool decode(BufReader& reader, DriveSnapshotV1& value) {
        if (reader.remaining() < kEncodedSize) return false;
        value.timestampUs = reader.getU64();
        value.stateGeneration = reader.getU64();
        value.slaveIndex = reader.getU16();
        value.alStatusCode = reader.getU16();
        value.statusWord = reader.getU16();
        value.controlWord = reader.getU16();
        value.faultCode = reader.getU16();
        value.qualityFlags = reader.getU32();
        value.alState = reader.getU8();
        value.ds402State = reader.getU8();
        value.targetMode = static_cast<int8_t>(reader.getU8());
        value.displayMode = static_cast<int8_t>(reader.getU8());
        value.targetPosition = reader.getI32();
        value.demandPosition = reader.getI32();
        value.actualPosition = reader.getI32();
        value.followingError = reader.getI32();
        value.targetVelocity = reader.getI32();
        value.actualVelocity = reader.getI32();
        value.targetTorque = static_cast<int16_t>(reader.getU16());
        value.actualTorque = static_cast<int16_t>(reader.getU16());
        value.homingState = reader.getU8();
        value.reserved = reader.getU8();
        return reader.ok();
    }
};

static_assert(DriveSnapshotV1::kEncodedSize == 64);

/// Return the fixed field descriptor used by DescribeStruct.
inline StructDescriptor driveSnapshotDescriptor(uint64_t entryId) {
    StructDescriptor descriptor;
    descriptor.entryId = entryId;
    descriptor.name = std::string{kDriveSnapshotSchema};
    descriptor.totalSize = DriveSnapshotV1::kEncodedSize;
    descriptor.fields = {
        {"timestamp_us", ValueType::U64, 0, 8, "us"},
        {"state_generation", ValueType::U64, 8, 8, ""},
        {"slave_index", ValueType::U16, 16, 2, ""},
        {"al_status_code", ValueType::U16, 18, 2, ""},
        {"status_word", ValueType::U16, 20, 2, ""},
        {"control_word", ValueType::U16, 22, 2, ""},
        {"fault_code", ValueType::U16, 24, 2, ""},
        {"quality_flags", ValueType::U32, 26, 4, "bitmask"},
        {"al_state", ValueType::U8, 30, 1, "enum"},
        {"ds402_state", ValueType::U8, 31, 1, "enum"},
        {"target_mode", ValueType::I8, 32, 1, "mode"},
        {"display_mode", ValueType::I8, 33, 1, "mode"},
        {"target_position", ValueType::I32, 34, 4, "drive-units"},
        {"demand_position", ValueType::I32, 38, 4, "drive-units"},
        {"actual_position", ValueType::I32, 42, 4, "drive-units"},
        {"following_error", ValueType::I32, 46, 4, "drive-units"},
        {"target_velocity", ValueType::I32, 50, 4, "drive-units/s"},
        {"actual_velocity", ValueType::I32, 54, 4, "drive-units/s"},
        {"target_torque", ValueType::I16, 58, 2, "drive-units"},
        {"actual_torque", ValueType::I16, 60, 2, "drive-units"},
        {"homing_state", ValueType::U8, 62, 1, "enum"},
        {"reserved", ValueType::U8, 63, 1, ""},
    };
    return descriptor;
}

} // namespace tether::io::cia402
