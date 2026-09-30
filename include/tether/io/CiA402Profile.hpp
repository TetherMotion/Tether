#pragma once

/**
 * @file CiA402Profile.hpp
 * @brief Fixed wire types for the machine.cia402.v1 IO profile.
 */

#include "tether/io/SchemaDigest.hpp"

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace tether::io::cia402 {

inline constexpr std::string_view kProfileName = "machine.cia402.v1";
inline constexpr uint32_t kProfileVersion = 1;
inline constexpr std::string_view kDriveSnapshotSchemaName =
    "tether.machine.cia402.DriveSnapshotV1";

inline SchemaKey driveSnapshotSchemaKey(uint8_t id) {
    SchemaKey key{};
    key[15] = id;
    return key;
}

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

/// Returns the dependency-closed V6 graph for the packed DriveSnapshotV1.
/// Its final node is the snapshot root; all prior nodes are reusable scalars.
inline SchemaGraph driveSnapshotSchemaGraph() {
    const auto makeScalar = [](uint8_t id, ValueType type) {
        SchemaNode node;
        node.key = driveSnapshotSchemaKey(id);
        node.kind = SchemaKind::Scalar;
        node.scalarType = type;
        return node;
    };
    SchemaNode u8 = makeScalar(1, ValueType::U8);
    SchemaNode u16 = makeScalar(2, ValueType::U16);
    SchemaNode u32 = makeScalar(3, ValueType::U32);
    SchemaNode u64 = makeScalar(4, ValueType::U64);
    SchemaNode i8 = makeScalar(5, ValueType::I8);
    SchemaNode i16 = makeScalar(6, ValueType::I16);
    SchemaNode i32 = makeScalar(7, ValueType::I32);

    const auto ref = [](const SchemaNode& node) {
        return SchemaRef{node.key, computeSchemaDigest(node)};
    };
    SchemaNode snapshot;
    snapshot.key = driveSnapshotSchemaKey(8);
    snapshot.kind = SchemaKind::Struct;
    snapshot.structEncoding = StructEncoding::Packed;
    snapshot.name = std::string{kDriveSnapshotSchemaName};
    snapshot.description = "Coherent CiA 402 drive status snapshot";
    snapshot.fields = {
        {1, 0, ref(u64), "timestamp_us", "Source timestamp in microseconds"},
        {2, 0, ref(u64), "state_generation", "Safety-relevant state generation"},
        {3, 0, ref(u16), "slave_index", "EtherCAT slave index"},
        {4, 0, ref(u16), "al_status_code", "EtherCAT AL status code"},
        {5, 0, ref(u16), "status_word", "CiA 402 statusword"},
        {6, 0, ref(u16), "control_word", "Applied controlword"},
        {7, 0, ref(u16), "fault_code", "CiA 402 fault code"},
        {8, 0, ref(u32), "quality_flags", "Drive quality bitmask"},
        {9, 0, ref(u8), "al_state", "EtherCAT AL state"},
        {10, 0, ref(u8), "ds402_state", "CiA 402 state"},
        {11, 0, ref(i8), "target_mode", "Requested operation mode"},
        {12, 0, ref(i8), "display_mode", "Displayed operation mode"},
        {13, 0, ref(i32), "target_position", "Target position in drive units"},
        {14, 0, ref(i32), "demand_position", "Demand position in drive units"},
        {15, 0, ref(i32), "actual_position", "Actual position in drive units"},
        {16, 0, ref(i32), "following_error", "Following error in drive units"},
        {17, 0, ref(i32), "target_velocity", "Target velocity in drive units/s"},
        {18, 0, ref(i32), "actual_velocity", "Actual velocity in drive units/s"},
        {19, 0, ref(i16), "target_torque", "Target torque in drive units"},
        {20, 0, ref(i16), "actual_torque", "Actual torque in drive units"},
        {21, 0, ref(u8), "homing_state", "Profile-defined homing state"},
        {22, 0, ref(u8), "reserved", "Reserved and zero in version 1"},
    };
    return {{std::move(u8), std::move(u16), std::move(u32), std::move(u64),
             std::move(i8), std::move(i16), std::move(i32), std::move(snapshot)}};
}

} // namespace tether::io::cia402
