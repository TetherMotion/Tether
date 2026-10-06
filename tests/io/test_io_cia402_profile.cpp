#include "tether/io/CiA402Profile.hpp"
#include "tether/io/CiA402MachineProfile.hpp"
#include "tether/io/Registry.hpp"
#include "tether/io/SchemaValueCodec.hpp"
#include "tether/io/SimulatedCiA402Fleet.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdio>
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

TEST(CiA402MachineProfileTest, SimulatedFleetPublishesSchemaValidatedReadOnlySignals) {
    const auto graph = machineProfileSchemaGraph();
    ASSERT_TRUE(validateSchemaGraph(graph));
    const auto manifest = machineProfileManifest(graph);
    ASSERT_EQ(manifest.size(), 31U);

    SchemaCatalog catalog;
    ASSERT_TRUE(SimulatedCiA402Fleet::installSchemas(graph, catalog));
    SimulatedCiA402Fleet fleet;
    Registry registry;
    ASSERT_TRUE(fleet.registerSignals(registry, catalog));
    ASSERT_EQ(fleet.axisCount(), 4U);
    ASSERT_EQ(registry.signalCount(), 7U);
    EXPECT_EQ(registry.paramCount(), 0U);
    ASSERT_EQ(registry.functionCount(), 1U);

    for (uint32_t offset = 0; offset < registry.signalCount(); ++offset) {
        const auto page = registry.signalPage(offset, 1);
        ASSERT_EQ(page.size(), 1U);
        const auto entry = page.front();
        if (entry.valueType() == ValueType::U64) {
            EXPECT_EQ(entry.name(), "machine.events.cursor");
            uint64_t cursor = 0;
            entry.read(&cursor);
            EXPECT_EQ(cursor, 1U);
            continue;
        }
        ASSERT_EQ(entry.valueType(), ValueType::Struct);
        ASSERT_FALSE(entry.writable());
        const auto* schema = catalog.resolve(catalog.epoch(), entry.schemaSlot());
        ASSERT_NE(schema, nullptr);
        EXPECT_EQ(schema->name, entry.name() == "machine.descriptor"
            ? kMachineDescriptorSchemaName
            : entry.name() == "machine.snapshot" ? kMachineSnapshotSchemaName
            : entry.name() == "machine.events.read" ? "tether.machine.cia402.EventPageV1"
                                                       : kDriveSnapshotSchemaName);

        std::vector<uint8_t> value(entry.maxValueSize());
        const size_t size = entry.readVar(value.data(), value.size());
        ASSERT_GT(size, 0U);
        value.resize(size);
        BufReader reader(value.data(), value.size());
        EXPECT_TRUE(validateSchemaValue(graph, schema->key, reader));
        EXPECT_EQ(reader.remaining(), 0U);
    }

    const auto drive = registry.findSignal(SimulatedCiA402Fleet::kDriveSignalIdBase + 1);
    ASSERT_TRUE(drive);
    std::array<uint8_t, DriveSnapshotV1::kEncodedSize> driveBytes{};
    const size_t driveSize = drive.readVar(driveBytes.data(), driveBytes.size());
    ASSERT_EQ(driveSize, DriveSnapshotV1::kEncodedSize);
    BufReader driveReader(driveBytes.data(), driveSize);
    DriveSnapshotV1 snapshot;
    ASSERT_TRUE(DriveSnapshotV1::decode(driveReader, snapshot));
    EXPECT_EQ(snapshot.slaveIndex, 0U);
    EXPECT_NE(snapshot.qualityFlags & static_cast<uint32_t>(DriveQuality::Simulated), 0U);

    snapshot.qualityFlags |= static_cast<uint32_t>(DriveQuality::Stale);
    ASSERT_TRUE(fleet.updateDriveSnapshot("sim-axis-x", snapshot));
    EXPECT_FALSE(fleet.updateDriveSnapshot("unknown-axis", snapshot));

    fleet.appendSimulationEvent("OperatorNote", "simulated-cia402-machine",
                                EventSeverity::Info, 0, "Test event");
    const auto readEvents = registry.findFunction(SimulatedCiA402Fleet::kEventReadFunctionId);
    ASSERT_TRUE(readEvents);
    ASSERT_EQ(readEvents.returnValue().schemaSlot,
              *catalog.slotFor(SchemaRef{schemaKey(MachineSchemaId::EventPage), computeSchemaDigest(
                  *graph.find(schemaKey(MachineSchemaId::EventPage)))}));
    std::vector<FunctionArgument> arguments(2);
    arguments[0].position = 0;
    arguments[0].type = ValueType::U64;
    arguments[0].value = std::vector<uint8_t>(8, 0);
    arguments[1].position = 1;
    arguments[1].type = ValueType::U32;
    arguments[1].value = {100, 0, 0, 0};
    const auto eventPage = readEvents.invoke(arguments);
    ASSERT_TRUE(eventPage.success) << eventPage.errorMessage;
    const auto* eventPageSchema = graph.find(schemaKey(MachineSchemaId::EventPage));
    ASSERT_NE(eventPageSchema, nullptr);
    BufReader eventPageReader(eventPage.returnValue.data(), eventPage.returnValue.size());
    EXPECT_TRUE(validateSchemaValue(graph, eventPageSchema->key, eventPageReader));
    EXPECT_EQ(eventPageReader.remaining(), 0U);
}

/// Cross-language digest fixtures: the browser client recomputes the same
/// canonical descriptor + BLAKE3 digests (schema-v6/handshake.ts). These
/// constants are mirrored in src/schema-v6/ of the TetherWebUI repo
/// handshake.test.ts — a change here must change there, or catalogs
/// negotiated between the implementations will reject each other.
TEST(CiA402MachineProfileTest, DigestFixturesMatchBrowserClient) {
    const auto graph = machineProfileSchemaGraph();
    const auto hex = [](const SchemaDigest& digest) {
        std::string out;
        for (const auto byte : digest) {
            char buf[3];
            std::snprintf(buf, sizeof(buf), "%02x", byte);
            out += buf;
        }
        return out;
    };
    // Scalar nodes: stable, dependency-free fixtures.
    EXPECT_EQ(hex(computeSchemaDigest(*graph.find(schemaKey(MachineSchemaId::Bool)))),
              "d41be435b890f0b0053c078844c3ad08b5ac355d6ec4bb209e58fb677df55fa1");
    EXPECT_EQ(hex(computeSchemaDigest(*graph.find(schemaKey(MachineSchemaId::F64)))),
              "b16d032d88099a23ae8358d9b42247e5f15ee9749f84b009807d9a6ebeda7b25");
    // String node exercises the String kind + maxBytes path.
    EXPECT_EQ(hex(computeSchemaDigest(*graph.find(schemaKey(MachineSchemaId::String128)))),
              "fff72e569203f9a95998a207598a2804d3c5cb84e94568f0f1b90ddac3a0cea5");
    // A struct with field refs exercises the full descriptor path.
    EXPECT_EQ(hex(computeSchemaDigest(*graph.find(schemaKey(MachineSchemaId::AuthorityLease)))),
              "4f29338832c015926f717cc39d5f0a37c4101417dcc3dcf04ee9cfa35027def4");
}

} // namespace
} // namespace tether::io::cia402
