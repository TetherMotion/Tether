#include <gtest/gtest.h>

#include "tether/io/MachineAuth.hpp"
#include "tether/io/Registry.hpp"
#include "tether/io/SchemaCatalog.hpp"
#include "tether/io/SimulatedMachineService.hpp"
#include "tether/io/EtherCatSdoAdapter.hpp"  // compile coverage for the master-backed adapter
// Compile-check the real-machine adapter surface (not instantiated here —
#include "tether/io/EtherCatDiagnosticsAdapter.hpp"  // compile coverage for the master-backed adapter
#include "tether/io/EtherCatSupervisionAdapter.hpp"  // compile coverage for the master-backed adapters
// it requires a live EtherCAT master).
#include "tether/io/DS402MachineAdapter.hpp"

#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>
#include <thread>
#include <vector>

using namespace tether::io;
using namespace tether::io::machine;
using namespace tether::io::machine::detail;

namespace {

SessionIdentity operatorSession(std::string id = "session-1") {
    return {std::move(id), "operator-a", "127.0.0.1", Role::Operator, true};
}

SessionIdentity observerSession(std::string id = "session-obs") {
    return {std::move(id), "observer", "127.0.0.1", Role::Observer, true};
}

SessionIdentity engineerSession(std::string id = "session-eng") {
    return {std::move(id), "engineer-a", "127.0.0.1", Role::ControlsEngineer, true};
}

SessionIdentity technicianSession(std::string id = "session-tech") {
    return {std::move(id), "technician-a", "127.0.0.1", Role::Technician, true};
}

FunctionInvokeContext contextFor(SessionIdentity identity) {
    FunctionInvokeContext context;
    context.identity = std::move(identity);
    return context;
}

FunctionArgument arg(uint32_t key, std::vector<uint8_t> value) {
    FunctionArgument argument;
    argument.key = key;
    argument.value = std::move(value);
    argument.provided = true;
    return argument;
}

FunctionArgument stringArg(uint32_t key, std::string_view value) {
    return arg(key, fieldString(value));
}

FunctionArgument u32Arg(uint32_t key, uint32_t value) { return arg(key, fieldScalar(value)); }
FunctionArgument u64Arg(uint32_t key, uint64_t value) { return arg(key, fieldScalar(value)); }

std::vector<uint8_t> stringArray(const std::vector<std::string>& values) {
    std::vector<std::vector<uint8_t>> elements;
    for (const auto& value : values) elements.push_back(fieldString(value));
    return encodeArray(elements);
}

std::vector<uint8_t> u64Array(const std::vector<uint64_t>& values) {
    std::vector<std::vector<uint8_t>> elements;
    for (const auto& value : values) elements.push_back(fieldScalar(value));
    return encodeArray(elements);
}

/// Decode a wire dynamic array into raw element payloads.
std::vector<std::vector<uint8_t>> elementArray(const std::vector<uint8_t>& packed) {
    size_t position = 0;
    const auto varint = [&]() -> uint64_t {
        uint64_t value = 0;
        for (unsigned i = 0; i < 10 && position < packed.size(); ++i) {
            const uint8_t byte = packed[position++];
            value |= static_cast<uint64_t>(byte & 0x7F) << (i * 7U);
            if ((byte & 0x80) == 0) break;
        }
        return value;
    };
    const uint64_t count = varint();
    std::vector<std::vector<uint8_t>> elements;
    for (uint64_t i = 0; i < count; ++i) {
        const uint64_t length = varint();
        if (position + length > packed.size()) break;
        elements.emplace_back(packed.begin() + static_cast<ptrdiff_t>(position),
                              packed.begin() + static_cast<ptrdiff_t>(position + length));
        position += static_cast<size_t>(length);
    }
    return elements;
}

/// Build a CommandRequestV1 tagged-struct payload.
std::vector<uint8_t> commandRequest(std::string_view uuid, std::string_view scope,
                                    uint64_t token, uint8_t action,
                                    const std::vector<std::string>& targets,
                                    const std::vector<uint64_t>& generations,
                                    uint64_t deadlineUs,
                                    const std::vector<uint8_t>& parameters = {}) {
    return encodeTagged({
        {1, fieldString(uuid)},
        {2, fieldString(scope)},
        {3, fieldScalar(token)},
        {4, fieldScalar(action)},
        {5, stringArray(targets)},
        {6, u64Array(generations)},
        {7, fieldScalar(deadlineUs)},
        {9, fieldBytes(parameters)},
    });
}

struct CommandReceiptFields {
    std::string requestUuid;
    uint8_t state = 0xFF;
    std::string message;
    std::string operationId;
    std::string auditId;
    std::vector<std::string> blockers;
};

CommandReceiptFields decodeReceipt(const std::vector<uint8_t>& bytes) {
    std::map<uint32_t, std::vector<uint8_t>> fields;
    EXPECT_TRUE(parseTagged(bytes, fields));
    CommandReceiptFields receipt;
    getString(fields[1], receipt.requestUuid);
    getScalar(fields[2], receipt.state);
    getString(fields[3], receipt.message);
    getString(fields[4], receipt.operationId);
    getString(fields[5], receipt.auditId);
    // Decode blocker string array
    const auto& packed = fields[6];
    size_t position = 0;
    const auto varint = [&]() -> uint64_t {
        uint64_t value = 0;
        unsigned shift = 0;
        while (position < packed.size()) {
            const uint8_t byte = packed[position++];
            value |= static_cast<uint64_t>(byte & 0x7F) << shift;
            if ((byte & 0x80) == 0) break;
            shift += 7;
        }
        return value;
    };
    const uint64_t count = varint();
    for (uint64_t index = 0; index < count; ++index) {
        const uint64_t length = varint();
        if (position + length > packed.size()) break;
        std::string blocker;
        getString(std::vector<uint8_t>(packed.begin() + static_cast<ptrdiff_t>(position),
                                       packed.begin() + static_cast<ptrdiff_t>(position + length)),
                  blocker);
        receipt.blockers.push_back(std::move(blocker));
        position += static_cast<size_t>(length);
    }
    return receipt;
}

class SimulatedMachineServiceTest : public ::testing::Test {
protected:
    void SetUp() override {
        // The catalog references the graph by pointer; it must outlive it.
        graph_ = cia402::machineProfileSchemaGraph();
        ASSERT_TRUE(cia402::SimulatedCiA402Fleet::installSchemas(graph_, catalog_));
        ASSERT_TRUE(stack_.fleet.registerSignals(registry_, catalog_));
        ASSERT_TRUE(stack_.install(registry_, catalog_));

        ParamEntry writable{};
        writable.id = kWritableParamId;
        writable.name = "test.writable_u32";
        writable.group = "test";
        writable.valueType = ValueType::U32;
        writable.readFn = [this](void* destination) {
            std::memcpy(destination, &paramValue_, sizeof(paramValue_));
        };
        writable.writeFn = [this](const void* source) {
            std::memcpy(&paramValue_, source, sizeof(paramValue_));
        };
        ASSERT_TRUE(registry_.addParam(std::move(writable)));

        ParamEntry readOnly{};
        readOnly.id = kReadOnlyParamId;
        readOnly.name = "test.read_only_u32";
        readOnly.group = "test";
        readOnly.valueType = ValueType::U32;
        readOnly.readFn = [this](void* destination) {
            std::memcpy(destination, &paramValue_, sizeof(paramValue_));
        };
        ASSERT_TRUE(registry_.addParam(std::move(readOnly)));
    }

    static constexpr uint64_t kWritableParamId = 0x5445535400000001ULL;
    static constexpr uint64_t kReadOnlyParamId = 0x5445535400000002ULL;

    FunctionCallResult invoke(uint64_t functionId, std::vector<FunctionArgument> arguments,
                              SessionIdentity identity) {
        const FunctionView function = registry_.findFunction(functionId);
        EXPECT_TRUE(function) << "function " << functionId << " not registered";
        return function.invoke(arguments, contextFor(std::move(identity)));
    }

    CommandReceiptFields sendCommand(const SessionIdentity& identity, uint64_t token,
                                     uint8_t action,
                                     const std::vector<std::string>& targets,
                                     const std::vector<uint64_t>& generations,
                                     std::string_view uuid = "req-1",
                                     std::string_view scope = "machine",
                                     const std::vector<uint8_t>& parameters = {}) {
        const auto request = commandRequest(uuid, scope, token, action, targets, generations,
                                            5'000'000, parameters);
        const auto result = invoke(MachineService::kCommandFnId,
                                   {arg(1, request)}, identity);
        EXPECT_TRUE(result.success) << result.errorMessage;
        return decodeReceipt(result.returnValue);
    }

    std::vector<uint64_t> generationsFor(const std::vector<std::string>& targets) {
        std::vector<uint64_t> out;
        for (const auto& target : targets) {
            const auto drive = stack_.fleet.driveSnapshot(target);
            EXPECT_TRUE(drive);
            out.push_back(drive ? drive->stateGeneration : 0);
        }
        return out;
    }

    uint64_t acquire(const SessionIdentity& identity, std::string_view scope = "machine") {
        const auto result = invoke(MachineService::kAuthorityAcquireFnId,
                                   {stringArg(1, scope), u32Arg(2, 30'000)}, identity);
        EXPECT_TRUE(result.success) << result.errorMessage;
        std::map<uint32_t, std::vector<uint8_t>> fields;
        EXPECT_TRUE(parseTagged(result.returnValue, fields));
        bool granted = false;
        uint64_t token = 0;
        getBool(fields[1], granted);
        getScalar(fields[3], token);
        if (!granted) return 0;
        return token;
    }

    /// Invoke machine.config.stage with encoded ConfigEntryV1 elements.
    FunctionCallResult stageConfig(
        const std::vector<std::pair<uint64_t, std::vector<uint8_t>>>& writes,
        SessionIdentity identity) {
        std::vector<std::vector<uint8_t>> elements;
        for (const auto& [id, value] : writes) {
            elements.push_back(encodeTagged({
                {1, fieldScalar(id)},
                {2, fieldBytes(value)},
            }));
        }
        return invoke(MachineService::kConfigStageFnId, {arg(1, encodeArray(elements))},
                      std::move(identity));
    }

    SchemaGraph graph_;
    SchemaCatalog catalog_;
    Registry registry_;
    SimulatedMachineStack stack_;
    uint32_t paramValue_ = 7;
};

TEST_F(SimulatedMachineServiceTest, AuthorityRequiresAuthenticatedEligibleRole) {
    // Unauthenticated observer is refused.
    auto anonymous = observerSession();
    anonymous.authenticated = false;
    EXPECT_EQ(acquire(anonymous), 0u);
    // Authenticated observer still cannot take motion authority.
    EXPECT_EQ(acquire(observerSession()), 0u);
    // Operator can.
    EXPECT_NE(acquire(operatorSession()), 0u);
}

TEST_F(SimulatedMachineServiceTest, LeaseConflictsAndReleaseViaFunction) {
    const auto owner = operatorSession("owner");
    const auto other = operatorSession("other");
    const uint64_t token = acquire(owner);
    ASSERT_NE(token, 0u);
    EXPECT_EQ(acquire(other), 0u);

    const auto released = invoke(MachineService::kAuthorityReleaseFnId,
                                 {u64Arg(1, token), stringArg(2, "machine")}, owner);
    ASSERT_TRUE(released.success);
    std::map<uint32_t, std::vector<uint8_t>> fields;
    ASSERT_TRUE(parseTagged(released.returnValue, fields));
    bool granted = false;
    getBool(fields[1], granted);
    EXPECT_TRUE(granted);
    EXPECT_NE(acquire(other), 0u);
}

TEST_F(SimulatedMachineServiceTest, CommandWithoutAuthorityIsRejectedWithBlockers) {
    const auto receipt = sendCommand(operatorSession(), 0,
                                     static_cast<uint8_t>(Action::Disable),
                                     {"sim-axis-x"}, generationsFor({"sim-axis-x"}));
    EXPECT_EQ(receipt.state, static_cast<uint8_t>(CommandState::Rejected));
    EXPECT_FALSE(receipt.blockers.empty());
    EXPECT_TRUE(receipt.operationId.empty());
}

TEST_F(SimulatedMachineServiceTest, AuthorizedCommandDispatchesAndCompletesOperation) {
    const auto identity = operatorSession();
    const uint64_t token = acquire(identity);
    ASSERT_NE(token, 0u);

    const auto receipt = sendCommand(identity, token,
                                     static_cast<uint8_t>(Action::Disable),
                                     {"sim-axis-x"}, generationsFor({"sim-axis-x"}));
    EXPECT_EQ(receipt.state, static_cast<uint8_t>(CommandState::Accepted))
        << receipt.message << " | blockers=" << receipt.blockers.size();
    EXPECT_FALSE(receipt.operationId.empty());
    EXPECT_FALSE(receipt.auditId.empty());

    const auto drive = stack_.fleet.driveSnapshot("sim-axis-x");
    ASSERT_TRUE(drive);
    EXPECT_EQ(drive->ds402State, 2u);

    const auto operation = invoke(MachineService::kOperationReadFnId,
                                  {stringArg(1, receipt.operationId)}, observerSession());
    ASSERT_TRUE(operation.success);
    std::map<uint32_t, std::vector<uint8_t>> fields;
    ASSERT_TRUE(parseTagged(operation.returnValue, fields));
    uint8_t state = 0;
    getScalar(fields[3], state);
    EXPECT_EQ(state, static_cast<uint8_t>(OperationState::Completed));
}

TEST_F(SimulatedMachineServiceTest, StaleGenerationIsRejectedAndDoesNotDispatch) {
    const auto identity = operatorSession();
    const uint64_t token = acquire(identity);
    ASSERT_NE(token, 0u);
    const auto before = stack_.fleet.driveSnapshot("sim-axis-x");
    ASSERT_TRUE(before);

    const auto receipt = sendCommand(identity, token,
                                     static_cast<uint8_t>(Action::Disable),
                                     {"sim-axis-x"}, {before->stateGeneration + 99});
    EXPECT_EQ(receipt.state, static_cast<uint8_t>(CommandState::Rejected));
    const auto after = stack_.fleet.driveSnapshot("sim-axis-x");
    EXPECT_EQ(before->ds402State, after->ds402State);
}

TEST_F(SimulatedMachineServiceTest, DuplicateRequestUuidReturnsOriginalReceipt) {
    const auto identity = operatorSession();
    const uint64_t token = acquire(identity);
    ASSERT_NE(token, 0u);
    const auto first = sendCommand(identity, token,
                                   static_cast<uint8_t>(Action::Disable),
                                   {"sim-axis-x"}, generationsFor({"sim-axis-x"}), "dup-1");
    ASSERT_EQ(first.state, static_cast<uint8_t>(CommandState::Accepted));

    // Enable again under a fresh generation, then retry the original UUID —
    // the cached receipt must come back unchanged, not dispatch a second time.
    const uint64_t token2 = token;
    const auto second = sendCommand(identity, token2,
                                    static_cast<uint8_t>(Action::Enable),
                                    {"sim-axis-x"}, generationsFor({"sim-axis-x"}), "dup-1");
    EXPECT_EQ(second.requestUuid, first.requestUuid);
    EXPECT_EQ(second.operationId, first.operationId);
}

TEST_F(SimulatedMachineServiceTest, AlarmAcknowledgeRequiresOperatorRole) {
    const uint64_t alarmId = stack_.alarms.raise(EventSeverity::Fault, 0x6010,
                                               "sim-axis-x", "Test fault");
    // Observer is denied at the function boundary.
    const auto denied = invoke(MachineService::kAlarmAckFnId,
                               {u64Arg(1, alarmId)}, observerSession());
    EXPECT_FALSE(denied.success);

    const auto acked = invoke(MachineService::kAlarmAckFnId,
                              {u64Arg(1, alarmId)}, operatorSession());
    ASSERT_TRUE(acked.success);
    uint8_t status = 0xFF;
    getScalar(acked.returnValue, status);
    EXPECT_EQ(status, 0u);

    const auto page = stack_.alarms.readAfter(0, 10);
    ASSERT_EQ(page.alarms.size(), 1u);
    EXPECT_EQ(page.alarms[0].state, AlarmState::Acknowledged);
    EXPECT_EQ(page.alarms[0].actor, "operator-a");
}

TEST_F(SimulatedMachineServiceTest, AlarmPageReportsGapAndCursor) {
    stack_.alarms.raise(EventSeverity::Warning, 1, "sim-axis-y", "first");
    stack_.alarms.raise(EventSeverity::Fault, 2, "sim-axis-y", "second");
    const auto page = stack_.alarms.readAfter(0, 100);
    EXPECT_EQ(page.alarms.size(), 2u);
    EXPECT_FALSE(page.gap);
    EXPECT_GT(page.latestCursor, 0u);

    // Acknowledge through the typed function.
    const auto read = invoke(MachineService::kAlarmReadFnId,
                             {u64Arg(1, 0), u32Arg(2, 100)}, observerSession());
    ASSERT_TRUE(read.success);
}

TEST_F(SimulatedMachineServiceTest, AlarmNotifierDedupEscalatesAndAudits) {
    struct Recorder final : IAlarmNotifier {
        std::vector<AlarmNotification> kinds;
        void notify(const AlarmRecordV1&, AlarmNotification kind) override {
            kinds.push_back(kind);
        }
    } recorder;
    AlarmNotifyPolicy policy;
    policy.escalationDelayUs = 1'000;
    policy.escalationIntervalUs = 500;
    policy.maxEscalations = 2;
    stack_.alarms.setNotifier(&recorder, policy, &stack_.fleet.eventJournal());

    // Raise → one Raised notification; re-raise while active is deduplicated.
    const uint64_t alarmId =
        stack_.alarms.raise(EventSeverity::Fault, 7, "sim-axis-x", "drive fault",
                            /*nowUs=*/100'000);
    EXPECT_EQ(recorder.kinds.size(), 1u);
    stack_.alarms.raise(EventSeverity::Fault, 7, "sim-axis-x", "drive fault",
                        101'000);
    EXPECT_EQ(recorder.kinds.size(), 1u);
    EXPECT_EQ(recorder.kinds[0], AlarmNotification::Raised);

    // Below the severity floor: no notification at all.
    stack_.alarms.raise(EventSeverity::Info, 8, "sim-axis-x", "noise", 102'000);
    EXPECT_EQ(recorder.kinds.size(), 1u);

    // Escalation: before the delay nothing; past it, bounded repeats.
    stack_.alarms.tick(100'999);
    EXPECT_EQ(recorder.kinds.size(), 1u);
    stack_.alarms.tick(101'000);
    stack_.alarms.tick(101'400);  // within interval — suppressed
    EXPECT_EQ(recorder.kinds.size(), 2u);
    stack_.alarms.tick(101'600);
    EXPECT_EQ(recorder.kinds.size(), 3u);
    stack_.alarms.tick(200'000);  // maxEscalations reached
    EXPECT_EQ(recorder.kinds.size(), 3u);
    EXPECT_EQ(recorder.kinds[1], AlarmNotification::Escalated);
    EXPECT_EQ(recorder.kinds[2], AlarmNotification::Escalated);

    // Acknowledgement notifies and stops further escalation.
    EXPECT_EQ(stack_.alarms.acknowledge(alarmId, operatorSession(), 300'000), 0u);
    EXPECT_EQ(recorder.kinds.back(), AlarmNotification::Acknowledged);
    stack_.alarms.tick(400'000);
    EXPECT_EQ(recorder.kinds.size(), 4u);

    // Denied acknowledgement emits nothing.
    EXPECT_EQ(stack_.alarms.acknowledge(alarmId, observerSession(), 400'000), 2u);
    EXPECT_EQ(recorder.kinds.size(), 4u);

    // Every emission is journaled for audit.
    size_t notified = 0;
    for (const auto& event : stack_.fleet.eventJournal().readAfter(0, 512).events)
        if (event.eventType.rfind("alarm.notify.", 0) == 0) ++notified;
    EXPECT_EQ(notified, 4u);

    // Clear notifies once.
    EXPECT_EQ(stack_.alarms.clear(alarmId, operatorSession(), 500'000), 0u);
    EXPECT_EQ(recorder.kinds.back(), AlarmNotification::Cleared);
}

TEST_F(SimulatedMachineServiceTest, SessionEndedReleasesLeaseAndCancelsJog) {
    const auto owner = operatorSession("owner");
    const uint64_t token = acquire(owner);
    ASSERT_NE(token, 0u);

    // Start a continuous jog: the operation stays Running.
    std::vector<uint8_t> parameters;
    putParameter(parameters, 4, static_cast<uint32_t>(500), 4);  // velocity
    putParameter(parameters, 5, 1000, 4);                        // duration ms
    const auto receipt = sendCommand(owner, token,
                                     static_cast<uint8_t>(Action::JogStart),
                                     {"sim-axis-x"}, generationsFor({"sim-axis-x"}),
                                     "jog-1", "machine", parameters);
    ASSERT_EQ(receipt.state, static_cast<uint8_t>(CommandState::Accepted));

    stack_.service.sessionEnded(owner.sessionId);
    EXPECT_FALSE(stack_.authority.current("machine", SteadyClock::now()));
    const auto operation = stack_.operations.get(receipt.operationId);
    ASSERT_TRUE(operation);
    EXPECT_EQ(operation->state, OperationState::Cancelled);
}

TEST_F(SimulatedMachineServiceTest, SessionEndedCancelsEveryInFlightJog) {
    const auto owner = operatorSession("owner-multi");
    const uint64_t token = acquire(owner);
    ASSERT_NE(token, 0u);

    const auto jog = [&](const char* axis, const char* uuid) {
        std::vector<uint8_t> parameters;
        putParameter(parameters, 4, static_cast<uint32_t>(500), 4);
        putParameter(parameters, 5, 1000, 4);
        return sendCommand(owner, token, static_cast<uint8_t>(Action::JogStart),
                           {axis}, generationsFor({axis}), uuid, "machine", parameters);
    };
    const auto jogX = jog("sim-axis-x", "jog-x");
    const auto jogY = jog("sim-axis-y", "jog-y");
    ASSERT_EQ(jogX.state, static_cast<uint8_t>(CommandState::Accepted));
    ASSERT_EQ(jogY.state, static_cast<uint8_t>(CommandState::Accepted));

    stack_.service.sessionEnded(owner.sessionId);
    for (const auto& id : {jogX.operationId, jogY.operationId}) {
        const auto operation = stack_.operations.get(id);
        ASSERT_TRUE(operation) << "operation " << id << " is missing";
        EXPECT_EQ(operation->state, OperationState::Cancelled);
        EXPECT_EQ(operation->message, "Owner session disconnected");
    }
}

TEST_F(SimulatedMachineServiceTest, SessionEndedFreesScopeForNewOwner) {
    const auto owner = operatorSession("owner-a");
    const auto other = operatorSession("owner-b");
    ASSERT_NE(acquire(owner), 0u);
    // The same scope conflicts while another session holds it.
    EXPECT_EQ(acquire(other), 0u);
    stack_.service.sessionEnded(owner.sessionId);
    EXPECT_NE(acquire(other), 0u);
}

TEST_F(SimulatedMachineServiceTest, CommandsWithReleasedTokenAreRejected) {
    const auto owner = operatorSession("owner-stale-token");
    const uint64_t token = acquire(owner);
    ASSERT_NE(token, 0u);
    stack_.service.sessionEnded(owner.sessionId);

    const auto receipt = sendCommand(owner, token,
                                     static_cast<uint8_t>(Action::Enable),
                                     {"sim-axis-x"}, generationsFor({"sim-axis-x"}),
                                     "post-disconnect-cmd");
    EXPECT_EQ(receipt.state, static_cast<uint8_t>(CommandState::Rejected));
    ASSERT_FALSE(receipt.blockers.empty());
}

TEST_F(SimulatedMachineServiceTest, ExpiredLeaseRejectsCommandsAndFreesScope) {
    const auto owner = operatorSession("owner-expired");
    // Acquire directly through the authority with an expiry already in the
    // past relative to the steady clock used by the command gate.
    const auto past = SteadyClock::now() - std::chrono::seconds(2);
    const auto acquired = stack_.authority.acquire("machine", owner,
                                                 std::chrono::milliseconds(1), past);
    ASSERT_TRUE(acquired.lease);

    const auto receipt = sendCommand(owner, acquired.lease->token,
                                     static_cast<uint8_t>(Action::Enable),
                                     {"sim-axis-x"}, generationsFor({"sim-axis-x"}),
                                     "expired-lease-cmd");
    EXPECT_EQ(receipt.state, static_cast<uint8_t>(CommandState::Rejected));
    ASSERT_FALSE(receipt.blockers.empty());

    // Renewal of an expired lease is rejected through the function surface.
    const auto renew = invoke(MachineService::kAuthorityRenewFnId,
                              {u64Arg(1, acquired.lease->token),
                               stringArg(2, "machine"), u32Arg(3, 10'000)},
                              owner);
    ASSERT_TRUE(renew.success);
    std::map<uint32_t, std::vector<uint8_t>> fields;
    ASSERT_TRUE(parseTagged(renew.returnValue, fields));
    bool granted = true;
    getBool(fields[1], granted);
    EXPECT_FALSE(granted);

    // The expired lease no longer blocks a different owner.
    EXPECT_NE(acquire(operatorSession("owner-after-expiry")), 0u);
}

TEST_F(SimulatedMachineServiceTest, AuthoritySnapshotListsActiveLeases) {
    EXPECT_NE(acquire(operatorSession()), 0u);
    const auto entry = registry_.findSignal(MachineService::kAuthoritySnapshotSignalId);
    ASSERT_TRUE(entry);
    std::vector<uint8_t> bytes(MachineService::kMaximumEncodedValue);
    const size_t size = entry.readVar(bytes.data(), bytes.size());
    ASSERT_GT(size, 0u);
    std::map<uint32_t, std::vector<uint8_t>> fields;
    ASSERT_TRUE(parseTagged({bytes.begin(), bytes.begin() + static_cast<ptrdiff_t>(size)}, fields));
    EXPECT_TRUE(fields.contains(2));
}

TEST_F(SimulatedMachineServiceTest, MalformedCommandPayloadIsRejected) {
    const auto result = invoke(MachineService::kCommandFnId,
                               {arg(1, {0xFF, 0xFF, 0xFF})}, operatorSession());
    EXPECT_FALSE(result.success);
    EXPECT_FALSE(result.errorMessage.empty());
}

TEST_F(SimulatedMachineServiceTest, ExpiredDeadlineIsRejected) {
    const auto identity = operatorSession();
    const uint64_t token = acquire(identity);
    ASSERT_NE(token, 0u);
    const auto receipt = sendCommand(identity, token,
                                     static_cast<uint8_t>(Action::Disable),
                                     {"sim-axis-x"}, generationsFor({"sim-axis-x"}),
                                     "expired-1");
    EXPECT_EQ(receipt.state, static_cast<uint8_t>(CommandState::Accepted));
    // Now with a zero deadline budget — rejected by the gate.
    const auto request = commandRequest("expired-2", "machine", token,
                                        static_cast<uint8_t>(Action::Enable),
                                        {"sim-axis-x"}, generationsFor({"sim-axis-x"}),
                                        0);
    const auto result = invoke(MachineService::kCommandFnId, {arg(1, request)}, identity);
    ASSERT_TRUE(result.success);
    const auto receipt2 = decodeReceipt(result.returnValue);
    EXPECT_EQ(receipt2.state, static_cast<uint8_t>(CommandState::Rejected));
}

TEST_F(SimulatedMachineServiceTest, CaptureConfigureValidatesAndReportsStatus) {
    // Fixed-size entries only: the U64 cursor signal plus the writable param.
    const auto result = invoke(MachineService::kCaptureConfigureFnId,
                               {stringArg(1, "commissioning-run"), u32Arg(2, 1000),
                                arg(3, fieldScalar<uint8_t>(1)),
                                arg(4, u64Array({cia402::SimulatedCiA402Fleet::kEventCursorSignalId,
                                                 kWritableParamId}))},
                               technicianSession());
    ASSERT_TRUE(result.success) << result.errorMessage;
    std::map<uint32_t, std::vector<uint8_t>> fields;
    ASSERT_TRUE(parseTagged(result.returnValue, fields));
    uint8_t state = 0;
    uint32_t fieldCount = 0;
    getScalar(fields[2], state);
    getScalar(fields[7], fieldCount);
    EXPECT_EQ(state, static_cast<uint8_t>(DatalogState::Recording));
    EXPECT_EQ(fieldCount, 2u);

    // Observer can query status.
    const auto status = invoke(MachineService::kCaptureStatusFnId, {}, observerSession());
    EXPECT_TRUE(status.success);

    // Unknown entry id is rejected.
    const auto unknown = invoke(MachineService::kCaptureConfigureFnId,
                                {stringArg(1, "bad"), u32Arg(2, 1000),
                                 arg(3, fieldScalar<uint8_t>(0)),
                                 arg(4, u64Array({0xDEADBEEF}))},
                                technicianSession());
    EXPECT_FALSE(unknown.success);
    // Variable-length entries are rejected.
    const auto variable = invoke(MachineService::kCaptureConfigureFnId,
                                 {stringArg(1, "var"), u32Arg(2, 1000),
                                  arg(3, fieldScalar<uint8_t>(0)),
                                  arg(4, u64Array({cia402::SimulatedCiA402Fleet::kMachineSnapshotSignalId}))},
                                 technicianSession());
    EXPECT_FALSE(variable.success);
    // Out-of-range rate is rejected.
    const auto rate = invoke(MachineService::kCaptureConfigureFnId,
                             {stringArg(1, "rate"), u32Arg(2, 0),
                              arg(3, fieldScalar<uint8_t>(0)), arg(4, u64Array({}))},
                             technicianSession());
    EXPECT_FALSE(rate.success);
}

TEST_F(SimulatedMachineServiceTest, CaptureTriggerKeepsPreAndPostWindow) {
    // Trigger on the writable U32 param: level 100, pre 5, post 3.
    double level = 100.0;
    std::vector<uint8_t> levelBytes(8);
    std::memcpy(levelBytes.data(), &level, 8);
    const auto result = invoke(MachineService::kCaptureConfigureFnId,
                               {stringArg(1, "triggered"), u32Arg(2, 1000),
                                arg(3, fieldScalar<uint8_t>(1)),
                                arg(4, u64Array({kWritableParamId})),
                                arg(5, fieldScalar<uint64_t>(kWritableParamId)),
                                arg(6, fieldScalar<uint8_t>(0)),
                                arg(7, std::move(levelBytes)),
                                arg(8, fieldScalar<uint32_t>(5)),
                                arg(9, fieldScalar<uint32_t>(3))},
                               technicianSession());
    ASSERT_TRUE(result.success) << result.errorMessage;
    std::map<uint32_t, std::vector<uint8_t>> fields;
    ASSERT_TRUE(parseTagged(result.returnValue, fields));
    uint8_t triggerState = 0;
    ASSERT_TRUE(getScalar(fields[9], triggerState));
    EXPECT_EQ(triggerState, static_cast<uint8_t>(CaptureTriggerState::Armed));

    // Below the level: ring stays at pre-trigger depth.
    paramValue_ = 42;
    for (int i = 0; i < 12; ++i) stack_.captureRecorder.sampleOnce();
    EXPECT_EQ(stack_.capture->triggerState(), CaptureTriggerState::Armed);
    EXPECT_EQ(stack_.capture->recordsAvailable(), 5u);

    // Cross the level: exactly pre + post records are retained and the
    // recorder stops itself.
    paramValue_ = 150;
    for (int i = 0; i < 10; ++i) stack_.captureRecorder.sampleOnce();
    EXPECT_EQ(stack_.capture->triggerState(), CaptureTriggerState::Complete);
    EXPECT_EQ(stack_.capture->recordsAvailable(), 8u);
    EXPECT_EQ(stack_.captureRecorder.status().state, DatalogState::Stopped);

    // Bounded export: request a tiny chunk and paginate.
    const auto chunk0 = invoke(MachineService::kCaptureExportFnId,
                               {arg(1, fieldScalar<uint64_t>(0)),
                                arg(2, fieldScalar<uint32_t>(40))},
                               technicianSession());
    ASSERT_TRUE(chunk0.success) << chunk0.errorMessage;
    fields.clear();
    ASSERT_TRUE(parseTagged(chunk0.returnValue, fields));
    uint64_t total = 0, offset = 0;
    uint32_t recordSize = 0;
    ASSERT_TRUE(getScalar(fields[2], total));
    ASSERT_TRUE(getScalar(fields[1], offset));
    ASSERT_TRUE(getScalar(fields[4], recordSize));
    EXPECT_EQ(total, 8u);
    EXPECT_EQ(offset, 0u);
    EXPECT_NE(recordSize, 0u);
    const uint64_t exportedRecords = fields[5].size() / recordSize;
    EXPECT_LT(exportedRecords, total);  // chunk was capped below the full set

    // Cancel discards retained records and stops the recorder.
    const auto cancel = invoke(MachineService::kCaptureCancelFnId, {}, technicianSession());
    ASSERT_TRUE(cancel.success);
    EXPECT_EQ(stack_.capture->recordsAvailable(), 0u);
    EXPECT_EQ(stack_.capture->triggerState(), CaptureTriggerState::None);
}

TEST_F(SimulatedMachineServiceTest, ConfigStageValidateCommitAppliesAndBumpsRevision) {
    uint32_t newValue = 42;
    std::vector<uint8_t> valueBytes(reinterpret_cast<const uint8_t*>(&newValue),
                                    reinterpret_cast<const uint8_t*>(&newValue) + 4);
    const auto staged = stageConfig({{kWritableParamId, valueBytes}}, technicianSession());
    ASSERT_TRUE(staged.success) << staged.errorMessage;
    std::map<uint32_t, std::vector<uint8_t>> fields;
    ASSERT_TRUE(parseTagged(staged.returnValue, fields));
    uint8_t state = 0;
    getScalar(fields[2], state);
    EXPECT_EQ(state, static_cast<uint8_t>(ConfigTxnState::Staged));

    // Commit before validate fails inside the typed status.
    auto early = invoke(MachineService::kConfigCommitFnId, {}, technicianSession());
    ASSERT_TRUE(early.success);
    fields.clear();
    ASSERT_TRUE(parseTagged(early.returnValue, fields));
    getScalar(fields[2], state);
    EXPECT_EQ(state, static_cast<uint8_t>(ConfigTxnState::Failed));
    EXPECT_EQ(paramValue_, 7u);

    // Re-stage, validate, then commit as operator — insufficient role.
    ASSERT_TRUE(stageConfig({{kWritableParamId, valueBytes}}, technicianSession()).success);
    ASSERT_TRUE(invoke(MachineService::kConfigValidateFnId, {}, technicianSession()).success);
    auto denied = invoke(MachineService::kConfigCommitFnId, {}, operatorSession());
    ASSERT_TRUE(denied.success);
    fields.clear();
    ASSERT_TRUE(parseTagged(denied.returnValue, fields));
    getScalar(fields[2], state);
    EXPECT_EQ(state, static_cast<uint8_t>(ConfigTxnState::Failed));
    EXPECT_EQ(paramValue_, 7u);

    // Technician commits; value applied and revision bumped.
    ASSERT_TRUE(stageConfig({{kWritableParamId, valueBytes}}, technicianSession()).success);
    ASSERT_TRUE(invoke(MachineService::kConfigValidateFnId, {}, technicianSession()).success);
    auto committed = invoke(MachineService::kConfigCommitFnId, {}, technicianSession());
    ASSERT_TRUE(committed.success);
    fields.clear();
    ASSERT_TRUE(parseTagged(committed.returnValue, fields));
    getScalar(fields[2], state);
    uint64_t revision = 0;
    getScalar(fields[3], revision);
    EXPECT_EQ(state, static_cast<uint8_t>(ConfigTxnState::Committed));
    EXPECT_EQ(paramValue_, 42u);
    EXPECT_GT(revision, 0u);
}

TEST_F(SimulatedMachineServiceTest, ConfigStageRejectsReadOnlyAndUnknownEntries) {
    uint32_t v = 1;
    std::vector<uint8_t> bytes(reinterpret_cast<const uint8_t*>(&v),
                               reinterpret_cast<const uint8_t*>(&v) + 4);
    auto staged = stageConfig({{kReadOnlyParamId, bytes}}, technicianSession());
    ASSERT_TRUE(staged.success);
    std::map<uint32_t, std::vector<uint8_t>> fields;
    ASSERT_TRUE(parseTagged(staged.returnValue, fields));
    uint8_t state = 0;
    getScalar(fields[2], state);
    EXPECT_EQ(state, static_cast<uint8_t>(ConfigTxnState::Failed));

    staged = stageConfig({{0xDEADBEEF, bytes}}, technicianSession());
    ASSERT_TRUE(staged.success);
    fields.clear();
    ASSERT_TRUE(parseTagged(staged.returnValue, fields));
    getScalar(fields[2], state);
    EXPECT_EQ(state, static_cast<uint8_t>(ConfigTxnState::Failed));
}

TEST_F(SimulatedMachineServiceTest, ConfigRollbackDiscardsStagedWrites) {
    uint32_t v = 9;
    std::vector<uint8_t> bytes(reinterpret_cast<const uint8_t*>(&v),
                               reinterpret_cast<const uint8_t*>(&v) + 4);
    ASSERT_TRUE(stageConfig({{kWritableParamId, bytes}}, technicianSession()).success);
    auto rolled = invoke(MachineService::kConfigRollbackFnId, {}, technicianSession());
    ASSERT_TRUE(rolled.success);
    std::map<uint32_t, std::vector<uint8_t>> fields;
    ASSERT_TRUE(parseTagged(rolled.returnValue, fields));
    uint8_t state = 0;
    getScalar(fields[2], state);
    EXPECT_EQ(state, static_cast<uint8_t>(ConfigTxnState::RolledBack));
    EXPECT_EQ(paramValue_, 7u);

    // Status is observer-readable.
    auto status = invoke(MachineService::kConfigStatusFnId, {}, observerSession());
    ASSERT_TRUE(status.success);
    fields.clear();
    ASSERT_TRUE(parseTagged(status.returnValue, fields));
    getScalar(fields[2], state);
    EXPECT_EQ(state, static_cast<uint8_t>(ConfigTxnState::RolledBack));
}

// ---- Authentication provider + durable audit -------------------------------

TEST(MachineAuthTest, StaticTokensAuthenticateAndReject) {
    StaticTokenAuthProvider provider;
    provider.addToken("0123456789abcdef-op", "alice", Role::Operator);
    provider.addToken("0123456789abcdef-tx", "bob", Role::Technician);

    const auto op = provider.authenticate("0123456789abcdef-op");
    ASSERT_TRUE(op);
    EXPECT_TRUE(op->authenticated);
    EXPECT_EQ(op->role, Role::Operator);
    EXPECT_EQ(op->actor, "alice");

    const auto tech = provider.authenticate("0123456789abcdef-tx");
    ASSERT_TRUE(tech);
    EXPECT_EQ(tech->role, Role::Technician);

    // Wrong token, near-miss length, and empty are all rejected.
    EXPECT_FALSE(provider.authenticate("0123456789abcdef-XX"));
    EXPECT_FALSE(provider.authenticate("0123456789abcdef-o"));
    EXPECT_FALSE(provider.authenticate(""));
}

TEST(MachineAuthTest, TokenFileParsesAndRejectsMalformed) {
    const auto path = std::filesystem::temp_directory_path() / "tether_auth_test.tsv";
    {
        std::ofstream out(path);
        out << "# comment\n"
            << "0123456789abcdef-op\talice\toperator\n"
            << "0123456789abcdef-tx\tbob\ttechnician\n";
    }
    const auto provider = StaticTokenAuthProvider::fromFile(path.string());
    EXPECT_EQ(provider.authenticate("0123456789abcdef-tx")->actor, "bob");
    std::filesystem::remove(path);

    const auto bad = std::filesystem::temp_directory_path() / "tether_auth_bad.tsv";
    {
        std::ofstream out(bad);
        out << "0123456789abcdef-op alice operator\n"; // spaces, not tabs
    }
    EXPECT_THROW(StaticTokenAuthProvider::fromFile(bad.string()), std::runtime_error);
    std::filesystem::remove(bad);
    EXPECT_THROW(StaticTokenAuthProvider::fromFile("/nonexistent/tokens.tsv"),
                 std::runtime_error);
}

TEST(MachineAuthTest, BearerCredentialExtraction) {
    EXPECT_EQ(bearerCredential("Bearer abc123"), "abc123");
    EXPECT_FALSE(bearerCredential("Bearer"));
    EXPECT_FALSE(bearerCredential("Bearer "));
    EXPECT_FALSE(bearerCredential("Basic abc123"));
    EXPECT_FALSE(bearerCredential("Bearer has space"));
    EXPECT_FALSE(bearerCredential(""));
}

TEST(MachineAuthTest, AuditSinkPersistsEscapedRecords) {
    const auto path = std::filesystem::temp_directory_path() / "tether_audit_test.tsv";
    std::filesystem::remove(path);
    EventJournal journal(4);
    {
        const auto sink = std::make_shared<FileEventJournalSink>(path.string());
        journal.setListener([sink](const EventRecordV1& record) { sink->write(record); });
        journal.append({0, 100, 1, "CommandIssued", "axis-x", EventSeverity::Info, 7,
                        "disable\twith\ttabs"});
        journal.append({0, 200, 1, "FaultSet", "axis-y", EventSeverity::Fault, 0x6010,
                        "line\nbreak"});
    }
    std::ifstream in(path);
    std::vector<std::string> lines;
    for (std::string line; std::getline(in, line);) lines.push_back(line);
    ASSERT_EQ(lines.size(), 2u);
    EXPECT_NE(lines[0].find("CommandIssued"), std::string::npos);
    // Tabs/newlines are escaped — every record is exactly one line.
    EXPECT_NE(lines[0].find("disable\\twith\\ttabs"), std::string::npos);
    EXPECT_NE(lines[1].find("line\\nbreak"), std::string::npos);
    // Evicted in-memory events still exist in the durable log.
    journal.append({0, 300, 1, "E", "s", EventSeverity::Info, 0, "d"});
    journal.append({0, 301, 1, "E", "s", EventSeverity::Info, 0, "d"});
    journal.append({0, 302, 1, "E", "s", EventSeverity::Info, 0, "d"});
    EXPECT_EQ(journal.retainedCount(), 4u);
    std::filesystem::remove(path);
}


TEST_F(SimulatedMachineServiceTest, TakeoverEvictsOwnerAndRequiresSeniorRole) {
    const uint64_t token = acquire(operatorSession(), "machine");
    ASSERT_NE(token, 0u);

    // An operator cannot take over — the lease stays with the first owner.
    auto result = invoke(MachineService::kAuthorityTakeoverFnId,
                         {stringArg(1, "machine"), u32Arg(2, 30'000)},
                         operatorSession("session-2"));
    ASSERT_TRUE(result.success);
    std::map<uint32_t, std::vector<uint8_t>> fields;
    ASSERT_TRUE(parseTagged(result.returnValue, fields));
    bool granted = true;
    getBool(fields[1], granted);
    EXPECT_FALSE(granted);

    {
        auto direct = stack_.authority.takeover("machine", engineerSession(),
                                                std::chrono::milliseconds(30'000));
        EXPECT_TRUE(direct.lease.has_value()) << direct.blocker;
    }
    // A controls engineer evicts the owner and gets a fresh token.
    result = invoke(MachineService::kAuthorityTakeoverFnId,
                    {stringArg(1, "machine"), u32Arg(2, 30'000)}, engineerSession());
    ASSERT_TRUE(result.success) << result.errorMessage;
    // parseTagged emplaces without overwriting — clear stale fields first.
    fields.clear();
    ASSERT_TRUE(parseTagged(result.returnValue, fields));
    ASSERT_TRUE(getBool(fields[1], granted));
    if (!granted) {
        std::string blocker;
        if (getString(fields[6], blocker)) ADD_FAILURE() << "blocker: " << blocker;
    }
    ASSERT_TRUE(granted);
    uint64_t newToken = 0;
    ASSERT_TRUE(getScalar(fields[3], newToken));
    EXPECT_NE(newToken, token);

    // The takeover is audited and names the evicted owner.
    bool sawTakeover = false;
    for (const auto& event : stack_.fleet.eventJournal().readAfter(0, 512).events) {
        if (event.eventType.find("takeover") != std::string::npos ||
            (event.eventType.find("authority") != std::string::npos &&
             event.description.find("takeover") != std::string::npos))
            sawTakeover = true;
    }
    EXPECT_TRUE(sawTakeover);
}

// ---------------------------------------------------------------------------
// SDO inspection surface
// ---------------------------------------------------------------------------

TEST_F(SimulatedMachineServiceTest, SdoListReadWriteAndRoleGating) {
    // List: authenticated observer sees the simulated CiA 402 table.
    auto result = invoke(MachineService::kSdoListFnId,
                         {arg(1, fieldScalar(uint16_t{0}))}, observerSession());
    ASSERT_TRUE(result.success) << result.errorMessage;
    // Array of tagged SdoEntryV1 elements.
    const auto elements = elementArray(result.returnValue);
    ASSERT_FALSE(elements.empty());
    std::map<uint32_t, std::vector<uint8_t>> entry;
    ASSERT_TRUE(parseTagged(elements.front(), entry));
    uint16_t index = 0;
    ASSERT_TRUE(getScalar(entry[1], index));
    EXPECT_EQ(index, 0x1000u);

    // Read the seeded statusword (0x6041:0 -> {0x50,0x02}).
    const auto request = encodeTagged({
        {1, fieldScalar(uint16_t{0})},
        {2, fieldScalar(uint16_t{0x6041})},
        {3, fieldScalar(uint8_t{0})},
        {4, fieldScalar(uint16_t{64})},
    });
    result = invoke(MachineService::kSdoReadFnId, {arg(1, request)},
                    observerSession());
    ASSERT_TRUE(result.success) << result.errorMessage;
    std::map<uint32_t, std::vector<uint8_t>> fields;
    ASSERT_TRUE(parseTagged(result.returnValue, fields));
    bool ok = false;
    ASSERT_TRUE(getBool(fields[1], ok));
    EXPECT_TRUE(ok);
    // Field 3 is a length-prefixed byte blob.
    ASSERT_GE(fields[3].size(), 3u);
    EXPECT_EQ(fields[3][1], 0x50);
    EXPECT_EQ(fields[3][2], 0x02);

    // Write requires technician+ and updates the simulated OD.
    const auto writeRequest = encodeTagged({
        {1, fieldScalar(uint16_t{0})},
        {2, fieldScalar(uint16_t{0x6040})},
        {3, fieldScalar(uint8_t{0})},
        {4, fieldBytes({0x06, 0x00})},
    });
    result = invoke(MachineService::kSdoWriteFnId, {arg(1, writeRequest)},
                    operatorSession());
    EXPECT_FALSE(result.success);  // operator role is not enough
    result = invoke(MachineService::kSdoWriteFnId, {arg(1, writeRequest)},
                    technicianSession());
    ASSERT_TRUE(result.success) << result.errorMessage;
    ASSERT_TRUE(parseTagged(result.returnValue, fields));
    ASSERT_TRUE(getBool(fields[1], ok));
    EXPECT_TRUE(ok);
}

TEST_F(SimulatedMachineServiceTest, SdoReadReportsAbortForUnknownObject) {
    const auto request = encodeTagged({
        {1, fieldScalar(uint16_t{0})},
        {2, fieldScalar(uint16_t{0x2FF1})},  // not in the simulated OD
        {3, fieldScalar(uint8_t{0})},
        {4, fieldScalar(uint16_t{64})},
    });
    const auto result = invoke(MachineService::kSdoReadFnId, {arg(1, request)},
                               technicianSession());
    ASSERT_TRUE(result.success) << result.errorMessage;
    std::map<uint32_t, std::vector<uint8_t>> fields;
    ASSERT_TRUE(parseTagged(result.returnValue, fields));
    bool ok = true;
    getBool(fields[1], ok);
    EXPECT_FALSE(ok);
    uint32_t abortCode = 0;
    ASSERT_TRUE(getScalar(fields[2], abortCode));
    EXPECT_EQ(abortCode, 0x06020000u);  // object does not exist
}

TEST_F(SimulatedMachineServiceTest, SdoSurfaceIsOptional) {
    // A second stack without the SDO backend must not register machine.sdo.*.
    SchemaGraph graph = cia402::machineProfileSchemaGraph();
    SchemaCatalog catalog;
    ASSERT_TRUE(cia402::SimulatedCiA402Fleet::installSchemas(graph, catalog));
    Registry registry;
    SimulatedMachineStack stack;
    ASSERT_TRUE(stack.fleet.registerSignals(registry, catalog));
    ASSERT_TRUE(stack.install(registry, catalog, /*enableSdo=*/false));
    EXPECT_FALSE(registry.findFunction(MachineService::kSdoListFnId));
    EXPECT_FALSE(registry.findFunction(MachineService::kSdoReadFnId));
    EXPECT_FALSE(registry.findFunction(MachineService::kSdoWriteFnId));
}

TEST_F(SimulatedMachineServiceTest, SdoWriteIsAudited) {
    const auto writeRequest = encodeTagged({
        {1, fieldScalar(uint16_t{1})},
        {2, fieldScalar(uint16_t{0x607A})},
        {3, fieldScalar(uint8_t{0})},
        {4, fieldBytes({0x10, 0x27, 0x00, 0x00})},
    });
    const auto result = invoke(MachineService::kSdoWriteFnId,
                               {arg(1, writeRequest)}, technicianSession());
    ASSERT_TRUE(result.success) << result.errorMessage;
    bool sawAudit = false;
    const auto page = stack_.fleet.eventJournal().readAfter(0, 512);
    for (const auto& event : page.events) {
        if (event.eventType == "audit.sdo.write") {
            sawAudit = true;
            EXPECT_NE(event.description.find("technician-a"), std::string::npos);
            EXPECT_NE(event.description.find("0x607a"), std::string::npos);
        }
    }
    EXPECT_TRUE(sawAudit) << "SDO write was not journaled";
}

} // namespace

TEST_F(SimulatedMachineServiceTest, DiagnosticsSignalReportsFleetCounters) {
    const auto entry = registry_.findSignal(MachineService::kDiagnosticsSignalId);
    ASSERT_TRUE(entry);
    EXPECT_EQ(entry.name(), "machine.diagnostics");
    std::vector<uint8_t> bytes(MachineService::kMaximumEncodedValue);
    const size_t size = entry.readVar(bytes.data(), bytes.size());
    ASSERT_GT(size, 0u);
    std::map<uint32_t, std::vector<uint8_t>> fields;
    ASSERT_TRUE(parseTagged({bytes.begin(), bytes.begin() + static_cast<ptrdiff_t>(size)}, fields));
    uint64_t cycleCount = 0;
    uint16_t detected = 0, operational = 0;
    uint32_t wkc = 0;
    EXPECT_TRUE(getScalar(fields[2], cycleCount));
    EXPECT_TRUE(getScalar(fields[18], detected));
    EXPECT_TRUE(getScalar(fields[19], operational));
    EXPECT_TRUE(getScalar(fields[20], wkc));
    EXPECT_EQ(detected, static_cast<uint16_t>(stack_.fleet.axisCount()));
    EXPECT_EQ(operational, detected);
    EXPECT_EQ(wkc, detected * 2U);
    EXPECT_GT(cycleCount, 0U);
}

TEST_F(SimulatedMachineServiceTest, DiagnosticsSurfaceIsOptional) {
    SchemaGraph graph = cia402::machineProfileSchemaGraph();
    SchemaCatalog catalog;
    ASSERT_TRUE(cia402::SimulatedCiA402Fleet::installSchemas(graph, catalog));
    Registry registry;
    SimulatedMachineStack stack;
    ASSERT_TRUE(stack.fleet.registerSignals(registry, catalog));
    ASSERT_TRUE(stack.install(registry, catalog, /*enableSdo=*/true,
                              /*enableDiagnostics=*/false));
    EXPECT_FALSE(registry.findSignal(MachineService::kDiagnosticsSignalId));
}

TEST_F(SimulatedMachineServiceTest, SdoWriteReadbackDecodesAbortNameAndRateLimits) {
    const auto writeRequest = encodeTagged({
        {1, fieldScalar(uint16_t{0})},
        {2, fieldScalar(uint16_t{0x607A})},
        {3, fieldScalar(uint8_t{0})},
        {4, fieldBytes({0x2A, 0x00, 0x00, 0x00})},
    });
    const auto write = invoke(MachineService::kSdoWriteFnId, {arg(1, writeRequest)},
                              technicianSession());
    ASSERT_TRUE(write.success) << write.errorMessage;
    std::map<uint32_t, std::vector<uint8_t>> fields;
    ASSERT_TRUE(parseTagged(write.returnValue, fields));
    bool ok = false;
    ASSERT_TRUE(getBool(fields[1], ok));
    EXPECT_TRUE(ok);
    // Field 6: verification readback (varint length + payload).
    ASSERT_GE(fields[6].size(), 5u);
    EXPECT_EQ(fields[6][0], 0x04);
    EXPECT_EQ(std::vector<uint8_t>(fields[6].begin() + 1, fields[6].end()),
              std::vector<uint8_t>({0x2A, 0x00, 0x00, 0x00}));

    // An immediate second write is rate-limited server-side.
    const auto tooSoon = invoke(MachineService::kSdoWriteFnId, {arg(1, writeRequest)},
                                technicianSession());
    EXPECT_FALSE(tooSoon.success);
    EXPECT_NE(tooSoon.errorMessage.find("rate limited"), std::string::npos);

    // Unknown-object aborts carry the decoded CiA 301 name in field 5.
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    const auto readRequest = encodeTagged({
        {1, fieldScalar(uint16_t{0})},
        {2, fieldScalar(uint16_t{0x9999})},
        {3, fieldScalar(uint8_t{0})},
        {4, fieldScalar(uint16_t{8})},
    });
    const auto aborted = invoke(MachineService::kSdoReadFnId, {arg(1, readRequest)},
                                observerSession());
    ASSERT_TRUE(aborted.success) << aborted.errorMessage;
    fields.clear();
    ASSERT_TRUE(parseTagged(aborted.returnValue, fields));
    std::string abortName;
    ASSERT_TRUE(getString(fields[5], abortName));
    EXPECT_EQ(abortName, "Object does not exist in the object dictionary");
}

TEST_F(SimulatedMachineServiceTest, PdoMapListsLogicalPlacement) {
    const auto result = invoke(MachineService::kPdoMapFnId, {}, observerSession());
    ASSERT_TRUE(result.success) << result.errorMessage;
    const auto elements = elementArray(result.returnValue);
    // 3 Rx + 3 Tx entries per axis.
    ASSERT_EQ(elements.size(), stack_.fleet.axisCount() * 6u);
    std::map<uint32_t, std::vector<uint8_t>> entry;
    ASSERT_TRUE(parseTagged(elements.front(), entry));
    uint16_t slave = 0, pdo = 0;
    uint8_t direction = 9;
    uint32_t offset = 0;
    EXPECT_TRUE(getScalar(entry[1], slave));
    EXPECT_TRUE(getScalar(entry[2], pdo));
    EXPECT_TRUE(getScalar(entry[3], direction));
    EXPECT_TRUE(getScalar(entry[4], offset));
    EXPECT_EQ(slave, 0u);
    EXPECT_EQ(direction, 0u);
    // Unauthenticated sessions are rejected.
    SessionIdentity anon;
    EXPECT_FALSE(
        invoke(MachineService::kPdoMapFnId, {}, anon).success);
}

TEST_F(SimulatedMachineServiceTest, SupervisorStatusRetryAndRateLimit) {
    // Fault slave 1 so the supervisor reports Critical.
    const auto axes = stack_.fleet.descriptor().axes;
    auto drive = *stack_.fleet.driveSnapshot(axes[1].stableId);
    drive.faultCode = 0x1234;
    drive.ds402State = 7;
    stack_.fleet.updateDriveSnapshot(axes[1].stableId, drive);

    const auto status = invoke(MachineService::kSupervisorStatusFnId, {}, observerSession());
    ASSERT_TRUE(status.success) << status.errorMessage;
    const auto elements = elementArray(status.returnValue);
    ASSERT_EQ(elements.size(), stack_.fleet.axisCount());
    std::map<uint32_t, std::vector<uint8_t>> second;
    ASSERT_TRUE(parseTagged(elements[1], second));
    uint8_t state = 0;
    ASSERT_TRUE(getScalar(second[2], state));
    EXPECT_EQ(state, 1u);  // Critical

    // Operator role is rejected; technician retries the faulted slave.
    auto retry = invoke(MachineService::kSupervisorRetryFnId,
                        {arg(1, fieldScalar(uint16_t{1}))}, operatorSession());
    EXPECT_FALSE(retry.success);
    retry = invoke(MachineService::kSupervisorRetryFnId,
                   {arg(1, fieldScalar(uint16_t{1}))}, technicianSession());
    ASSERT_TRUE(retry.success) << retry.errorMessage;
    std::map<uint32_t, std::vector<uint8_t>> fields;
    ASSERT_TRUE(parseTagged(retry.returnValue, fields));
    bool ok = false;
    ASSERT_TRUE(getBool(fields[1], ok));
    EXPECT_TRUE(ok);

    // The retry cleared the simulated fault and is journaled.
    const auto cleared = stack_.fleet.driveSnapshot(axes[1].stableId);
    EXPECT_EQ(cleared->faultCode, 0u);
    bool sawAudit = false;
    for (const auto& event : stack_.fleet.eventJournal().readAfter(0, 512).events) {
        if (event.eventType == "audit.supervisor.retry") sawAudit = true;
    }
    EXPECT_TRUE(sawAudit);

    // A second retry within the same second is rate-limited.
    const auto tooSoon = invoke(MachineService::kSupervisorRetryFnId,
                                {arg(1, fieldScalar(uint16_t{1}))}, technicianSession());
    EXPECT_FALSE(tooSoon.success);
    EXPECT_NE(tooSoon.errorMessage.find("rate limited"), std::string::npos);
}

TEST_F(SimulatedMachineServiceTest, PdoAndSupervisorSurfacesAreOptional) {
    SchemaGraph graph = cia402::machineProfileSchemaGraph();
    SchemaCatalog catalog;
    ASSERT_TRUE(cia402::SimulatedCiA402Fleet::installSchemas(graph, catalog));
    Registry registry;
    SimulatedMachineStack stack;
    ASSERT_TRUE(stack.fleet.registerSignals(registry, catalog));
    ASSERT_TRUE(stack.install(registry, catalog, /*enableSdo=*/true,
                              /*enableDiagnostics=*/false));
    EXPECT_FALSE(registry.findFunction(MachineService::kPdoMapFnId));
    EXPECT_FALSE(registry.findFunction(MachineService::kSupervisorStatusFnId));
    EXPECT_FALSE(registry.findFunction(MachineService::kSupervisorRetryFnId));
    EXPECT_FALSE(registry.findFunction(MachineService::kChecklistListFnId));
    EXPECT_FALSE(registry.findFunction(MachineService::kChecklistReportFnId));
}

TEST_F(SimulatedMachineServiceTest, ConfigExportListsWritableParamsAndDiffFindsChanges) {
    const auto exported = invoke(MachineService::kConfigExportFnId, {}, observerSession());
    ASSERT_TRUE(exported.success) << exported.errorMessage;
    std::map<uint32_t, std::vector<uint8_t>> header;
    ASSERT_TRUE(parseTagged(exported.returnValue, header));
    const auto elements = elementArray(header[2]);
    bool sawWritable = false;
    uint64_t baselineRevision = 0;
    getScalar(header[1], baselineRevision);
    for (const auto& element : elements) {
        std::map<uint32_t, std::vector<uint8_t>> fields;
        ASSERT_TRUE(parseTagged(element, fields));
        uint64_t id = 0;
        ASSERT_TRUE(getScalar(fields[1], id));
        if (id == kWritableParamId) sawWritable = true;
    }
    EXPECT_TRUE(sawWritable);

    // Re-submit the export as a baseline: no diffs.
    auto diff = invoke(MachineService::kConfigDiffFnId,
                       {arg(1, header[2])}, observerSession());
    ASSERT_TRUE(diff.success) << diff.errorMessage;
    std::map<uint32_t, std::vector<uint8_t>> d;
    ASSERT_TRUE(parseTagged(diff.returnValue, d));
    uint32_t diffCount = 1;
    ASSERT_TRUE(getScalar(d[2], diffCount));
    EXPECT_EQ(diffCount, 0u);

    // Change the live value and re-diff against the same baseline.
    paramValue_ = 99;
    diff = invoke(MachineService::kConfigDiffFnId, {arg(1, header[2])}, observerSession());
    ASSERT_TRUE(diff.success);
    std::map<uint32_t, std::vector<uint8_t>> d2;
    ASSERT_TRUE(parseTagged(diff.returnValue, d2));
    ASSERT_TRUE(getScalar(d2[2], diffCount));
    ASSERT_EQ(diffCount, 1u);
    const auto mismatch = elementArray(d2[3]);
    ASSERT_EQ(mismatch.size(), 1u);
    std::map<uint32_t, std::vector<uint8_t>> diffFields;
    ASSERT_TRUE(parseTagged(mismatch[0], diffFields));
    uint64_t id = 0;
    ASSERT_TRUE(getScalar(diffFields[1], id));
    EXPECT_EQ(id, kWritableParamId);
    paramValue_ = 7;
}

TEST_F(SimulatedMachineServiceTest, ConfigImportStagesTransactionAndRejectsOutOfRange) {
    // Import stages a transaction; a range-bounded param rejects bad values.
    uint32_t bounded = 5;
    ParamEntry ranged{};
    ranged.id = 0x5445535400000003ULL;
    ranged.name = "test.bounded";
    ranged.group = "test";
    ranged.valueType = ValueType::U32;
    ranged.readFn = [&bounded](void* d) { std::memcpy(d, &bounded, 4); };
    ranged.writeFn = [&bounded](const void* v) { std::memcpy(&bounded, v, 4); };
    ranged.metadata["range.min"] = "1";
    ranged.metadata["range.max"] = "100";
    ASSERT_TRUE(registry_.addParam(std::move(ranged)));

    const auto inRange = fieldScalar(uint32_t{42});
    const auto outOfRange = fieldScalar(uint32_t{900});
    const auto encode = [](std::pair<uint64_t, std::vector<uint8_t>> w) {
        return encodeArray({encodeTagged({
            {1, fieldScalar(w.first)}, {2, fieldBytes(w.second)}})});
    };

    auto result = invoke(MachineService::kConfigImportFnId,
                         {arg(1, encode({ranged.id, outOfRange}))},
                         technicianSession());
    ASSERT_TRUE(result.success);
    std::map<uint32_t, std::vector<uint8_t>> fields;
    ASSERT_TRUE(parseTagged(result.returnValue, fields));
    uint8_t state = 0;
    ASSERT_TRUE(getScalar(fields[2], state));
    EXPECT_EQ(state, 4u);  // Failed — out of declared range

    result = invoke(MachineService::kConfigImportFnId,
                    {arg(1, encode({ranged.id, inRange}))}, technicianSession());
    ASSERT_TRUE(result.success);
    std::map<uint32_t, std::vector<uint8_t>> fields2;
    ASSERT_TRUE(parseTagged(result.returnValue, fields2));
    ASSERT_TRUE(getScalar(fields2[2], state));
    EXPECT_EQ(state, 1u);  // Staged — still requires validate + commit

    // Observer role cannot import.
    EXPECT_FALSE(invoke(MachineService::kConfigImportFnId,
                        {arg(1, encode({ranged.id, inRange}))},
                        observerSession()).success);
}

TEST_F(SimulatedMachineServiceTest, ChecklistEvaluatesAndReportGatesOnRequiredItems) {
    // Required item "configuration committed" fails until a commit happens.
    const auto list = invoke(MachineService::kChecklistListFnId, {}, observerSession());
    ASSERT_TRUE(list.success) << list.errorMessage;
    const auto items = elementArray(list.returnValue);
    ASSERT_GE(items.size(), 3u);
    bool sawCommitItem = false;
    for (const auto& element : items) {
        std::map<uint32_t, std::vector<uint8_t>> f;
        ASSERT_TRUE(parseTagged(element, f));
        uint32_t itemId = 0;
        ASSERT_TRUE(getScalar(f[1], itemId));
        bool passed = false;
        ASSERT_TRUE(getBool(f[4], passed));
        if (itemId == 3u) { sawCommitItem = true; EXPECT_FALSE(passed); }
    }
    EXPECT_TRUE(sawCommitItem);

    // The acceptance report refuses while required items fail.
    EXPECT_FALSE(
        invoke(MachineService::kChecklistReportFnId, {}, technicianSession()).success);

    // Commit a config transaction → required item 3 now passes.
    ASSERT_TRUE(stageConfig({{kWritableParamId, fieldScalar(uint32_t{11})}},
                            technicianSession()).success);
    ASSERT_TRUE(invoke(MachineService::kConfigValidateFnId, {},
                       technicianSession()).success);
    ASSERT_TRUE(invoke(MachineService::kConfigCommitFnId, {},
                       technicianSession()).success);

    const auto report =
        invoke(MachineService::kChecklistReportFnId, {}, technicianSession());
    ASSERT_TRUE(report.success) << report.errorMessage;
    std::map<uint32_t, std::vector<uint8_t>> fields;
    ASSERT_TRUE(parseTagged(report.returnValue, fields));
    uint8_t allPassed = 0;
    ASSERT_TRUE(getScalar(fields[3], allPassed));
    EXPECT_EQ(allPassed, 1u);
    uint64_t revision = 0;
    ASSERT_TRUE(getScalar(fields[2], revision));
    EXPECT_GT(revision, 0u);

    bool audited = false;
    for (const auto& event : stack_.fleet.eventJournal().readAfter(0, 512).events) {
        if (event.eventType == "audit.checklist.report") audited = true;
    }
    EXPECT_TRUE(audited);
}

TEST_F(SimulatedMachineServiceTest, CaptureServiceTemplateSelectsFixedSizeSignals) {
    auto configured = invoke(MachineService::kCaptureConfigureFnId,
                             {stringArg(1, "svc"), u32Arg(2, 100),
                              arg(3, fieldScalar(uint8_t{1})),
                              arg(4, std::vector<uint8_t>{0}),
                              arg(5, fieldScalar(uint64_t{0})),
                              arg(6, fieldScalar(uint8_t{0})),
                              arg(7, std::vector<uint8_t>(8, 0)),
                              arg(8, fieldScalar(uint32_t{0})),
                              arg(9, fieldScalar(uint32_t{0})),
                              arg(10, fieldScalar(uint8_t{1}))},
                             technicianSession());
    ASSERT_TRUE(configured.success) << configured.errorMessage;
    std::map<uint32_t, std::vector<uint8_t>> fields;
    ASSERT_TRUE(parseTagged(configured.returnValue, fields));
    uint32_t fieldCount = 0;
    ASSERT_TRUE(getScalar(fields[7], fieldCount));
    EXPECT_GT(fieldCount, 0u);

    // Unknown template ids are rejected.
    auto bad = invoke(MachineService::kCaptureConfigureFnId,
                      {stringArg(1, "svc"), u32Arg(2, 100),
                       arg(3, fieldScalar(uint8_t{1})),
                       arg(4, std::vector<uint8_t>{0}),
                       arg(5, fieldScalar(uint64_t{0})),
                       arg(6, fieldScalar(uint8_t{0})),
                       arg(7, std::vector<uint8_t>(8, 0)),
                       arg(8, fieldScalar(uint32_t{0})),
                       arg(9, fieldScalar(uint32_t{0})),
                       arg(10, fieldScalar(uint8_t{9}))},
                      technicianSession());
    EXPECT_FALSE(bad.success);
}

TEST_F(SimulatedMachineServiceTest, AppProfileSignalServesVerifiedDocument) {
    const auto signal = registry_.findSignal(MachineService::kAppProfileSignalId);
    ASSERT_TRUE(signal) << "machine.app.profile not registered";
    std::vector<uint8_t> bytes(MachineService::kMaximumEncodedValue);
    const size_t size = signal.readVar(bytes.data(), bytes.size());
    ASSERT_GT(size, 0u);
    bytes.resize(size);
    std::map<uint32_t, std::vector<uint8_t>> fields;
    ASSERT_TRUE(parseTagged(bytes, fields));
    std::string name, version;
    ASSERT_TRUE(getString(fields[1], name));
    ASSERT_TRUE(getString(fields[2], version));
    EXPECT_EQ(name, "tether.demo");
    EXPECT_EQ(version, "1.0");

    const auto stripPrefix = [](const std::vector<uint8_t>& packed) {
        size_t position = 0;
        while (position < packed.size() && (packed[position] & 0x80)) ++position;
        ++position;  // final varint byte
        return std::vector<uint8_t>(packed.begin() + static_cast<ptrdiff_t>(position),
                                    packed.end());
    };
    const auto doc = stripPrefix(fields[3]);
    const auto mac = stripPrefix(fields[4]);
    EXPECT_EQ(mac.size(), 32u);
    const std::string docText(doc.begin(), doc.end());
    EXPECT_NE(docText.find("tether.app.profile.v1"), std::string::npos);

    // A tampered or unsigned document must not install.
    ProfileKey key{};
    for (size_t i = 0; i < key.size(); ++i) key[i] = static_cast<uint8_t>(0xA5 + i);
    StaticAppProfileSource source(key);
    std::vector<uint8_t> tampered = doc;
    tampered[10] ^= 0xFF;
    ProfileMac forged{};
    std::copy(mac.begin(), mac.end(), forged.begin());
    EXPECT_FALSE(source.install("x", "1", tampered, forged));
    EXPECT_FALSE(source.installed());
    const auto goodMac = signProfileDocument(doc.data(), doc.size(), key);
    EXPECT_TRUE(source.install("x", "1", doc, goodMac));
}

TEST(SimulatedMachineServiceStatic, AppProfileSourceValidatesDocumentContract) {
    ProfileKey key{};
    for (size_t i = 0; i < key.size(); ++i) key[i] = static_cast<uint8_t>(0xA5 + i);
    const auto bytes = [](std::string_view text) {
        return std::vector<uint8_t>(text.begin(), text.end());
    };
    const auto installs = [&](const std::vector<uint8_t>& doc) {
        StaticAppProfileSource source(key);
        return source.install("x", "1", doc,
                              signProfileDocument(doc.data(), doc.size(), key));
    };

    // A correctly signed but malformed document must not install: integrity
    // is necessary but not sufficient (plan item 86).
    EXPECT_FALSE(installs(bytes("not json")));
    EXPECT_FALSE(installs(bytes(R"JSON({"format":"other","panels":[]})JSON")));
    EXPECT_FALSE(installs(bytes(R"JSON({"format":"tether.app.profile.v1","panels":[{
        "id": "p", "title": "P",
        "widgets": [{"kind": "jog", "label": "J"}]}]})JSON")));  // missing axis
    EXPECT_FALSE(installs(bytes(R"JSON({"format":"tether.app.profile.v1","panels":[{
        "id": "p", "title": "P",
        "widgets": [{"kind": "value", "label": "V"}]}]})JSON")));  // missing entry
    EXPECT_FALSE(installs(bytes(R"JSON({"format":"tether.app.profile.v1","panels":[{
        "id": "p", "title": "P",
        "widgets": [{"kind": "unknown", "label": "U"}]}]})JSON")));
    EXPECT_FALSE(installs(bytes(R"JSON({"format":"tether.app.profile.v1","panels":[{
        "id": "p", "title": "P",
        "widgets": [{"kind": "scene", "label": "S"}]}]})JSON")));  // missing axes
    EXPECT_FALSE(installs(bytes(R"JSON({"format":"tether.app.profile.v1","panels":[{
        "id": "p", "title": "P",
        "widgets": [{"kind": "scene", "label": "S",
                     "axes": [{"entry": "sig"}]}]}]})JSON")));  // missing field
    EXPECT_FALSE(installs(bytes(R"JSON({"format":"tether.app.profile.v1","panels":[
        {"id": "p", "title": "A"}, {"id": "p", "title": "B"}]})JSON")));  // dup ids
    EXPECT_FALSE(installs(
        bytes(std::string(kMaxProfileDocumentBytes + 1, ' '))));

    // Valid documents install.
    EXPECT_TRUE(installs(bytes(R"JSON({"format":"tether.app.profile.v1"})JSON")));
    EXPECT_TRUE(installs(bytes(R"JSON({"format":"tether.app.profile.v1","panels":[{
        "id": "p", "title": "P",
        "widgets": [
            {"kind": "gauge", "label": "G", "entry": "sine_wave", "min": -1, "max": 1},
            {"kind": "jog", "label": "J", "axis": "sim-axis-x"},
            {"kind": "command", "label": "E", "axis": "sim-axis-x", "action": 0},
            {"kind": "button", "label": "B", "fn": "machine.checklist.report"},
            {"kind": "scene", "label": "S", "axes": [
                {"name": "x", "entry": "drive.x.snapshot", "field": "actual_position",
                 "targetField": "target_position", "min": -100, "max": 100}]}]}]})JSON")));
}

TEST_F(SimulatedMachineServiceTest, RecipeApplyStagesTransactionAndIsAudited) {
    stack_.recipes->add("warmup", "Warm-up position presets",
                        {{kWritableParamId, fieldScalar(uint32_t{42})}});

    const auto list = invoke(MachineService::kRecipeListFnId, {}, observerSession());
    ASSERT_TRUE(list.success) << list.errorMessage;
    const auto elements = elementArray(list.returnValue);
    ASSERT_EQ(elements.size(), 1u);
    std::map<uint32_t, std::vector<uint8_t>> entry;
    ASSERT_TRUE(parseTagged(elements[0], entry));
    std::string name;
    ASSERT_TRUE(getString(entry[1], name));
    EXPECT_EQ(name, "warmup");

    // Observer cannot apply; technician stages (does not commit).
    EXPECT_FALSE(
        invoke(MachineService::kRecipeApplyFnId, {stringArg(1, "warmup")},
               observerSession()).success);
    const auto applied =
        invoke(MachineService::kRecipeApplyFnId, {stringArg(1, "warmup")},
               technicianSession());
    ASSERT_TRUE(applied.success) << applied.errorMessage;
    std::map<uint32_t, std::vector<uint8_t>> status;
    ASSERT_TRUE(parseTagged(applied.returnValue, status));
    uint8_t state = 0;
    ASSERT_TRUE(getScalar(status[2], state));
    EXPECT_EQ(state, 1u);  // Staged
    EXPECT_EQ(paramValue_, 7u);  // not committed yet

    bool audited = false;
    for (const auto& event : stack_.fleet.eventJournal().readAfter(0, 512).events)
        if (event.eventType == "audit.recipe.apply") audited = true;
    EXPECT_TRUE(audited);

    // Unknown recipe names are rejected.
    EXPECT_FALSE(
        invoke(MachineService::kRecipeApplyFnId, {stringArg(1, "missing")},
               technicianSession()).success);
}

TEST_F(SimulatedMachineServiceTest, ExtensionSurfacesAbsentWhenDisabled) {
    SimulatedMachineStack disabled;
    Registry registry2;
    SchemaCatalog catalog2;
    const auto graph = cia402::machineProfileSchemaGraph();
    ASSERT_TRUE(cia402::SimulatedCiA402Fleet::installSchemas(graph, catalog2));
    ASSERT_TRUE(disabled.fleet.registerSignals(registry2, catalog2));
    ASSERT_TRUE(disabled.install(registry2, catalog2, true, true, false));
    EXPECT_FALSE(registry2.findSignal(MachineService::kAppProfileSignalId));
    EXPECT_FALSE(registry2.findFunction(MachineService::kRecipeListFnId));
    EXPECT_FALSE(registry2.findFunction(MachineService::kRecipeApplyFnId));
}
