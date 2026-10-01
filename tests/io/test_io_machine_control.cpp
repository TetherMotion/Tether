#include <gtest/gtest.h>

#include "tether/io/MachineControl.hpp"

#include <atomic>
#include <chrono>
#include <stdexcept>
#include <string>

using namespace std::chrono_literals;
using namespace tether::io::machine;

namespace {
SessionIdentity operatorSession(std::string id = "session-1") {
    return {std::move(id), "operator-a", "127.0.0.1", Role::Operator, true};
}
}

TEST(MachineControlAuthorityTest, RequiresAuthenticatedOperatorAndExposesOwner) {
    ControlAuthority authority(5s, [](const AuthorityEvent&) {});
    auto observer = operatorSession("observer");
    observer.role = Role::Observer;
    EXPECT_FALSE(authority.acquire("machine", observer, 2s).lease);
    auto administrator = operatorSession("administrator");
    administrator.role = Role::Administrator;
    EXPECT_FALSE(authority.acquire("machine", administrator, 2s).lease);

    const auto now = SteadyClock::now();
    const auto identity = operatorSession();
    const auto acquired = authority.acquire("machine", identity, 2s, now);
    ASSERT_TRUE(acquired.lease);
    EXPECT_EQ(acquired.lease->ownerSession, identity.sessionId);
    EXPECT_TRUE(authority.owns(acquired.lease->token, "machine", identity, now));

    const auto contender = authority.acquire("machine", operatorSession("session-2"), 2s, now);
    EXPECT_FALSE(contender.lease);
    EXPECT_FALSE(contender.blocker.empty());
}

TEST(MachineControlAuthorityTest, LeaseExpiryAndRenewalAreServerEnforced) {
    ControlAuthority authority(5s, [](const AuthorityEvent&) {});
    const auto start = SteadyClock::now();
    const auto identity = operatorSession();
    const auto acquired = authority.acquire("group-a", identity, 2s, start);
    ASSERT_TRUE(acquired.lease);
    EXPECT_TRUE(authority.renew(acquired.lease->token, "group-a", identity, 3s, start + 1s).lease);
    EXPECT_TRUE(authority.owns(acquired.lease->token, "group-a", identity, start + 2s));
    auto downgraded = identity;
    downgraded.role = Role::Observer;
    EXPECT_FALSE(authority.renew(acquired.lease->token, "group-a", downgraded, 3s, start + 2s).lease);
    EXPECT_FALSE(authority.owns(acquired.lease->token, "group-a", downgraded, start + 2s));
    EXPECT_FALSE(authority.owns(acquired.lease->token, "group-a", identity, start + 5s));
}

TEST(MachineCommandGateTest, ChecksGenerationAndIdempotentlyQueuesOneDispatch) {
    ControlAuthority authority(10s, [](const AuthorityEvent&) {});
    const auto now = SteadyClock::now();
    const auto identity = operatorSession();
    const auto lease = authority.acquire("machine", identity, 5s, now);
    ASSERT_TRUE(lease.lease);

    std::atomic<int> dispatches{0};
    std::atomic<int> audits{0};
    CommandEnvironment environment;
    environment.configurationRevision = 12;
    ResourceCommandState axis;
    axis.generation = 8;
    axis.supportedActions.insert(Action::StepMove);
    environment.resources.emplace("axis-uuid", axis);
    MachineCommandGate gate(
        authority,
        [&environment](const CommandRequest&) { return std::optional{environment}; },
        [&dispatches](const CommandRequest&, const SessionIdentity&) {
            ++dispatches;
            return DispatchResult{true, "operation-1", "Queued by native motion executor"};
        },
        [&audits](const AuditRecord&) {
            return "audit-" + std::to_string(++audits);
        });

    CommandRequest request;
    request.requestUuid = "request-uuid-1";
    request.scope = "machine";
    request.authorityToken = lease.lease->token;
    request.targets = {"axis-uuid"};
    request.expectedGenerations.emplace("axis-uuid", 8);
    request.configurationRevision = 12;
    request.action = Action::StepMove;
    request.deadline = now + 2s;

    const auto accepted = gate.submit(request, identity, now);
    EXPECT_EQ(accepted.state, CommandState::Accepted);
    EXPECT_EQ(accepted.operationId, "operation-1");
    EXPECT_FALSE(accepted.auditId.empty());
    const auto duplicate = gate.submit(request, identity, now + 1ms);
    EXPECT_EQ(duplicate.state, CommandState::Accepted);
    EXPECT_EQ(dispatches.load(), 1);

    request.requestUuid = "request-uuid-2";
    request.expectedGenerations["axis-uuid"] = 7;
    const auto stale = gate.submit(request, identity, now + 2ms);
    EXPECT_EQ(stale.state, CommandState::Rejected);
    EXPECT_FALSE(stale.blockers.empty());
    EXPECT_EQ(dispatches.load(), 1);
}

TEST(MachineCommandGateTest, RejectsMissingAuthenticationAuthorityAndExpiredDeadlines) {
    ControlAuthority authority(30s, [](const AuthorityEvent&) {});
    const auto now = SteadyClock::now();
    const auto identity = operatorSession();
    int dispatches = 0;
    MachineCommandGate gate(
        authority,
        [](const CommandRequest&) { return std::optional<CommandEnvironment>{CommandEnvironment{}}; },
        [&dispatches](const CommandRequest&, const SessionIdentity&) {
            ++dispatches;
            return DispatchResult{true, "unexpected", {}};
        },
        [](const AuditRecord&) { return std::string{"audit"}; });

    CommandRequest request;
    request.requestUuid = "expired";
    request.scope = "machine";
    request.targets = {"axis"};
    request.expectedGenerations.emplace("axis", 1);
    request.deadline = now - 1ms;
    const auto expired = gate.submit(request, identity, now);
    EXPECT_EQ(expired.state, CommandState::Rejected);
    EXPECT_EQ(dispatches, 0);

    auto unauthenticated = identity;
    unauthenticated.authenticated = false;
    request.requestUuid = "unauthenticated";
    request.deadline = now + 1s;
    EXPECT_EQ(gate.submit(request, unauthenticated, now).state, CommandState::Rejected);
    EXPECT_EQ(dispatches, 0);
}

TEST(MachineCommandGateTest, AppliesRolePolicyPerActionAndAllowsExplicitOverrides) {
    ControlAuthority authority(30s, [](const AuthorityEvent&) {});
    const auto now = SteadyClock::now();
    const auto operatorIdentity = operatorSession();
    const auto lease = authority.acquire("machine", operatorIdentity, 5s, now);
    ASSERT_TRUE(lease.lease);

    CommandEnvironment environment;
    environment.resources.emplace("axis", ResourceCommandState{
        1, {Action::ConfigurationCommit}, {}});
    int dispatches = 0;
    MachineCommandGate gate(
        authority,
        [&environment](const CommandRequest&) { return std::optional{environment}; },
        [&dispatches](const CommandRequest&, const SessionIdentity&) {
            ++dispatches;
            return DispatchResult{true, "config-op", "Queued"};
        },
        [](const AuditRecord&) { return std::string{"audit"}; });

    CommandRequest request;
    request.requestUuid = "operator-config";
    request.scope = "machine";
    request.authorityToken = lease.lease->token;
    request.targets = {"axis"};
    request.expectedGenerations.emplace("axis", 1);
    request.action = Action::ConfigurationCommit;
    request.deadline = now + 2s;
    EXPECT_EQ(gate.submit(request, operatorIdentity, now).state, CommandState::Rejected);
    EXPECT_EQ(dispatches, 0);

    CommandPolicy policy;
    policy.actionRoles[Action::ConfigurationCommit] = {Role::Operator};
    MachineCommandGate overridden(
        authority,
        [&environment](const CommandRequest&) { return std::optional{environment}; },
        [&dispatches](const CommandRequest&, const SessionIdentity&) {
            ++dispatches;
            return DispatchResult{true, "config-op", "Queued"};
        },
        [](const AuditRecord&) { return std::string{"audit"}; }, policy);
    request.requestUuid = "operator-config-policy-override";
    EXPECT_EQ(overridden.submit(request, operatorIdentity, now).state, CommandState::Accepted);
    EXPECT_EQ(dispatches, 1);
}

TEST(MachineCommandGateTest, FailsClosedWhenIntentAuditCannotBePersisted) {
    ControlAuthority authority(30s, [](const AuthorityEvent&) {});
    const auto now = SteadyClock::now();
    const auto identity = operatorSession();
    const auto lease = authority.acquire("machine", identity, 5s, now);
    ASSERT_TRUE(lease.lease);

    CommandEnvironment environment;
    environment.resources.emplace("axis", ResourceCommandState{1, {Action::StepMove}, {}});
    int dispatches = 0;
    int auditCalls = 0;
    MachineCommandGate gate(
        authority,
        [&environment](const CommandRequest&) { return std::optional{environment}; },
        [&dispatches](const CommandRequest&, const SessionIdentity&) {
            ++dispatches;
            return DispatchResult{true, "unexpected", {}};
        },
        [&auditCalls](const AuditRecord&) -> std::string {
            ++auditCalls;
            throw std::runtime_error("audit store unavailable");
        });

    CommandRequest request;
    request.requestUuid = "audit-failure";
    request.scope = "machine";
    request.authorityToken = lease.lease->token;
    request.targets = {"axis"};
    request.expectedGenerations.emplace("axis", 1);
    request.action = Action::StepMove;
    request.deadline = now + 1s;

    const auto receipt = gate.submit(request, identity, now);
    EXPECT_EQ(receipt.state, CommandState::Rejected);
    EXPECT_EQ(dispatches, 0);
    EXPECT_EQ(auditCalls, 2);
    EXPECT_FALSE(receipt.blockers.empty());
}
