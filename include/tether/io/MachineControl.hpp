#pragma once

// Application-facing, non-real-time control boundary for machine-profile adapters.
// This module is deliberately fail-closed: transport/session code must supply a
// trusted identity and state provider before any command can be accepted.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace tether::io::machine {

enum class Role : uint8_t { Observer = 0, Operator = 1, Technician = 2, ControlsEngineer = 3, Administrator = 4 };

enum class Action : uint8_t {
    Enable, Disable, QuickStop, FaultReset, Recover, SetMode,
    JogStart, JogRenew, JogStop, StepMove, MoveToPosition, GroupMove,
    HomePrepare, HomeStart, HomeCancel, ConfigurationStage,
    ConfigurationValidate, ConfigurationCommit, ConfigurationRollback,
    CancelOperation, AcknowledgeAlarm
};

enum class CommandState : uint8_t { InProgress, Accepted, Rejected, Failed };

struct SessionIdentity {
    std::string sessionId;
    std::string actor;
    std::string source;
    Role role = Role::Observer;
    bool authenticated = false;
};

using SteadyClock = std::chrono::steady_clock;
using TimePoint = SteadyClock::time_point;
using Duration = SteadyClock::duration;

struct AuthorityLease {
    uint64_t token = 0;
    std::string scope;
    std::string ownerSession;
    std::string ownerActor;
    Role ownerRole = Role::Observer;
    TimePoint expiresAt{};
};

struct AuthorityResult {
    std::optional<AuthorityLease> lease;
    std::string blocker;
};

struct AuthorityEvent {
    std::string action;
    std::string scope;
    std::string actor;
    std::string sessionId;
    std::string reason;
    uint64_t token = 0;
    TimePoint timestamp{};
};

/// Server-owned control ownership. Calls are intended for an application executor,
/// never from the EtherCAT cyclic thread.
class ControlAuthority final {
public:
    using Audit = std::function<void(const AuthorityEvent&)>;

    explicit ControlAuthority(
        Duration maximumLease = std::chrono::seconds(30), Audit audit = {},
        std::unordered_set<Role> eligibleRoles = {
            Role::Operator, Role::Technician, Role::ControlsEngineer})
        : maximumLease_(maximumLease), audit_(std::move(audit)),
          eligibleRoles_(std::move(eligibleRoles)) {}

    AuthorityResult acquire(const std::string& scope, const SessionIdentity& identity,
                            Duration requested, TimePoint now = SteadyClock::now()) {
        AuthorityResult result;
        AuthorityEvent event{"acquire", scope, identity.actor, identity.sessionId, {}, 0, now};
        {
            std::lock_guard lock(mutex_);
            expireLocked(scope, now);
            if (scope.empty()) result.blocker = "A machine or motion-group scope is required";
            else if (!identity.authenticated || identity.sessionId.empty()) result.blocker = "An authenticated session is required";
            else if (!eligibleRoles_.contains(identity.role)) result.blocker = "Role cannot acquire motion authority";
            else if (requested <= Duration::zero() || requested > maximumLease_) result.blocker = "Requested lease duration is outside server policy";
            else if (const auto found = leases_.find(scope); found != leases_.end() && found->second.ownerSession != identity.sessionId)
                result.blocker = "Control authority is held by another session";
            else {
                auto& lease = leases_[scope];
                if (lease.token == 0) lease.token = nextToken_++;
                lease.scope = scope;
                lease.ownerSession = identity.sessionId;
                lease.ownerActor = identity.actor;
                lease.ownerRole = identity.role;
                lease.expiresAt = now + requested;
                result.lease = lease;
                event.token = lease.token;
            }
        }
        if (result.blocker.empty()) event.action = "acquire";
        else event.reason = result.blocker;
        if (result.lease && !record(event)) {
            std::lock_guard lock(mutex_);
            const auto found = leases_.find(scope);
            if (found != leases_.end() && found->second.token == result.lease->token) leases_.erase(found);
            result.lease.reset();
            result.blocker = "Authority audit service is unavailable; lease was not granted";
        } else if (!result.blocker.empty()) {
            (void)record(event);
        }
        return result;
    }

    AuthorityResult renew(uint64_t token, const std::string& scope,
                          const SessionIdentity& identity, Duration requested,
                          TimePoint now = SteadyClock::now()) {
        AuthorityResult result;
        AuthorityEvent event{"renew", scope, identity.actor, identity.sessionId, {}, token, now};
        std::optional<TimePoint> previousExpiry;
        {
            std::lock_guard lock(mutex_);
            expireLocked(scope, now);
            auto found = leases_.find(scope);
            if (found == leases_.end() || found->second.token != token ||
                found->second.ownerSession != identity.sessionId || !identity.authenticated) {
                result.blocker = "Authority lease is absent, expired, or owned by another session";
            } else if (!eligibleRoles_.contains(identity.role)) {
                result.blocker = "Role cannot renew motion authority";
            } else if (requested <= Duration::zero() || requested > maximumLease_) {
                result.blocker = "Requested lease duration is outside server policy";
            } else {
                previousExpiry = found->second.expiresAt;
                found->second.expiresAt = now + requested;
                result.lease = found->second;
            }
        }
        event.reason = result.blocker;
        if (result.lease && !record(event)) {
            std::lock_guard lock(mutex_);
            const auto found = leases_.find(scope);
            if (found != leases_.end() && found->second.token == token && previousExpiry)
                found->second.expiresAt = *previousExpiry;
            result.lease.reset();
            result.blocker = "Authority audit service is unavailable; lease renewal was rejected";
        } else if (!result.blocker.empty()) {
            (void)record(event);
        }
        return result;
    }

    bool release(uint64_t token, const std::string& scope, const SessionIdentity& identity,
                 TimePoint now = SteadyClock::now()) {
        bool released = false;
        std::optional<AuthorityLease> previousLease;
        AuthorityEvent event{"release", scope, identity.actor, identity.sessionId, {}, token, now};
        {
            std::lock_guard lock(mutex_);
            auto found = leases_.find(scope);
            if (found != leases_.end() && found->second.token == token &&
                found->second.ownerSession == identity.sessionId && identity.authenticated) {
                previousLease = found->second;
                leases_.erase(found);
                released = true;
            }
        }
        if (!released) event.reason = "Authority lease could not be released by this session";
        if (released && !record(event)) {
            std::lock_guard lock(mutex_);
            if (!leases_.contains(scope) && previousLease) leases_.emplace(scope, *previousLease);
            return false;
        }
        if (!released) (void)record(event);
        return released;
    }

    /// Force-take a scope lease from its current owner. Restricted to
    /// ControlsEngineer and Administrator — an operator must wait for expiry
    /// or a cooperative release. The evicted owner is recorded in the audit
    /// event reason so takeovers are attributable.
    AuthorityResult takeover(const std::string& scope, const SessionIdentity& identity,
                             Duration requested, TimePoint now = SteadyClock::now()) {
        AuthorityResult result;
        AuthorityEvent event{"takeover", scope, identity.actor, identity.sessionId, {}, 0, now};
        std::string previousOwner;
        {
            std::lock_guard lock(mutex_);
            expireLocked(scope, now);
            if (scope.empty()) result.blocker = "A machine or motion-group scope is required";
            else if (!identity.authenticated || identity.sessionId.empty())
                result.blocker = "An authenticated session is required";
            else if (identity.role < Role::ControlsEngineer)
                result.blocker = "Takeover requires a controls-engineer or administrator role";
            else if (requested <= Duration::zero() || requested > maximumLease_)
                result.blocker = "Requested lease duration is outside server policy";
            else {
                const auto found = leases_.find(scope);
                if (found != leases_.end()) previousOwner = found->second.ownerActor;
                auto& lease = leases_[scope];
                lease.token = nextToken_++;
                lease.scope = scope;
                lease.ownerSession = identity.sessionId;
                lease.ownerActor = identity.actor;
                lease.ownerRole = identity.role;
                lease.expiresAt = now + requested;
                result.lease = lease;
                event.token = lease.token;
                if (!previousOwner.empty())
                    event.reason = "Evicted previous owner: " + previousOwner;
            }
        }
        if (result.lease && !record(event)) {
            std::lock_guard lock(mutex_);
            if (const auto found = leases_.find(scope);
                found != leases_.end() && found->second.token == result.lease->token)
                leases_.erase(found);
            result.lease.reset();
            result.blocker = "Authority audit service is unavailable; takeover was rejected";
        } else if (!result.blocker.empty()) {
            (void)record(event);
        }
        return result;
    }

    std::optional<AuthorityLease> current(const std::string& scope,
                                          TimePoint now = SteadyClock::now()) {
        std::lock_guard lock(mutex_);
        expireLocked(scope, now);
        const auto found = leases_.find(scope);
        if (found == leases_.end()) return std::nullopt;
        return found->second;
    }

    bool owns(uint64_t token, const std::string& scope, const SessionIdentity& identity,
              TimePoint now = SteadyClock::now()) {
        const auto lease = current(scope, now);
        return lease && identity.authenticated && eligibleRoles_.contains(identity.role) && lease->token == token &&
               lease->ownerSession == identity.sessionId;
    }

    /// Snapshot of every active lease, ordered by scope (for the
    /// machine.authority snapshot signal).
    std::vector<AuthorityLease> leases(TimePoint now = SteadyClock::now()) {
        std::lock_guard lock(mutex_);
        std::vector<AuthorityLease> out;
        for (auto it = leases_.begin(); it != leases_.end();) {
            if (it->second.expiresAt <= now) it = leases_.erase(it);
            else { out.push_back(it->second); ++it; }
        }
        std::sort(out.begin(), out.end(),
                  [](const AuthorityLease& a, const AuthorityLease& b) { return a.scope < b.scope; });
        return out;
    }

    /// Force-release every lease owned by a session (disconnect/expiry stop
    /// semantics). Returns the number released; each emits an audit event.
    size_t releaseAll(const std::string& sessionId, TimePoint now = SteadyClock::now()) {
        std::vector<AuthorityEvent> events;
        size_t released = 0;
        {
            std::lock_guard lock(mutex_);
            for (auto it = leases_.begin(); it != leases_.end();) {
                if (it->second.ownerSession == sessionId) {
                    events.push_back({"release", it->first, it->second.ownerActor, sessionId,
                                      "Session ended; lease force-released", it->second.token, now});
                    it = leases_.erase(it);
                    ++released;
                } else ++it;
            }
        }
        for (const auto& event : events) (void)record(event);
        return released;
    }

private:
    void expireLocked(const std::string& scope, TimePoint now) {
        const auto found = leases_.find(scope);
        if (found != leases_.end() && found->second.expiresAt <= now) leases_.erase(found);
    }
    bool record(const AuthorityEvent& event) const {
        if (!audit_) return false;
        try { audit_(event); return true; }
        catch (...) { return false; }
    }

    Duration maximumLease_;
    Audit audit_;
    std::unordered_set<Role> eligibleRoles_;
    mutable std::mutex mutex_;
    std::unordered_map<std::string, AuthorityLease> leases_;
    uint64_t nextToken_ = 1;
};

struct CommandRequest {
    std::string requestUuid;
    std::string scope;
    uint64_t authorityToken = 0;
    std::vector<std::string> targets;
    std::unordered_map<std::string, uint64_t> expectedGenerations;
    Action action = Action::Enable;
    TimePoint deadline{};
    std::optional<uint64_t> configurationRevision;
    std::vector<uint8_t> parameters;
};

struct ResourceCommandState {
    uint64_t generation = 0;
    std::unordered_set<Action> supportedActions;
    std::vector<std::string> blockers;
};

struct CommandEnvironment {
    uint64_t configurationRevision = 0;
    std::vector<std::string> machineBlockers;
    std::unordered_map<std::string, ResourceCommandState> resources;
};

struct DispatchResult {
    bool accepted = false;
    std::string operationId;
    std::string message;
};

struct CommandReceipt {
    std::string requestUuid;
    CommandState state = CommandState::Rejected;
    std::string message;
    std::string operationId;
    std::string auditId;
    std::vector<std::string> blockers;
};

struct AuditRecord {
    std::string requestUuid;
    std::string auditId;
    std::string actor;
    std::string role;
    std::string sessionId;
    std::string source;
    std::string scope;
    Action action = Action::Enable;
    std::vector<std::string> targets;
    CommandState result = CommandState::Rejected;
    std::unordered_map<std::string, uint64_t> expectedGenerations;
    std::string message;
    TimePoint timestamp{};
};

struct CommandPolicy {
    Duration idempotencyRetention = std::chrono::minutes(10);
    size_t maximumTargets = 128;
    std::unordered_map<Action, std::unordered_set<Role>> actionRoles;
};

/// Validates and atomically queues application commands outside the real-time loop.
/// The application-provided dispatcher must enqueue work to its native motion executor;
/// it must not perform blocking EtherCAT/mailbox work on the caller thread.
class MachineCommandGate final {
public:
    using StateProvider = std::function<std::optional<CommandEnvironment>(const CommandRequest&)>;
    using Dispatcher = std::function<DispatchResult(const CommandRequest&, const SessionIdentity&)>;
    using Audit = std::function<std::string(const AuditRecord&)>;

    MachineCommandGate(ControlAuthority& authority, StateProvider stateProvider,
                       Dispatcher dispatcher, Audit audit, CommandPolicy policy = {})
        : authority_(authority), stateProvider_(std::move(stateProvider)),
          dispatcher_(std::move(dispatcher)), audit_(std::move(audit)), policy_(policy) {}

    CommandReceipt submit(const CommandRequest& request, const SessionIdentity& identity,
                          TimePoint now = SteadyClock::now()) {
        if (request.requestUuid.empty()) return rejected(request, identity, "Command request UUID is required", now);
        {
            std::lock_guard lock(mutex_);
            pruneLocked(now);
            const auto found = idempotency_.find(request.requestUuid);
            if (found != idempotency_.end()) {
                if (found->second.sessionId != identity.sessionId)
                    return rejected(request, identity, "Request UUID belongs to another session", now);
                return found->second.receipt;
            }
            CommandReceipt pending{request.requestUuid, CommandState::InProgress, "Command validation in progress", {}, {}, {}};
            idempotency_.emplace(request.requestUuid, Cached{identity.sessionId, pending, now});
        }

        CommandReceipt receipt{request.requestUuid, CommandState::Rejected, {}, {}, {}, {}};
        if (!identity.authenticated || identity.sessionId.empty()) addBlocker(receipt, "An authenticated session is required");
        if (!isActionAllowed(request.action, identity.role))
            addBlocker(receipt, "Role is not authorized for this machine action");
        if (!audit_) addBlocker(receipt, "Command audit service is unavailable");
        if (request.targets.empty()) addBlocker(receipt, "At least one target resource is required");
        if (request.targets.size() > policy_.maximumTargets) addBlocker(receipt, "Target count exceeds server policy");
        if (request.deadline == TimePoint{} || request.deadline <= now) addBlocker(receipt, "Command deadline has expired or is missing");
        if (!authority_.owns(request.authorityToken, request.scope, identity, now))
            addBlocker(receipt, "Current session does not hold valid control authority for this scope");

        std::optional<CommandEnvironment> environment;
        try {
            if (receipt.blockers.empty() && stateProvider_) environment = stateProvider_(request);
        } catch (...) {
            addBlocker(receipt, "Machine state provider failed; command was not dispatched");
        }
        if (receipt.blockers.empty() && !environment) addBlocker(receipt, "Machine state is unavailable; command was not dispatched");
        if (environment) validateEnvironment(request, *environment, receipt);
        if (receipt.blockers.empty()) {
            receipt.state = CommandState::InProgress;
            receipt.message = "Preflight passed; dispatch pending";
            receipt.auditId = persistAudit(request, identity, receipt, now);
            if (receipt.auditId.empty()) {
                receipt.state = CommandState::Rejected;
                receipt.message = "Command audit service failed; command was not dispatched";
                receipt.blockers.push_back(receipt.message);
            } else {
                try {
                    const auto dispatched = dispatcher_ ? dispatcher_(request, identity) : DispatchResult{};
                    if (dispatched.accepted) {
                        receipt.state = CommandState::Accepted;
                        receipt.message = dispatched.message;
                        receipt.operationId = dispatched.operationId;
                    } else {
                        receipt.state = CommandState::Rejected;
                        receipt.message = dispatched.message.empty() ? "Native command dispatcher rejected the request" : dispatched.message;
                        receipt.blockers.push_back(receipt.message);
                    }
                } catch (...) {
                    receipt.state = CommandState::Failed;
                    receipt.message = "Native command dispatcher failed; inspect operation state before retrying";
                    receipt.blockers.push_back(receipt.message);
                }
            }
        } else {
            receipt.state = CommandState::Rejected;
            receipt.message = receipt.blockers.front();
        }
        const auto completionAuditId = persistAudit(request, identity, receipt, now);
        if (receipt.auditId.empty()) receipt.auditId = completionAuditId;
        {
            std::lock_guard lock(mutex_);
            const auto found = idempotency_.find(request.requestUuid);
            if (found != idempotency_.end()) found->second.receipt = receipt;
        }
        return receipt;
    }

private:
    struct Cached { std::string sessionId; CommandReceipt receipt; TimePoint insertedAt; };

    static void addBlocker(CommandReceipt& receipt, std::string reason) {
        receipt.blockers.push_back(std::move(reason));
    }
    static std::string roleName(Role role) {
        switch (role) {
            case Role::Observer: return "observer";
            case Role::Operator: return "operator";
            case Role::Technician: return "technician";
            case Role::ControlsEngineer: return "controls_engineer";
            case Role::Administrator: return "administrator";
        }
        return "unknown";
    }
    bool isActionAllowed(Action action, Role role) const {
        if (const auto configured = policy_.actionRoles.find(action);
            configured != policy_.actionRoles.end()) {
            return configured->second.contains(role);
        }
        switch (action) {
            case Action::Recover:
                return role == Role::Technician || role == Role::ControlsEngineer;
            case Action::ConfigurationStage:
            case Action::ConfigurationValidate:
            case Action::ConfigurationCommit:
            case Action::ConfigurationRollback:
                return role == Role::ControlsEngineer;
            case Action::Enable:
            case Action::Disable:
            case Action::QuickStop:
            case Action::FaultReset:
            case Action::SetMode:
            case Action::JogStart:
            case Action::JogRenew:
            case Action::JogStop:
            case Action::StepMove:
            case Action::MoveToPosition:
            case Action::GroupMove:
            case Action::HomePrepare:
            case Action::HomeStart:
            case Action::HomeCancel:
            case Action::CancelOperation:
            case Action::AcknowledgeAlarm:
                return role == Role::Operator || role == Role::Technician ||
                       role == Role::ControlsEngineer;
        }
        return false;
    }
    static void validateEnvironment(const CommandRequest& request, const CommandEnvironment& environment,
                                    CommandReceipt& receipt) {
        for (const auto& blocker : environment.machineBlockers) addBlocker(receipt, blocker);
        if (request.configurationRevision && *request.configurationRevision != environment.configurationRevision)
            addBlocker(receipt, "Configuration revision changed; refresh before retrying");
        std::unordered_set<std::string> unique;
        for (const auto& target : request.targets) {
            if (!unique.insert(target).second) {
                addBlocker(receipt, "Duplicate target resource: " + target);
                continue;
            }
            const auto expected = request.expectedGenerations.find(target);
            const auto actual = environment.resources.find(target);
            if (expected == request.expectedGenerations.end()) {
                addBlocker(receipt, "Expected state generation is missing for " + target);
                continue;
            }
            if (actual == environment.resources.end()) {
                addBlocker(receipt, "Target resource is unavailable: " + target);
                continue;
            }
            if (expected->second != actual->second.generation)
                addBlocker(receipt, "State generation changed for " + target);
            if (!actual->second.supportedActions.contains(request.action))
                addBlocker(receipt, "Action is not advertised for " + target);
            for (const auto& blocker : actual->second.blockers)
                addBlocker(receipt, target + ": " + blocker);
        }
    }
    CommandReceipt rejected(const CommandRequest& request, const SessionIdentity& identity,
                            std::string reason, TimePoint now) {
        CommandReceipt receipt{request.requestUuid, CommandState::Rejected, reason, {}, {}, {reason}};
        receipt.auditId = persistAudit(request, identity, receipt, now);
        return receipt;
    }
    std::string persistAudit(const CommandRequest& request, const SessionIdentity& identity,
                             const CommandReceipt& receipt, TimePoint now) const {
        if (!audit_) return {};
        AuditRecord record{request.requestUuid, receipt.auditId, identity.actor, roleName(identity.role),
            identity.sessionId, identity.source, request.scope, request.action, request.targets,
            receipt.state, request.expectedGenerations, receipt.message, now};
        try { return audit_(record); }
        catch (...) { return {}; }
    }
    void pruneLocked(TimePoint now) {
        for (auto it = idempotency_.begin(); it != idempotency_.end();) {
            if (now - it->second.insertedAt > policy_.idempotencyRetention) it = idempotency_.erase(it);
            else ++it;
        }
    }

    ControlAuthority& authority_;
    StateProvider stateProvider_;
    Dispatcher dispatcher_;
    Audit audit_;
    CommandPolicy policy_;
    std::mutex mutex_;
    std::unordered_map<std::string, Cached> idempotency_;
};

} // namespace tether::io::machine
