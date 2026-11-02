/**
 * @file tether_io_machine_auth.cpp
 * @brief The machine-auth boundary: bearer token → SessionIdentity →
 *        role-gated control authority.
 *
 * Clients never self-assert a role — a transport proves a credential,
 * an IMachineAuthProvider maps it to a SessionIdentity, and the
 * machine-control surface fails closed on unauthenticated or
 * ineligible identities. StaticTokenAuthProvider is the built-in
 * provider (constant-time compare, token-file format
 * "<token>\t<actor>\t<role>"). ControlAuthority is the enforcement
 * point: acquire() grants a scoped motion lease only to authenticated
 * sessions holding an eligible role.
 *
 *   ./tether_io_machine_auth
 */

#include "tether/io/MachineAuth.hpp"
#include "tether/io/MachineControl.hpp"

#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

using namespace tether::io::machine;

namespace {
const char* roleName(Role r) {
    switch (r) {
        case Role::Observer: return "observer";
        case Role::Operator: return "operator";
        case Role::Technician: return "technician";
        case Role::ControlsEngineer: return "engineer";
        case Role::Administrator: return "admin";
    }
    return "?";
}
} // namespace

int main() {
    // Token file: one "<token>\t<actor>\t<role>" record per line.
    const char* path = "/tmp/tether_auth_tokens.txt";
    {
        std::ofstream f(path);
        f << "# telemetry viewer\n"
          << "tok-observer-0000001\tgrafana\tobserver\n"
          << "tok-operator-0000002\tcell-op\toperator\n";
    }
    StaticTokenAuthProvider auth;
    try {
        auth = StaticTokenAuthProvider::fromFile(path);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "token file: %s\n", e.what());
        return 1;
    }
    std::remove(path);

    // Authority grants require an audit sink — without one the
    // acquire fails closed.
    std::vector<std::string> auditLog;
    ControlAuthority authority(
        std::chrono::seconds(30),
        [&](const AuthorityEvent& e) {
            auditLog.push_back(e.action + " " + e.scope + " " + e.actor);
        });
    bool ok = true;

    auto tryAcquire = [&](const char* label,
                          const std::optional<std::string>& credential) {
        // The transport hands a credential; the provider decides who
        // this session is — an unauthenticated identity on rejection.
        SessionIdentity id;
        id.sessionId = label;
        if (credential) {
            if (const auto proven = auth.authenticate(*credential)) {
                id = *proven;
                id.sessionId = label;
            }
        }
        const auto r = authority.acquire("motion-group-0", id,
                                       std::chrono::seconds(10));
        if (r.lease) {
            std::printf("[%s] role=%-9s -> lease token=%llu\n", label,
                        roleName(id.role),
                        static_cast<unsigned long long>(r.lease->token));
        } else {
            std::printf("[%s] role=%-9s -> blocked: %s\n", label,
                        roleName(id.role), r.blocker.c_str());
        }
        return r.lease.has_value();
    };

    // "Bearer <token>" is the wire credential shape.
    ok &= !tryAcquire("anon", std::nullopt);          // unauthenticated
    ok &= !tryAcquire("bad-token", std::string("nope-nope-nope"));
    ok &= !tryAcquire(
        "viewer",
        bearerCredential("Bearer tok-observer-0000001"));  // role too low
    ok &= tryAcquire(
        "operator",
        bearerCredential("Bearer tok-operator-0000002"));

    std::printf("[audit] %zu events\n", auditLog.size());
    for (const auto& line : auditLog)
        std::printf("        %s\n", line.c_str());

    return ok ? 0 : 1;
}
