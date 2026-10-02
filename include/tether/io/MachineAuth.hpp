#pragma once

/**
 * @file MachineAuth.hpp
 * @brief Authentication boundary for the machine.cia402.v1 surface.
 *
 * Every control path in MachineService already fails closed on
 * SessionIdentity::authenticated == false or an insufficient role.  An
 * IMachineAuthProvider is what turns a transport credential (bearer token,
 * mTLS identity, proxy header) into that identity.  Providers are the only
 * component allowed to mark a session authenticated — clients never self-
 * assert a role.
 */

#include "tether/io/MachineControl.hpp" // SessionIdentity, Role

#include <fstream>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>

namespace tether::io::machine {

/// Maps one transport credential to a session identity, or rejects it.
class IMachineAuthProvider {
public:
    virtual ~IMachineAuthProvider() = default;
    /// Returns an authenticated identity for `credential`, or nullopt.
    virtual std::optional<SessionIdentity> authenticate(std::string_view credential) const = 0;
};

/**
 * Static bearer-token provider for deployments without an external IdP.
 * Tokens are compared in constant time and never logged.  The token file is
 * one record per line: `<token>\t<actor>\t<role>` with role in
 * {observer, operator, technician, engineer, admin}.  `#` starts a comment.
 */
class StaticTokenAuthProvider final : public IMachineAuthProvider {
public:
    void addToken(std::string token, std::string actor, Role role) {
        if (token.size() < 16 || token.size() > 256 || actor.empty() || actor.size() > 127)
            throw std::invalid_argument("auth token or actor is outside bounds");
        entries_[std::move(token)] = {std::move(actor), role};
    }

    static StaticTokenAuthProvider fromFile(const std::string& path) {
        std::ifstream input(path);
        if (!input) throw std::runtime_error("cannot open auth token file: " + path);
        StaticTokenAuthProvider provider;
        std::string line;
        size_t lineNumber = 0;
        while (std::getline(input, line)) {
            ++lineNumber;
            if (line.empty() || line.front() == '#') continue;
            const auto first = line.find('\t');
            const auto second = first == std::string::npos
                ? std::string::npos : line.find('\t', first + 1);
            if (first == std::string::npos || second == std::string::npos ||
                line.find('\t', second + 1) != std::string::npos)
                throw std::runtime_error("auth token file line " +
                                         std::to_string(lineNumber) + " is malformed");
            provider.addToken(line.substr(0, first),
                              line.substr(first + 1, second - first - 1),
                              parseRoleName(line.substr(second + 1)));
        }
        if (provider.entries_.empty())
            throw std::runtime_error("auth token file has no usable tokens: " + path);
        return provider;
    }

    bool empty() const { return entries_.empty(); }

    std::optional<SessionIdentity> authenticate(std::string_view credential) const override {
        if (credential.empty() || credential.size() > 512) return std::nullopt;
        for (const auto& [token, entry] : entries_) {
            if (token.size() != credential.size()) continue;
            // Constant-time-ish compare: no early exit on mismatch.
            uint8_t diff = 0;
            for (size_t i = 0; i < token.size(); ++i)
                diff |= static_cast<uint8_t>(token[i] ^ credential[i]);
            if (diff == 0) {
                SessionIdentity identity;
                identity.actor = entry.actor;
                identity.source = "bearer-token";
                identity.role = entry.role;
                identity.authenticated = true;
                return identity;
            }
        }
        return std::nullopt;
    }

private:
    static Role parseRoleName(std::string_view name) {
        if (name == "observer") return Role::Observer;
        if (name == "operator") return Role::Operator;
        if (name == "technician") return Role::Technician;
        if (name == "engineer") return Role::ControlsEngineer;
        if (name == "admin") return Role::Administrator;
        throw std::runtime_error("unknown role in auth token file: " + std::string(name));
    }

    struct Entry {
        std::string actor;
        Role role;
    };
    std::map<std::string, Entry> entries_;
};

/// Extracts a bearer credential from an HTTP Authorization header value.
/// Returns nullopt for absent/malformed/non-Bearer values.
inline std::optional<std::string> bearerCredential(std::string_view authorization) {
    constexpr std::string_view prefix = "Bearer ";
    if (authorization.size() <= prefix.size() ||
        authorization.substr(0, prefix.size()) != prefix)
        return std::nullopt;
    const auto token = authorization.substr(prefix.size());
    if (token.empty() || token.find_first_of(" \t\r\n") != std::string_view::npos)
        return std::nullopt;
    return std::string(token);
}

} // namespace tether::io::machine
