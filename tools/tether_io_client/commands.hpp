/**
 * @file commands.hpp
 * @brief CLI command dispatcher for tether_io_client.
 *
 * dispatch() is shared by the direct CLI, the `shell` REPL, and the TUI `:`
 * prompt. Global output options (json/format/filter) come through `Ctx`.
 */
#pragma once

#include "client_session.hpp"

#include <ostream>
#include <string>
#include <vector>

namespace tether::io::client {

struct Ctx {
    IoSession& session;
    bool json = false;          ///< -j/--json: machine-readable output
    bool verbose = false;       ///< --verbose: per-request timing to err
    std::ostream& out;
    std::ostream& err;
};

/// Execute one command line (args[0] = command name).
/// Returns 0 on success, nonzero on failure. Prints human-readable
/// errors to ctx.err.
int dispatch(Ctx& ctx, const std::vector<std::string>& args);

/// dispatch() + --verbose timing and stale-catalog/schema notices.
int dispatchTimed(Ctx& ctx, const std::vector<std::string>& args);

/// Split a command line into tokens honoring "..." and '...' quoting and
/// backslash escapes (for the `shell` REPL / TUI `:` prompt).
std::vector<std::string> tokenize(const std::string& line);

} // namespace tether::io::client
