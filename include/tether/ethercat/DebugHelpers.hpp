#pragma once

#include <set>
#include <string>

namespace EtherCAT {

class Master;

namespace debug {

/// Parse a comma-separated debug-flags spec into a set of trimmed tokens.
std::set<std::string> parseFlags(const std::string& spec);

/**
 * @brief Apply a set of debug-flag tokens to a Master instance.
 *
 * Supports the per-master/per-slave filter syntax, e.g.:
 *   rx-pdo,stateMachine:(slaves:0,2,5),coe-reads:(slaves:1-3)
 *
 * Unknown flag names are reported via the log.
 *
 * @param flags   Set of flag tokens from parseFlags()
 * @param master  Target master to configure debug flags on
 * @param tag     ESP-style log tag for diagnostics
 */
void applyFlags(const std::set<std::string>& flags,
                Master& master,
                const char* tag);

/**
 * @brief Apply debug start/stop condition specs to a Master's debug gate.
 *
 * Parses the condition strings (e.g. "state:pre-op") and adds them to the
 * master's DebugGate as global start/stop conditions.
 *
 * @param startCond  Start condition spec, or empty
 * @param stopCond   Stop condition spec, or empty
 * @param master     Target master
 * @param tag        Log tag for diagnostics
 * @return true if all conditions parsed successfully (or were empty)
 */
bool applyGateConditions(const std::string& startCond,
                         const std::string& stopCond,
                         Master& master,
                         const char* tag);

/// Print the available debug flags and filter syntax to stdout and return
/// true when @p spec is the "help" token.
bool printFlagsHelp(const std::string& spec);

/// Print debug condition help and return true when @p spec is "help".
bool printConditionHelp(const std::string& spec);

} // namespace debug
} // namespace EtherCAT
