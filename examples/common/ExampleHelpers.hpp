#pragma once

#include <argparse/argparse.hpp>
#include <optional>
#include <set>
#include <string>

#include "tether/ethercat/VLANRouter.hpp"

namespace Tether::Examples {

// ============================================================================
// Common argument-parsing helpers for EtherCAT host examples
// ============================================================================

/// Add `-i` / `--interface` to an ArgumentParser.
void addInterfaceArg(argparse::ArgumentParser& program,
                     const std::string& defaultValue = "eth0");

/// Add `--debug` to an ArgumentParser.
void addDebugArg(argparse::ArgumentParser& program);

/// Add `--debug-start` and `--debug-stop` to an ArgumentParser.
void addDebugConditionArgs(argparse::ArgumentParser& program);

/// Print detailed debug-flag help to stdout and return true if @p debugStr is "help".
bool printDebugHelpIfRequested(const std::string& debugStr);

/// Print debug condition help and return true if @p startStr is "help".
bool printDebugConditionHelpIfRequested(const std::string& startStr);

/// Add `--rx-vlan` and `--tx-vlan` to an ArgumentParser.
void addVlanArgs(argparse::ArgumentParser& program);

/// Add `-s` / `--slave` to an ArgumentParser.
void addSlaveArg(argparse::ArgumentParser& program, int defaultValue = 0);

/// Add `-t` / `--time` to an ArgumentParser.
void addDurationArg(argparse::ArgumentParser& program, double defaultValue = 0.0);

// ============================================================================
// Debug-flag helpers
// ============================================================================

/// Parse a comma-separated debug-flags string into a set of trimmed tokens.
std::set<std::string> parseDebugFlags(const std::string& debugStr);

/**
 * @brief Apply debug flags parsed from CLI to a Master instance.
 *
 * Uses the new per-master/per-slave syntax, e.g.:
 *   --debug rx-pdo,stateMachine:(slaves:0,2,5),coe-reads:(slaves:1-3)
 *
 * @param flags   Set of flag strings from parseDebugFlags()
 * @param master  Target master to configure debug flags on
 * @param tag     ESP-style log tag for diagnostics
 */
void applyDebugFlags(const std::set<std::string>& flags,
                     EtherCAT::Master& master,
                     const char* tag);

/**
 * @brief Apply debug start/stop conditions from CLI to a Master's debug gate.
 *
 * Parses the --debug-start and --debug-stop condition strings and adds them
 * to the master's DebugGate as global start/stop conditions.
 *
 * @param startCond  Start condition string (e.g. "state:pre-op"), or empty
 * @param stopCond   Stop condition string (e.g. "state:op"), or empty
 * @param master     Target master
 * @param tag        Log tag for diagnostics
 * @return true if all conditions parsed successfully (or were empty)
 */
bool applyDebugGateConditions(const std::string& startCond,
                              const std::string& stopCond,
                              EtherCAT::Master& master,
                              const char* tag);

// ============================================================================
// VLAN helpers
// ============================================================================

struct VlanConfig {
    bool enabled = false;
    std::optional<uint16_t> txVlan;
    bool rxAny = false;
    std::optional<EtherCAT::VLANRouter::VLANRange> rxRange;
};

/// Parse `--rx-vlan` / `--tx-vlan` strings into a VlanConfig.
/// Returns false and prints to stderr on invalid input.
bool parseVlanArgs(const std::string& rxVlanStr,
                   const std::string& txVlanStr,
                   VlanConfig& out,
                   const char* tag);

/// Log the VLAN configuration via TETHER_LOGI.
void logVlanConfig(const VlanConfig& config, const char* tag);

// ============================================================================
// Mailbox helpers
// ============================================================================

struct MailboxSizeConfig {
    uint16_t inSize = 256;
    uint16_t outSize = 256;
};

struct MailboxAddressConfig {
    uint16_t inAddress = 0x1000;
    uint16_t outAddress = 0x1200;
};

/// Add `-M` / `--mailbox-size` to an ArgumentParser.
void addMailboxSizeArg(argparse::ArgumentParser& program);

/// Add `--mailbox-address` to an ArgumentParser.
void addMailboxAddressArg(argparse::ArgumentParser& program);

/// Parse `--mailbox-size` value.  Single number sets both; `in:X,out:Y` sets independently.
/// Returns false and prints to stderr on invalid input.
bool parseMailboxSize(const std::string& str, MailboxSizeConfig& out);

/// Parse `--mailbox-address` value.  Expected format: `in:<hex>,out:<hex>`.
/// Validates that in-address != out-address.
/// Returns false and prints to stderr on invalid input.
bool parseMailboxAddress(const std::string& str, MailboxAddressConfig& out);

/// Log the mailbox configuration via TETHER_LOGI.
void logMailboxConfig(const MailboxSizeConfig& size,
                      const MailboxAddressConfig& addr,
                      const char* tag);

} // namespace Tether::Examples
