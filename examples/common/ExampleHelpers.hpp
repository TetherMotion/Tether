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

/// Enable the appropriate EtherCAT debug logging for each flag in @p flags.
/// If @p knownFlags is non-null, warn about unknown flags.
void applyDebugFlags(const std::set<std::string>& flags,
                     const std::set<std::string>* knownFlags,
                     const char* tag);

/// Full set of known debug flags used by most examples.
const std::set<std::string>& allKnownDebugFlags();

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

} // namespace Tether::Examples
