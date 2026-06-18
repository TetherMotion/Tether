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

/// Print detailed debug-flag help to stdout and return true if @p debugStr is "help".
bool printDebugHelpIfRequested(const std::string& debugStr);

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
/// Warns about unknown flags by comparing against the debug-module registry.
void applyDebugFlags(const std::set<std::string>& flags, const char* tag);

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
