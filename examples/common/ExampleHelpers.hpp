#pragma once

#include <argparse/argparse.hpp>
#include <cstdint>
#include <initializer_list>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "tether/ethercat/DebugHelpers.hpp"
#include "tether/ethercat/Encapsulation.hpp"
#include "tether/ethercat/HostSetup.hpp"
#include "tether/ethercat/MailboxEndpointConfig.hpp"
#include "tether/ethercat/SdoAbortReport.hpp"
#include "tether/hal/InterfaceHelpers.hpp"
#include "tether/platform/CancellableLoop.hpp"
#include "tether/utils/StreamPrint.hpp"

#include "ParseArgs.hpp"

#ifdef TETHER_HAS_TERMINAL_UI
#include "tether/terminal_ui/Session.hpp"
#endif

namespace Tether::Examples {

// ============================================================================
// Common argument-parsing helpers for EtherCAT host examples
//
// Only the argparse-bound CLI conventions live here; the generic
// machinery they feed has moved into Tether proper:
//   - EtherCAT::EncapsulationConfig/parseEncapsulationSpec/setupEncapsulation
//     → tether/ethercat/Encapsulation.hpp
//   - EtherCAT::HostEtherNetSession/initHostEthernet/bringUpHost/...
//     → tether/ethercat/HostSetup.hpp
//   - EtherCAT::debug::parseFlags/applyFlags/applyGateConditions
//     → tether/ethercat/DebugHelpers.hpp
//   - EtherCAT::HAL::resolveInterface/listPhysicalInterfaces/
//     logPermissionDeniedError → tether/hal/InterfaceHelpers.hpp
//   - EtherCAT::MailboxSizeConfig/MailboxAddressConfig/parseMailboxSize/
//     parseMailboxAddress/logMailboxConfig
//     → tether/ethercat/MailboxEndpointConfig.hpp
//   - EtherCAT::reportSdoAbort → tether/ethercat/SdoAbortReport.hpp
//   - Tether::Platform::runCancellableLoop
//     → tether/platform/CancellableLoop.hpp
//   - Tether::Utils::printTimestamped → tether/utils/StreamPrint.hpp
// ============================================================================

/// Add `-i` / `--interface` (plus `--encapsulation`) to an ArgumentParser.
/// Default is empty  -  use EtherCAT::HAL::resolveInterface() to
/// auto-select when not given.
void addInterfaceArg(argparse::ArgumentParser& program,
                     const std::string& defaultValue = "");

/// Add `--list-interfaces` to an ArgumentParser.
/// Pair with EtherCAT::HAL::listPhysicalInterfaces().
void addListInterfacesArg(argparse::ArgumentParser& program);

/// Add `--debug` to an ArgumentParser.
/// Pair with EtherCAT::debug::printFlagsHelp()/parseFlags()/applyFlags().
void addDebugArg(argparse::ArgumentParser& program);

/// Add `--debug-start` and `--debug-stop` to an ArgumentParser.
/// Pair with EtherCAT::debug::printConditionHelp()/applyGateConditions().
void addDebugConditionArgs(argparse::ArgumentParser& program);

/// Add `-s` / `--slave` to an ArgumentParser.
void addSlaveArg(argparse::ArgumentParser& program, int defaultValue = 0);

/// Add `-t` / `--time` to an ArgumentParser.
void addDurationArg(argparse::ArgumentParser& program, double defaultValue = 0.0);

/// Add the `--interactive`/`--stream` output-mode flag pair used by every
/// dual-mode example.  `streamHelp` documents what the plain-lines mode
/// prints (e.g. "Print status as plain lines instead of the TUI").
void addStreamArgs(argparse::ArgumentParser& program,
                   std::string_view streamHelp);

/// Resolve --stream/--interactive into the output mode: --stream wins;
/// otherwise interactive unless the TUI can't run, in which case a
/// forced --interactive logs `unavailable_reason` under `tag` and the
/// result is stream.  Back-end for wantsInteractive().
bool resolveInteractiveMode(const argparse::ArgumentParser& program,
                            const char* tag, bool tui_available,
                            const char* unavailable_reason);

/// Output mode for the dual-mode examples: --stream wins, else
/// interactive when a TUI session is usable (TUI::Session::available()
/// when built with TETHER_HAS_TERMINAL_UI; never otherwise).  A forced
/// --interactive that can't be honored logs the reason and falls back
/// to stream — identical to the 16-line block this replaces.
inline bool wantsInteractive(const argparse::ArgumentParser& program,
                             const char* tag) {
#ifdef TETHER_HAS_TERMINAL_UI
    return resolveInteractiveMode(
        program, tag, Tether::TUI::Session::available(),
        "no usable terminal  —  falling back to --stream");
#else
    return resolveInteractiveMode(program, tag, false,
                                  "built without ncurses  —  using --stream mode");
#endif
}

/**
 * @brief Resolve a chain of slave-index arguments (e.g. "--slave",
 *        "--slave2", ...) into validated uint16 indices.
 *
 * Each argument after the first defaults to the previous index + 1 when
 * the parser produced a negative value (matching the
 * `.default_value(-1)` convention for extra --slaveN args).  The first
 * argument must resolve to a non-negative value.  All indices must fit
 * uint16 and be distinct.
 *
 * @return the index vector in argument order, or std::nullopt after
 *         printing an error to stderr.
 */
std::optional<std::vector<uint16_t>> resolveSlaveIndices(
    const argparse::ArgumentParser& program,
    std::initializer_list<std::string_view> arg_names);

// ============================================================================
// Mailbox helpers
// ============================================================================

/// Add `-M` / `--mailbox-size` to an ArgumentParser.
/// `default_size` sets the default mailbox size (bytes) for both MbxIn and
/// MbxOut.  ESC211 demos should pass 512: the 256-byte FNI/RSP/SDD sections
/// plus the SDO header do not fit into a 256-byte mailbox, which forces
/// segmented downloads that the slave rejects.
/// Pair with EtherCAT::parseMailboxSize().
void addMailboxSizeArg(argparse::ArgumentParser& program,
                       uint16_t default_size = 256);

/// Add `--mailbox-address` to an ArgumentParser.
/// Pair with EtherCAT::parseMailboxAddress().
void addMailboxAddressArg(argparse::ArgumentParser& program);

// ============================================================================
// ESI (EtherCAT Slave Information) XML helpers
// ============================================================================

/// Add `--esi-xml` to an ArgumentParser (default empty string).
/// When non-empty, examples use it to construct an EtherCAT::ESIFile and
/// drive mailbox/PDO configuration from the ESI XML instead of SII EEPROM.
void addEsiXmlArg(argparse::ArgumentParser& program,
                  const std::string& defaultValue = "");

} // namespace Tether::Examples
