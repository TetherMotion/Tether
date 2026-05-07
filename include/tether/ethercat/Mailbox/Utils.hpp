#pragma once

#include <cstdint>

namespace EtherCAT {
class EtherCATMaster;
class CiA402Drive;

namespace Mailbox {
namespace Utils {

/**
 * @brief Log statusword warnings, following-errors and internal limits.
 *
 * This helper encapsulates the bit‑mask checks previously performed inline
 * in examples.  It maintains a simple warning-active state that the caller
 * must persist across invocations and prints messages when warnings are
 * asserted or cleared.  Calling code should pass the current cycle count so
 * the helper can log durations.
 *
 * @param statusword           Status word read from the drive (TxPDO)
 * @param warning_active       Reference to boolean state tracking whether the
 *                             warning bit was active on the previous call.
 * @param warning_first_cycle  Reference to cycle count when the warning first
 *                             became active (only updated on rising edge).
 * @param cycle                Current cycle count (for timestamping logs).
 * @param tag                  Optional log tag (defaults to "Mailbox").
 */
void logStatuswordDiagnostics(uint16_t statusword,
                              bool& warning_active,
                              uint64_t& warning_first_cycle,
                              uint64_t cycle,
                              const char* tag = "Mailbox");

/**
 * @brief Dump mailbox header and SM status registers for a slave.
 *
 * This helper is a direct extract of the diagnostic code previously
 * embedded in several examples. It reads the configured mailbox addresses
 * from the master's SDO subsystem, prints SM0/SM1 watchdog state via the
 * slave accessor, and then peeks at the six-byte mailbox header for both
 * the write (master→slave) and read (slave→master) buffers.  A warning is
 * printed if the APRD fails or if the mailbox configuration is unavailable.
 *
 * @param master    Reference to EtherCATMaster instance.
 * @param slave_idx 0-based slave index.
 * @param tag       Optional log tag (defaults to "Mailbox").
 */
void dumpHeaderAndStatus(EtherCATMaster& master,
                         uint16_t slave_idx,
                         const char* tag = "Mailbox");

/**
 * @brief Convenience helper that dumps both sync-manager registers and
 *        mailbox state for a CiA402Drive object.
 *
 * The implementation simply invokes dumpHeaderAndStatus() and then
 * performs the same AL-status read that was previously found in the
 * example.  If the supplied drive has no associated master, the function
 * logs a warning and returns immediately.
 *
 * @param drive  Drive object whose master and index will be queried.
 * @param tag    Optional log tag (defaults to "Mailbox").
 */
void dumpSlaveSyncAndMailboxInfo(const CiA402Drive& drive,
                                 const char* tag = "Mailbox");

} // namespace Utils
} // namespace Mailbox
} // namespace EtherCAT
