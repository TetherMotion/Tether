#pragma once

#include <cstdint>

namespace EtherCAT {

class Slave;

/**
 * @brief Print a clear, user-facing explanation of the last CoE SDO abort
 *        reported by the slave on @p slave.
 *
 * Call this immediately after a Slave::sdoRead/sdoWrite family call returns
 * SlaveError::SDOAborted. It reads Slave::lastSdoAbortCode(), decodes the
 * standard 32-bit CoE abort code into its human-readable meaning, and prints
 * the result to both the TETHER log and stderr so the user sees it on the
 * console regardless of log routing. It also reports the operation direction
 * (read vs. write) and the attempted payload length from
 * Slave::lastSdoAttemptedLength().
 *
 * For the length-mismatch family of abort codes (0x06070010 / 0x06070012 /
 * 0x06070013) it additionally prints a hint pointing the user at the object
 * dictionary / ESI file to find the correct payload size  -  this is the
 * signature of the original "slave rejects the write because the payload is
 * the wrong size" failure.
 *
 * @param slave  The slave whose last SDO call was aborted.
 * @param tag    ESP-style log tag for the TETHER_LOGE line.
 * @return The abort code (0 if no abort was recorded  -  in that case nothing
 *         is printed).
 */
uint32_t reportSdoAbort(const Slave& slave, const char* tag);

} // namespace EtherCAT
