#pragma once

/**
 * @file SyncDiagnostics.hpp
 * @brief Human-readable EtherCAT synchronisation diagnostics for one slave
 *
 * Collects every synchronisation-related reading a slave offers and formats
 * them as a well-structured report for logging:
 *
 *  - Standard SM synchronisation parameters 0x1C32 (outputs) / 0x1C33
 *    (inputs): sync mode, cycle times, missed/exceeded/shift counters and
 *    the sync error flag.
 *  - Distributed-clock ESC registers (optional, via a register-read
 *    callback): sync activation, system time offset/delay/difference,
 *    speed counter, SYNC0 start time and cycle.
 *  - Vendor-specific objects (optional): the AS715N / StepperOnline A6
 *    objects 0x2013 (EtherCAT parameter incl. sync-lost counter) and
 *    0x2040 (operation monitoring incl. EtherCAT SyncPeriod and
 *    Sync/IRQ phases).  Reads that fail are simply omitted, so the
 *    utility is safe to use with other drives.
 *
 * Every field is std::optional: a failed SDO or register read leaves the
 * field empty and is rendered as "n/a" instead of aborting the report.
 * collect() additionally derives a list of plain-text findings (e.g.
 * "SYNC0 cycle does not match the SM cycle", "SM events were missed"),
 * which is usually the fastest way to spot why a slave raised AL 0x001A /
 * 0x002C.
 *
 * Typical usage (from a supervisor critical-condition callback, while the
 * slave mailbox may still respond):
 *
 * @code
 *   auto reg_read = [&master](uint16_t s, uint16_t reg, void* d, uint16_t n) {
 *       return master.readRegister(EtherCAT::SlaveAddress(s), reg, d, n, 200);
 *   };
 *   auto report = EtherCAT::SyncDiagnostics::collect(
 *       slave_index, master.sdoManager(slave_index), reg_read);
 *   TETHER_LOGW(tag, "Slave {} sync diagnostics (AL 0x{:04X}):\n{}",
 *               slave_index, al_code,
 *               EtherCAT::SyncDiagnostics::format(report));
 * @endcode
 */

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "tether/ethercat/CoEManager.hpp"

namespace EtherCAT {

/// Register read callback: (slave_index, reg_addr, out, size) -> success.
/// Bind Master::readRegister / IDCTransport::readRegister / a test double.
using SyncDiagRegRead =
    std::function<bool(uint16_t slave_index, uint16_t reg_addr,
                       void* data, uint16_t size)>;

struct SyncDiagOptions {
    /// SDO options for all object reads.  Keep the timeout short when
    /// calling from an error path — the slave may already be degraded.
    CoE::CoETransactionOptions sdo_opts{.timeout_ms = 250, .max_retries = 0};
    bool read_sm_input   = true;  ///< also read 0x1C33 (SM3/input)
    bool read_dc_regs    = true;  ///< read DC registers via the RegRead func
    bool read_vendor     = true;  ///< probe vendor objects (absence tolerated)
};

/// One standard SM synchronisation parameter object (0x1C32 or 0x1C33).
struct SyncDiagSmSync {
    std::optional<uint16_t> sync_mode;            ///< :01
    std::optional<uint32_t> cycle_time_ns;        ///< :02
    std::optional<uint16_t> sync_modes_supported; ///< :04
    std::optional<uint32_t> min_cycle_time_ns;    ///< :05
    std::optional<uint32_t> calc_copy_time_ns;    ///< :06
    std::optional<uint32_t> sync0_cycle_time_ns;  ///< :0A
    std::optional<uint16_t> sm_event_missed;      ///< :0B
    std::optional<uint16_t> cycle_exceeded;       ///< :0C
    std::optional<uint16_t> shift_too_short;      ///< :0D
    std::optional<uint8_t>  sync_error;           ///< :20
};

/// Distributed-clock ESC register snapshot.
struct SyncDiagDcRegs {
    std::optional<uint8_t>  sync_act;         ///< 0x0981 DCSyncAct
    std::optional<int64_t>  sys_offset_ns;    ///< 0x0920 system time offset
    std::optional<uint32_t> sys_delay_ns;     ///< 0x0928 transmission delay
    std::optional<int32_t>  sys_time_diff_ns; ///< 0x092C system time difference
    std::optional<uint32_t> speed_counter;    ///< 0x0930 speed counter
    std::optional<uint32_t> time_filter;      ///< 0x0934 time filter
    std::optional<uint64_t> sync0_start_ns;   ///< 0x0990 SYNC0 start time
    std::optional<uint32_t> cycle0_ns;        ///< 0x09A0 SYNC0 cycle
};

/// Vendor objects — currently the AS715N / StepperOnline A6 set.
struct SyncDiagVendor {
    std::optional<uint16_t> sync_lost_window;     ///< 0x2013:03
    std::optional<uint16_t> sync_lost_counter;    ///< 0x2013:05
    std::optional<uint16_t> sync_mode_set;        ///< 0x2013:06
    std::optional<uint16_t> sync_error_window;    ///< 0x2013:07
    std::optional<uint16_t> csp_increment_over;   ///< 0x2013:08
    std::optional<int32_t>  ecat_sync_period;     ///< 0x2040:59
    std::optional<int32_t>  sync_irq_phases;      ///< 0x2040:61
};

struct SyncDiagReport {
    uint16_t slave_index = 0;
    SyncDiagSmSync sm_out;      ///< 0x1C32 (SM2, outputs)
    SyncDiagSmSync sm_in;       ///< 0x1C33 (SM3, inputs)
    SyncDiagDcRegs dc;
    SyncDiagVendor vendor;
    /// Derived interpretation lines ("findings"), empty when nothing
    /// remarkable was detected.
    std::vector<std::string> findings;
};

class SyncDiagnostics {
public:
    /// Collect all sync diagnostics for one slave.
    /// @param reg_read  Register access for the DC section; pass nullptr
    ///                  (or set Options::read_dc_regs=false) to skip it.
    static SyncDiagReport collect(uint16_t slave_index,
                                  CoE::CoEManager& coe,
                                  SyncDiagRegRead reg_read = nullptr,
                                  const SyncDiagOptions& opts = {});

    /// Render a collected report as a well-formatted multi-line string.
    static std::string format(const SyncDiagReport& r);

    /// Human-readable name of a 0x1C32/0x1C33 sync-mode value.
    static const char* syncModeName(uint16_t mode);
};

} // namespace EtherCAT
