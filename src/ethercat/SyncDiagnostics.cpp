/**
 * @file SyncDiagnostics.cpp
 * @brief Implementation of human-readable EtherCAT sync diagnostics
 */

#include "tether/ethercat/SyncDiagnostics.hpp"
#include "tether/platform/EspCompat.hpp"

#include <cstring>
#include <format>

namespace EtherCAT {

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

namespace {

template <typename T>
std::optional<T> readLE(const SyncDiagRegRead& reg_read,
                        uint16_t slave, uint16_t reg)
{
    if (!reg_read) return std::nullopt;
    uint8_t buf[sizeof(T)] = {0};
    if (!reg_read(slave, reg, buf, sizeof(buf))) return std::nullopt;
    T v{};
    std::memcpy(&v, buf, sizeof(T));   // EtherCAT registers are little-endian
    return v;
}

std::string opt_u(const std::optional<uint32_t>& v, const char* unit = "")
{
    if (!v) return "n/a";
    return unit[0] ? std::format("{} {}", *v, unit) : std::format("{}", *v);
}

std::string opt_i(const std::optional<int64_t>& v, const char* unit = "")
{
    if (!v) return "n/a";
    return unit[0] ? std::format("{} {}", *v, unit) : std::format("{}", *v);
}

void line(std::string& out, const char* label, const std::string& value)
{
    out += std::format("  {:<38} : {}\n", label, value);
}

void smSection(std::string& out, const char* title,
               const SyncDiagSmSync& sm)
{
    out += std::format("{}\n", title);
    line(out, "Sync mode (:01)",
         sm.sync_mode
             ? std::format("{} ({})", *sm.sync_mode,
                           SyncDiagnostics::syncModeName(*sm.sync_mode))
             : "n/a");
    line(out, "Cycle time (:02)", opt_u(sm.cycle_time_ns, "ns"));
    if (sm.sync_modes_supported) {
        line(out, "Sync modes supported (:04)",
             std::format("0x{:04X}", *sm.sync_modes_supported));
    } else {
        line(out, "Sync modes supported (:04)", "n/a");
    }
    line(out, "Min cycle time (:05)", opt_u(sm.min_cycle_time_ns, "ns"));
    line(out, "Calc & copy time (:06)", opt_u(sm.calc_copy_time_ns, "ns"));
    line(out, "SYNC0 cycle time (:0A)", opt_u(sm.sync0_cycle_time_ns, "ns"));
    line(out, "SM event missed (:0B)",
         sm.sm_event_missed ? std::format("{}", *sm.sm_event_missed) : "n/a");
    line(out, "Cycle exceeded (:0C)",
         sm.cycle_exceeded ? std::format("{}", *sm.cycle_exceeded) : "n/a");
    line(out, "Shift too short (:0D)",
         sm.shift_too_short ? std::format("{}", *sm.shift_too_short) : "n/a");
    line(out, "Sync error (:20)",
         sm.sync_error ? std::format("{}", (unsigned)*sm.sync_error) : "n/a");
}

} // namespace

const char* SyncDiagnostics::syncModeName(uint16_t mode)
{
    switch (mode) {
    case 0: return "Free Run";
    case 1: return "SM synchronous";
    case 2: return "DC SYNC0";
    case 3: return "DC SYNC1";
    default: return "unknown";
    }
}

// ---------------------------------------------------------------------------
// collect()
// ---------------------------------------------------------------------------

SyncDiagReport SyncDiagnostics::collect(uint16_t slave_index,
                                        CoE::CoEManager& coe,
                                        SyncDiagRegRead reg_read,
                                        const SyncDiagOptions& opts)
{
    SyncDiagReport r;
    r.slave_index = slave_index;
    const auto& o = opts.sdo_opts;

    auto u8  = [&](uint16_t i, uint8_t s) { auto v = coe.readU8(i, s, o);  return v ? std::optional<uint8_t>(*v)  : std::nullopt; };
    auto u16 = [&](uint16_t i, uint8_t s) { auto v = coe.readU16(i, s, o); return v ? std::optional<uint16_t>(*v) : std::nullopt; };
    auto u32 = [&](uint16_t i, uint8_t s) { auto v = coe.readU32(i, s, o); return v ? std::optional<uint32_t>(*v) : std::nullopt; };
    auto i32 = [&](uint16_t i, uint8_t s) { auto v = coe.readI32(i, s, o); return v ? std::optional<int32_t>(*v)  : std::nullopt; };

    auto read_sm = [&](uint16_t idx, SyncDiagSmSync& sm) {
        sm.sync_mode            = u16(idx, 0x01);
        sm.cycle_time_ns        = u32(idx, 0x02);
        sm.sync_modes_supported = u16(idx, 0x04);
        sm.min_cycle_time_ns    = u32(idx, 0x05);
        sm.calc_copy_time_ns    = u32(idx, 0x06);
        sm.sync0_cycle_time_ns  = u32(idx, 0x0A);
        sm.sm_event_missed      = u16(idx, 0x0B);
        sm.cycle_exceeded       = u16(idx, 0x0C);
        sm.shift_too_short      = u16(idx, 0x0D);
        sm.sync_error           = u8 (idx, 0x20);
    };

    read_sm(0x1C32, r.sm_out);
    if (opts.read_sm_input)
        read_sm(0x1C33, r.sm_in);

    if (opts.read_dc_regs && reg_read) {
        r.dc.sync_act         = readLE<uint8_t>(reg_read, slave_index, 0x0981);
        r.dc.sys_offset_ns    = readLE<int64_t>(reg_read, slave_index, 0x0920);
        r.dc.sys_delay_ns     = readLE<uint32_t>(reg_read, slave_index, 0x0928);
        r.dc.sys_time_diff_ns = readLE<int32_t>(reg_read, slave_index, 0x092C);
        r.dc.speed_counter    = readLE<uint32_t>(reg_read, slave_index, 0x0930);
        r.dc.time_filter      = readLE<uint32_t>(reg_read, slave_index, 0x0934);
        r.dc.sync0_start_ns   = readLE<uint64_t>(reg_read, slave_index, 0x0990);
        r.dc.cycle0_ns        = readLE<uint32_t>(reg_read, slave_index, 0x09A0);
    }

    if (opts.read_vendor) {
        r.vendor.sync_lost_window   = u16(0x2013, 0x03);
        r.vendor.sync_lost_counter  = u16(0x2013, 0x05);
        r.vendor.sync_mode_set      = u16(0x2013, 0x06);
        r.vendor.sync_error_window  = u16(0x2013, 0x07);
        r.vendor.csp_increment_over = u16(0x2013, 0x08);
        r.vendor.ecat_sync_period   = i32(0x2040, 59);
        r.vendor.sync_irq_phases    = i32(0x2040, 61);
    }

    // ---- Derived findings --------------------------------------------------
    auto& f = r.findings;

    if (r.sm_out.sync_mode && *r.sm_out.sync_mode != 2)
        f.push_back(std::format(
            "output SM is not DC-SYNC0 synchronised (mode={} {})",
            *r.sm_out.sync_mode, syncModeName(*r.sm_out.sync_mode)));

    if (r.sm_out.sync0_cycle_time_ns && r.sm_out.cycle_time_ns &&
        *r.sm_out.sync0_cycle_time_ns != *r.sm_out.cycle_time_ns)
        f.push_back(std::format(
            "SYNC0 period ({} ns) != SM cycle time ({} ns): drive consumes "
            "setpoints at a different rate than the master sends them",
            *r.sm_out.sync0_cycle_time_ns, *r.sm_out.cycle_time_ns));

    if (r.sm_out.sm_event_missed && *r.sm_out.sm_event_missed > 0)
        f.push_back(std::format(
            "{} SM event(s) missed: process data arrived late relative to SYNC0",
            *r.sm_out.sm_event_missed));
    if (r.sm_out.cycle_exceeded && *r.sm_out.cycle_exceeded > 0)
        f.push_back(std::format(
            "{} cycle(s) exceeded: slave could not finish its cyclic work in time",
            *r.sm_out.cycle_exceeded));
    if (r.sm_out.shift_too_short && *r.sm_out.shift_too_short > 0)
        f.push_back(std::format(
            "{} shift-too-short event(s): SYNC0 shift leaves too little time "
            "between process data arrival and the sync pulse",
            *r.sm_out.shift_too_short));
    if (r.sm_out.sync_error && *r.sm_out.sync_error != 0)
        f.push_back("drive reports an active SM sync error (:20)");

    if (r.dc.sync_act && !(*r.dc.sync_act & 0x01))
        f.push_back("DC sync unit is not activated (0x0981 bit0 = 0)");
    if (r.dc.sys_time_diff_ns) {
        const int64_t d = *r.dc.sys_time_diff_ns;
        if (d > 100000 || d < -100000)
            f.push_back(std::format(
                "system time difference is {} ns: slave clock is still "
                "drifting against the DC reference", d));
    }
    if (r.dc.cycle0_ns && r.sm_out.sync0_cycle_time_ns &&
        *r.dc.cycle0_ns != *r.sm_out.sync0_cycle_time_ns)
        f.push_back(std::format(
            "ESC SYNC0 cycle register ({} ns) != slave-reported cycle "
            "({} ns)", *r.dc.cycle0_ns, *r.sm_out.sync0_cycle_time_ns));

    if (r.vendor.sync_lost_counter && *r.vendor.sync_lost_counter > 0)
        f.push_back(std::format(
            "vendor sync-lost counter = {} (window={}): drive has declared "
            "sync lost before", *r.vendor.sync_lost_counter,
            r.vendor.sync_lost_window ? std::format("{}", *r.vendor.sync_lost_window)
                                      : "n/a"));
    if (r.vendor.csp_increment_over && *r.vendor.csp_increment_over > 0)
        f.push_back(std::format(
            "vendor CSP increment-over counter = {}: position setpoint "
            "increments exceeded the expected per-cycle limit",
            *r.vendor.csp_increment_over));
    if (r.vendor.ecat_sync_period && r.dc.cycle0_ns &&
        *r.vendor.ecat_sync_period > 0 &&
        *r.vendor.ecat_sync_period != static_cast<int32_t>(*r.dc.cycle0_ns))
        f.push_back(std::format(
            "drive-detected EtherCAT sync period ({} ns) != programmed SYNC0 "
            "cycle ({} ns)", *r.vendor.ecat_sync_period, *r.dc.cycle0_ns));

    if (f.empty())
        f.push_back("no anomalies detected");

    return r;
}

// ---------------------------------------------------------------------------
// format()
// ---------------------------------------------------------------------------

std::string SyncDiagnostics::format(const SyncDiagReport& r)
{
    std::string out;
    out += std::format("== Sync diagnostics: slave {} ==\n", r.slave_index);

    smSection(out, "SM output sync (0x1C32):", r.sm_out);
    smSection(out, "SM input  sync (0x1C33):", r.sm_in);

    const SyncDiagDcRegs& d = r.dc;
    if (d.sync_act || d.sys_offset_ns || d.sys_time_diff_ns || d.cycle0_ns) {
        out += "Distributed clocks (ESC registers):\n";
        if (d.sync_act) {
            line(out, "Sync activation (0981)",
                 std::format("0x{:02X} [{} {} {} {}]", *d.sync_act,
                             (*d.sync_act & 0x01) ? "DC" : "-",
                             (*d.sync_act & 0x02) ? "SYNC0" : "-",
                             (*d.sync_act & 0x04) ? "SYNC1" : "-",
                             (*d.sync_act & 0x08) ? "AUTO" : "-"));
        }
        line(out, "Sys time offset (0920)", opt_i(d.sys_offset_ns, "ns"));
        line(out, "Sys tx delay (0928)", opt_u(d.sys_delay_ns, "ns"));
        line(out, "Sys time diff (092C)", opt_i(d.sys_time_diff_ns, "ns"));
        if (d.speed_counter)
            line(out, "Speed counter (0930)",
                 std::format("start={} diff={}",
                             *d.speed_counter & 0xFFFF,
                             (*d.speed_counter >> 16) & 0xFFFF));
        line(out, "Time filter (0934)", opt_u(d.time_filter, "ns"));
        line(out, "SYNC0 start time (0990)",
             d.sync0_start_ns ? std::format("{} ns", *d.sync0_start_ns) : "n/a");
        line(out, "SYNC0 cycle (09A0)", opt_u(d.cycle0_ns, "ns"));
    }

    const SyncDiagVendor& v = r.vendor;
    if (v.sync_lost_counter || v.sync_lost_window || v.sync_mode_set ||
        v.ecat_sync_period || v.sync_irq_phases || v.csp_increment_over) {
        out += "Vendor objects (AS715N):\n";
        line(out, "Sync lost window (2013:03)",
             v.sync_lost_window ? std::format("{}", *v.sync_lost_window) : "n/a");
        line(out, "Sync lost counter (2013:05)",
             v.sync_lost_counter ? std::format("{}", *v.sync_lost_counter) : "n/a");
        line(out, "Sync mode set (2013:06)",
             v.sync_mode_set ? std::format("{}", *v.sync_mode_set) : "n/a");
        line(out, "Sync error window (2013:07)",
             v.sync_error_window ? std::format("{}", *v.sync_error_window) : "n/a");
        line(out, "CSP increment over (2013:08)",
             v.csp_increment_over ? std::format("{}", *v.csp_increment_over) : "n/a");
        line(out, "EtherCAT SyncPeriod (2040:59)",
             v.ecat_sync_period ? std::format("{} ns", *v.ecat_sync_period) : "n/a");
        line(out, "Sync and IRQ phases (2040:61)",
             v.sync_irq_phases ? std::format("{}", *v.sync_irq_phases) : "n/a");
    }

    out += "Findings:\n";
    for (const auto& s : r.findings)
        out += std::format("  - {}\n", s);

    return out;
}

} // namespace EtherCAT
