/**
 * @file beckhoff_tui.cpp
 * @brief Beckhoff Device Explorer  -  unified interactive terminal UI
 *
 * Scans the EtherCAT chain once, classifies every Beckhoff terminal by
 * product code against all implemented driver families, brings every
 * recognized terminal to OP (each family in its own PDO group / shared
 * logical address space), and presents a navigable coupler/terminal
 * device tree with a live detail pane.
 *
 * Interaction per selected device:
 *   digital out / combined I/O : '1'..'9' toggle channel, ','/'.' select,
 *                                space toggle, 'a'/'o' invert all channels
 *                                of the selected terminal
 *   analog out                 : '['/']' select channel, '+'/'-' adjust,
 *                                '0' zero
 *   PWM                        : '+'/'-' duty, 'e' enable, '0' off
 *   pulse-train                : '+'/'-' frequency, 'f'/'r' direction,
 *                                '0' stop
 *   stepper / DC motor         : 'e' enable toggle, '+'/'-' velocity,
 *                                '0' stop, 'x' reset
 *   CiA402 drive               : 'e' FSA enable step, 'd' disable
 *                                voltage, '+'/'-' target velocity (CSV),
 *                                '0' stop
 *   serial/IO-Link             : 't' transmit test bytes
 *   power meter                : '+'/'-' index selector 0
 *   inputs / analog in / position / oversampling / power : live view
 *   TwinSAFE                   : classified only  -  needs FSoE config
 *                                (run ./beckhoff_safety_monitor)
 *
 * Usage (Linux, requires root or CAP_NET_RAW):
 *   ./beckhoff_tui -i eth0
 *   ./beckhoff_tui -i enp3s0 -t 60     # auto-quit after 60 s
 */

#include <algorithm>
#include <atomic>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include <unistd.h>

#include "tether/Beckhoff/AnalogInputTerminal.hpp"
#include "tether/Beckhoff/AnalogOutputTerminal.hpp"
#include "tether/Beckhoff/CombinedIoTerminal.hpp"
#include "tether/Beckhoff/CommTerminal.hpp"
#include "tether/Beckhoff/CompactDriveTerminal.hpp"
#include "tether/Beckhoff/DcMotorTerminal.hpp"
#include "tether/Beckhoff/InputTerminal.hpp"
#include "tether/Beckhoff/MultiAnalogInputTerminal.hpp"
#include "tether/Beckhoff/MultiAnalogOutputTerminal.hpp"
#include "tether/Beckhoff/MultiCombinedIoTerminal.hpp"
#include "tether/Beckhoff/MultiInputTerminal.hpp"
#include "tether/Beckhoff/MultiOutputTerminal.hpp"
#include "tether/Beckhoff/MultiPositionInputTerminal.hpp"
#include "tether/Beckhoff/OversamplingTerminal.hpp"
#include "tether/Beckhoff/OutputTerminal.hpp"
#include "tether/Beckhoff/PositionInputTerminal.hpp"
#include "tether/Beckhoff/PowerMeterTerminal.hpp"
#include "tether/Beckhoff/PulseTrainTerminal.hpp"
#include "tether/Beckhoff/PwmTerminal.hpp"
#include "tether/Beckhoff/SafetyTerminal.hpp"
#include "tether/Beckhoff/StepperTerminal.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/SlaveDiscoveryManager.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/platform/Platform.hpp"
#include "tether/utils/SignalHandler.hpp"

#ifdef TETHER_HAS_TERMINAL_UI
#include "tether/terminal_ui/Session.hpp"
#include "tether/terminal_ui/TreeScreen.hpp"
#include "common/DeviceTree.hpp"
#endif

#include "common/ExampleHelpers.hpp"
#include "common/EtherCATHostSetup.hpp"

#ifdef TETHER_HAS_TERMINAL_UI
#include <clocale>
#include <ncurses.h>
#endif

static const char* TAG = "beckhoff_tui";

namespace Beckhoff = EtherCAT::Beckhoff;
using EtherCAT::DiscoveredSlave;

static std::atomic<bool> g_cancel{false};

// ============================================================================
// Device classification  -  each slave is assigned to exactly one family
// ============================================================================

enum class DeviceFamily : uint8_t {
    None, Input, Output, Combined, AnalogIn, AnalogOut, Position,
    Stepper, Drive, Pwm, Pulse, DcMotor, PowerMeter, Oversampling,
    Comm, Safety,
};

struct DeviceFamilyEntry {
    DeviceFamily                                   family;
    std::span<const Beckhoff::DeviceIdentity> registry;
    const char*                           label;
    const char*                           example;
};

// Precedence order  -  the first registry containing the product code wins.
// More specific/capable families precede generic digital I/O so devices
// registered in both (e.g. EL2034 outputs + diagnostics, timestamped
// EL125x inputs) land in CombinedIo.
static const DeviceFamilyEntry kDeviceFamilies[] = {
    {DeviceFamily::Safety,       Beckhoff::Devices::kSafetyTerminals,
     "TwinSAFE FSoE",      "beckhoff_safety_monitor"},
    {DeviceFamily::Comm,         Beckhoff::Devices::kCommTerminals,
     "serial/IO-Link",     "beckhoff_serial_bridge"},
    {DeviceFamily::Oversampling, Beckhoff::Devices::kOversamplingTerminals,
     "oversampling",       "beckhoff_oversampling_monitor"},
    {DeviceFamily::PowerMeter,   Beckhoff::Devices::kPowerMeterTerminals,
     "power measurement",  "beckhoff_power_meter"},
    {DeviceFamily::DcMotor,      Beckhoff::Devices::kDcMotorTerminals,
     "DC motor",           "beckhoff_dcmotor_jog"},
    {DeviceFamily::Drive,        Beckhoff::Devices::kCompactDriveTerminals,
     "CiA402 drive",       "beckhoff_drive_move"},
    {DeviceFamily::Stepper,      Beckhoff::Devices::kStepperTerminals,
     "POS stepper",        "beckhoff_stepper_jog"},
    {DeviceFamily::Pulse,        Beckhoff::Devices::kPulseTrainTerminals,
     "pulse-train",        "beckhoff_pulse_jog"},
    {DeviceFamily::Pwm,          Beckhoff::Devices::kPwmTerminals,
     "PWM",                "beckhoff_pwm_sweep"},
    {DeviceFamily::Position,     Beckhoff::Devices::kPositionInputTerminals,
     "position/encoder",   "beckhoff_position_monitor"},
    {DeviceFamily::AnalogOut,    Beckhoff::Devices::kAnalogOutputTerminals,
     "analog output",      "beckhoff_analog_output_sweep"},
    {DeviceFamily::AnalogIn,     Beckhoff::Devices::kAnalogInputTerminals,
     "analog input",       "beckhoff_analog_input_monitor"},
    {DeviceFamily::Combined,     Beckhoff::Devices::kCombinedIoTerminals,
     "combined I/O",       "beckhoff_combined_io_toggle"},
    {DeviceFamily::Output,       Beckhoff::Devices::kOutputTerminals,
     "digital output",     "beckhoff_output_toggle"},
    {DeviceFamily::Input,        Beckhoff::Devices::kInputTerminals,
     "digital input",      "beckhoff_input_monitor"},
};

struct SlaveClass {
    DeviceFamily                              family = DeviceFamily::None;
    const Beckhoff::DeviceIdentity*  identity  = nullptr;
    const DeviceFamilyEntry*                    entry = nullptr;
};

static SlaveClass classify(const DiscoveredSlave& s) {
    const uint32_t vendor  = s.vendor_id ? *s.vendor_id : 0;
    const uint32_t product = s.product_code ? *s.product_code : 0;
    if (!product) return {};
    for (const auto& f : kDeviceFamilies) {
        for (const auto& identity : f.registry) {
            if (identity.product_code == product && identity.vendor_id == vendor)
                return {f.family, &identity, &f};
        }
    }
    return {};
}

// ============================================================================
// Drivers  -  every recognized slave brought to OP in one process
// ============================================================================

struct Drivers {
    // Chained families  -  one shared logical address space each.
    std::optional<Beckhoff::MultiInputTerminal<>>         din;
    std::optional<Beckhoff::MultiOutputTerminal<>>        dout;
    std::optional<Beckhoff::MultiCombinedIoTerminal<>>    combo;
    std::optional<Beckhoff::MultiAnalogInputTerminal<>>   ain;
    std::optional<Beckhoff::MultiAnalogOutputTerminal<>>  aout;
    std::optional<Beckhoff::MultiPositionInputTerminal<>> pos;

    // Single-terminal families  -  one driver per slave.
    std::vector<std::unique_ptr<Beckhoff::StepperTerminal>>      steppers;
    std::vector<std::unique_ptr<Beckhoff::CompactDriveTerminal>> drives;
    std::vector<std::unique_ptr<Beckhoff::PwmTerminal>>          pwms;
    std::vector<std::unique_ptr<Beckhoff::PulseTrainTerminal>>   pulses;
    std::vector<std::unique_ptr<Beckhoff::DcMotorTerminal>>      dcmotors;
    std::vector<std::unique_ptr<Beckhoff::PowerMeterTerminal>>   meters;
    std::vector<std::unique_ptr<Beckhoff::OversamplingTerminal>> overs;
    std::vector<std::unique_ptr<Beckhoff::CommTerminal>>         comms;

    /// Chain PDO groups needing an explicit exchangeAll() in the RT loop
    /// (single terminals ride on the default master_.pdo() group).
    std::vector<EtherCAT::PDOManager*> groups;

    /// slave index -> bring-up error text (empty = OK or unmanaged).
    std::map<uint16_t, std::string> errors;
};

template <typename Vec>
static typename Vec::value_type::element_type* findBySlave(Vec& v,
                                                           uint16_t idx) {
    for (auto& p : v) if (p->slaveIndex() == idx) return p.get();
    return nullptr;
}

/// Module index of `slave` inside a Multi chain, or SIZE_MAX.
template <typename Chain>
static size_t moduleOf(const Chain& c, uint16_t slave) {
    for (size_t m = 0; m < c.moduleCount(); ++m)
        if (c.slaveIndex(m) == slave) return m;
    return SIZE_MAX;
}

/// Bring every classified slave to OP.  Phase A configures all chains and
/// single terminals to SAFE-OP; phase B starts one realtime loop that
/// exchanges the default PDO group (single terminals) plus every chain's
/// dedicated group; phase C requests OP on all devices.
static void bringUp(EtherCAT::Master& master,
                    std::span<const DiscoveredSlave> slaves,
                    const std::map<uint16_t, SlaveClass>& assignments,
                    Drivers& drivers) {
    using Beckhoff::StartOptions;
    namespace Dev = Beckhoff::Devices;

    auto of = [&](DeviceFamily f) {
        std::vector<DiscoveredSlave> out;
        for (const auto& s : slaves) {
            auto it = assignments.find(s.index);
            if (it != assignments.end() && it->second.family == f) out.push_back(s);
        }
        return out;
    };
    auto idOf = [&](uint16_t idx) -> const Beckhoff::DeviceIdentity* {
        auto it = assignments.find(idx);
        return it != assignments.end() ? it->second.identity : nullptr;
    };
    auto fail = [&](uint16_t idx, const char* what, Beckhoff::Error e) {
        drivers.errors[idx] = std::string(what) + ": " + Beckhoff::errorToString(e);
        TETHER_LOGW(TAG, "slave {}: {}: {}", idx, what,
                    Beckhoff::errorToString(e));
    };

    // ---- Phase A: configure -------------------------------------------------
    auto chainCfg = [&](auto& opt, DeviceFamily f,
                        std::span<const Beckhoff::DeviceIdentity> reg,
                        const char* name) {
        auto scan = of(f);
        if (scan.empty()) return;
        opt.emplace(master);
        if (auto r = opt->detect(reg, scan); !r || *r == 0) return;
        if (auto r = opt->configure(); !r) {
            fail(opt->slaveIndex(opt->lastErrorModule() == SIZE_MAX
                     ? 0 : opt->lastErrorModule()),
                 name, r.error());
            return;
        }
        if (opt->pdoManager()) drivers.groups.push_back(opt->pdoManager());
        TETHER_LOGI(TAG, "{}: {} device(s) configured", name,
                    opt->moduleCount());
    };

    chainCfg(drivers.din,   DeviceFamily::Input,     Dev::kInputTerminals,         "inputs");
    chainCfg(drivers.dout,  DeviceFamily::Output,    Dev::kOutputTerminals,        "outputs");
    chainCfg(drivers.combo, DeviceFamily::Combined,  Dev::kCombinedIoTerminals,    "combo-io");
    chainCfg(drivers.ain,   DeviceFamily::AnalogIn,  Dev::kAnalogInputTerminals,   "analog-in");
    chainCfg(drivers.aout,  DeviceFamily::AnalogOut, Dev::kAnalogOutputTerminals,  "analog-out");
    chainCfg(drivers.pos,   DeviceFamily::Position,  Dev::kPositionInputTerminals, "position");

    auto singleCfg = [&](auto& vec, DeviceFamily f, const char* name) {
        for (const auto& s : of(f)) {
            vec.push_back(std::make_unique<typename
                std::remove_reference_t<decltype(vec)>::value_type::element_type>(
                    master, s, *idOf(s.index)));
            if (auto r = vec.back()->configure(); !r)
                fail(s.index, name, r.error());
        }
    };

    singleCfg(drivers.steppers,  DeviceFamily::Stepper,      "stepper");
    singleCfg(drivers.drives,    DeviceFamily::Drive,        "drive");
    singleCfg(drivers.pwms,      DeviceFamily::Pwm,          "pwm");
    singleCfg(drivers.pulses,    DeviceFamily::Pulse,        "pulse-train");
    singleCfg(drivers.dcmotors,  DeviceFamily::DcMotor,      "dc-motor");
    singleCfg(drivers.meters,    DeviceFamily::PowerMeter,   "power-meter");
    singleCfg(drivers.overs,     DeviceFamily::Oversampling, "oversampling");
    singleCfg(drivers.comms,     DeviceFamily::Comm,         "comm");

    const bool any = drivers.din || drivers.dout || drivers.combo || drivers.ain || drivers.aout || drivers.pos
                  || !drivers.steppers.empty() || !drivers.drives.empty() || !drivers.pwms.empty()
                  || !drivers.pulses.empty() || !drivers.dcmotors.empty()
                  || !drivers.meters.empty() || !drivers.overs.empty() || !drivers.comms.empty();
    if (!any) {
        TETHER_LOGW(TAG, "no recognized terminals  —  tree view only");
        return;
    }

    // ---- Phase B: one RT loop exchanging every group ------------------------
    auto* groups = &drivers.groups;
    master.setMotionControlCallback(
        [&master, groups](double) {
            master.pdo().exchangeAll();
            for (auto* g : *groups) g->exchangeAll();
            return !g_cancel.load();
        });
    EtherCAT::Master::RealtimeMotionLoopConfig cfg;
    cfg.cycle_period_us           = 1000;
    cfg.enable_dc_synchronization = false;
    if (!master.startRealtimeMotionControlLoop(cfg)) {
        TETHER_LOGE(TAG, "realtime loop failed to start");
        return;
    }

    // ---- Phase C: request OP -------------------------------------------------
    StartOptions noLoop;
    noLoop.manage_realtime_loop = false;

    auto chainOp = [&](auto& opt, const char* name) {
        // Only chains that configured in phase A: their group is already
        // in the exchange list.  start() would otherwise retry configure
        // and, on late success, create a group nobody exchanges.
        if (!opt || !opt->pdoManager()
            || std::find(drivers.groups.begin(), drivers.groups.end(),
                         opt->pdoManager()) == drivers.groups.end()) return;
        if (auto r = opt->start(noLoop); !r)
            fail(opt->slaveIndex(opt->lastErrorModule() == SIZE_MAX
                     ? 0 : opt->lastErrorModule()),
                 name, r.error());
    };
    chainOp(drivers.din, "inputs");   chainOp(drivers.dout, "outputs");
    chainOp(drivers.combo, "combo");  chainOp(drivers.ain, "analog-in");
    chainOp(drivers.aout, "analog-out"); chainOp(drivers.pos, "position");

    auto singleOp = [&](auto& vec, const char* name) {
        for (auto& t : vec) {
            if (auto r = t->start(noLoop); !r)
                fail(t->slaveIndex(), name, r.error());
        }
    };
    singleOp(drivers.steppers, "stepper");   singleOp(drivers.drives, "drive");
    singleOp(drivers.pwms, "pwm");           singleOp(drivers.pulses, "pulse-train");
    singleOp(drivers.dcmotors, "dc-motor");  singleOp(drivers.meters, "power-meter");
    singleOp(drivers.overs, "oversampling"); singleOp(drivers.comms, "comm");
}

// ============================================================================
// TUI
// ============================================================================

#ifdef TETHER_HAS_TERMINAL_UI
namespace TUI = Tether::TUI;

namespace {

// -- small drawing helpers ---------------------------------------------------

void put(WINDOW* w, int& row, const char* fmt, ...) {
    char buf[256];
    va_list ap; va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    mvwprintw(w, row++, 1, "%s", buf);
}

// Aligned "key        value" row — the detail pane's table structure.
void kv(WINDOW* w, int& row, const char* key, const char* fmt, ...) {
    char buf[256];
    va_list ap; va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    mvwprintw(w, row++, 1, "%-10s %s", key, buf);
}

// Dim horizontal rule across the pane width.
void rule(WINDOW* w, int& row) {
    const int width = getmaxx(w);
    wattron(w, A_DIM);
    for (int c = 1; c < width - 1; ++c)
        mvwprintw(w, row, c, "%s", "\xE2\x94\x80");   // ─
    wattroff(w, A_DIM);
    ++row;
}

void bitRow(WINDOW* w, int row, const char* label,
            size_t n, const std::function<bool(size_t)>& get) {
    mvwprintw(w, row, 1, "%-10s", label);
    for (size_t k = 0; k < n && k < 32; ++k) {
        const bool on = get(k);
        wattron(w, on ? (COLOR_PAIR(TUI::PalValue) | A_BOLD) : A_DIM);
        mvwprintw(w, row, 11 + static_cast<int>(k) * 2, "%s",
                  on ? "\xE2\x96\xA0" : "\xE2\x96\xA1");   // ■ □
        wattroff(w, on ? (COLOR_PAIR(TUI::PalValue) | A_BOLD) : A_DIM);
    }
}

const char* driveStateName(Beckhoff::DriveState s) {
    using Beckhoff::DriveState;
    switch (s) {
    case DriveState::NotReadyToSwitchOn:  return "not ready";
    case DriveState::SwitchOnDisabled:    return "switch-on disabled";
    case DriveState::ReadyToSwitchOn:     return "ready to switch on";
    case DriveState::SwitchedOn:          return "switched on";
    case DriveState::OperationEnabled:    return "OPERATION ENABLED";
    case DriveState::QuickStopActive:     return "quick stop";
    case DriveState::FaultReactionActive: return "fault reaction";
    case DriveState::Fault:               return "FAULT";
    default:                              return "unknown";
    }
}

} // namespace

// -- per-family detail renderers ----------------------------------------------

struct UiState {
    /// Selected channel/axis per slave (for '[',']' selection).
    std::map<uint16_t, size_t> sel;
    /// Input transition counters, per slave then channel.
    std::map<uint16_t, std::vector<uint32_t>> transitions;
    std::map<uint16_t, uint64_t>              prevBits;
};

static void renderLive(WINDOW* w, int& row, uint16_t slave, DeviceFamily family,
                       Drivers& drivers, UiState& uiState, const std::string& iface) {
    using namespace Beckhoff;

    auto hint = [&](const char* s) {
        ++row;
        wattron(w, COLOR_PAIR(TUI::PalHint));
        mvwprintw(w, row++, 1, "%s", s);
        wattroff(w, COLOR_PAIR(TUI::PalHint));
    };
    auto errLine = [&]() {
        auto it = drivers.errors.find(slave);
        if (it == drivers.errors.end()) return;
        wattron(w, COLOR_PAIR(TUI::PalError));
        mvwprintw(w, row++, 1, "bring-up: %s", it->second.c_str());
        wattroff(w, COLOR_PAIR(TUI::PalError));
    };

    switch (family) {
    case DeviceFamily::Input: {
        if (!drivers.din) break;
        const size_t m = moduleOf(*drivers.din, slave);
        if (m == SIZE_MAX) break;
        const auto& mod = drivers.din->module(m);
        kv(w, row, "channels", "%zu", mod.bitCount());
        bitRow(w, row++, "in", mod.bitCount(),
               [&](size_t k) { return mod.bit(k); });
        auto& tr = uiState.transitions[slave];
        if (tr.size() == mod.bitCount()) {
            mvwprintw(w, row, 1, "%-10s", "changes");
            for (size_t k = 0; k < mod.bitCount() && k < 32; ++k)
                mvwprintw(w, row, 11 + (int)k * 4, "%-4u", tr[k]);
            ++row;
        }
        break;
    }
    case DeviceFamily::Output: {
        if (!drivers.dout) break;
        const size_t m = moduleOf(*drivers.dout, slave);
        if (m == SIZE_MAX) break;
        const auto& mod = drivers.dout->module(m);
        const size_t n = mod.bitCount();
        kv(w, row, "channels", "%zu", n);
        bitRow(w, row++, "out", n,
               [&](size_t k) { return mod.bit(k); });
        size_t& sel = uiState.sel[slave];
        if (n > 0) sel = std::min(sel, n - 1);
        kv(w, row, "selected", "%zu", sel + 1);
        hint("1-9 toggle  ,/. select  space toggle  a/o invert node");
        break;
    }
    case DeviceFamily::Combined: {
        if (!drivers.combo) break;
        const size_t m = moduleOf(*drivers.combo, slave);
        if (m == SIZE_MAX) break;
        const auto& mod = drivers.combo->module(m);
        const size_t n = mod.outputChannelCount();
        kv(w, row, "channels", "in %zu  out %zu",
           mod.inputChannelCount(), n);
        bitRow(w, row++, "in", mod.inputChannelCount(),
               [&](size_t k) { return mod.input(k); });
        bitRow(w, row++, "out", n,
               [&](size_t k) { return mod.output(k); });
        size_t& sel = uiState.sel[slave];
        if (n > 0) sel = std::min(sel, n - 1);
        kv(w, row, "selected", "%zu", sel + 1);
        hint("1-9 toggle  ,/. select  space toggle  a/o invert node");
        break;
    }
    case DeviceFamily::AnalogIn: {
        if (!drivers.ain) break;
        const size_t m = moduleOf(*drivers.ain, slave);
        if (m == SIZE_MAX) break;
        const auto& mod = drivers.ain->module(m);
        for (size_t k = 0; k < mod.channelCount(); ++k) {
            if (mod.hasStatus(k))
                put(w, row, "ch %-3zu %12d   status 0x%04X",
                    k, mod.value(k), mod.status(k));
            else
                put(w, row, "ch %-3zu %12d", k, mod.value(k));
        }
        break;
    }
    case DeviceFamily::AnalogOut: {
        if (!drivers.aout) break;
        const size_t m = moduleOf(*drivers.aout, slave);
        if (m == SIZE_MAX) break;
        const auto& mod = drivers.aout->module(m);
        size_t& sel = uiState.sel[slave];
        if (sel >= mod.channelCount()) sel = 0;
        for (size_t k = 0; k < mod.channelCount(); ++k) {
            if (k == sel) wattron(w, A_REVERSE);
            put(w, row, "ch %-3zu %12d", k,
                mod.value(k));
            if (k == sel) wattroff(w, A_REVERSE);
        }
        hint("[/] channel  +/- value  0 zero all");
        break;
    }
    case DeviceFamily::Position: {
        if (!drivers.pos) break;
        const size_t m = moduleOf(*drivers.pos, slave);
        if (m == SIZE_MAX) break;
        const auto& mod = drivers.pos->module(m);
        for (size_t k = 0; k < mod.channelCount(); ++k) {
            if (mod.hasLatch(k))
                put(w, row, "ch %-3zu pos %-12lld latch %lld",
                    k, static_cast<long long>(mod.position(k)),
                    static_cast<long long>(mod.latch(k)));
            else
                put(w, row, "ch %-3zu pos %-12lld",
                    k, static_cast<long long>(mod.position(k)));
        }
        break;
    }
    case DeviceFamily::Stepper: {
        auto* t = findBySlave(drivers.steppers, slave);
        if (!t) break;
        kv(w, row, "status", "0x%04X   enc 0x%04X",
           t->stmStatusWord(), t->encStatusWord());
        kv(w, row, "flags", "rdyEn %d  rdy %d  warn %d  err %d",
           t->readyToEnable(), t->ready(), t->warning(), t->error());
        kv(w, row, "motion", "moving %+d   torqueReduced %d",
           t->movingPositive() ? 1 : t->movingNegative() ? -1 : 0,
           t->torqueReduced());
        kv(w, row, "counter", "%d   latch %d%s",
           t->counterValue(), t->latchValue(),
           t->latchValid() ? " (valid)" : "");
        kv(w, row, "din", "din1 %d  din2 %d  syncErr %d",
           t->digitalInput1(), t->digitalInput2(), t->syncError());
        hint("e enable  +/- velocity  0 stop  x reset");
        break;
    }
    case DeviceFamily::Drive: {
        auto* t = findBySlave(drivers.drives, slave);
        if (!t) break;
        size_t sel = 0;
        if (auto it = uiState.sel.find(slave); it != uiState.sel.end())
            sel = it->second;
        for (size_t a = 0; a < t->axisCount(); ++a) {
            put(w, row, "%s ax %zu    sw 0x%04X  %s",
                a == sel ? ">" : " ",
                a, t->statusword(a),
                driveStateName(t->driveState(a)));
            if (t->hasActualPosition(a) || t->hasActualVelocity(a))
                put(w, row, "          pos %-10d vel %-10d tq %d",
                    t->actualPosition(a), t->actualVelocity(a),
                    t->actualTorque(a));
        }
        hint("[/] axis  e enable step  d disable  +/- velocity  0 stop");
        break;
    }
    case DeviceFamily::Pwm: {
        auto* t = findBySlave(drivers.pwms, slave);
        if (!t) break;
        size_t sel = 0;
        if (auto it = uiState.sel.find(slave); it != uiState.sel.end())
            sel = it->second;
        for (size_t c = 0; c < t->channelCount(); ++c) {
            put(w, row, "%s ch %-3zu duty %-6u%s%s%s",
                c == sel ? ">" : " ",
                c, t->duty(c),
                t->hasStatus(c) && t->warning(c) ? "  WARN" : "",
                t->hasStatus(c) && t->error(c)   ? "  ERR"  : "",
                t->hasStatus(c) && t->digitalInput(c) ? "  DIN" : "");
        }
        hint("[/] channel  +/- duty  e enable  0 off");
        break;
    }
    case DeviceFamily::Pulse: {
        auto* t = findBySlave(drivers.pulses, slave);
        if (!t) break;
        size_t sel = 0;
        if (auto it = uiState.sel.find(slave); it != uiState.sel.end())
            sel = it->second;
        for (size_t c = 0; c < t->channelCount(); ++c) {
            put(w, row, "%s ch %-3zu freq %-6u%s%s%s",
                c == sel ? ">" : " ",
                c, t->frequency(c),
                t->rampActive(c) ? "  ramp" : "",
                t->error(c)      ? "  ERR"  : "",
                t->syncError(c)  ? "  SYNC" : "");
            if (t->hasEnc(c))
                put(w, row, "          enc %-10d latch %d",
                    t->counterValue(c), t->latchValue(c));
        }
        hint("[/] channel  +/- freq  f/r dir  0 stop");
        break;
    }
    case DeviceFamily::DcMotor: {
        auto* t = findBySlave(drivers.dcmotors, slave);
        if (!t) break;
        size_t sel = 0;
        if (auto it = uiState.sel.find(slave); it != uiState.sel.end())
            sel = it->second;
        for (size_t a = 0; a < t->axisCount(); ++a) {
            put(w, row, "%s ax %zu    sw 0x%04X  rdyEn %d rdy %d warn %d err %d",
                a == sel ? ">" : " ",
                a, t->motStatusWord(a), t->readyToEnable(a),
                t->ready(a), t->warning(a), t->error(a));
            if (t->hasEncoder(a))
                put(w, row, "          enc %-10d latch %d",
                    t->counterValue(a), t->latchValue(a));
        }
        hint("[/] axis  e enable  +/- velocity  0 stop  x reset");
        break;
    }
    case DeviceFamily::PowerMeter: {
        auto* t = findBySlave(drivers.meters, slave);
        if (!t) break;
        for (size_t p = 0; p < t->phaseCount(); ++p) {
            auto v = t->voltage(p), i = t->current(p),
                 pw = t->activePower(p);
            char vb[16], ib[16], pb[16];
            snprintf(vb, sizeof(vb), v  ? "%.2f" : "-", v  ? *v  : 0.0);
            snprintf(ib, sizeof(ib), i  ? "%.3f" : "-", i  ? *i  : 0.0);
            snprintf(pb, sizeof(pb), pw ? "%.1f" : "-", pw ? *pw : 0.0);
            put(w, row, "L%zu   %9s V  %9s A  %9s W", p + 1, vb, ib, pb);
        }
        if (t->indexSelectors())
            hint("+/- index selector 0");
        break;
    }
    case DeviceFamily::Oversampling: {
        auto* t = findBySlave(drivers.overs, slave);
        if (!t) break;
        for (size_t c = 0; c < t->channels(); ++c) {
            const size_t n = t->samplesPerCycle(c);
            put(w, row, "ch %-3zu %zu x %u-bit", c, n, t->sampleBits(c));
            std::string line;
            for (size_t i = 0; i < n && i < 8; ++i) {
                char b[24];
                snprintf(b, sizeof(b), " %d", t->sample(c, i));
                line += b;
            }
            if (n > 8) line += " …";
            put(w, row, "          %s", line.c_str());
        }
        break;
    }
    case DeviceFamily::Comm: {
        auto* t = findBySlave(drivers.comms, slave);
        if (!t) break;
        for (size_t c = 0; c < t->channels(); ++c) {
            put(w, row, "ch %-3zu ctrl 0x%04X  stat 0x%04X  fifo %zu/%zu",
                c, t->ctrlWord(c), t->statusWord(c),
                t->rxCapacity(c), t->txCapacity(c));
            auto data = t->dataIn(c);
            if (!data.empty()) {
                std::string hex, asc;
                for (size_t i = 0; i < data.size() && i < 24; ++i) {
                    char b[8];
                    snprintf(b, sizeof(b), "%02X ", data[i]);
                    hex += b;
                    asc += (data[i] >= 32 && data[i] < 127)
                         ? (char)data[i] : '.';
                }
                put(w, row, "  rx       %s", hex.c_str());
                put(w, row, "           %s", asc.c_str());
            }
        }
        hint("t transmit test bytes");
        break;
    }
    case DeviceFamily::Safety:
        ++row;
        put(w, row, "TwinSAFE needs FSoE master config");
        put(w, row, "run: ./beckhoff_safety_monitor -i %s "
                    "--safety-addr <n>", iface.c_str());
        break;
    default:
        break;
    }

    // Bring-up error is always worth showing, for any family.
    errLine();
}

// -- key handling -------------------------------------------------------------

// Per-slave adjustment state for velocity/index selectors.
static std::map<uint16_t, int> g_vel;
static std::map<uint16_t, int> g_idx;
static int& stepperVel(uint16_t s)   { return g_vel[s]; }
static int& axisVel(uint16_t s)      { return g_vel[0x8000 | s]; }
static int& meterIndexSel(uint16_t s){ return g_idx[s]; }

static bool handleKey(int key, uint16_t slave, DeviceFamily family,
                      Drivers& drivers, UiState& uiState) {
    using namespace Beckhoff;

    switch (family) {
    case DeviceFamily::Output: {
        if (!drivers.dout) return false;
        const size_t m = moduleOf(*drivers.dout, slave);
        if (m == SIZE_MAX) return false;
        const size_t base = drivers.dout->bitOffset(m);
        const size_t n    = drivers.dout->module(m).bitCount();
        if (n == 0) return false;
        size_t& sel = uiState.sel[slave];
        if (sel >= n) sel = 0;
        if (key == ',' || key == '[') {
            if (sel > 0) --sel; return true;
        }
        if (key == '.' || key == ']') {
            if (sel + 1 < n) ++sel; return true;
        }
        if (key == ' ' || key == 't' || key == 'T') {
            const size_t ch = base + sel;
            drivers.dout->setBit(ch, !drivers.dout->bit(ch));
            return true;
        }
        if (key >= '1' && key <= '9' && size_t(key - '1') < n) {
            const size_t ch = base + size_t(key - '1');
            drivers.dout->setBit(ch, !drivers.dout->bit(ch));
            return true;
        }
        // 'a'/'o' invert every channel of THIS terminal only — the old
        // allOn()/allOff() acted on the whole chain.
        if (key == 'a' || key == 'o') {
            for (size_t k = 0; k < n; ++k) {
                const size_t ch = base + k;
                drivers.dout->setBit(ch, !drivers.dout->bit(ch));
            }
            return true;
        }
        return false;
    }
    case DeviceFamily::Combined: {
        if (!drivers.combo) return false;
        const size_t m = moduleOf(*drivers.combo, slave);
        if (m == SIZE_MAX) return false;
        const size_t base = drivers.combo->outputOffset(m);
        const size_t n    = drivers.combo->module(m).outputChannelCount();
        if (n == 0) return false;
        size_t& sel = uiState.sel[slave];
        if (sel >= n) sel = 0;
        if (key == ',' || key == '[') {
            if (sel > 0) --sel; return true;
        }
        if (key == '.' || key == ']') {
            if (sel + 1 < n) ++sel; return true;
        }
        if (key == ' ' || key == 't' || key == 'T') {
            const size_t ch = base + sel;
            drivers.combo->setOutput(ch, !drivers.combo->output(ch));
            return true;
        }
        if (key >= '1' && key <= '9' && size_t(key - '1') < n) {
            const size_t ch = base + size_t(key - '1');
            drivers.combo->setOutput(ch, !drivers.combo->output(ch));
            return true;
        }
        if (key == 'a' || key == 'o') {
            for (size_t k = 0; k < n; ++k) {
                const size_t ch = base + k;
                drivers.combo->setOutput(ch, !drivers.combo->output(ch));
            }
            return true;
        }
        return false;
    }
    case DeviceFamily::AnalogOut: {
        if (!drivers.aout) return false;
        const size_t m = moduleOf(*drivers.aout, slave);
        if (m == SIZE_MAX) return false;
        const size_t base = drivers.aout->channelOffset(m);
        const size_t n    = drivers.aout->module(m).channelCount();
        size_t& sel = uiState.sel[slave];
        if (sel >= n) sel = 0;
        if (key == '[') { sel = (sel + n - 1) % n; return true; }
        if (key == ']') { sel = (sel + 1) % n;     return true; }
        if (key == '+' || key == '-') {
            const int32_t delta = key == '+' ? 256 : -256;
            drivers.aout->setValue(base + sel,
                             drivers.aout->value(base + sel) + delta);
            return true;
        }
        if (key == '0') {
            for (size_t k = 0; k < n; ++k) drivers.aout->setValue(base + k, 0);
            return true;
        }
        return false;
    }
    case DeviceFamily::Pwm: {
        auto* t = findBySlave(drivers.pwms, slave);
        if (!t || !t->channelCount()) return false;
        size_t& sel = uiState.sel[slave];
        if (sel >= t->channelCount()) sel = 0;
        if (key == '[') {
            sel = (sel + t->channelCount() - 1) % t->channelCount();
            return true;
        }
        if (key == ']') { sel = (sel + 1) % t->channelCount(); return true; }
        if (key == '+' || key == '-') {
            const int d = key == '+' ? 1024 : -1024;
            const int v = static_cast<int>(t->duty(sel)) + d;
            t->setDuty(sel, static_cast<uint16_t>(std::clamp(v, 0, 0xFFFF)));
            return true;
        }
        if (key == 'e') { t->setEnable(sel, true);  return true; }
        if (key == '0') { t->allOff(); return true; }
        return false;
    }
    case DeviceFamily::Pulse: {
        auto* t = findBySlave(drivers.pulses, slave);
        if (!t || !t->channelCount()) return false;
        size_t& sel = uiState.sel[slave];
        if (sel >= t->channelCount()) sel = 0;
        if (key == '[') {
            sel = (sel + t->channelCount() - 1) % t->channelCount();
            return true;
        }
        if (key == ']') { sel = (sel + 1) % t->channelCount(); return true; }
        if (key == '+' || key == '-') {
            const int d = key == '+' ? 100 : -100;
            const int v = static_cast<int>(t->frequency(sel)) + d;
            t->setFrequency(sel,
                            static_cast<uint16_t>(std::clamp(v, 0, 0xFFFF)));
            return true;
        }
        if (key == 'f') { t->setForward(sel, true);  t->setReverse(sel, false); return true; }
        if (key == 'r') { t->setReverse(sel, true);  t->setForward(sel, false); return true; }
        if (key == '0') { t->setFrequency(sel, 0); return true; }
        return false;
    }
    case DeviceFamily::Stepper: {
        auto* t = findBySlave(drivers.steppers, slave);
        if (!t) return false;
        int& vel = stepperVel(slave);
        if (key == 'e') { t->setEnable(true);  return true; }
        if (key == 'x') { t->setReset(true);   return true; }
        if (key == '+' || key == '-') {
            vel += (key == '+') ? 200 : -200;
            vel = std::clamp(vel, -32768, 32767);
            t->setVelocity(static_cast<int16_t>(vel));
            return true;
        }
        if (key == '0') { vel = 0; t->setVelocity(0); return true; }
        return false;
    }
    case DeviceFamily::DcMotor: {
        auto* t = findBySlave(drivers.dcmotors, slave);
        if (!t || !t->axisCount()) return false;
        size_t& sel = uiState.sel[slave];
        if (sel >= t->axisCount()) sel = 0;
        int& vel = axisVel(slave);
        if (key == '[') {
            sel = (sel + t->axisCount() - 1) % t->axisCount();
            return true;
        }
        if (key == ']') { sel = (sel + 1) % t->axisCount(); return true; }
        if (key == 'e') { t->setEnable(sel, true); return true; }
        if (key == 'x') { t->setReset(sel, true);  return true; }
        if (key == '+' || key == '-') {
            vel += (key == '+') ? 500 : -500;
            vel = std::clamp(vel, -32768, 32767);
            t->setVelocity(sel, static_cast<int16_t>(vel));
            return true;
        }
        if (key == '0') { vel = 0; t->setVelocity(sel, 0); return true; }
        return false;
    }
    case DeviceFamily::Drive: {
        auto* t = findBySlave(drivers.drives, slave);
        if (!t || !t->axisCount()) return false;
        size_t& sel = uiState.sel[slave];
        if (sel >= t->axisCount()) sel = 0;
        int& vel = axisVel(slave);
        if (key == '[') {
            sel = (sel + t->axisCount() - 1) % t->axisCount();
            return true;
        }
        if (key == ']') { sel = (sel + 1) % t->axisCount(); return true; }
        if (key == 'e') {
            // Step the CiA402 FSA toward Operation Enabled.
            using DS = Beckhoff::DriveState;
            switch (t->driveState(sel)) {
            case DS::SwitchOnDisabled:    t->requestShutdown(sel);         break;
            case DS::ReadyToSwitchOn:     t->requestSwitchOn(sel);         break;
            case DS::SwitchedOn:          t->requestEnableOperation(sel);  break;
            case DS::Fault:               t->requestFaultReset(sel);       break;
            default:                      t->requestShutdown(sel);         break;
            }
            return true;
        }
        if (key == 'd') { t->requestDisableVoltage(sel); return true; }
        if (key == '+' || key == '-') {
            vel += (key == '+') ? 100 : -100;
            if (t->hasModesOfOperation(sel))
                t->setModesOfOperation(sel, 2);   // CSV
            if (t->hasTargetVelocity(sel))
                t->setTargetVelocity(sel, vel);
            return true;
        }
        if (key == '0') {
            vel = 0;
            if (t->hasTargetVelocity(sel)) t->setTargetVelocity(sel, 0);
            return true;
        }
        return false;
    }
    case DeviceFamily::Comm: {
        auto* t = findBySlave(drivers.comms, slave);
        if (!t || !t->channels()) return false;
        if (key == 't') {
            const char* msg = "tether\r\n";
            t->writeData(0, std::span<const uint8_t>(
                reinterpret_cast<const uint8_t*>(msg), strlen(msg)));
            return true;
        }
        return false;
    }
    case DeviceFamily::PowerMeter: {
        auto* t = findBySlave(drivers.meters, slave);
        if (!t || !t->indexSelectors()) return false;
        if (key == '+' || key == '-') {
            int& idx = meterIndexSel(slave);
            idx = std::clamp(idx + (key == '+' ? 1 : -1), 0, 255);
            t->setIndexSelector(0, static_cast<uint8_t>(idx));
            return true;
        }
        return false;
    }
    default:
        return false;
    }
}



static void runTui(std::span<const DiscoveredSlave> slaves,
                   const std::map<uint16_t, SlaveClass>& assignments,
                   Drivers& drivers, double duration_sec,
                   const std::string& iface) {
    using namespace Tether::Examples;

    UiState uiState;

    // Seed input transition counters.
    if (drivers.din) {
        for (size_t m = 0; m < drivers.din->moduleCount(); ++m) {
            const uint16_t s = drivers.din->slaveIndex(m);
            uiState.transitions[s].assign(drivers.din->module(m).bitCount(), 0);
            uiState.prevBits[s] = drivers.din->module(m).bits();
        }
    }

    auto managed = [&](uint16_t idx) {
        auto it = assignments.find(idx);
        return it != assignments.end() && it->second.family != DeviceFamily::None;
    };

    TUI::TreeScreen* screenPtr = nullptr;

    // Compact live bit indicator drawn right-aligned on the tree row:
    // [■□■□] for pure in/out terminals, i[…]o[…] for combined I/O.
    auto bitBadge = [](size_t n, const std::function<bool(size_t)>& get) {
        std::string b = "[";
        for (size_t k = 0; k < n && k < 16; ++k)
            b += get(k) ? "\xE2\x96\xA0" : "\xE2\x96\xA1";   // ■ □
        if (n > 16) b += "\xE2\x80\xA6";                    // …
        b += "]";
        return b;
    };

    TUI::TreeScreenHooks hooks;
    hooks.keyHints = "see right pane for device keys";

    hooks.onTick = [&]() {
        if (drivers.din) {
            for (size_t m = 0; m < drivers.din->moduleCount(); ++m) {
                const uint16_t s  = drivers.din->slaveIndex(m);
                const auto&    mod = drivers.din->module(m);
                const uint64_t now = mod.bits();
                uint64_t&      prv = uiState.prevBits[s];
                auto&          tr  = uiState.transitions[s];
                const uint64_t chg = now ^ prv;
                for (size_t k = 0; k < mod.bitCount() && k < 64; ++k)
                    if ((chg >> k) & 1u) ++tr[k];
                prv = now;
                if (screenPtr)
                    screenPtr->tree().setBadge(s, bitBadge(mod.bitCount(),
                        [&](size_t k) { return mod.bit(k); }));
            }
        }
        if (!screenPtr) return;
        if (drivers.dout) {
            for (size_t m = 0; m < drivers.dout->moduleCount(); ++m) {
                const auto& mod = drivers.dout->module(m);
                screenPtr->tree().setBadge(drivers.dout->slaveIndex(m),
                    bitBadge(mod.bitCount(),
                             [&](size_t k) { return mod.bit(k); }));
            }
        }
        if (drivers.combo) {
            for (size_t m = 0; m < drivers.combo->moduleCount(); ++m) {
                const auto& mod = drivers.combo->module(m);
                screenPtr->tree().setBadge(drivers.combo->slaveIndex(m),
                    "i" + bitBadge(mod.inputChannelCount(),
                                   [&](size_t k) { return mod.input(k); })
                        + "o" + bitBadge(mod.outputChannelCount(),
                                   [&](size_t k) { return mod.output(k); }));
            }
        }
    };

    hooks.renderDetail = [&](TUI::TermWindow* w, const TUI::TreeNode& node) {
        WINDOW* win = static_cast<WINDOW*>(w);
        int row = 1;

        const auto* s = slaveByIndex(slaves, node.tag);
        if (!s) {
            if (!node.children.empty())
                mvwprintw(win, row++, 1, "%zu terminal(s) below",
                          node.children.size());
            else
                mvwprintw(win, row++, 1, "(bus root)");
            return;
        }

        wattron(win, A_BOLD);
        mvwprintw(win, row++, 1, "%s",
                  s->device_name ? s->device_name->c_str() : "?");
        wattroff(win, A_BOLD);
        rule(win, row);
        kv(win, row, "slave",   "%u", s->index);
        kv(win, row, "vendor",  "0x%08X",
           s->vendor_id ? *s->vendor_id : 0);
        kv(win, row, "product", "0x%08X",
           s->product_code ? *s->product_code : 0);

        auto it = assignments.find(s->index);
        const SlaveClass empty{};
        const SlaveClass& assigned = it != assignments.end() ? it->second : empty;

        if (assigned.family != DeviceFamily::None && assigned.entry) {
            kv(win, row, "family", "%s", assigned.entry->label);
            rule(win, row);
            renderLive(win, row, s->index, assigned.family, drivers, uiState, iface);
            rule(win, row);
            wattron(win, A_DIM);
            kv(win, row, "example", "./%s -i %s",
               assigned.entry->example, iface.c_str());
            wattroff(win, A_DIM);
        } else {
            rule(win, row);
            wattron(win, A_DIM);
            if (isCouplerDevice(*s))
                mvwprintw(win, row++, 1, "(coupler  —  %zu terminal(s))",
                          node.children.size());
            else
                mvwprintw(win, row++, 1,
                          "(product not in any driver registry)");
            wattroff(win, A_DIM);
        }
    };

    hooks.onKey = [&](int key) {
        const TUI::TreeNode* sel = screenPtr ? screenPtr->selected()
                                             : nullptr;
        if (!sel || sel->tag < 0) return false;
        auto it = assignments.find(static_cast<uint16_t>(sel->tag));
        if (it == assignments.end()) return false;
        return handleKey(key, it->first, it->second.family, drivers, uiState);
    };

    // The tree is built from the ALREADY-discovered slave list  -  no
    // second bus scan here (a full DiscoveryOption::All re-read of every
    // slave's SII EEPROM takes seconds and previously looked like a hang).
    TUI::TreeScreen screen(
        std::string("Beckhoff Device Explorer  —  ") + iface,
        buildDeviceTree(slaves, managed), std::move(hooks));
    screenPtr = &screen;
    screen.run(g_cancel, duration_sec);
}
#endif // TETHER_HAS_TERMINAL_UI

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
    argparse::ArgumentParser program("beckhoff_tui", "1.0",
                                     argparse::default_arguments::help);
    Tether::Examples::addInterfaceArg(program);
    Tether::Examples::addListInterfacesArg(program);
    Tether::Examples::addDebugArg(program);
    Tether::Examples::addDurationArg(program, 0.0);
    program.add_argument("--interactive")
        .help("Force the interactive ncurses TUI")
        .flag();

    try { program.parse_args(argc, argv); }
    catch (const std::runtime_error& err) {
        std::cerr << err.what() << "\n" << program;
        return 1;
    }

    if (program.get<bool>("--list-interfaces")) {
        Tether::Examples::listPhysicalInterfaces(TAG);
        return 0;
    }

    std::string iface = Tether::Examples::resolveInterface(
        program.get<std::string>("--interface"), TAG);
    if (iface.empty()) return 1;

    std::string debug_str = program.get<std::string>("--debug");
    if (Tether::Examples::printDebugHelpIfRequested(debug_str)) return 0;
    auto debug_flags = Tether::Examples::parseDebugFlags(debug_str);

    Tether::Examples::EncapsulationConfig encapsulation;
    if (!Tether::Examples::parseEncapsulationArg(program.get<std::string>("--encapsulation"), encapsulation, TAG)) {
        return 1;
    }

    const double duration_sec = program.get<double>("--time");

    bool interactive = true;
#ifdef TETHER_HAS_TERMINAL_UI
    if (interactive && !Tether::TUI::Session::available()) {
        if (program.get<bool>("--interactive")) {
            TETHER_LOGW(TAG, "no usable terminal  —  TUI unavailable");
        }
        interactive = false;
    }
#else
    if (program.get<bool>("--interactive")) {
        TETHER_LOGW(TAG, "built without ncurses  —  TUI unavailable");
    }
    interactive = false;
#endif

    if (!interactive) {
        TETHER_LOGE(TAG, "beckhoff_tui requires a terminal with ncurses");
        return 2;
    }

    Tether::Platform::ensureRealtimeKernelOrExit();
    Tether::Utils::SignalHandler sig_handler(g_cancel);

    Tether::Examples::HostEtherNetSession session;
    if (!Tether::Examples::initHostEthernet(session, iface, TAG)) return 3;

    EtherCAT::Master master;
    sig_handler.setCancelCallback([&master]() { master.requestCancel(); });
    Tether::Examples::applyDebugFlags(debug_flags, master, TAG);

    if (!Tether::Examples::setupEncapsulation(session, master, encapsulation, TAG)) {
        Tether::Examples::shutdownHostEthernet(session);
        return 5;
    }
    Tether::Examples::startHostPollThread(session, TAG);
    if (!Tether::Examples::startHostMaster(session, master, TAG)) {
        Tether::Examples::shutdownHostEthernet(session);
        return 5;
    }

    auto slaves = master.discovery().discover(EtherCAT::DiscoveryOption::All);
    if (slaves.empty()) {
        TETHER_LOGE(TAG, "No slaves discovered");
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 4;
    }

    TETHER_LOGI(TAG, "=== Discovered {} slave(s) ===", slaves.size());
    for (const auto& s : slaves) {
        TETHER_LOGI(TAG, "Slave {}: {} (vendor=0x{:08X} product=0x{:08X})",
                    s.index,
                    s.device_name ? s.device_name->c_str() : "?",
                    s.vendor_id ? *s.vendor_id : 0,
                    s.product_code ? *s.product_code : 0);
    }

    // Classify every slave into exactly one family.
    std::map<uint16_t, SlaveClass> assignments;
    for (const auto& s : slaves) {
        auto assigned = classify(s);
        if (assigned.family != DeviceFamily::None) {
            assignments[s.index] = assigned;
            TETHER_LOGI(TAG, "  s{} -> {} ({})", s.index,
                        assigned.entry->label, assigned.identity->name);
        }
    }

    // Bring all recognized terminals to OP (chains in their own PDO
    // groups, singles on the default group, one shared RT loop).
    Drivers drivers;
    bringUp(master, slaves, assignments, drivers);

#ifdef TETHER_HAS_TERMINAL_UI
    runTui(slaves, assignments, drivers, duration_sec, iface);
#else
    (void)duration_sec;
#endif

    if (master.isMotionControlLoopRunning())
        master.stopMotionControlLoop();
    master.stop();
    Tether::Examples::shutdownHostEthernet(session);
    TETHER_LOGI(TAG, "Done.");
    return 0;
}
