/**
 * @file beckhoff_tui.cpp
 * @brief Beckhoff Device Explorer — unified interactive terminal UI
 *
 * Scans the EtherCAT chain once, classifies every Beckhoff terminal by
 * product code against all implemented driver families, brings every
 * recognized terminal to OP (each family in its own PDO group / shared
 * logical address space), and presents a navigable coupler/terminal
 * device tree with a live detail pane.
 *
 * Interaction per selected device:
 *   digital out / combined I/O : '1'..'9' toggle channel, 'a' all on,
 *                                'o' all off
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
 *   TwinSAFE                   : classified only — needs FSoE config
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
// Device classification — each slave is assigned to exactly one family
// ============================================================================

enum class Fam : uint8_t {
    None, Input, Output, Combined, AnalogIn, AnalogOut, Position,
    Stepper, Drive, Pwm, Pulse, DcMotor, PowerMeter, Oversampling,
    Comm, Safety,
};

struct FamDef {
    Fam                                   fam;
    std::span<const Beckhoff::DeviceIdentity> registry;
    const char*                           label;
    const char*                           example;
};

// Precedence order — the first registry containing the product code wins.
// More specific/capable families precede generic digital I/O so devices
// registered in both (e.g. EL2034 outputs + diagnostics, timestamped
// EL125x inputs) land in CombinedIo.
static const FamDef kFamilies[] = {
    {Fam::Safety,       Beckhoff::Devices::kSafetyTerminals,
     "TwinSAFE FSoE",      "beckhoff_safety_monitor"},
    {Fam::Comm,         Beckhoff::Devices::kCommTerminals,
     "serial/IO-Link",     "beckhoff_serial_bridge"},
    {Fam::Oversampling, Beckhoff::Devices::kOversamplingTerminals,
     "oversampling",       "beckhoff_oversampling_monitor"},
    {Fam::PowerMeter,   Beckhoff::Devices::kPowerMeterTerminals,
     "power measurement",  "beckhoff_power_meter"},
    {Fam::DcMotor,      Beckhoff::Devices::kDcMotorTerminals,
     "DC motor",           "beckhoff_dcmotor_jog"},
    {Fam::Drive,        Beckhoff::Devices::kCompactDriveTerminals,
     "CiA402 drive",       "beckhoff_drive_move"},
    {Fam::Stepper,      Beckhoff::Devices::kStepperTerminals,
     "POS stepper",        "beckhoff_stepper_jog"},
    {Fam::Pulse,        Beckhoff::Devices::kPulseTrainTerminals,
     "pulse-train",        "beckhoff_pulse_jog"},
    {Fam::Pwm,          Beckhoff::Devices::kPwmTerminals,
     "PWM",                "beckhoff_pwm_sweep"},
    {Fam::Position,     Beckhoff::Devices::kPositionInputTerminals,
     "position/encoder",   "beckhoff_position_monitor"},
    {Fam::AnalogOut,    Beckhoff::Devices::kAnalogOutputTerminals,
     "analog output",      "beckhoff_analog_output_sweep"},
    {Fam::AnalogIn,     Beckhoff::Devices::kAnalogInputTerminals,
     "analog input",       "beckhoff_analog_input_monitor"},
    {Fam::Combined,     Beckhoff::Devices::kCombinedIoTerminals,
     "combined I/O",       "beckhoff_combined_io_toggle"},
    {Fam::Output,       Beckhoff::Devices::kOutputTerminals,
     "digital output",     "beckhoff_output_toggle"},
    {Fam::Input,        Beckhoff::Devices::kInputTerminals,
     "digital input",      "beckhoff_input_monitor"},
};

struct SlaveClass {
    Fam                              fam = Fam::None;
    const Beckhoff::DeviceIdentity*  id  = nullptr;
    const FamDef*                    def = nullptr;
};

static SlaveClass classify(const DiscoveredSlave& s) {
    const uint32_t vendor  = s.vendor_id ? *s.vendor_id : 0;
    const uint32_t product = s.product_code ? *s.product_code : 0;
    if (!product) return {};
    for (const auto& f : kFamilies) {
        for (const auto& id : f.registry) {
            if (id.product_code == product && id.vendor_id == vendor)
                return {f.fam, &id, &f};
        }
    }
    return {};
}

// ============================================================================
// Drivers — every recognized slave brought to OP in one process
// ============================================================================

struct Drivers {
    // Chained families — one shared logical address space each.
    std::optional<Beckhoff::MultiInputTerminal<>>         din;
    std::optional<Beckhoff::MultiOutputTerminal<>>        dout;
    std::optional<Beckhoff::MultiCombinedIoTerminal<>>    combo;
    std::optional<Beckhoff::MultiAnalogInputTerminal<>>   ain;
    std::optional<Beckhoff::MultiAnalogOutputTerminal<>>  aout;
    std::optional<Beckhoff::MultiPositionInputTerminal<>> pos;

    // Single-terminal families — one driver per slave.
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
                    const std::map<uint16_t, SlaveClass>& cls,
                    Drivers& d) {
    using Beckhoff::StartOptions;
    namespace Dev = Beckhoff::Devices;

    auto of = [&](Fam f) {
        std::vector<DiscoveredSlave> out;
        for (const auto& s : slaves) {
            auto it = cls.find(s.index);
            if (it != cls.end() && it->second.fam == f) out.push_back(s);
        }
        return out;
    };
    auto idOf = [&](uint16_t idx) -> const Beckhoff::DeviceIdentity* {
        auto it = cls.find(idx);
        return it != cls.end() ? it->second.id : nullptr;
    };
    auto fail = [&](uint16_t idx, const char* what, Beckhoff::Error e) {
        d.errors[idx] = std::string(what) + ": " + Beckhoff::errorToString(e);
        TETHER_LOGW(TAG, "slave {}: {}: {}", idx, what,
                    Beckhoff::errorToString(e));
    };

    // ---- Phase A: configure -------------------------------------------------
    auto chainCfg = [&](auto& opt, Fam f,
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
        if (opt->pdoManager()) d.groups.push_back(opt->pdoManager());
        TETHER_LOGI(TAG, "{}: {} device(s) configured", name,
                    opt->moduleCount());
    };

    chainCfg(d.din,   Fam::Input,     Dev::kInputTerminals,         "inputs");
    chainCfg(d.dout,  Fam::Output,    Dev::kOutputTerminals,        "outputs");
    chainCfg(d.combo, Fam::Combined,  Dev::kCombinedIoTerminals,    "combo-io");
    chainCfg(d.ain,   Fam::AnalogIn,  Dev::kAnalogInputTerminals,   "analog-in");
    chainCfg(d.aout,  Fam::AnalogOut, Dev::kAnalogOutputTerminals,  "analog-out");
    chainCfg(d.pos,   Fam::Position,  Dev::kPositionInputTerminals, "position");

    auto singleCfg = [&](auto& vec, Fam f, const char* name) {
        for (const auto& s : of(f)) {
            vec.push_back(std::make_unique<typename
                std::remove_reference_t<decltype(vec)>::value_type::element_type>(
                    master, s, *idOf(s.index)));
            if (auto r = vec.back()->configure(); !r)
                fail(s.index, name, r.error());
        }
    };

    singleCfg(d.steppers,  Fam::Stepper,      "stepper");
    singleCfg(d.drives,    Fam::Drive,        "drive");
    singleCfg(d.pwms,      Fam::Pwm,          "pwm");
    singleCfg(d.pulses,    Fam::Pulse,        "pulse-train");
    singleCfg(d.dcmotors,  Fam::DcMotor,      "dc-motor");
    singleCfg(d.meters,    Fam::PowerMeter,   "power-meter");
    singleCfg(d.overs,     Fam::Oversampling, "oversampling");
    singleCfg(d.comms,     Fam::Comm,         "comm");

    const bool any = d.din || d.dout || d.combo || d.ain || d.aout || d.pos
                  || !d.steppers.empty() || !d.drives.empty() || !d.pwms.empty()
                  || !d.pulses.empty() || !d.dcmotors.empty()
                  || !d.meters.empty() || !d.overs.empty() || !d.comms.empty();
    if (!any) {
        TETHER_LOGW(TAG, "no recognized terminals — tree view only");
        return;
    }

    // ---- Phase B: one RT loop exchanging every group ------------------------
    auto* groups = &d.groups;
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
            || std::find(d.groups.begin(), d.groups.end(),
                         opt->pdoManager()) == d.groups.end()) return;
        if (auto r = opt->start(noLoop); !r)
            fail(opt->slaveIndex(opt->lastErrorModule() == SIZE_MAX
                     ? 0 : opt->lastErrorModule()),
                 name, r.error());
    };
    chainOp(d.din, "inputs");   chainOp(d.dout, "outputs");
    chainOp(d.combo, "combo");  chainOp(d.ain, "analog-in");
    chainOp(d.aout, "analog-out"); chainOp(d.pos, "position");

    auto singleOp = [&](auto& vec, const char* name) {
        for (auto& t : vec) {
            if (auto r = t->start(noLoop); !r)
                fail(t->slaveIndex(), name, r.error());
        }
    };
    singleOp(d.steppers, "stepper");   singleOp(d.drives, "drive");
    singleOp(d.pwms, "pwm");           singleOp(d.pulses, "pulse-train");
    singleOp(d.dcmotors, "dc-motor");  singleOp(d.meters, "power-meter");
    singleOp(d.overs, "oversampling"); singleOp(d.comms, "comm");
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

void bitRow(WINDOW* w, int row, const char* label,
            size_t n, const std::function<bool(size_t)>& get) {
    mvwprintw(w, row, 1, "%s", label);
    for (size_t k = 0; k < n && k < 40; ++k) {
        const bool on = get(k);
        wattron(w, on ? (COLOR_PAIR(TUI::PalValue) | A_BOLD) : A_DIM);
        mvwprintw(w, row, 10 + static_cast<int>(k) * 3, "%s",
                  on ? "*" : ".");
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

static void renderLive(WINDOW* w, int& row, uint16_t slave, Fam fam,
                       Drivers& d, UiState& ui, const std::string& iface) {
    using namespace Beckhoff;

    auto hint = [&](const char* s) {
        ++row;
        wattron(w, COLOR_PAIR(TUI::PalHint));
        mvwprintw(w, row++, 1, "%s", s);
        wattroff(w, COLOR_PAIR(TUI::PalHint));
    };
    auto errLine = [&]() {
        auto it = d.errors.find(slave);
        if (it == d.errors.end()) return;
        wattron(w, COLOR_PAIR(TUI::PalError));
        mvwprintw(w, row++, 1, "bring-up: %s", it->second.c_str());
        wattroff(w, COLOR_PAIR(TUI::PalError));
    };

    switch (fam) {
    case Fam::Input: {
        if (!d.din) break;
        const size_t m = moduleOf(*d.din, slave);
        if (m == SIZE_MAX) break;
        const auto& mod = d.din->module(m);
        put(w, row, "channels: %zu", mod.bitCount());
        bitRow(w, row++, "in:", mod.bitCount(),
               [&](size_t k) { return mod.bit(k); });
        auto& tr = ui.transitions[slave];
        if (tr.size() == mod.bitCount()) {
            mvwprintw(w, row, 1, "chg:");
            for (size_t k = 0; k < mod.bitCount() && k < 40; ++k)
                mvwprintw(w, row, 10 + (int)k * 3, "%-2u", tr[k]);
            ++row;
        }
        break;
    }
    case Fam::Output: {
        if (!d.dout) break;
        const size_t m = moduleOf(*d.dout, slave);
        if (m == SIZE_MAX) break;
        const auto& mod = d.dout->module(m);
        put(w, row, "channels: %zu", mod.bitCount());
        bitRow(w, row++, "out:", mod.bitCount(),
               [&](size_t k) { return mod.bit(k); });
        hint("1-9: toggle ch  a: all on  o: all off");
        break;
    }
    case Fam::Combined: {
        if (!d.combo) break;
        const size_t m = moduleOf(*d.combo, slave);
        if (m == SIZE_MAX) break;
        const auto& mod = d.combo->module(m);
        put(w, row, "in: %zu  out: %zu",
            mod.inputChannelCount(), mod.outputChannelCount());
        bitRow(w, row++, "in:", mod.inputChannelCount(),
               [&](size_t k) { return mod.input(k); });
        bitRow(w, row++, "out:", mod.outputChannelCount(),
               [&](size_t k) { return mod.output(k); });
        hint("1-9: toggle out  o: all off");
        break;
    }
    case Fam::AnalogIn: {
        if (!d.ain) break;
        const size_t m = moduleOf(*d.ain, slave);
        if (m == SIZE_MAX) break;
        const auto& mod = d.ain->module(m);
        for (size_t k = 0; k < mod.channelCount(); ++k) {
            if (mod.hasStatus(k))
                put(w, row, "ch%zu: %11d   (status 0x%04X)",
                    k, mod.value(k), mod.status(k));
            else
                put(w, row, "ch%zu: %11d", k, mod.value(k));
        }
        break;
    }
    case Fam::AnalogOut: {
        if (!d.aout) break;
        const size_t m = moduleOf(*d.aout, slave);
        if (m == SIZE_MAX) break;
        const auto& mod = d.aout->module(m);
        size_t& sel = ui.sel[slave];
        if (sel >= mod.channelCount()) sel = 0;
        for (size_t k = 0; k < mod.channelCount(); ++k) {
            if (k == sel) wattron(w, A_REVERSE);
            put(w, row, "ch%zu: %11d", k,
                mod.value(k));
            if (k == sel) wattroff(w, A_REVERSE);
        }
        hint("[/]: channel  +/-: value  0: zero all");
        break;
    }
    case Fam::Position: {
        if (!d.pos) break;
        const size_t m = moduleOf(*d.pos, slave);
        if (m == SIZE_MAX) break;
        const auto& mod = d.pos->module(m);
        for (size_t k = 0; k < mod.channelCount(); ++k) {
            put(w, row, "ch%zu: pos %lld",
                k, static_cast<long long>(mod.position(k)));
            if (mod.hasLatch(k))
                put(w, row, "     latch %lld",
                    static_cast<long long>(mod.latch(k)));
        }
        break;
    }
    case Fam::Stepper: {
        auto* t = findBySlave(d.steppers, slave);
        if (!t) break;
        put(w, row, "status 0x%04X  enc 0x%04X",
            t->stmStatusWord(), t->encStatusWord());
        put(w, row, "readyToEnable %d  ready %d  warn %d  err %d",
            t->readyToEnable(), t->ready(), t->warning(), t->error());
        put(w, row, "moving %+d   torqueReduced %d",
            t->movingPositive() ? 1 : t->movingNegative() ? -1 : 0,
            t->torqueReduced());
        put(w, row, "counter %d   latch %d%s",
            t->counterValue(), t->latchValue(),
            t->latchValid() ? " (valid)" : "");
        put(w, row, "din1 %d  din2 %d  syncErr %d",
            t->digitalInput1(), t->digitalInput2(), t->syncError());
        hint("e: enable  +/-: velocity  0: stop  x: reset");
        break;
    }
    case Fam::Drive: {
        auto* t = findBySlave(d.drives, slave);
        if (!t) break;
        size_t sel = 0;
        if (auto it = ui.sel.find(slave); it != ui.sel.end())
            sel = it->second;
        for (size_t a = 0; a < t->axisCount(); ++a) {
            put(w, row, "%s axis %zu  sw 0x%04X  %s",
                a == sel ? ">" : " ",
                a, t->statusword(a),
                driveStateName(t->driveState(a)));
            if (t->hasActualPosition(a) || t->hasActualVelocity(a))
                put(w, row, "  pos %d  vel %d  tq %d",
                    t->actualPosition(a), t->actualVelocity(a),
                    t->actualTorque(a));
        }
        hint("e: enable step  d: disable  +/-: velocity  0: stop");
        break;
    }
    case Fam::Pwm: {
        auto* t = findBySlave(d.pwms, slave);
        if (!t) break;
        size_t sel = 0;
        if (auto it = ui.sel.find(slave); it != ui.sel.end())
            sel = it->second;
        for (size_t c = 0; c < t->channelCount(); ++c) {
            put(w, row, "%s ch%zu: duty %-5u%s%s%s",
                c == sel ? ">" : " ",
                c, t->duty(c),
                t->hasStatus(c) && t->warning(c) ? "  WARN" : "",
                t->hasStatus(c) && t->error(c)   ? "  ERR"  : "",
                t->hasStatus(c) && t->digitalInput(c) ? "  DIN" : "");
        }
        hint("[/]: channel  +/-: duty  e: enable  0: off");
        break;
    }
    case Fam::Pulse: {
        auto* t = findBySlave(d.pulses, slave);
        if (!t) break;
        size_t sel = 0;
        if (auto it = ui.sel.find(slave); it != ui.sel.end())
            sel = it->second;
        for (size_t c = 0; c < t->channelCount(); ++c) {
            put(w, row, "%s ch%zu: freq %-6u%s%s%s",
                c == sel ? ">" : " ",
                c, t->frequency(c),
                t->rampActive(c) ? "  ramp" : "",
                t->error(c)      ? "  ERR"  : "",
                t->syncError(c)  ? "  SYNC" : "");
            if (t->hasEnc(c))
                put(w, row, "      enc %d  latch %d",
                    t->counterValue(c), t->latchValue(c));
        }
        hint("[/]: channel  +/-: freq  f/r: dir  0: stop");
        break;
    }
    case Fam::DcMotor: {
        auto* t = findBySlave(d.dcmotors, slave);
        if (!t) break;
        size_t sel = 0;
        if (auto it = ui.sel.find(slave); it != ui.sel.end())
            sel = it->second;
        for (size_t a = 0; a < t->axisCount(); ++a) {
            put(w, row, "%s ax%zu: sw 0x%04X rdyE %d rdy %d warn %d err %d",
                a == sel ? ">" : " ",
                a, t->motStatusWord(a), t->readyToEnable(a),
                t->ready(a), t->warning(a), t->error(a));
            if (t->hasEncoder(a))
                put(w, row, "      enc %d  latch %d",
                    t->counterValue(a), t->latchValue(a));
        }
        hint("[/]: axis  e: enable  +/-: velocity  0: stop  x: reset");
        break;
    }
    case Fam::PowerMeter: {
        auto* t = findBySlave(d.meters, slave);
        if (!t) break;
        for (size_t p = 0; p < t->phaseCount(); ++p) {
            auto v = t->voltage(p), i = t->current(p),
                 pw = t->activePower(p);
            put(w, row, "L%zu: %s%s%s%s%s%s",
                p + 1,
                v  ? std::to_string(*v).c_str()  : "-", v  ? " V "  : " ",
                i  ? std::to_string(*i).c_str()  : "-", i  ? " A "  : " ",
                pw ? std::to_string(*pw).c_str() : "-", pw ? " W"   : "");
        }
        if (t->indexSelectors())
            hint("+/-: index selector 0");
        break;
    }
    case Fam::Oversampling: {
        auto* t = findBySlave(d.overs, slave);
        if (!t) break;
        for (size_t c = 0; c < t->channels(); ++c) {
            const size_t n = t->samplesPerCycle(c);
            put(w, row, "ch%zu: %zu x %u-bit", c, n, t->sampleBits(c));
            std::string line;
            for (size_t i = 0; i < n && i < 8; ++i) {
                char b[24];
                snprintf(b, sizeof(b), " %d", t->sample(c, i));
                line += b;
            }
            if (n > 8) line += " ...";
            put(w, row, "     %s", line.c_str());
        }
        break;
    }
    case Fam::Comm: {
        auto* t = findBySlave(d.comms, slave);
        if (!t) break;
        for (size_t c = 0; c < t->channels(); ++c) {
            put(w, row, "ch%zu: ctrl 0x%04X  stat 0x%04X  fifo %zu/%zu",
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
                put(w, row, "  rx: %s", hex.c_str());
                put(w, row, "      %s", asc.c_str());
            }
        }
        hint("t: transmit test bytes");
        break;
    }
    case Fam::Safety:
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

static bool handleKey(int key, uint16_t slave, Fam fam,
                      Drivers& d, UiState& ui) {
    using namespace Beckhoff;

    switch (fam) {
    case Fam::Output: {
        if (!d.dout) return false;
        const size_t m = moduleOf(*d.dout, slave);
        if (m == SIZE_MAX) return false;
        const size_t base = d.dout->bitOffset(m);
        const size_t n    = d.dout->module(m).bitCount();
        if (key >= '1' && key <= '9' && size_t(key - '1') < n) {
            const size_t ch = base + size_t(key - '1');
            d.dout->setBit(ch, !d.dout->bit(ch));
            return true;
        }
        if (key == 'a') { d.dout->allOn();  return true; }
        if (key == 'o') { d.dout->allOff(); return true; }
        return false;
    }
    case Fam::Combined: {
        if (!d.combo) return false;
        const size_t m = moduleOf(*d.combo, slave);
        if (m == SIZE_MAX) return false;
        const size_t base = d.combo->outputOffset(m);
        const size_t n    = d.combo->module(m).outputChannelCount();
        if (key >= '1' && key <= '9' && size_t(key - '1') < n) {
            const size_t ch = base + size_t(key - '1');
            d.combo->setOutput(ch, !d.combo->output(ch));
            return true;
        }
        if (key == 'o') { d.combo->allOutputsOff(); return true; }
        return false;
    }
    case Fam::AnalogOut: {
        if (!d.aout) return false;
        const size_t m = moduleOf(*d.aout, slave);
        if (m == SIZE_MAX) return false;
        const size_t base = d.aout->channelOffset(m);
        const size_t n    = d.aout->module(m).channelCount();
        size_t& sel = ui.sel[slave];
        if (sel >= n) sel = 0;
        if (key == '[') { sel = (sel + n - 1) % n; return true; }
        if (key == ']') { sel = (sel + 1) % n;     return true; }
        if (key == '+' || key == '-') {
            const int32_t delta = key == '+' ? 256 : -256;
            d.aout->setValue(base + sel,
                             d.aout->value(base + sel) + delta);
            return true;
        }
        if (key == '0') {
            for (size_t k = 0; k < n; ++k) d.aout->setValue(base + k, 0);
            return true;
        }
        return false;
    }
    case Fam::Pwm: {
        auto* t = findBySlave(d.pwms, slave);
        if (!t || !t->channelCount()) return false;
        size_t& sel = ui.sel[slave];
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
    case Fam::Pulse: {
        auto* t = findBySlave(d.pulses, slave);
        if (!t || !t->channelCount()) return false;
        size_t& sel = ui.sel[slave];
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
    case Fam::Stepper: {
        auto* t = findBySlave(d.steppers, slave);
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
    case Fam::DcMotor: {
        auto* t = findBySlave(d.dcmotors, slave);
        if (!t || !t->axisCount()) return false;
        size_t& sel = ui.sel[slave];
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
    case Fam::Drive: {
        auto* t = findBySlave(d.drives, slave);
        if (!t || !t->axisCount()) return false;
        size_t& sel = ui.sel[slave];
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
    case Fam::Comm: {
        auto* t = findBySlave(d.comms, slave);
        if (!t || !t->channels()) return false;
        if (key == 't') {
            const char* msg = "tether\r\n";
            t->writeData(0, std::span<const uint8_t>(
                reinterpret_cast<const uint8_t*>(msg), strlen(msg)));
            return true;
        }
        return false;
    }
    case Fam::PowerMeter: {
        auto* t = findBySlave(d.meters, slave);
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
                   const std::map<uint16_t, SlaveClass>& cls,
                   Drivers& d, double duration_sec,
                   const std::string& iface) {
    using namespace Tether::Examples;

    UiState ui;

    // Seed input transition counters.
    if (d.din) {
        for (size_t m = 0; m < d.din->moduleCount(); ++m) {
            const uint16_t s = d.din->slaveIndex(m);
            ui.transitions[s].assign(d.din->module(m).bitCount(), 0);
            ui.prevBits[s] = d.din->module(m).bits();
        }
    }

    auto managed = [&](uint16_t idx) {
        auto it = cls.find(idx);
        return it != cls.end() && it->second.fam != Fam::None;
    };

    TUI::TreeScreenHooks hooks;
    hooks.keyHints = "see right pane for device keys";

    hooks.onTick = [&]() {
        if (!d.din) return;
        for (size_t m = 0; m < d.din->moduleCount(); ++m) {
            const uint16_t s  = d.din->slaveIndex(m);
            const auto&    mod = d.din->module(m);
            const uint64_t now = mod.bits();
            uint64_t&      prv = ui.prevBits[s];
            auto&          tr  = ui.transitions[s];
            const uint64_t chg = now ^ prv;
            for (size_t k = 0; k < mod.bitCount() && k < 64; ++k)
                if ((chg >> k) & 1u) ++tr[k];
            prv = now;
        }
    };

    TUI::TreeScreen* screenPtr = nullptr;

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

        mvwprintw(win, row++, 1, "%s",
                  s->device_name ? s->device_name->c_str() : "?");
        mvwprintw(win, row++, 1, "slave   : %u", s->index);
        mvwprintw(win, row++, 1, "vendor  : 0x%08X",
                  s->vendor_id ? *s->vendor_id : 0);
        mvwprintw(win, row++, 1, "product : 0x%08X",
                  s->product_code ? *s->product_code : 0);

        auto it = cls.find(s->index);
        const SlaveClass empty{};
        const SlaveClass& sc = it != cls.end() ? it->second : empty;

        if (sc.fam != Fam::None && sc.def) {
            ++row;
            wattron(win, A_BOLD);
            mvwprintw(win, row++, 1, "family  : %s", sc.def->label);
            wattroff(win, A_BOLD);
            renderLive(win, row, s->index, sc.fam, d, ui, iface);
            ++row;
            wattron(win, A_DIM);
            mvwprintw(win, row++, 1, "also: ./%s -i %s",
                      sc.def->example, iface.c_str());
            wattroff(win, A_DIM);
        } else {
            ++row;
            wattron(win, A_DIM);
            if (isCouplerDevice(*s))
                mvwprintw(win, row++, 1, "(coupler — %zu terminal(s))",
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
        auto it = cls.find(static_cast<uint16_t>(sel->tag));
        if (it == cls.end()) return false;
        return handleKey(key, it->first, it->second.fam, d, ui);
    };

    // The tree is built from the ALREADY-discovered slave list — no
    // second bus scan here (a full DiscoveryOption::All re-read of every
    // slave's SII EEPROM takes seconds and previously looked like a hang).
    TUI::TreeScreen screen(
        std::string("Beckhoff Device Explorer — ") + iface,
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
    Tether::Examples::addVlanArgs(program);
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

    Tether::Examples::VlanConfig vlan;
    if (!Tether::Examples::parseVlanArgs(
            program.get<std::string>("--rx-vlan"),
            program.get<std::string>("--tx-vlan"), vlan, TAG)) {
        return 1;
    }

    const double duration_sec = program.get<double>("--time");

    bool interactive = true;
#ifdef TETHER_HAS_TERMINAL_UI
    if (interactive && !Tether::TUI::Session::available()) {
        if (program.get<bool>("--interactive")) {
            TETHER_LOGW(TAG, "no usable terminal — TUI unavailable");
        }
        interactive = false;
    }
#else
    if (program.get<bool>("--interactive")) {
        TETHER_LOGW(TAG, "built without ncurses — TUI unavailable");
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

    if (!Tether::Examples::setupVlanAndRxCallback(session, master, vlan, TAG)) {
        Tether::Examples::shutdownHostEthernet(session);
        return 5;
    }
    Tether::Examples::startHostPollThread(session, TAG);
    if (!Tether::Examples::startHostMaster(session, master, vlan, TAG)) {
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
    std::map<uint16_t, SlaveClass> cls;
    for (const auto& s : slaves) {
        auto sc = classify(s);
        if (sc.fam != Fam::None) {
            cls[s.index] = sc;
            TETHER_LOGI(TAG, "  s{} -> {} ({})", s.index,
                        sc.def->label, sc.id->name);
        }
    }

    // Bring all recognized terminals to OP (chains in their own PDO
    // groups, singles on the default group, one shared RT loop).
    Drivers drivers;
    bringUp(master, slaves, cls, drivers);

#ifdef TETHER_HAS_TERMINAL_UI
    runTui(slaves, cls, drivers, duration_sec, iface);
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
