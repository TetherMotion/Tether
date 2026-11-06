/**
 * @file Imago500Map.hpp
 * @brief JUMO Imago 500 (703590) Modbus/RTU register map — the complete
 *        grouped description of every documented address (datasheet
 *        B 70.3590.9, "Interface description Modbus", §5 Address tables).
 *
 *  Layout facts from the datasheet:
 *   - One flat word-address space; FC3/FC4 read (max 127 words), FC6/FC16
 *     write. R/O regs are modelled as InputRegister, R/W as
 *     HoldingRegister.
 *   - FLOAT values = IEEE-754, LOW 16-bit word first on the wire
 *     (datasheet: 25.0 -> words "0000 41C8") => ScalarCodec WordSwap.
 *   - LONG (int32) = HIGH word first ("00 01 02 03" = 66051)
 *     => ScalarCodec BigEndian — the two 32-bit types differ!
 *   - INT = 16-bit big-endian; status/command words carry documented
 *     bit fields (see the JumoBit tables below).
 *   - Setpoint limits are NOT checked by the device for interface
 *     writes; EEPROM wears (~10k cycles) — use the RAM setpoint block
 *     (§5.9) for frequent changes.
 *   - Unit id 1..254 (0 = broadcast writes only); baud 9600/19200/38400;
 *     8-1-none/odd/even, 8-2-none. A configurable minimum response
 *     time (0..500 ms) may add latency between request and reply.
 */
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "tether/modbus/profile/Profile.hpp"

namespace tether::modbus::jumo::imago500 {

// ---- wire encodings (datasheet §4) -----------------------------------------
inline const ScalarSpec kInt   {ValueType::I16, WordOrder::BigEndian};
inline const ScalarSpec kUInt  {ValueType::U16, WordOrder::BigEndian};
inline const ScalarSpec kFloat {ValueType::F32, WordOrder::WordSwap};
inline const ScalarSpec kLong  {ValueType::I32, WordOrder::BigEndian};

// ---- group names (RegisterDecl::group) --------------------------------------
namespace grp {
    inline constexpr std::string_view status       = "status";
    inline constexpr std::string_view process      = "process";
    inline constexpr std::string_view program      = "program_state";
    inline constexpr std::string_view setpoints    = "setpoints";
    inline constexpr std::string_view manual       = "manual";
    inline constexpr std::string_view params       = "controller_params";
    inline constexpr std::string_view prog_start   = "program_start";
    inline constexpr std::string_view prog_tx      = "program_tx";
    inline constexpr std::string_view commands     = "commands";
    inline constexpr std::string_view ram          = "ram";
    inline constexpr std::string_view ident        = "device_id";
    inline constexpr std::string_view clock        = "clock";
}

// ---- base addresses -----------------------------------------------------------
namespace addr {
    // §5.1 process data — status words
    inline constexpr uint16_t genStatus      = 0x008B;
    inline constexpr uint16_t ctlStatus      = 0x008C;
    inline constexpr uint16_t logicOutputs   = 0x008D;
    inline constexpr uint16_t er8Outputs     = 0x008E;
    inline constexpr uint16_t logicInputs    = 0x008F;
    inline constexpr uint16_t limitComp      = 0x0090;
    inline constexpr uint16_t logicModule    = 0x0091;
    inline constexpr uint16_t samplingTime   = 0x00A4;
    inline constexpr uint16_t internalPt100  = 0x00A6;
    inline constexpr uint16_t analogInBase   = 0x00A8;   // +2n, n=0..7
    inline constexpr uint16_t mathBase       = 0x00B8;   // +2n, n=0..7
    inline constexpr uint16_t chBlockBase    = 0x00C8;   // +0x14*(c-1), c=1..4
    inline constexpr uint16_t chSwHeatBase   = 0x00DA;   // +0x14*(c-1)
    inline constexpr uint16_t chSwCoolBase   = 0x00DB;
    inline constexpr uint16_t chParamSetBase = 0x0118;   // +(c-1)
    inline constexpr uint16_t programNumber  = 0x011C;
    inline constexpr uint16_t pchSegNumBase  = 0x011D;   // +(pch-1)
    inline constexpr uint16_t pchLastSegBase = 0x0121;
    inline constexpr uint16_t tolBandSignal  = 0x0125;
    inline constexpr uint16_t freeSegments   = 0x0126;
    inline constexpr uint16_t pchGenSpBase   = 0x0127;   // +2*(pch-1), odd!
    inline constexpr uint16_t controlContacts= 0x012F;
    inline constexpr uint16_t pchParamSetBase= 0x0130;
    inline constexpr uint16_t progRunTime    = 0x0136;   // LONG
    inline constexpr uint16_t progRemainTime = 0x0138;
    inline constexpr uint16_t pchProgTimeBase= 0x013A;   // +2*(pch-1)
    inline constexpr uint16_t pchSegRunBase  = 0x0142;   // +6*(pch-1)
    inline constexpr uint16_t pchSegRemBase  = 0x0144;
    inline constexpr uint16_t pchSegTimeBase = 0x0146;
    // §5.2 EEPROM setpoints: base + 0x08*(c-1) + 2*(w-1), w=1..4
    inline constexpr uint16_t setpointBase   = 0x083C;
    // §5.3 manual output: +2*(c-1)
    inline constexpr uint16_t manualOutBase  = 0x016A;
    // §5.4 controller params: base + 0x48*(c-1) + 0x24*(set-1) + off
    inline constexpr uint16_t paramsBase     = 0x085C;
    inline constexpr uint16_t paramsChStride = 0x0048;
    inline constexpr uint16_t paramsSetStride= 0x0024;
    // §5.5 program start
    inline constexpr uint16_t progBuffer     = 0x01B4;   // bit8 = start
    inline constexpr uint16_t progStartNo    = 0x01B5;
    inline constexpr uint16_t progStartDelay = 0x01B6;   // LONG
    inline constexpr uint16_t progDateBase   = 0x01B8;   // y/m/d
    inline constexpr uint16_t progTimeBase   = 0x01BB;   // s/min/h
    inline constexpr uint16_t progStartSeg   = 0x01BE;
    inline constexpr uint16_t progSegRemain  = 0x01BF;   // LONG
    // §5.6 manual operating mode
    inline constexpr uint16_t manBuffer      = 0x01C1;   // bit9 -> manual
    inline constexpr uint16_t manSetpointBase= 0x01C2;   // +2*(pch-1)
    inline constexpr uint16_t manContacts    = 0x01CA;
    inline constexpr uint16_t manParamSetBase= 0x01CB;   // +(pch-1)
    inline constexpr uint16_t manCtlFuncCtl  = 0x01CF;
    inline constexpr uint16_t manCtlFuncLim  = 0x01D0;
    // §5.7 program transmission
    inline constexpr uint16_t ptxActivate    = 0x01D1;
    inline constexpr uint16_t ptxFunction    = 0x01D2;
    inline constexpr uint16_t ptxError       = 0x01D3;
    inline constexpr uint16_t ptxProgram     = 0x01D4;
    inline constexpr uint16_t ptxChannel     = 0x01D5;
    inline constexpr uint16_t ptxSegment     = 0x01D6;
    inline constexpr uint16_t ptxMaxSeg      = 0x01D7;
    inline constexpr uint16_t ptxSetpoint    = 0x01D8;   // FLOAT
    inline constexpr uint16_t ptxSetpointT2  = 0x01DA;
    inline constexpr uint16_t ptxContacts    = 0x01DC;
    inline constexpr uint16_t ptxTolMin      = 0x01DD;   // FLOAT (odd addr)
    inline constexpr uint16_t ptxTolMax      = 0x01DF;
    inline constexpr uint16_t ptxSegTime     = 0x01E1;   // LONG (odd addr)
    inline constexpr uint16_t ptxRepeats     = 0x01E3;
    inline constexpr uint16_t ptxTargetSeg   = 0x01E4;
    inline constexpr uint16_t ptxParamSet    = 0x01E5;
    // §5.8 commands
    inline constexpr uint16_t cmdProgram     = 0x0172;
    inline constexpr uint16_t cmdCtlBase     = 0x0173;   // +(c-1)
    inline constexpr uint16_t cmdOperation   = 0x0177;
    inline constexpr uint16_t chSpChangeBase = 0x017A;   // +3*(c-1)
    inline constexpr uint16_t chPsChangeBase = 0x017B;
    inline constexpr uint16_t chPvChangeBase = 0x017C;
    inline constexpr uint16_t programSelect  = 0x0186;
    inline constexpr uint16_t relayOutputs   = 0x01B3;
    // §5.9/5.10 volatile (RAM) setpoint + process value
    inline constexpr uint16_t ramSetpointBase= 0x015A;   // +4*(c-1)
    inline constexpr uint16_t ramPvBase      = 0x015C;
    // §5.11 identification
    inline constexpr uint16_t identOutBoard  = 0x019F;   // +n, n=0..5
    inline constexpr uint16_t identAnalogIn  = 0x01A5;   // +n, n=0..3
    inline constexpr uint16_t identSetup     = 0x01A9;
    inline constexpr uint16_t identInterface = 0x01AA;
    inline constexpr uint16_t identProfibus  = 0x01AB;
    inline constexpr uint16_t identEr8       = 0x01AC;
    inline constexpr uint16_t identDataBuf   = 0x01AD;
    // §5.12 clock
    inline constexpr uint16_t clockBase      = 0x007E;   // y/mo/d/h/mi/s
    inline constexpr uint16_t clockSetCmd    = 0x0084;
}

// ---- bit fields -------------------------------------------------------------
/// One documented bit inside a word register.
struct JumoBit {
    const char* name;   ///< e.g. "automatic", "program_end"
    uint8_t     bit;
};

/// 0x008B generator status.
inline constexpr std::array<JumoBit, 7> kGenStatusBits{{
    {"delay",          2},
    {"overrange",      3},   ///< over-/underrange
    {"program_end",    4},
    {"automatic",      8},
    {"manual",         9},   ///< manual / automatic-manual
    {"basic_status",  11},
    {"standstill",    13},
}};

/// 0x008C controller status — per-channel {manual, self_optimization}
/// bit pairs: C4 in bits 0/1, C3 in 4/5, C2 in 8/9, C1 in 12/13.
inline uint8_t ctlStatusManualBit(int channel)   // c = 1..4
    { return uint8_t((4 - channel) * 4 + 0); }
inline uint8_t ctlStatusSelfOptBit(int channel)
    { return uint8_t((4 - channel) * 4 + 1); }

/// Odd ordering shared by 0x008D logic outputs and 0x01B3 relay
/// outputs: bit0=out1, bit1=out7, bit2=out2, bit3=out8, ...
/// i.e. output n lives at bit ((n-1)/2 + (n even ? 0 : 0)) — see map:
///   out: 1 2 3 4 5 6 7 8 9 10 11 12 -> bit: 0 2 4 6 8 10 1 3 5 7 9 11
inline uint8_t logicOutputBit(int n) {           // n = 1..12
    const uint8_t map[12] = {0,2,4,6,8,10,1,3,5,7,9,11};
    return map[n - 1];
}
inline constexpr std::array<JumoBit, 12> kLogicOutputBits{{
    {"out1",0},{"out7",1},{"out2",2},{"out8",3},{"out3",4},{"out9",5},
    {"out4",6},{"out10",7},{"out5",8},{"out11",9},{"out6",10},{"out12",11},
}};

/// 0x0172 command word "program controller".
inline constexpr std::array<JumoBit, 8> kCmdProgramBits{{
    {"fast_forward",          0},
    {"temp_alteration",       1},
    {"segment_change",        2},
    {"ack_limit_comparators", 5},
    {"automatic",             8},
    {"manual",                9},
    {"cancel_program_ramp",  11},
    {"start_last_program",   12},
}};
inline constexpr uint8_t kCmdProgramPauseBit = 13;  ///< program/ramp pause

/// 0x0173..0x0176 command words "controller C1..C4" (same layout).
inline constexpr std::array<JumoBit, 5> kCmdControllerBits{{
    {"inhibit_manual",    6},
    {"start_self_opt",    7},
    {"automatic",         8},
    {"manual",            9},
    {"cancel_self_opt",  10},
}};

/// 0x0177 command word "operation".
inline constexpr std::array<JumoBit, 6> kCmdOperationBits{{
    {"display_switch",       10},  ///< edge-triggered
    {"inhibit_prog_start",   11},
    {"display_off",          12},
    {"inhibit_prog_editor",  13},
    {"inhibit_config",       14},
    {"key_inhibit",          15},
}};

/// 0x01B3 relay output word — same odd ordering as logic outputs,
/// plus bit15 = activation.
inline constexpr uint8_t kRelayActivateBit = 15;

/// 0x01B4 program buffer.
inline constexpr uint8_t kProgBufferStartBit = 8;
/// 0x01C1 program buffer (manual mode).
inline constexpr uint8_t kManBufferManualBit = 9;

/// §5.7 program-transmission function codes (0x01D2).
enum class ProgTxFn : uint16_t {
    ReadSegment     = 1,
    TransmitSegment = 2,
    DeleteProgram   = 6,
    DeleteMemory    = 7,
    DeleteChannel   = 10,
};

// ---- register tables ----------------------------------------------------------
namespace detail {
    inline RegisterDecl ro(std::string name, uint16_t a,
                           const ScalarSpec& spec, int pollMs,
                           std::string unit = {},
                           std::string group = {},
                           std::string desc = {}) {
        RegisterDecl d;
        d.name = std::move(name);
        d.addr = Address{TableId::InputRegister, a};
        d.spec = spec; d.pollMs = pollMs; d.writable = false;
        d.unit = std::move(unit); d.group = std::move(group);
        d.description = std::move(desc);
        return d;
    }
    inline RegisterDecl rw(std::string name, uint16_t a,
                           const ScalarSpec& spec,
                           std::string unit = {},
                           std::string group = {},
                           std::string desc = {}) {
        RegisterDecl d;
        d.name = std::move(name);
        d.addr = Address{TableId::HoldingRegister, a};
        d.spec = spec; d.writable = true;
        d.unit = std::move(unit); d.group = std::move(group);
        d.description = std::move(desc);
        return d;
    }
} // namespace detail

/// §5.1 — status + process data (all R/O).
inline std::vector<RegisterDecl> statusRegisters() {
    using detail::ro;
    std::vector<RegisterDecl> v;
    const std::string g{grp::status};
    v.push_back(ro("gen_status", addr::genStatus, kInt, 250, "", g,
        "bit2 delay, bit3 over/underrange, bit4 program_end, bit8 "
        "automatic, bit9 manual, bit11 basic_status, bit13 standstill"));
    v.push_back(ro("ctl_status", addr::ctlStatus, kInt, 500, "", g,
        "per channel {manual, self_opt}: C4=bit0/1, C3=bit4/5, "
        "C2=bit8/9, C1=bit12/13"));
    v.push_back(ro("logic_outputs", addr::logicOutputs, kInt, 250, "", g,
        "logic outputs 1-12, bit order: 1,7,2,8,3,9,4,10,5,11,6,12"));
    v.push_back(ro("er8_outputs", addr::er8Outputs, kInt, 250, "", g,
        "option board ER8 outputs 1-8"));
    v.push_back(ro("logic_inputs", addr::logicInputs, kInt, 250, "", g,
        "logic inputs 1-6 (1=closed)"));
    v.push_back(ro("limit_comparators", addr::limitComp, kInt, 250, "", g,
        "limit comparators 1-16 (1=on)"));
    v.push_back(ro("logic_module", addr::logicModule, kInt, 250, "", g,
        "logic module outputs 1-8"));
    for (int i = 0; i < 9; ++i)
        v.push_back(ro("reserved" + std::to_string(i),
                       uint16_t(0x0092 + 2 * i), kFloat, 0, "", g));
    v.push_back(ro("sampling_time", addr::samplingTime, kFloat, 0, "s", g));
    v.push_back(ro("internal_pt100", addr::internalPt100, kFloat, 500,
                   "°C", g, "internal Pt100 (cold junction)"));
    for (int i = 0; i < 8; ++i)
        v.push_back(ro("analog_in" + std::to_string(i + 1),
                       uint16_t(addr::analogInBase + 2 * i), kFloat, 250,
                       "", g));
    for (int i = 0; i < 8; ++i)
        v.push_back(ro("math" + std::to_string(i + 1),
                       uint16_t(addr::mathBase + 2 * i), kFloat, 500,
                       "", g));
    return v;
}

/// §5.1 — per-channel process values (all R/O; c = 1..4).
inline std::vector<RegisterDecl> channelRegisters(int c) {
    using detail::ro;
    const uint16_t b = uint16_t(addr::chBlockBase + 0x14 * (c - 1));
    const std::string g{grp::process}, p = "c" + std::to_string(c) + ".";
    const std::string sw = " (1=contact closed)";
    return {
        ro(p + "ramp_end",    b + 0x00, kFloat, 250, "", g),
        ro(p + "pv_filtered", b + 0x02, kFloat, 250, "", g),
        ro(p + "pv_raw",      b + 0x04, kFloat, 250, "", g),
        ro(p + "setpoint",    b + 0x06, kFloat, 250, "", g),
        ro(p + "output",      b + 0x08, kFloat, 250, "%", g,
           "displayed output -100..100%"),
        ro(p + "out_heating", b + 0x0A, kFloat, 250, "%", g),
        ro(p + "out_cooling", b + 0x0C, kFloat, 250, "%", g),
        ro(p + "ctrl_diff",   b + 0x0E, kFloat, 500, "", g,
           "control difference"),
        ro(p + "ctrl_dev",    b + 0x10, kFloat, 500, "", g,
           "control deviation"),
        ro(p + "sw_heating",  uint16_t(addr::chSwHeatBase + 0x14*(c-1)),
           kInt, 250, "", g, "heating contact" + sw),
        ro(p + "sw_cooling",  uint16_t(addr::chSwCoolBase + 0x14*(c-1)),
           kInt, 250, "", g, "cooling contact" + sw),
        ro(p + "param_set",   uint16_t(addr::chParamSetBase + c - 1),
           kInt, 0, "", g, "active parameter set 1..2"),
    };
}

/// §5.1 — program-channel run state (all R/O; pch = 1..4).
inline std::vector<RegisterDecl> programStateRegisters(int pch) {
    using detail::ro;
    const std::string g{grp::program},
        p = "pch" + std::to_string(pch) + ".";
    const int i = pch - 1;
    return {
        ro(p + "seg_num",   uint16_t(addr::pchSegNumBase + i), kInt, 250,
           "", g, "current segment 1..99"),
        ro(p + "last_seg",  uint16_t(addr::pchLastSegBase + i), kInt, 0,
           "", g),
        ro(p + "gen_sp",    uint16_t(addr::pchGenSpBase + 2*i), kFloat,
           250, "", g, "generator setpoint per operating mode"),
        ro(p + "param_set", uint16_t(addr::pchParamSetBase + i), kInt, 0,
           "", g),
        ro(p + "prog_time", uint16_t(addr::pchProgTimeBase + 2*i), kLong,
           1000, "s", g),
        ro(p + "seg_run",   uint16_t(addr::pchSegRunBase + 6*i), kLong,
           1000, "s", g),
        ro(p + "seg_remain",uint16_t(addr::pchSegRemBase + 6*i), kLong,
           1000, "s", g),
        ro(p + "seg_time",  uint16_t(addr::pchSegTimeBase + 6*i), kLong,
           1000, "s", g),
    };
}

/// §5.1 — global program state (all R/O).
inline std::vector<RegisterDecl> programRegisters() {
    using detail::ro;
    const std::string g{grp::program};
    return {
        ro("program_number",  addr::programNumber, kInt, 0, "", g,
           "1..99"),
        ro("tol_band_signal", addr::tolBandSignal, kInt, 250, "", g),
        ro("free_segments",   addr::freeSegments, kInt, 0, "", g),
        ro("control_contacts",addr::controlContacts, kInt, 250, "", g),
        ro("reserved_0134",   0x0134, kInt, 0, "", g),
        ro("reserved_0135",   0x0135, kInt, 0, "", g),
        ro("prog_run_time",   addr::progRunTime, kLong, 1000, "s", g),
        ro("prog_remain_time",addr::progRemainTime, kLong, 1000, "s", g),
    };
}

/// §5.2 — EEPROM setpoints W1..W4 (R/W — infrequent writes only;
/// device does not check limits; EEPROM ~10k cycles).
inline std::vector<RegisterDecl> setpointRegisters(int c) {
    using detail::rw;
    const std::string g{grp::setpoints},
        p = "c" + std::to_string(c) + ".w";
    const uint16_t b = uint16_t(addr::setpointBase + 0x08 * (c - 1));
    std::vector<RegisterDecl> v;
    for (int w = 0; w < 4; ++w)
        v.push_back(rw(p + std::to_string(w + 1),
                       uint16_t(b + 2 * w), kFloat, "", g,
                       "EEPROM setpoint — write rarely (10k cycles)"));
    return v;
}

/// §5.3 — manual output (R/W).
inline std::vector<RegisterDecl> manualOutRegisters() {
    using detail::rw;
    std::vector<RegisterDecl> v;
    for (int c = 1; c <= 4; ++c)
        v.push_back(rw("c" + std::to_string(c) + ".manual_output",
                       uint16_t(addr::manualOutBase + 2*(c-1)), kFloat,
                       "%", std::string(grp::manual)));
    return v;
}

/// §5.4 — controller parameters. Param slots per (channel, set):
/// struct1/struct2 INT then XP1..TK2 FLOAT.
inline constexpr std::array<std::string_view, 19> kParamNames{{
    "struct1", "struct2", "xp1", "xp2", "tv1", "tv2", "tn1", "tn2",
    "cy1", "cy2", "xsh", "xd1", "xd2", "tt", "y0", "y1", "y2", "tk1",
    "tk2",
}};
/// Word offset of param slot i within a (channel, set) block
/// (ints first, floats at +2 stride).
inline uint16_t paramOffset(int i) { return uint16_t(i < 2 ? i : 2*(i-1)); }
inline bool        paramIsInt(int i) { return i < 2; }

inline std::vector<RegisterDecl> paramRegisters(int c, int set) {
    using detail::rw;
    const std::string g{grp::params},
        p = "c" + std::to_string(c) + ".p" + std::to_string(set) + ".";
    const uint16_t b = uint16_t(addr::paramsBase +
        addr::paramsChStride * (c - 1) + addr::paramsSetStride * (set - 1));
    std::vector<RegisterDecl> v;
    for (size_t i = 0; i < kParamNames.size(); ++i) {
        const bool isInt = paramIsInt(int(i));
        v.push_back(rw(p + std::string(kParamNames[i]),
                       uint16_t(b + paramOffset(int(i))),
                       isInt ? kInt : kFloat, "", g));
    }
    return v;
}

/// §5.5 — program start with start data (R/W).
inline std::vector<RegisterDecl> programStartRegisters() {
    using detail::rw;
    const std::string g{grp::prog_start};
    return {
        rw("prog_start.buffer",   addr::progBuffer, kInt, "", g,
           "set bit8=1 to start the program"),
        rw("prog_start.number",   addr::progStartNo, kInt, "", g),
        rw("prog_start.delay_s",  addr::progStartDelay, kLong, "s", g),
        rw("prog_start.year",     addr::progDateBase + 0, kInt, "", g),
        rw("prog_start.month",    addr::progDateBase + 1, kInt, "", g),
        rw("prog_start.day",      addr::progDateBase + 2, kInt, "", g),
        rw("prog_start.secs",     addr::progTimeBase + 0, kInt, "", g,
           "-1 = instant start"),
        rw("prog_start.mins",     addr::progTimeBase + 1, kInt, "", g),
        rw("prog_start.hours",    addr::progTimeBase + 2, kInt, "", g),
        rw("prog_start.segment",  addr::progStartSeg, kInt, "", g,
           "1..99, 0 = program start"),
        rw("prog_start.seg_remain", addr::progSegRemain, kLong, "s", g),
    };
}

/// §5.6 — manual operating mode data (R/W).
inline std::vector<RegisterDecl> manualModeRegisters() {
    using detail::rw;
    const std::string g{grp::manual};
    std::vector<RegisterDecl> v = {
        rw("man.buffer",  addr::manBuffer, kInt, "", g,
           "set bit9=1 to switch to Manual mode"),
        rw("man.contacts",addr::manContacts, kInt, "", g,
           "control contacts 1-8 (1=on)"),
        rw("man.ctl_func_ctrl",   addr::manCtlFuncCtl, kInt, "", g,
           "control function: controllers 1-4 (bit0-3, 1=active)"),
        rw("man.ctl_func_limits", addr::manCtlFuncLim, kInt, "", g,
           "control function: limit comparators 1-16"),
    };
    for (int pch = 1; pch <= 4; ++pch) {
        v.push_back(rw("man.pch" + std::to_string(pch) + ".setpoint",
                       uint16_t(addr::manSetpointBase + 2*(pch-1)),
                       kFloat, "", g));
        v.push_back(rw("man.pch" + std::to_string(pch) + ".param_set",
                       uint16_t(addr::manParamSetBase + (pch-1)),
                       kInt, "", g, "0 = parameter set 1"));
    }
    return v;
}

/// §5.7 — program transmission (R/W + 2 R/O feedback regs).
inline std::vector<RegisterDecl> programTxRegisters() {
    using detail::ro;
    using detail::rw;
    const std::string g{grp::prog_tx};
    return {
        rw("ptx.activate",   addr::ptxActivate, kInt, "", g,
           "write 1 to execute the selected function"),
        rw("ptx.function",   addr::ptxFunction, kInt, "", g,
           "1=read seg, 2=transmit seg, 6=del prog, 7=del memory, "
           "10=del channel"),
        ro("ptx.error",      addr::ptxError, kInt, 0, "", g,
           "error feedback after function"),
        rw("ptx.program",    addr::ptxProgram, kInt, "", g),
        rw("ptx.channel",    addr::ptxChannel, kInt, "", g),
        rw("ptx.segment",    addr::ptxSegment, kInt, "", g, "1..100"),
        ro("ptx.max_seg",    addr::ptxMaxSeg, kInt, 0, "", g),
        rw("ptx.setpoint",   addr::ptxSetpoint, kFloat, "", g),
        rw("ptx.setpoint_t2",addr::ptxSetpointT2, kFloat, "", g,
           "trace 2"),
        rw("ptx.contacts",   addr::ptxContacts, kInt, "", g,
           "control contacts 1-8"),
        rw("ptx.tol_min",    addr::ptxTolMin, kFloat, "", g),
        rw("ptx.tol_max",    addr::ptxTolMax, kFloat, "", g),
        rw("ptx.seg_time",   addr::ptxSegTime, kLong, "s", g),
        rw("ptx.repeats",    addr::ptxRepeats, kInt, "", g,
           "0=none, -1=endless"),
        rw("ptx.target_seg", addr::ptxTargetSeg, kInt, "", g,
           "for repeats"),
        rw("ptx.param_set",  addr::ptxParamSet, kInt, "", g,
           "0 = parameter set 1"),
    };
}

/// §5.8 — command words + changeovers (R/W bitfields).
inline std::vector<RegisterDecl> commandRegisters() {
    using detail::rw;
    const std::string g{grp::commands};
    std::vector<RegisterDecl> v = {
        rw("cmd.program", addr::cmdProgram, kInt, "", g,
           "bit0 ff, bit1 temp-alt, bit2 seg-change, bit5 ack-limit, "
           "bit8 auto, bit9 manual, bit11 cancel, bit12 start-last, "
           "bit13 pause"),
        rw("cmd.operation", addr::cmdOperation, kInt, "", g,
           "bit10 disp-switch, bit11 inhibit-start, bit12 disp-off, "
           "bit13 inhibit-editor, bit14 inhibit-config, bit15 "
           "key-inhibit"),
        rw("program_select", addr::programSelect, kInt, "", g,
           "-1=off, 1..16"),
        rw("relay_outputs", addr::relayOutputs, kInt, "", g,
           "outputs 1-12 (odd order), bit15 = activation"),
    };
    for (int c = 1; c <= 4; ++c) {
        const std::string p = "cmd.c" + std::to_string(c);
        v.push_back(rw(p, uint16_t(addr::cmdCtlBase + c - 1), kInt, "", g,
            "bit6 inhibit-manual, bit7 start-selfopt, bit8 auto, "
            "bit9 manual, bit10 cancel-selfopt"));
        v.push_back(rw(p + ".sp_change",
                       uint16_t(addr::chSpChangeBase + 3*(c-1)), kInt,
                       "", g, "0=off, 1..4 = W1..W4"));
        v.push_back(rw(p + ".ps_change",
                       uint16_t(addr::chPsChangeBase + 3*(c-1)), kInt,
                       "", g, "0=off, 1=P1, 2=P2"));
        v.push_back(rw(p + ".pv_change",
                       uint16_t(addr::chPvChangeBase + 3*(c-1)), kInt,
                       "", g, "0=off, 2..4 = analog input"));
    }
    return v;
}

/// §5.9 + §5.10 — volatile RAM setpoints & process values (frequent
/// writes; lost on power cycle, no EEPROM wear, no limit check).
inline std::vector<RegisterDecl> ramRegisters() {
    using detail::rw;
    const std::string g{grp::ram};
    std::vector<RegisterDecl> v;
    for (int c = 1; c <= 4; ++c) {
        v.push_back(rw("c" + std::to_string(c) + ".ram_setpoint",
                       uint16_t(addr::ramSetpointBase + 4*(c-1)), kFloat,
                       "", g, "volatile setpoint — frequent writes OK"));
        v.push_back(rw("c" + std::to_string(c) + ".ram_pv",
                       uint16_t(addr::ramPvBase + 4*(c-1)), kFloat,
                       "", g, "process value via interface (volatile)"));
    }
    return v;
}

/// §5.11 — device identification (R/O).
inline std::vector<RegisterDecl> identRegisters() {
    using detail::ro;
    const std::string g{grp::ident};
    std::vector<RegisterDecl> v;
    for (int i = 0; i < 6; ++i)
        v.push_back(ro("id.out_board" + std::to_string(i + 1),
                       uint16_t(addr::identOutBoard + i), kInt, 0, "", g));
    for (int i = 0; i < 4; ++i)
        v.push_back(ro("id.analog_in" + std::to_string(i + 1),
                       uint16_t(addr::identAnalogIn + i), kInt, 0, "", g));
    v.push_back(ro("id.setup",     addr::identSetup, kInt, 0, "", g));
    v.push_back(ro("id.interface", addr::identInterface, kInt, 0, "", g));
    v.push_back(ro("id.profibus",  addr::identProfibus, kInt, 0, "", g));
    v.push_back(ro("id.er8",       addr::identEr8, kInt, 0, "", g));
    v.push_back(ro("id.data_buf",  addr::identDataBuf, kInt, 0, "", g));
    return v;
}

/// §5.12 — real-time clock (R/W + set command).
inline std::vector<RegisterDecl> clockRegisters() {
    using detail::rw;
    const std::string g{grp::clock};
    return {
        rw("clock.year",   addr::clockBase + 0, kInt, "", g,
           "without century"),
        rw("clock.month",  addr::clockBase + 1, kInt, "", g),
        rw("clock.day",    addr::clockBase + 2, kInt, "", g),
        rw("clock.hours",  addr::clockBase + 3, kInt, "", g),
        rw("clock.minutes",addr::clockBase + 4, kInt, "", g),
        rw("clock.seconds",addr::clockBase + 5, kInt, "", g),
        rw("clock.set",    addr::clockSetCmd, kInt, "", g,
           "write 1 to apply the date/time fields"),
    };
}

// ---- whole-device profile ---------------------------------------------------
/**
 * Every documented register, tagged by datasheet group — feed
 * `DeviceProfile::registers` to a BoundDevice / exposer and all
 * attributes become addressable by name ("c1.pv_filtered",
 * "c3.p2.tn1", "prog_start.number", ...).
 */
inline std::vector<RegisterDecl> allRegisters() {
    std::vector<RegisterDecl> v = statusRegisters();
    for (int c = 1; c <= 4; ++c) {
        auto r = channelRegisters(c);
        v.insert(v.end(), r.begin(), r.end());
    }
    auto pr = programRegisters();
    v.insert(v.end(), pr.begin(), pr.end());
    for (int pch = 1; pch <= 4; ++pch) {
        auto r = programStateRegisters(pch);
        v.insert(v.end(), r.begin(), r.end());
    }
    for (int c = 1; c <= 4; ++c) {
        auto r = setpointRegisters(c);
        v.insert(v.end(), r.begin(), r.end());
    }
    auto mo = manualOutRegisters();
    v.insert(v.end(), mo.begin(), mo.end());
    for (int c = 1; c <= 4; ++c)
        for (int s = 1; s <= 2; ++s) {
            auto r = paramRegisters(c, s);
            v.insert(v.end(), r.begin(), r.end());
        }
    auto ps = programStartRegisters();
    v.insert(v.end(), ps.begin(), ps.end());
    auto mm = manualModeRegisters();
    v.insert(v.end(), mm.begin(), mm.end());
    auto tx = programTxRegisters();
    v.insert(v.end(), tx.begin(), tx.end());
    auto cm = commandRegisters();
    v.insert(v.end(), cm.begin(), cm.end());
    auto rm = ramRegisters();
    v.insert(v.end(), rm.begin(), rm.end());
    auto id = identRegisters();
    v.insert(v.end(), id.begin(), id.end());
    auto ck = clockRegisters();
    v.insert(v.end(), ck.begin(), ck.end());
    return v;
}

/// Complete DeviceProfile — registers carry `group`, `unit`,
/// `writable` and `pollMs` hints so an exposer can surface everything.
inline DeviceProfile imago500Profile(uint8_t unit = 1) {
    DeviceProfile p;
    p.id = "jumo/imago500";
    p.vendor = "JUMO";
    p.model = "Imago 500 (703590)";
    p.defaultUnit = unit;
    p.registers = allRegisters();
    // JUMO Modbus-float layout: low word first (WordSwap for F32);
    // datasheet read limit is 127 words per request.
    p.quirks.maxReadQty = 127;
    p.quirks.interFrameDelayUs = 0;   // raise if unit needs it
    return p;
}

} // namespace tether::modbus::jumo::imago500
