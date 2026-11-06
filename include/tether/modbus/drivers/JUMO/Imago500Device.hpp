/**
 * @file Imago500Device.hpp
 * @brief JumoImago500 — typed Modbus/RTU driver over the Imago500Map
 *        register description. One class = one controller (units 1..254),
 *        covering all 4 controller channels C1..C4 and every datasheet
 *        group (status, process, program state, setpoints, manual output,
 *        params, program start, manual mode, program transmission,
 *        commands, RAM values, identification, clock).
 *
 *  Wire encodings handled per type (see Imago500Map.hpp):
 *    INT   -> I16 big-endian      (1 reg)
 *    FLOAT -> F32, low word first (2 regs, ScalarCodec WordSwap)
 *    LONG  -> I32, high word first(2 regs, ScalarCodec BigEndian)
 */
#pragma once

#include "tether/modbus/client/ModbusClient.hpp"
#include "tether/modbus/drivers/JUMO/Imago500Map.hpp"
#include "tether/modbus/typed/ScalarCodec.hpp"

namespace tether::modbus::jumo::imago500 {

class JumoImago500 {
public:
    explicit JumoImago500(ModbusClient& cli, uint8_t unit = 1) : cli_(cli) {
        cli_.setUnit(unit);
    }
    static DeviceProfile profile(uint8_t unit = 1) {
        return imago500Profile(unit);
    }

    // ---- typed scalar helpers ----------------------------------------------
    Expected<double> readF(uint16_t a, const ScalarSpec& s,
                           const TransactionOptions& o = {}) {
        auto r = cli_.readInput(a, uint16_t(valueSize(s.type)), o);
        if (!r) return err<double>(r.error());
        return decodeScalar(*r, s);
    }
    Expected<double> readH(uint16_t a, const ScalarSpec& s,
                           const TransactionOptions& o = {}) {
        auto r = cli_.readHolding(a, uint16_t(valueSize(s.type)), o);
        if (!r) return err<double>(r.error());
        return decodeScalar(*r, s);
    }
    Expected<void> writeScaled(uint16_t a, double v, const ScalarSpec& s,
                             const TransactionOptions& o = {}) {
        auto regs = encodeScalar(v, s);
        if (!regs) return err<void>(regs.error());
        return regs->size() == 1
            ? cli_.writeReg(a, regs->front(), o)
            : cli_.writeRegs(a, *regs, o);
    }

    // ---- §5.1 status words ----------------------------------------------------
    struct GenStatus {
        uint16_t raw = 0;
        bool delay, overRange, programEnd, automatic, manual,
             basicStatus, standstill;
    };
    Expected<GenStatus> generatorStatus() {
        auto w = word(addr::genStatus);
        if (!w) return err<GenStatus>(w.error());
        GenStatus s; s.raw = *w;
        s.delay       = (*w >> 2) & 1;  s.overRange   = (*w >> 3) & 1;
        s.programEnd  = (*w >> 4) & 1;  s.automatic   = (*w >> 8) & 1;
        s.manual      = (*w >> 9) & 1;  s.basicStatus = (*w >> 11) & 1;
        s.standstill  = (*w >> 13) & 1;
        return s;
    }
    /// Per-channel manual/self-optimization status from 0x008C.
    Expected<bool> ctlManualActive(int c) {
        return statusBit(addr::ctlStatus, ctlStatusManualBit(c));
    }
    Expected<bool> ctlSelfOptActive(int c) {
        return statusBit(addr::ctlStatus, ctlStatusSelfOptBit(c));
    }
    Expected<uint16_t> logicOutputMask()  { return word(addr::logicOutputs); }
    Expected<uint16_t> er8OutputMask()    { return word(addr::er8Outputs); }
    Expected<uint16_t> logicInputMask()   { return word(addr::logicInputs); }
    Expected<uint16_t> limitCompMask()    { return word(addr::limitComp); }
    Expected<uint16_t> logicModuleMask()  { return word(addr::logicModule); }
    /// Logic output n = 1..12 (datasheet bit order 1,7,2,8,3,9,...).
    Expected<bool> logicOutput(int n) {
        if (n < 1 || n > 12)
            return err<bool>(ModbusError::codec("logic output 1..12"));
        return statusBit(addr::logicOutputs, logicOutputBit(n));
    }
    Expected<bool> logicInput(int n) {     // n = 1..6
        if (n < 1 || n > 6)
            return err<bool>(ModbusError::codec("logic input 1..6"));
        return statusBit(addr::logicInputs, uint8_t(n - 1));
    }
    Expected<bool> limitComparator(int n) { // n = 1..16
        if (n < 1 || n > 16)
            return err<bool>(ModbusError::codec("limit comp 1..16"));
        return statusBit(addr::limitComp, uint8_t(n - 1));
    }

    // ---- §5.1 process data -----------------------------------------------------
    Expected<double> samplingTimeS()   { return readF(addr::samplingTime, kFloat); }
    Expected<double> internalPt100()   { return readF(addr::internalPt100, kFloat); }
    Expected<double> analogInput(int n) {       // n = 1..8
        if (n < 1 || n > 8) return err<double>(ModbusError::codec("ai 1..8"));
        return readF(uint16_t(addr::analogInBase + 2*(n-1)), kFloat);
    }
    Expected<double> math(int n) {              // n = 1..8
        if (n < 1 || n > 8) return err<double>(ModbusError::codec("math 1..8"));
        return readF(uint16_t(addr::mathBase + 2*(n-1)), kFloat);
    }

    /// Per-channel process values — one block read (10 regs).
    struct ChannelPv {
        double rampEnd, pvFiltered, pvRaw, setpoint, output,
               outHeating, outCooling, ctrlDiff, ctrlDev;
        bool   heatingOn, coolingOn;
    };
    Expected<ChannelPv> channelPv(int c) {
        if (c < 1 || c > 4) return err<ChannelPv>(ModbusError::codec("c 1..4"));
        const uint16_t b = uint16_t(addr::chBlockBase + 0x14*(c-1));
        auto r = cli_.readInput(b, 20);   // 9 floats + 2 switch words
        if (!r) return err<ChannelPv>(r.error());
        const auto d = [&](int o) {
            return decodeScalar(std::span<const uint16_t>(&(*r)[o], 2),
                                kFloat);
        };
        ChannelPv p;
        p.rampEnd = d(0); p.pvFiltered = d(2); p.pvRaw = d(4);
        p.setpoint = d(6); p.output = d(8); p.outHeating = d(10);
        p.outCooling = d(12); p.ctrlDiff = d(14); p.ctrlDev = d(16);
        p.heatingOn = (*r)[18] != 0; p.coolingOn = (*r)[19] != 0;
        return p;
    }
    Expected<double> pvFiltered(int c) { return chF(c, 0x02); }
    Expected<double> pvRaw(int c)      { return chF(c, 0x04); }
    Expected<double> setpointNow(int c){ return chF(c, 0x06); }
    Expected<double> outputPct(int c)  { return chF(c, 0x08); }
    Expected<double> outHeating(int c) { return chF(c, 0x0A); }
    Expected<double> outCooling(int c) { return chF(c, 0x0C); }
    Expected<double> ctrlDiff(int c)   { return chF(c, 0x0E); }
    Expected<double> ctrlDev(int c)    { return chF(c, 0x10); }
    Expected<bool>   heatingOn(int c)  {
        if (c < 1 || c > 4) return err<bool>(ModbusError::codec("c 1..4"));
        return statusBit(uint16_t(addr::chSwHeatBase + 0x14*(c-1)), 0);
    }
    Expected<bool>   coolingOn(int c)  {
        if (c < 1 || c > 4) return err<bool>(ModbusError::codec("c 1..4"));
        return statusBit(uint16_t(addr::chSwCoolBase + 0x14*(c-1)), 0);
    }

    // ---- §5.1 program state ----------------------------------------------------
    Expected<int> paramSetNumber(int c) {
        if (c < 1 || c > 4) return err<int>(ModbusError::codec("c 1..4"));
        return iword(uint16_t(addr::chParamSetBase + c - 1));
    }
    Expected<int> programNumber()      { return iword(addr::programNumber); }
    Expected<int> segmentNumber(int pch) {
        if (badPch(pch)) return err<int>(ModbusError::codec("pch 1..4"));
        return iword(uint16_t(addr::pchSegNumBase + pch - 1));
    }
    Expected<int> lastSegment(int pch) {
        if (badPch(pch)) return err<int>(ModbusError::codec("pch 1..4"));
        return iword(uint16_t(addr::pchLastSegBase + pch - 1));
    }
    Expected<int> toleranceBandSignal(){ return iword(addr::tolBandSignal); }
    Expected<int> freeSegments()       { return iword(addr::freeSegments); }
    Expected<double> generatorSetpoint(int pch) {
        if (badPch(pch)) return err<double>(ModbusError::codec("pch 1..4"));
        return readF(uint16_t(addr::pchGenSpBase + 2*(pch-1)), kFloat);
    }
    Expected<int> controlContacts() { return iword(addr::controlContacts); }
    Expected<int> pchParamSet(int pch) {
        if (badPch(pch)) return err<int>(ModbusError::codec("pch 1..4"));
        return iword(uint16_t(addr::pchParamSetBase + pch - 1));
    }
    Expected<long> programRunTimeS()  { return lword(addr::progRunTime); }
    Expected<long> programRemainingS(){ return lword(addr::progRemainTime); }
    Expected<long> programTimeS(int pch) {
        if (badPch(pch)) return err<long>(ModbusError::codec("pch 1..4"));
        return lword(uint16_t(addr::pchProgTimeBase + 2*(pch-1)));
    }
    Expected<long> segmentRunTimeS(int pch) {
        if (badPch(pch)) return err<long>(ModbusError::codec("pch 1..4"));
        return lword(uint16_t(addr::pchSegRunBase + 6*(pch-1)));
    }
    Expected<long> segmentRemainingS(int pch) {
        if (badPch(pch)) return err<long>(ModbusError::codec("pch 1..4"));
        return lword(uint16_t(addr::pchSegRemBase + 6*(pch-1)));
    }
    Expected<long> segmentTimeS(int pch) {
        if (badPch(pch)) return err<long>(ModbusError::codec("pch 1..4"));
        return lword(uint16_t(addr::pchSegTimeBase + 6*(pch-1)));
    }

    // ---- §5.2 EEPROM setpoints (infrequent writes!) ----------------------------
    Expected<double> setpoint(int c, int w) {
        if (c < 1 || c > 4 || w < 1 || w > 4)
            return err<double>(ModbusError::codec("c 1..4, w 1..4"));
        return readH(uint16_t(addr::setpointBase + 0x08*(c-1) + 2*(w-1)),
                     kFloat);
    }
    Expected<void> setSetpoint(int c, int w, double v) {
        if (c < 1 || c > 4 || w < 1 || w > 4)
            return err<void>(ModbusError::codec("c 1..4, w 1..4"));
        return writeScaled(
            uint16_t(addr::setpointBase + 0x08*(c-1) + 2*(w-1)), v, kFloat);
    }
    // ---- §5.3 manual output --------------------------------------------------
    Expected<void> setManualOutput(int c, double pct) {
        if (c < 1 || c > 4)
            return err<void>(ModbusError::codec("c 1..4"));
        return writeScaled(uint16_t(addr::manualOutBase + 2*(c-1)), pct,
                           kFloat);
    }
    Expected<double> manualOutput(int c) {
        if (c < 1 || c > 4) return err<double>(ModbusError::codec("c 1..4"));
        return readH(uint16_t(addr::manualOutBase + 2*(c-1)), kFloat);
    }

    // ---- §5.4 controller parameters --------------------------------------------
    enum class Param : int {
        Struct1 = 0, Struct2, XP1, XP2, TV1, TV2, TN1, TN2,
        CY1, CY2, XSH, XD1, XD2, TT, Y0, Y1, Y2, TK1, TK2,
    };
    static uint16_t paramAddr(int c, int set, Param p) {
        return uint16_t(addr::paramsBase + addr::paramsChStride*(c-1)
                        + addr::paramsSetStride*(set-1)
                        + paramOffset(int(p)));
    }
    Expected<double> param(int c, int set, Param p) {
        if (c < 1 || c > 4 || set < 1 || set > 2)
            return err<double>(ModbusError::codec("c 1..4, set 1..2"));
        return readH(paramAddr(c, set, p),
                     paramIsInt(int(p)) ? kInt : kFloat);
    }
    Expected<void> setParam(int c, int set, Param p, double v) {
        if (c < 1 || c > 4 || set < 1 || set > 2)
            return err<void>(ModbusError::codec("c 1..4, set 1..2"));
        return writeScaled(paramAddr(c, set, p), v,
                           paramIsInt(int(p)) ? kInt : kFloat);
    }

    // ---- §5.5 program start ------------------------------------------------------
    struct ProgramStart {
        int  programNo = 1;
        long delayS = 0;                       // seconds
        int  year = -1, month = 0, day = 0;    // -1 = no date wait
        int  secs = -1, mins = 0, hours = 0;   // -1 = instant start
        int  startSegment = 0;                 // 0 = program start
        long startSegRemainS = 0;
    };
    Expected<void> startProgram(const ProgramStart& ps) {
        if (auto e = wr(addr::progStartNo, uint16_t(ps.programNo)); !e)
            return e;
        if (auto e = writeScaled(addr::progStartDelay, double(ps.delayS),
                                 kLong); !e) return e;
        const uint16_t dt[3] = {uint16_t(ps.year), uint16_t(ps.month),
                                uint16_t(ps.day)};
        if (auto e = cli_.writeRegs(addr::progDateBase,
                                    std::span<const uint16_t>(dt, 3)); !e)
            return e;
        const uint16_t tm[3] = {uint16_t(ps.secs), uint16_t(ps.mins),
                                uint16_t(ps.hours)};
        if (auto e = cli_.writeRegs(addr::progTimeBase,
                                    std::span<const uint16_t>(tm, 3)); !e)
            return e;
        if (auto e = wr(addr::progStartSeg, uint16_t(ps.startSegment)); !e)
            return e;
        if (auto e = writeScaled(addr::progSegRemain,
                                 double(ps.startSegRemainS), kLong); !e)
            return e;
        return wr(addr::progBuffer,
                  uint16_t(1u << kProgBufferStartBit));
    }

    // ---- §5.6 manual operating mode ----------------------------------------------
    struct ManualModeData {
        double    setpoint[4] = {0,0,0,0};     // per PCh
        uint8_t   contacts = 0;                // bit per contact 1-8
        uint8_t   paramSet[4] = {0,0,0,0};     // 0 = set 1
        uint8_t   ctlFuncControllers = 0x0F;   // C1..C4 active bits
        uint16_t  ctlFuncLimits = 0xFFFF;      // comparators 1-16
    };
    Expected<void> setManualMode(const ManualModeData& m) {
        for (int pch = 0; pch < 4; ++pch)
            if (auto e = writeScaled(
                    uint16_t(addr::manSetpointBase + 2*pch),
                    m.setpoint[pch], kFloat); !e) return e;
        if (auto e = wr(addr::manContacts, m.contacts); !e) return e;
        for (int pch = 0; pch < 4; ++pch)
            if (auto e = wr(uint16_t(addr::manParamSetBase + pch),
                            m.paramSet[pch]); !e) return e;
        if (auto e = wr(addr::manCtlFuncCtl, m.ctlFuncControllers); !e)
            return e;
        if (auto e = wr(addr::manCtlFuncLim, m.ctlFuncLimits); !e)
            return e;
        return wr(addr::manBuffer, uint16_t(1u << kManBufferManualBit));
    }

    // ---- §5.7 program transmission ----------------------------------------------
    struct SegmentData {
        int    program = 1, channel = 1, segment = 1;
        double setpoint = 0, setpointTrace2 = 0;
        uint8_t contacts = 0;
        double tolMin = 0, tolMax = 0;
        long   timeS = 0;
        int    repeats = 0;          // 0 none, -1 endless
        int    targetSegment = 0;
        int    paramSet = 0;         // 0 = set 1
    };
    Expected<int> progTxError()       { return iword(addr::ptxError); }
    Expected<int> progTxMaxSegments() { return iword(addr::ptxMaxSeg); }
    /// Write one segment then activate TransmitSegment; poll
    /// progTxError() for completion (0 = ok).
    Expected<void> transmitSegment(const SegmentData& s) {
        if (auto e = wr(addr::ptxFunction,
                        uint16_t(ProgTxFn::TransmitSegment)); !e) return e;
        if (auto e = wr(addr::ptxProgram, uint16_t(s.program)); !e) return e;
        if (auto e = wr(addr::ptxChannel, uint16_t(s.channel)); !e) return e;
        if (auto e = wr(addr::ptxSegment, uint16_t(s.segment)); !e) return e;
        if (auto e = writeScaled(addr::ptxSetpoint, s.setpoint, kFloat); !e)
            return e;
        if (auto e = writeScaled(addr::ptxSetpointT2, s.setpointTrace2,
                                 kFloat); !e) return e;
        if (auto e = wr(addr::ptxContacts, s.contacts); !e) return e;
        if (auto e = writeScaled(addr::ptxTolMin, s.tolMin, kFloat); !e)
            return e;
        if (auto e = writeScaled(addr::ptxTolMax, s.tolMax, kFloat); !e)
            return e;
        if (auto e = writeScaled(addr::ptxSegTime, double(s.timeS), kLong);
            !e) return e;
        if (auto e = wr(addr::ptxRepeats, uint16_t(s.repeats)); !e) return e;
        if (auto e = wr(addr::ptxTargetSeg, uint16_t(s.targetSegment)); !e)
            return e;
        if (auto e = wr(addr::ptxParamSet, uint16_t(s.paramSet)); !e) return e;
        return wr(addr::ptxActivate, 1);
    }
    /// Activate a management function (delete program/memory/channel).
    Expected<void> progTxRun(ProgTxFn fn, int program = 0, int channel = 0) {
        if (auto e = wr(addr::ptxFunction, uint16_t(fn)); !e) return e;
        if (program)
            if (auto e = wr(addr::ptxProgram, uint16_t(program)); !e) return e;
        if (channel)
            if (auto e = wr(addr::ptxChannel, uint16_t(channel)); !e) return e;
        return wr(addr::ptxActivate, 1);
    }

    // ---- §5.8 commands ---------------------------------------------------------
    Expected<void> progCmdBits(uint16_t bits) { return wr(addr::cmdProgram, bits); }
    Expected<void> progFastForward()      { return progCmdBits(1u << 0); }
    Expected<void> progTempAlteration()   { return progCmdBits(1u << 1); }
    Expected<void> progSegmentChange()    { return progCmdBits(1u << 2); }
    Expected<void> progAckLimits()        { return progCmdBits(1u << 5); }
    Expected<void> programAutomatic()     { return progCmdBits(1u << 8); }
    Expected<void> programManual()        { return progCmdBits(1u << 9); }
    Expected<void> programCancel()        { return progCmdBits(1u << 11); }
    Expected<void> programStartLast()     { return progCmdBits(1u << 12); }
    Expected<void> programPause()         { return progCmdBits(1u << kCmdProgramPauseBit); }

    Expected<void> ctlCmdBits(int c, uint16_t bits) {
        if (c < 1 || c > 4)
            return err<void>(ModbusError::codec("c 1..4"));
        return wr(uint16_t(addr::cmdCtlBase + c - 1), bits);
    }
    Expected<void> ctlInhibitManual(int c)  { return ctlCmdBits(c, 1u << 6); }
    Expected<void> ctlStartSelfOpt(int c)   { return ctlCmdBits(c, 1u << 7); }
    Expected<void> ctlAutomatic(int c)      { return ctlCmdBits(c, 1u << 8); }
    Expected<void> ctlManual(int c)         { return ctlCmdBits(c, 1u << 9); }
    Expected<void> ctlCancelSelfOpt(int c)  { return ctlCmdBits(c, 1u << 10); }

    Expected<void> operationCmd(uint16_t bits) { return wr(addr::cmdOperation, bits); }
    Expected<void> displaySwitch()          { return operationCmd(1u << 10); }
    Expected<void> inhibitProgramStart()    { return operationCmd(1u << 11); }
    Expected<void> displayOff()             { return operationCmd(1u << 12); }
    Expected<void> inhibitProgramEditor()   { return operationCmd(1u << 13); }
    Expected<void> inhibitConfig()          { return operationCmd(1u << 14); }
    Expected<void> keyInhibit()             { return operationCmd(1u << 15); }

    /// Changeover selects: w=0 off, 1..4 -> W1..W4; ps: 0 off,1=P1,2=P2;
    /// pv: 0 off, 2..4 = analog input.
    Expected<void> selectSetpoint(int c, int w) {
        if (c < 1 || c > 4 || w < 0 || w > 4)
            return err<void>(ModbusError::codec("c 1..4, w 0..4"));
        return wr(uint16_t(addr::chSpChangeBase + 3*(c-1)), uint16_t(w));
    }
    Expected<void> selectParamSet(int c, int ps) {
        if (c < 1 || c > 4 || ps < 0 || ps > 2)
            return err<void>(ModbusError::codec("c 1..4, ps 0..2"));
        return wr(uint16_t(addr::chPsChangeBase + 3*(c-1)), uint16_t(ps));
    }
    Expected<void> selectPvSource(int c, int ai) {
        if (c < 1 || c > 4 || ai < 0 || ai > 4 || ai == 1)
            return err<void>(ModbusError::codec("c 1..4, ai 0|2..4"));
        return wr(uint16_t(addr::chPvChangeBase + 3*(c-1)), uint16_t(ai));
    }
    /// Program selection: -1 = off, 1..16.
    Expected<void> selectProgram(int n) {
        if (n < -1 || n > 16)
            return err<void>(ModbusError::codec("program -1..16"));
        return wr(addr::programSelect, uint16_t(int16_t(n)));
    }
    /// Relay outputs 1-12 (odd bit order) + activation bit15.
    Expected<void> setRelayOutputs(uint16_t outMask, bool activate = true) {
        return wr(addr::relayOutputs,
                  uint16_t(outMask | (activate ? (1u << kRelayActivateBit)
                                              : 0)));
    }

    // ---- §5.9/5.10 RAM setpoints + PV via interface (volatile — no
    //      EEPROM wear; use these for frequent updates) ---------------------
    Expected<void> setSetpointRam(int c, double v) {
        if (c < 1 || c > 4) return err<void>(ModbusError::codec("c 1..4"));
        return writeScaled(uint16_t(addr::ramSetpointBase + 4*(c-1)), v,
                           kFloat);
    }
    Expected<double> setpointRam(int c) {
        if (c < 1 || c > 4) return err<double>(ModbusError::codec("c 1..4"));
        return readH(uint16_t(addr::ramSetpointBase + 4*(c-1)), kFloat);
    }
    Expected<void> setProcessValueRam(int c, double v) {
        if (c < 1 || c > 4) return err<void>(ModbusError::codec("c 1..4"));
        return writeScaled(uint16_t(addr::ramPvBase + 4*(c-1)), v, kFloat);
    }
    Expected<double> processValueRam(int c) {
        if (c < 1 || c > 4) return err<double>(ModbusError::codec("c 1..4"));
        return readH(uint16_t(addr::ramPvBase + 4*(c-1)), kFloat);
    }

    // ---- §5.11 device identification ----------------------------------------
    struct Ident {
        int outBoard[6], analogIn[4];
        int setup, interface, profibus, er8, dataBuf;
    };
    Expected<Ident> deviceIdent() {
        auto r = cli_.readInput(addr::identOutBoard, 15);  // 0x019F..0x01AD
        if (!r) return err<Ident>(r.error());
        Ident id;
        for (int i = 0; i < 6; ++i) id.outBoard[i] = int16_t((*r)[i]);
        for (int i = 0; i < 4; ++i) id.analogIn[i] = int16_t((*r)[6 + i]);
        id.setup     = int16_t((*r)[10]);
        id.interface = int16_t((*r)[11]);
        id.profibus  = int16_t((*r)[12]);
        id.er8       = int16_t((*r)[13]);
        id.dataBuf   = int16_t((*r)[14]);
        return id;
    }

    // ---- §5.12 clock ---------------------------------------------------------
    Expected<void> setClock(int year, int month, int day,
                          int hours, int minutes, int seconds) {
        const uint16_t v[6] = {uint16_t(year), uint16_t(month),
                               uint16_t(day), uint16_t(hours),
                               uint16_t(minutes), uint16_t(seconds)};
        if (auto e = cli_.writeRegs(addr::clockBase,
                                    std::span<const uint16_t>(v, 6)); !e)
            return e;
        return wr(addr::clockSetCmd, 1);
    }

    // ---- generic bit helpers --------------------------------------------------
    Expected<bool> statusBit(uint16_t wordAddr, uint8_t bit) {
        auto w = word(wordAddr);
        if (!w) return err<bool>(w.error());
        return ((*w >> bit) & 1) != 0;
    }
    /// Raw 16-bit read of an R/O (input) status word.
    Expected<uint16_t> word(uint16_t a) {
        auto r = cli_.readInput(a, 1);
        if (!r) return err<uint16_t>(r.error());
        return (*r)[0];
    }

private:
    static bool badPch(int pch) { return pch < 1 || pch > 4; }
    Expected<int> iword(uint16_t a) {
        auto r = cli_.readInput(a, 1);
        if (!r) return err<int>(r.error());
        return int(int16_t((*r)[0]));
    }
    Expected<long> lword(uint16_t a) {
        auto r = cli_.readInput(a, 2);
        if (!r) return err<long>(r.error());
        return long(int32_t(orderToBe(*r, WordOrder::BigEndian)));
    }
    Expected<void> wr(uint16_t a, uint16_t v) {
        return cli_.writeReg(a, v);
    }
    Expected<double> chF(int c, uint16_t off) {
        if (c < 1 || c > 4) return err<double>(ModbusError::codec("c 1..4"));
        return readF(uint16_t(addr::chBlockBase + 0x14*(c-1) + off), kFloat);
    }
    ModbusClient& cli_;
};

} // namespace tether::modbus::jumo::imago500
