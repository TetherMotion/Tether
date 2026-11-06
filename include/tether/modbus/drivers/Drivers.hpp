/**
 * @file Drivers.hpp
 * @brief Modbus "device drivers" (plan §14.4) — typed, io-free device
 *        wrappers in the spirit of tether::drives (AS715N & friends):
 *        domain calls on top of the register layer.
 *
 *  - ModbusVfd:       variable-frequency drive bound to a DeviceProfile
 *                     (semantic names, e.g. generic/vfd).
 *  - ModbusRemoteIo:  remote-I/O block (modbus I/O module) mapped onto
 *                     raw tables — discrete inputs / coils / input and
 *                     holding registers, no profile needed.
 *
 * Expose either kind to tether::io via bridge/ModbusIoExposer.hpp.
 */
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "tether/modbus/client/ModbusClient.hpp"
#include "tether/modbus/profile/Profile.hpp"

namespace tether::modbus {

// ---------------------------------------------------------------------------
// ModbusVfd — VFD semantics over a BoundDevice profile
// ---------------------------------------------------------------------------
/**
 * Register-name map defaults match `generic/vfd`. Rebind names for other
 * profiles (e.g. `.run = "start_cmd"`).
 */
class ModbusVfd {
public:
    struct Names {
        std::string run        = "run";        ///< coil: run command
        std::string freqSet    = "freq_set";   ///< HR: frequency setpoint (Hz)
        std::string freqAct    = "freq_act";   ///< IR/HR: output frequency
        std::string current    = "current";    ///< output current (A)
        std::string fault      = "fault";      ///< fault code register
        std::string faultReset = "fault_reset";///< coil/reg: fault reset
    };

    ModbusVfd(BoundDevice& dev, Names names)
        : dev_(dev), n_(std::move(names)) {}
    explicit ModbusVfd(BoundDevice& dev);   // defaulted Names

    // ---- commands ---------------------------------------------------------
    Expected<void> run(bool on)   { return dev_.write(n_.run, on ? 1 : 0); }
    Expected<void> stop()         { return dev_.write(n_.run, 0); }
    Expected<void> setFrequencyHz(double hz) {
        return dev_.write(n_.freqSet, hz);
    }
    Expected<void> resetFault()   { return dev_.write(n_.faultReset, 1); }

    // ---- telemetry --------------------------------------------------------
    Expected<double>   outputFrequencyHz() { return dev_.read(n_.freqAct); }
    Expected<double>   outputCurrent()     { return dev_.read(n_.current); }
    Expected<double>   faultCode()         { return dev_.read(n_.fault); }
    Expected<double>   running()           { return dev_.read(n_.run); }

    BoundDevice& device() { return dev_; }
    const Names& names()  const { return n_; }

private:
    BoundDevice& dev_;
    Names        n_;
};

// Out-of-line: nested-class NSDMI is only complete after ModbusVfd ends.
inline ModbusVfd::ModbusVfd(BoundDevice& dev)
    : ModbusVfd(dev, Names{}) {}

// ---------------------------------------------------------------------------
// ModbusRemoteIo — remote I/O module over raw tables (no profile needed)
// ---------------------------------------------------------------------------
/**
 * Typical mapping: DI → DiscreteInputs, DO → Coils, AI → InputRegisters,
 * AO → HoldingRegisters. Counts + base addresses configurable; channel
 * index i is `base + i`.
 */
class ModbusRemoteIo {
public:
    struct Cfg {
        uint16_t nDi = 0, baseDi = 0;   ///< discrete inputs
        uint16_t nDo = 0, baseDo = 0;   ///< coils (outputs)
        uint16_t nAi = 0, baseAi = 0;   ///< input registers
        uint16_t nAo = 0, baseAo = 0;   ///< holding registers (outputs)
    };

    ModbusRemoteIo(ModbusClient& cli, Cfg cfg) : cli_(cli), c_(cfg) {}

    const Cfg& cfg() const { return c_; }

    Expected<bool> digitalInput(uint16_t i) const {
        if (i >= c_.nDi)
            return err<bool>(ModbusError::codec("di index out of range"));
        auto r = cli_.readDiscreteInputs(uint16_t(c_.baseDi + i), 1);
        return r ? Expected<bool>{r->front()} : err<bool>(r.error());
    }
    Expected<bool> digitalOutput(uint16_t i) const {
        if (i >= c_.nDo)
            return err<bool>(ModbusError::codec("do index out of range"));
        auto r = cli_.readCoils(uint16_t(c_.baseDo + i), 1);
        return r ? Expected<bool>{r->front()} : err<bool>(r.error());
    }
    Expected<void> setDigitalOutput(uint16_t i, bool on) {
        if (i >= c_.nDo)
            return err<void>(ModbusError::codec("do index out of range"));
        return cli_.writeCoil(uint16_t(c_.baseDo + i), on);
    }
    Expected<uint16_t> analogInput(uint16_t i) const {
        if (i >= c_.nAi)
            return err<uint16_t>(ModbusError::codec("ai index out of range"));
        auto r = cli_.readInput(uint16_t(c_.baseAi + i), 1);
        return r ? Expected<uint16_t>{r->front()} : err<uint16_t>(r.error());
    }
    Expected<uint16_t> analogOutput(uint16_t i) const {
        if (i >= c_.nAo)
            return err<uint16_t>(ModbusError::codec("ao index out of range"));
        auto r = cli_.readHolding(uint16_t(c_.baseAo + i), 1);
        return r ? Expected<uint16_t>{r->front()} : err<uint16_t>(r.error());
    }
    Expected<void> setAnalogOutput(uint16_t i, uint16_t v) {
        if (i >= c_.nAo)
            return err<void>(ModbusError::codec("ao index out of range"));
        return cli_.writeReg(uint16_t(c_.baseAo + i), v);
    }

    /// Snapshot reads (single block transactions).
    Expected<std::vector<bool>> inputs() const {
        return c_.nDi ? cli_.readDiscreteInputs(c_.baseDi, c_.nDi)
                      : Expected<std::vector<bool>>{
                            std::vector<bool>{}};
    }
    Expected<std::vector<bool>> outputs() const {
        return c_.nDo ? cli_.readCoils(c_.baseDo, c_.nDo)
                      : Expected<std::vector<bool>>{
                            std::vector<bool>{}};
    }
    Expected<std::vector<uint16_t>> analogInputs() const {
        return c_.nAi ? cli_.readInput(c_.baseAi, c_.nAi)
                      : Expected<std::vector<uint16_t>>{
                            std::vector<uint16_t>{}};
    }

private:
    ModbusClient& cli_;
    Cfg           c_;
};

} // namespace tether::modbus
