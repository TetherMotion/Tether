#pragma once

/**
 * @file SimulatedCiA402Drive.hpp
 * @brief Simulated CiA 402 drive fleet behind IMachineSdoAccess.
 *
 * One deterministic simulated DS402 axis per EtherCAT slave index. Object
 * reads/writes go through the standard CiA 402 object dictionary
 * (0x6040 controlword, 0x6041 statusword, 0x6060/0x6061 modes,
 * 0x6064/0x606C/0x6077 actuals, 0x607A/0x60FF/0x6071 targets,
 * 0x6065 follow window, 0x603F error, 0x607C home offset) and drive a
 * CiA402 controlword-command state machine plus profile dynamics.
 *
 * Dynamics advance on an injected microsecond clock. The host sets time
 * via setClock()/advanceTo(); reads and writes advance lazily to the
 * current clock so callers stay coherent without an explicit tick. This
 * is the simulation backend for machine.sdo.*; swap in the EtherCAT SDO
 * adapter for real drives without changing the wire surface.
 */

#include "tether/io/MachineService.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace tether::io::machine {

namespace sim402 {

// CiA 402 states; the enum value is the statusword low-bit pattern.
enum State : int32_t {
    kNotReady        = 0x00,
    kSwitchOnDis     = 0x40,
    kReadyToSwitchOn = 0x21,
    kSwitchedOn      = 0x23,
    kOpEnabled       = 0x27,
    kQuickStopActive = 0x07,
    kFaultReact      = 0x0F,
    kFault           = 0x08,
};

// Modes of operation.
enum Mode : int8_t {
    kModeNone = 0,
    kModePP = 1, kModeVL = 2, kModePV = 3, kModeTQ = 4,
    kModeHM = 6, kModeCSP = 8, kModeCSV = 9, kModeCST = 10,
};

// Controlword bits (CiA 402).
inline constexpr uint16_t kCwSwitchOn       = 0x0001;
inline constexpr uint16_t kCwEnableVoltage  = 0x0002;
inline constexpr uint16_t kCwQuickStop      = 0x0004; // 0 = quick stop
inline constexpr uint16_t kCwEnableOp       = 0x0008;
inline constexpr uint16_t kCwNewSetPoint    = 0x0010;
inline constexpr uint16_t kCwFaultReset     = 0x0080;

// Statusword flag bits OR-ed onto the state pattern.
inline constexpr uint16_t kSwVoltageEnabled = 0x0010;
inline constexpr uint16_t kSwQuickStop      = 0x0020;
inline constexpr uint16_t kSwTargetReached  = 0x0400;
inline constexpr uint16_t kSwFollowingErr   = 0x2000;
inline constexpr uint16_t kSwRemote         = 0x0200;

// Object dictionary indices (subindex 0 for scalars).
inline constexpr uint16_t kIdxErrorCode    = 0x603F;
inline constexpr uint16_t kIdxControlword  = 0x6040;
inline constexpr uint16_t kIdxStatusword   = 0x6041;
inline constexpr uint16_t kIdxModeOp       = 0x6060;
inline constexpr uint16_t kIdxModeDisp     = 0x6061;
inline constexpr uint16_t kIdxPosActual    = 0x6064;
inline constexpr uint16_t kIdxFollowWin    = 0x6065;
inline constexpr uint16_t kIdxVelActual    = 0x606C;
inline constexpr uint16_t kIdxTorqueTarget = 0x6071;
inline constexpr uint16_t kIdxTorqueActual = 0x6077;
inline constexpr uint16_t kIdxPosTarget    = 0x607A;
inline constexpr uint16_t kIdxHomeOffset   = 0x607C;
inline constexpr uint16_t kIdxVelTarget    = 0x60FF;

inline constexpr double kVelEps = 1e-6;
inline constexpr double kUsPerSec = 1e6;
/**Largest integration step; larger gaps are sub-stepped for stability.*/
inline constexpr uint64_t kMaxStepUs = 100;

/**Per-axis configuration and runtime state.*/
struct Axis {
    // Configuration (set via configure(); the OD exposes the standard subset).
    int32_t unitsPerRev = 1000;
    int32_t maxVelocity = 100000;
    int32_t maxAcceleration = 500000;
    int32_t maxDeceleration = 500000;
    int32_t posMin = -2147483647;
    int32_t posMax = 2147483647;
    int32_t homeOffset = 0;
    uint32_t followWindow = 8;
    uint32_t flags = 1; // bit0 = enforce position limits

    // Runtime state.
    int32_t state = kSwitchOnDis;
    int8_t modeOp = kModeNone;
    uint16_t controlword = 0;
    uint16_t prevControlword = 0;
    double pos = 0.0, vel = 0.0, torque = 0.0;
    int32_t targetPos = 0, targetVel = 0, targetTorque = 0;
    uint32_t followWindowCur = 8;
    int32_t errorCode = 0;
    bool moveActive = false, targetReached = false, homingDone = false;
    uint64_t lastUs = 0;
    bool started = false;
};

inline uint16_t statuswordOf(const Axis& a) {
    uint16_t sw = uint16_t(a.state) & 0x7F;
    if (a.state != kSwitchOnDis && a.state != kNotReady && a.state != kFault)
        sw |= kSwVoltageEnabled;
    sw |= kSwRemote;
    if (a.state != kQuickStopActive) sw |= kSwQuickStop;
    if (a.targetReached) sw |= kSwTargetReached;
    if (a.moveActive && !a.targetReached) {
        const double goal = (a.modeOp == kModeHM)
                                ? double(a.homeOffset) : double(a.targetPos);
        const double window = a.followWindowCur > 0
                                  ? double(a.followWindowCur) : 1.0;
        if (std::fabs(goal - a.pos) > window) sw |= kSwFollowingErr;
    }
    return sw;
}

inline void updateState(Axis& a) {
    const uint16_t cw = a.controlword;
    if (a.state == kFault || a.state == kFaultReact) {
        if (cw & kCwFaultReset) { a.state = kSwitchOnDis; a.errorCode = 0; }
        return;
    }
    if (a.state == kNotReady) { a.state = kSwitchOnDis; return; }

    const bool ev = (cw & kCwEnableVoltage) != 0;
    const bool qs = (cw & kCwQuickStop) != 0;
    const bool so = (cw & kCwSwitchOn) != 0;
    const bool eo = (cw & kCwEnableOp) != 0;

    if (!ev)       a.state = kSwitchOnDis;     // DisableVoltage 0x0000
    else if (!qs)  a.state = kQuickStopActive; // QuickStop      0x0002
    else if (!so)  a.state = kReadyToSwitchOn; // Shutdown       0x0006
    else if (!eo)  a.state = kSwitchedOn;      // SwitchOn       0x0007
    else           a.state = kOpEnabled;       // EnableOperation 0x000F
}

inline void profileStep(Axis& a, double goal, double dt) {
    const double window = a.followWindowCur > 0
                              ? double(a.followWindowCur) : 1.0;
    double dist = goal - a.pos;
    if (std::fabs(dist) <= window) {
        a.pos = goal; a.vel = 0.0; a.targetReached = true; a.moveActive = false;
        return;
    }
    const double dir = dist >= 0.0 ? 1.0 : -1.0;
    const double adist = std::fabs(dist);
    const double vmaxDecel = std::sqrt(2.0 * double(a.maxDeceleration) * adist);
    double vmax = double(a.maxVelocity);
    if (vmaxDecel < vmax) vmax = vmaxDecel;
    const double desiredVel = dir * vmax;
    double dv = desiredVel - a.vel;
    const double rate = dv * desiredVel < 0.0
                            ? double(a.maxDeceleration) : double(a.maxAcceleration);
    const double maxDv = rate * dt;
    if (dv > maxDv) dv = maxDv;
    else if (dv < -maxDv) dv = -maxDv;
    a.vel += dv;
    a.pos += a.vel * dt;
    if (a.flags & 1u) {
        if (a.pos < double(a.posMin)) { a.pos = double(a.posMin); if (a.vel < 0) a.vel = 0; }
        else if (a.pos > double(a.posMax)) { a.pos = double(a.posMax); if (a.vel > 0) a.vel = 0; }
    }
}

inline void stepAxis(Axis& a, double dt) {
    updateState(a);

    const bool newSp = (a.controlword & kCwNewSetPoint) &&
                       !(a.prevControlword & kCwNewSetPoint);
    a.prevControlword = a.controlword;
    if (newSp && a.state == kOpEnabled &&
        (a.modeOp == kModePP || a.modeOp == kModeHM)) {
        a.moveActive = true;
        a.targetReached = false;
        if (a.modeOp == kModeHM) a.homingDone = false;
    }

    if (a.state != kOpEnabled) {
        a.vel *= (1.0 - std::min(1.0, 8.0 * dt));
        a.pos += a.vel * dt;
        if (std::fabs(a.vel) < kVelEps) {
            a.vel = 0.0; a.moveActive = false;
            if (a.modeOp == kModePP) a.targetReached = true;
        }
        return;
    }

    switch (a.modeOp) {
        case kModePP:
            if (a.moveActive) profileStep(a, double(a.targetPos), dt);
            break;
        case kModeHM:
            if (a.moveActive) profileStep(a, double(a.homeOffset), dt);
            if (a.targetReached && !a.homingDone) {
                a.pos = double(a.homeOffset); a.homingDone = true;
            }
            break;
        case kModeVL:
            a.vel = double(a.targetVel);
            a.pos += a.vel * dt;
            a.targetReached = (a.targetVel == 0);
            break;
        case kModePV: {
            double dv = double(a.targetVel) - a.vel;
            const double rate = dv * double(a.targetVel) < 0.0
                                    ? double(a.maxDeceleration) : double(a.maxAcceleration);
            const double maxDv = rate * dt;
            if (dv > maxDv) dv = maxDv;
            else if (dv < -maxDv) dv = -maxDv;
            a.vel += dv;
            a.pos += a.vel * dt;
            a.targetReached = (std::fabs(a.vel - double(a.targetVel)) < 1.0);
            break;
        }
        case kModeTQ:
            a.torque = double(a.targetTorque);
            a.vel += a.torque * 0.001 * dt;
            if (a.vel > double(a.maxVelocity)) a.vel = double(a.maxVelocity);
            else if (a.vel < -double(a.maxVelocity)) a.vel = -double(a.maxVelocity);
            a.pos += a.vel * dt;
            a.targetReached = true;
            break;
        case kModeCSP:
            a.pos = double(a.targetPos); a.vel = 0.0; a.targetReached = true;
            break;
        case kModeCSV:
            a.vel = double(a.targetVel);
            a.pos += a.vel * dt;
            a.targetReached = true;
            break;
        case kModeCST:
            a.torque = double(a.targetTorque);
            a.vel += a.torque * 0.001 * dt;
            if (a.vel > double(a.maxVelocity)) a.vel = double(a.maxVelocity);
            else if (a.vel < -double(a.maxVelocity)) a.vel = -double(a.maxVelocity);
            a.pos += a.vel * dt;
            a.targetReached = true;
            break;
        default:
            break;
    }
}

/**Little-endian encode of the low `size` bytes of `v`.*/
inline void putLE(std::vector<uint8_t>& out, uint64_t v, size_t size) {
    for (size_t i = 0; i < size; ++i)
        out.push_back(static_cast<uint8_t>(v >> (i * 8U)));
}

/**Little-endian decode to int64 (sign-extended).*/
inline int64_t getLE(const std::vector<uint8_t>& in) {
    uint64_t v = 0;
    const size_t n = std::min(in.size(), sizeof(uint64_t));
    for (size_t i = 0; i < n; ++i)
        v |= static_cast<uint64_t>(in[i]) << (i * 8U);
    if (n > 0 && n < 8 && (in[n - 1] & 0x80)) v |= ~uint64_t(0) << (n * 8U);
    return static_cast<int64_t>(v);
}

/**
 * @brief Simulated CiA 402 drive fleet behind IMachineSdoAccess.
 *
 * One Axis per slave index. Configure axes with configure(); the OD
 * read/write path interprets the standard objects and advances dynamics.
 */
class SimulatedCiA402Drive final : public IMachineSdoAccess {
public:
    using ClockFn = uint64_t (*)();

    explicit SimulatedCiA402Drive(ClockFn nowFn = nullptr) : nowFn_(nowFn) {}

    void setClock(ClockFn fn) { nowFn_ = fn; }

    /**Advance all axes to `nowUs` (monotonic microseconds). Gaps larger
     * than kMaxStepUs are integrated in sub-steps so sparse client polls
     * do not overshoot the motion profile. Thread-safe; IO calls and an
     * external ticker may advance concurrently.*/
    void advanceTo(uint64_t nowUs) {
        std::lock_guard<std::recursive_mutex> lk(mtx_);
        for (auto& [slave, axis] : axes_) {
            if (!axis.started) { axis.started = true; axis.lastUs = nowUs; continue; }
            if (nowUs <= axis.lastUs) continue;
            uint64_t t = axis.lastUs;
            axis.lastUs = nowUs;
            while (t < nowUs) {
                const uint64_t chunk = std::min<uint64_t>(nowUs - t, kMaxStepUs);
                stepAxis(axis, double(chunk) / kUsPerSec);
                t += chunk;
            }
        }
        nowUs_ = nowUs;
    }

    /**Current simulated time in microseconds.*/
    uint64_t nowUs() const {
        std::lock_guard<std::recursive_mutex> lk(mtx_);
        return nowFn_ ? nowFn_() : nowUs_;
    }

    /**Configure (or create) the simulated axis for `slave`.*/
    Axis& configure(uint16_t slave) { return axes_[slave]; }
    const Axis* axis(uint16_t slave) const {
        const auto it = axes_.find(slave);
        return it == axes_.end() ? nullptr : &it->second;
    }
    Axis* axis(uint16_t slave) {
        const auto it = axes_.find(slave);
        return it == axes_.end() ? nullptr : &it->second;
    }

    std::vector<SdoObjectInfoV1> listObjects(uint16_t slave) override {
        (void)slave;
        return objectTable();
    }

    SdoTransferResult read(uint16_t slave, uint16_t index, uint8_t subindex,
                           size_t maxBytes) override {
        std::lock_guard<std::recursive_mutex> lk(mtx_);
        advanceTo(nowUs());
        SdoTransferResult r;
        const auto it = axes_.find(slave);
        if (it == axes_.end()) {
            r.abortCode = 0x06020000;
            r.error = "No simulated CiA402 axis on this slave";
            return r;
        }
        const Axis& a = it->second;
        int64_t value = 0;
        size_t size = 0;
        bool known = true;
        switch (index) {
            case kIdxErrorCode:    value = a.errorCode; size = 2; break;
            case kIdxControlword:  value = a.controlword; size = 2; break;
            case kIdxStatusword:   value = statuswordOf(a); size = 2; break;
            case kIdxModeOp:       value = a.modeOp; size = 1; break;
            case kIdxModeDisp:     value = a.modeOp; size = 1; break;
            case kIdxPosActual:    value = int32_t(std::lrint(a.pos)); size = 4; break;
            case kIdxFollowWin:    value = int32_t(a.followWindowCur); size = 4; break;
            case kIdxVelActual:    value = int32_t(std::lrint(a.vel)); size = 4; break;
            case kIdxTorqueTarget: value = a.targetTorque; size = 2; break;
            case kIdxTorqueActual: value = int32_t(std::lrint(a.torque)); size = 2; break;
            case kIdxPosTarget:    value = a.targetPos; size = 4; break;
            case kIdxHomeOffset:   value = a.homeOffset; size = 4; break;
            case kIdxVelTarget:    value = a.targetVel; size = 4; break;
            default: known = false; break;
        }
        if (!known || subindex != 0) {
            r.abortCode = 0x06020000;
            r.error = "Object does not exist in the simulated dictionary";
            return r;
        }
        r.ok = true;
        putLE(r.data, static_cast<uint64_t>(value), size);
        if (r.data.size() > maxBytes) r.data.resize(maxBytes);
        return r;
    }

    SdoTransferResult write(uint16_t slave, uint16_t index, uint8_t subindex,
                            const std::vector<uint8_t>& data) override {
        std::lock_guard<std::recursive_mutex> lk(mtx_);
        advanceTo(nowUs());
        SdoTransferResult r;
        const auto it = axes_.find(slave);
        if (it == axes_.end()) {
            r.abortCode = 0x06020000;
            r.error = "No simulated CiA402 axis on this slave";
            return r;
        }
        if (subindex != 0 || data.empty()) {
            r.abortCode = 0x06090011;
            r.error = "Unsupported subindex or empty write";
            return r;
        }
        Axis& a = it->second;
        const int64_t value = getLE(data);
        bool known = true;
        switch (index) {
            case kIdxControlword:
                a.controlword = uint16_t(value & 0xFFFF); break;
            case kIdxModeOp:
                switch (int8_t(value)) {
                    case kModeNone: case kModePP: case kModeVL: case kModePV:
                    case kModeTQ: case kModeHM: case kModeCSP: case kModeCSV:
                    case kModeCST:
                        a.modeOp = int8_t(value); break;
                    default:
                        r.abortCode = 0x06090030;
                        r.error = "Unsupported mode of operation";
                        return r;
                }
                break;
            case kIdxPosActual: a.pos = double(value); break;
            case kIdxPosTarget: a.targetPos = int32_t(value); break;
            case kIdxVelTarget: a.targetVel = int32_t(value); break;
            case kIdxTorqueTarget: a.targetTorque = int32_t(value & 0xFFFF); break;
            case kIdxFollowWin: a.followWindowCur = uint32_t(value < 0 ? 0 : value); break;
            case kIdxHomeOffset: a.homeOffset = int32_t(value); break;
            default: known = false; break;
        }
        if (!known) {
            r.abortCode = 0x06020000;
            r.error = "Object is not writable in the simulated dictionary";
            return r;
        }
        stepAxis(a, 0.0); // apply controlword/mode/target immediately
        r.ok = true;
        return r;
    }

    /**Static CiA 402 object table for listObjects().*/
    static std::vector<SdoObjectInfoV1> objectTable() {
        return {
            {kIdxErrorCode, 0, 0x0006, 1, "Error code"},
            {kIdxControlword, 0, 0x0006, 3, "Controlword"},
            {kIdxStatusword, 0, 0x0006, 1, "Statusword"},
            {kIdxModeOp, 0, 0x0002, 3, "Modes of operation"},
            {kIdxModeDisp, 0, 0x0002, 1, "Modes of operation display"},
            {kIdxPosActual, 0, 0x0004, 1, "Position actual value"},
            {kIdxFollowWin, 0, 0x0007, 3, "Following error window"},
            {kIdxVelActual, 0, 0x0004, 1, "Velocity actual value"},
            {kIdxTorqueTarget, 0, 0x0003, 3, "Target torque"},
            {kIdxTorqueActual, 0, 0x0003, 1, "Torque actual value"},
            {kIdxPosTarget, 0, 0x0004, 3, "Target position"},
            {kIdxHomeOffset, 0, 0x0004, 3, "Home offset"},
            {kIdxVelTarget, 0, 0x0004, 3, "Target velocity"},
        };
    }

private:
    std::map<uint16_t, Axis> axes_;
    ClockFn nowFn_ = nullptr;
    uint64_t nowUs_ = 0;
    mutable std::recursive_mutex mtx_;
};

} // namespace sim402

} // namespace tether::io::machine
