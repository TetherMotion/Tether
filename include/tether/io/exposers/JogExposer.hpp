/**
 * @file JogExposer.hpp
 * @brief Tether IO exposer for the generic JogController.
 *
 * Registers the complete operator-jogging surface of a
 * tether::control::JogController into an io::Registry:
 *
 *   Functions
 *     jog.describe                -> String  JSON: axes, groups, limits,
 *                                            presets (UIs render from this)
 *     jog.stop_all                -> Bool
 *     jog.<axis>.jog(rate, lease) -> Bool    continuous jog; rate clamped to
 *                                            maxRate, lease to maxLeaseMs
 *     jog.<axis>.move(distance)   -> F64     bounded increment; returns the
 *                                            applied (clamped) distance
 *     jog.<axis>.stop             -> Bool    ramped stop (convenience — the
 *                                            lease is the real failsafe)
 *
 *   Signals (per axis)
 *     jog.<axis>.state            Binary snapshot (StructDescriptor attached)
 *     jog.<axis>.state.<field>    scalar: mode, active, rate, position,
 *                                 remaining, leaseLeftMs
 *
 * All commands take the current time from a caller-provided nowMs clock so
 * the lease deadline shares the motion thread's time source.
 *
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#pragma once

#include "tether/control/JogController.hpp"
#include "tether/io/ParameterExposer.hpp"

#include <chrono>
#include <cstring>
#include <deque>
#include <functional>
#include <string>

namespace tether::io::exposers {

class JogExposer : public IParameterExposer {
public:
    /// Millisecond clock used for lease deadlines.  Must be the same
    /// monotonic time source the host passes to JogController::update().
    using NowFn = std::function<int64_t()>;

    /**
     * @param jog     Controller to expose (must outlive the registry).
     * @param nowMs   Monotonic millisecond clock; defaults to
     *                steady_clock.
     * @param prefix  IO name prefix ("jog" -> "jog.x.jog", "jog.describe").
     */
    explicit JogExposer(tether::control::JogController& jog,
                        NowFn nowMs = defaultClock, const char* prefix = "jog")
        : jog_(jog), nowMs_(std::move(nowMs)), prefix_(prefix) {}

    const char* moduleName() const override { return "jog"; }

    void expose(Registry& registry, uint64_t idBase) override {
        // Global functions ------------------------------------------------
        addFunction(registry, idBase, kDescribeId, prefix_ + ".describe",
                    "JSON description of jog axes, groups, limits and presets",
                    {}, ValueType::String,
                    [this](const std::vector<FunctionArgument>&) {
                        FunctionCallResult r;
                        r.success = true;
                        const std::string json = jog_.describeJson();
                        r.returnValue.assign(json.begin(), json.end());
                        return r;
                    });

        addFunction(registry, idBase, kStopAllId, prefix_ + ".stop_all",
                    "Stop jogging on every axis (ramped stop)", {},
                    ValueType::Bool,
                    [this](const std::vector<FunctionArgument>&) {
                        jog_.stopAll();
                        return boolResult(true);
                    });

        // Per-axis functions + state signals ------------------------------
        for (size_t i = 0; i < jog_.axisCount(); ++i) {
            const std::string axis = jog_.axisConfig(i).name;
            const std::string base = prefix_ + "." + axis;
            const std::string grp = prefix_ + "." + axis;
            const uint32_t lid = kAxisBase + static_cast<uint32_t>(i) * kAxisStride;

            FunctionParameter rate;
            rate.name = "rate";
            rate.description = "Jog rate in " + jog_.axisConfig(i).unit +
                               "/s (server-clamped to ±maxRate; 0 = hold)";
            rate.type = ValueType::F64;
            FunctionParameter lease;
            lease.name = "lease_ms";
            lease.description =
                "Lease duration in ms (server-clamped to maxLeaseMs; "
                "0 = default). Motion stops when it expires.";
            lease.type = ValueType::U32;
            addFunction(registry, idBase, lid + kJogFn, base + ".jog",
                        "Continuous jog — refresh before the lease expires",
                        {rate, lease}, ValueType::Bool,
                        [this, i](const std::vector<FunctionArgument>& args) {
                            return jogCall(i, args);
                        });

            FunctionParameter dist;
            dist.name = "distance";
            dist.description = "Incremental distance in " +
                               jog_.axisConfig(i).unit +
                               " (server-clamped to ±maxIncrement)";
            dist.type = ValueType::F64;
            addFunction(registry, idBase, lid + kMoveFn, base + ".move",
                        "Bounded incremental move — returns applied distance",
                        {dist}, ValueType::F64,
                        [this, i](const std::vector<FunctionArgument>& args) {
                            return moveCall(i, args);
                        });

            addFunction(registry, idBase, lid + kStopFn, base + ".stop",
                        "Ramped stop (convenience — the lease is the failsafe)",
                        {}, ValueType::Bool,
                        [this, i](const std::vector<FunctionArgument>&) {
                            jog_.stop(i);
                            return boolResult(true);
                        });

            exposeState(registry, idBase, lid, i, base, grp);
        }
    }

    /// Default monotonic millisecond clock (steady_clock).
    static int64_t defaultClock() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }

private:
    // Local-id layout: 1 = describe, 2 = stop_all, then per-axis blocks of
    // kAxisStride starting at kAxisBase.
    static constexpr uint32_t kDescribeId = 1;
    static constexpr uint32_t kStopAllId  = 2;
    static constexpr uint32_t kAxisBase   = 0x100;
    static constexpr uint32_t kAxisStride = 16;
    static constexpr uint32_t kJogFn      = 0;
    static constexpr uint32_t kMoveFn     = 1;
    static constexpr uint32_t kStopFn     = 2;
    static constexpr uint32_t kStateSnap  = 3;
    static constexpr uint32_t kStateFirst = 4;  // scalar fields follow

    using Snapshot = tether::control::JogController::AxisSnapshot;

    FunctionCallResult jogCall(size_t axis,
                               const std::vector<FunctionArgument>& args) {
        double rate = 0.0;
        uint32_t lease = 0;
        if (args.size() > 0 && args[0].value.size() == sizeof(double))
            std::memcpy(&rate, args[0].value.data(), sizeof(double));
        if (args.size() > 1 && args[1].value.size() == sizeof(uint32_t))
            std::memcpy(&lease, args[1].value.data(), sizeof(uint32_t));
        if (!jog_.jog(axis, rate, lease, nowMs_())) {
            FunctionCallResult r;
            r.errorMessage = "jog rejected (invalid axis or rate)";
            return r;
        }
        return boolResult(true);
    }

    FunctionCallResult moveCall(size_t axis,
                                const std::vector<FunctionArgument>& args) {
        double dist = 0.0;
        if (args.size() > 0 && args[0].value.size() == sizeof(double))
            std::memcpy(&dist, args[0].value.data(), sizeof(double));
        const double applied = jog_.move(axis, dist, nowMs_());
        FunctionCallResult r;
        if (applied == 0.0 && dist != 0.0) {
            r.error = ErrorCode::ResourceBusy;
            r.errorMessage =
                "move rejected (axis busy with continuous jog or stopping)";
            return r;
        }
        r.success = true;
        r.returnValue.resize(sizeof(double));
        std::memcpy(r.returnValue.data(), &applied, sizeof(double));
        return r;
    }

    static FunctionCallResult boolResult(bool ok) {
        FunctionCallResult r;
        r.success = true;
        r.returnValue = {static_cast<uint8_t>(ok ? 1 : 0)};
        return r;
    }

    void addFunction(Registry& registry, uint64_t idBase, uint32_t localId,
                     const std::string& name, const std::string& desc,
                     std::vector<FunctionParameter> params,
                     ValueType retType, FunctionCallback cb) {
        FunctionEntry fn;
        fn.id = makeId(idBase, localId);
        fn.name = name;
        fn.description = desc;
        fn.group = prefix_;
        fn.parameters = std::move(params);
        fn.returnValue.present = true;
        fn.returnValue.name = "result";
        fn.returnValue.type = retType;
        fn.callback = std::move(cb);
        registry.addFunction(std::move(fn));
    }

    void exposeState(Registry& registry, uint64_t idBase, uint32_t lid,
                     size_t axis, const std::string& base,
                     const std::string& grp) {
        descriptors_.push_back(buildStateDescriptor(makeId(idBase, lid + kStateSnap),
                                                    base + ".state"));
        const StructDescriptor* sd = &descriptors_.back();

        auto snap = [this, axis](void* dest) {
            jog_.fillSnapshot(axis, *static_cast<Snapshot*>(dest), nowMs_());
        };

        SignalEntry s;
        s.id = makeId(idBase, lid + kStateSnap);
        s.name = base + ".state";
        s.description = "Jog state snapshot for axis '" +
                        jog_.axisConfig(axis).name + "'";
        s.group = grp;
        s.valueType = ValueType::Binary;
        s.maxValueSize = sizeof(Snapshot);
        s.structDesc = sd;
        s.varReadFn = [snap](void* d, size_t maxLen) -> size_t {
            if (maxLen < sizeof(Snapshot)) return 0;
            snap(d);
            return sizeof(Snapshot);
        };
        registry.addSignal(std::move(s));

        // Scalar field signals — same layout as the snapshot POD.
        struct Field {
            const char* name;
            ValueType type;
            uint16_t offset;
            uint16_t size;
        };
        static constexpr Field kFields[] = {
            {"mode",         ValueType::U8,  offsetof(Snapshot, mode),        1},
            {"active",       ValueType::U8,  offsetof(Snapshot, active),      1},
            {"rate",         ValueType::F64, offsetof(Snapshot, rate),        8},
            {"position",     ValueType::F64, offsetof(Snapshot, position),    8},
            {"remaining",    ValueType::F64, offsetof(Snapshot, remaining),   8},
            {"leaseLeftMs",  ValueType::U32, offsetof(Snapshot, leaseLeftMs), 4},
        };
        uint32_t local = lid + kStateFirst;
        for (const auto& f : kFields) {
            SignalEntry fs;
            fs.id = makeId(idBase, local++);
            fs.name = base + ".state." + f.name;
            fs.description = "Jog " + std::string(f.name) + " for axis '" +
                             jog_.axisConfig(axis).name + "'";
            fs.group = grp;
            fs.valueType = f.type;
            const uint16_t off = f.offset;
            const uint16_t sz = f.size;
            fs.readFn = [snap, off, sz](void* d) {
                Snapshot buf;
                snap(&buf);
                std::memcpy(d, reinterpret_cast<const uint8_t*>(&buf) + off,
                            sz);
            };
            registry.addSignal(std::move(fs));
        }
    }

    static StructDescriptor buildStateDescriptor(uint64_t id,
                                                 const std::string& name) {
        StructDescriptor sd;
        sd.entryId = id;
        sd.name = name;
        sd.totalSize = sizeof(Snapshot);
        sd.fields = {
            {"mode",        ValueType::U8,  offsetof(Snapshot, mode),        1, ""},
            {"active",      ValueType::U8,  offsetof(Snapshot, active),      1, ""},
            {"rate",        ValueType::F64, offsetof(Snapshot, rate),        8, "units/s"},
            {"position",    ValueType::F64, offsetof(Snapshot, position),    8, "units"},
            {"remaining",   ValueType::F64, offsetof(Snapshot, remaining),   8, "units"},
            {"leaseLeftMs", ValueType::U32, offsetof(Snapshot, leaseLeftMs), 4, "ms"},
            {"maxLeaseMs",  ValueType::U32, offsetof(Snapshot, maxLeaseMs),  4, "ms"},
            {"maxRate",     ValueType::F64, offsetof(Snapshot, maxRate),     8, "units/s"},
            {"maxIncrement",ValueType::F64, offsetof(Snapshot, maxIncrement),8, "units"},
        };
        return sd;
    }

    tether::control::JogController& jog_;
    NowFn nowMs_;
    std::string prefix_;
    std::deque<StructDescriptor> descriptors_;  ///< Stable descriptor storage
};

} // namespace tether::io::exposers
