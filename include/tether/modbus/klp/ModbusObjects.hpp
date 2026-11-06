/**
 * @file ModbusObjects.hpp
 * @brief Klipper printer objects backed by Modbus registers (plan §15):
 *
 *   [modbus_sensor <name>]  → ModbusSensorObject: read a profile
 *                            register; status exposes value/unit/ok/
 *                            error/age_ms. update() is driven by the
 *                            app (or a Poller) — status() never blocks.
 *
 *   [modbus_output <name>]  → ModbusOutputObject: writable value
 *                            (coil or holding register).
 *
 *   ModbusGcodes: extendedCommand handler for
 *     MODBUS_READ BUS=x REG=name
 *     MODBUS_WRITE BUS=x REG=name VALUE=v
 *     SET_MODBUS_OUTPUT NAME=x VALUE=0|1
 *     QUERY_MODBUS_OUTPUT NAME=x
 *
 * Header-only; compiled only when klipper + modbus coexist
 * (TETHER_MODBUS_KLIPPER set by cmake when the klipper target exists).
 */
#pragma once

#if defined(TETHER_MODBUS_KLIPPER) && !defined(ESP_PLATFORM)

#include <chrono>
#include <cstdio>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "tether/klipper/klippy/GCodeParser.hpp"
#include "tether/klipper/klippy/PrinterObject.hpp"
#include "tether/modbus/profile/Profile.hpp"

namespace tether::modbus::klp {

using tether::klipper::klippy::GcodeLine;
using tether::klipper::klippy::JsonValue;
using tether::klipper::klippy::PrinterObject;

namespace detail {
inline std::string fmtVal(double v) {
    char buf[48];
    if (v == double(int64_t(v)) && std::abs(v) < 1e15)
        std::snprintf(buf, sizeof buf, "%lld", (long long)v);
    else
        std::snprintf(buf, sizeof buf, "%.4g", v);
    return buf;
}
inline std::string named(const GcodeLine& l, const std::string& k) {
    auto it = l.namedParams.find(k);
    return it == l.namedParams.end() ? std::string{} : it->second;
}
} // namespace detail

// ---------------------------------------------------------------------------
// [modbus_sensor <name>] — read-only register sensor
// ---------------------------------------------------------------------------
class ModbusSensorObject final : public PrinterObject {
public:
    ModbusSensorObject(std::string name, BoundDevice& dev,
                       std::string regName)
        : name_("modbus_sensor " + name), dev_(dev),
          reg_(std::move(regName)) {}

    std::string name() const override { return name_; }

    /// Called by the app/poller tick — performs the (possibly blocking)
    /// device read and caches the result for status().
    void update() {
        auto v = dev_.read(reg_);
        std::lock_guard g(m_);
        if (v) {
            value_ = *v;
            err_.clear();
            lastOk_ = std::chrono::steady_clock::now();
        } else {
            err_ = v.error().format();
        }
    }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        std::lock_guard g(m_);
        s.add("value", JsonValue(value_));
        s.add("ok", JsonValue(err_.empty()));
        if (!err_.empty()) s.add("error", JsonValue(err_));
        if (const auto* d = dev_.profile().find(reg_);
            d && !d->unit.empty())
            s.add("unit", JsonValue(d->unit));
        const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - lastOk_).count();
        s.add("age_ms", JsonValue(int64_t(age)));
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override {
        return {"value", "ok", "error", "unit", "age_ms"};
    }

private:
    std::string   name_, reg_;
    BoundDevice&  dev_;
    double        value_ = 0;
    std::string   err_ = "never read";
    std::chrono::steady_clock::time_point lastOk_{};
    mutable std::mutex m_;
};

// ---------------------------------------------------------------------------
// [modbus_output <name>] — writable coil/holding register
// ---------------------------------------------------------------------------
class ModbusOutputObject final : public PrinterObject {
public:
    ModbusOutputObject(std::string name, BoundDevice& dev,
                       std::string regName)
        : name_("modbus_output " + name), dev_(dev),
          reg_(std::move(regName)) {}

    std::string name() const override { return name_; }

    Expected<void> set(double v) {
        auto r = dev_.write(reg_, v);
        std::lock_guard g(m_);
        if (r) { value_ = v; err_.clear(); }
        else err_ = r.error().format();
        return r;
    }
    double value() const {
        std::lock_guard g(m_);
        return value_;
    }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        std::lock_guard g(m_);
        s.add("value", JsonValue(value_));
        s.add("ok", JsonValue(err_.empty()));
        if (!err_.empty()) s.add("error", JsonValue(err_));
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override {
        return {"value", "ok", "error"};
    }

private:
    std::string  name_, reg_;
    BoundDevice& dev_;
    double       value_ = 0;
    std::string  err_;
    mutable std::mutex m_;
};

// ---------------------------------------------------------------------------
// G-code extended commands
// ---------------------------------------------------------------------------
/**
 * Usage: wire as the klipper `extendedCommand` callback (or chain):
 *   callbacks.extendedCommand = gcodes.handler();
 */
class ModbusGcodes {
public:
    /// `devices` maps BUS= names; `outputs` maps NAME= for SET/QUERY.
    ModbusGcodes(std::map<std::string, BoundDevice*> devices,
                 std::map<std::string, ModbusOutputObject*> outputs,
                 std::function<void(const std::string&)> out)
        : devs_(std::move(devices)), outs_(std::move(outputs)),
          out_(std::move(out)) {}

    bool operator()(const GcodeLine& line) const {
        if (line.code == "MODBUS_READ") {
            auto* d = dev(detail::named(line, "BUS"));
            const auto reg = detail::named(line, "REG");
            if (!d || reg.empty()) return fail("usage: MODBUS_READ BUS=x REG=name");
            auto v = d->read(reg);
            say(v ? "modbus " + reg + " = " + detail::fmtVal(*v)
                  : "modbus read failed: " + v.error().format());
            return true;
        }
        if (line.code == "MODBUS_WRITE") {
            auto* d = dev(detail::named(line, "BUS"));
            const auto reg = detail::named(line, "REG");
            const auto vs = detail::named(line, "VALUE");
            if (!d || reg.empty() || vs.empty())
                return fail("usage: MODBUS_WRITE BUS=x REG=name VALUE=v");
            auto r = d->write(reg, std::stod(vs));
            say(r ? "ok" : "modbus write failed: " + r.error().format());
            return true;
        }
        if (line.code == "SET_MODBUS_OUTPUT") {
            auto* o = out(detail::named(line, "NAME"));
            const auto vs = detail::named(line, "VALUE");
            if (!o || vs.empty())
                return fail("usage: SET_MODBUS_OUTPUT NAME=x VALUE=0|1");
            auto r = o->set(std::stod(vs));
            say(r ? "ok" : "modbus output failed: " + r.error().format());
            return true;
        }
        if (line.code == "QUERY_MODBUS_OUTPUT") {
            auto* o = out(detail::named(line, "NAME"));
            if (!o) return fail("usage: QUERY_MODBUS_OUTPUT NAME=x");
            say("modbus_output " + detail::named(line, "NAME") + " = " +
                detail::fmtVal(o->value()));
            return true;
        }
        return false;
    }

    std::function<bool(const GcodeLine&)> handler() {
        return [this](const GcodeLine& l) { return (*this)(l); };
    }

private:
    BoundDevice* dev(const std::string& n) const {
        auto it = devs_.find(n);
        return it == devs_.end() ? nullptr : it->second;
    }
    ModbusOutputObject* out(const std::string& n) const {
        auto it = outs_.find(n);
        return it == outs_.end() ? nullptr : it->second;
    }
    void say(const std::string& s) const { if (out_) out_(s); }
    bool fail(const std::string& m) const { say(m); return true; }

    std::map<std::string, BoundDevice*>        devs_;
    std::map<std::string, ModbusOutputObject*> outs_;
    std::function<void(const std::string&)>    out_;
};

} // namespace tether::modbus::klp

#endif // TETHER_MODBUS_KLIPPER && !ESP_PLATFORM
