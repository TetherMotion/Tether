#pragma once

/// @file PrinterObjectsPeripherals.hpp
/// @brief Peripheral printer objects (digital_out, pwm_out, spi, i2c, endstop, trsync)

#include "tether/klipper/klippy/KlippyUdsServer.hpp"
#include "tether/klipper/objects/Peripherals.hpp"

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace tether::klipper::klippy {

/// @brief Digital output pin printer object.
class DigitalOutObject : public PrinterObject {
public:
    explicit DigitalOutObject(std::shared_ptr<objects::DigitalOut> dev,
                              const std::string& name = "output_pin")
        : dev_(std::move(dev)), name_(name) {}

    std::string name() const override { return name_; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        if (dev_) {
            s.add("value", JsonValue(static_cast<double>(dev_->value())));
        }
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override {
        return {"value"};
    }

private:
    std::shared_ptr<objects::DigitalOut> dev_;
    std::string name_;
};

/// @brief PWM output pin printer object.
class PwmOutObject : public PrinterObject {
public:
    explicit PwmOutObject(std::shared_ptr<objects::PwmOut> dev,
                          const std::string& name = "pwm_tool")
        : dev_(std::move(dev)), name_(name) {}

    std::string name() const override { return name_; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        if (dev_) {
            s.add("value", JsonValue(dev_->dutyDouble()));
        }
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override {
        return {"value"};
    }

private:
    std::shared_ptr<objects::PwmOut> dev_;
    std::string name_;
};

/// @brief SPI bus printer object.
class SpiObject : public PrinterObject {
public:
    explicit SpiObject(std::shared_ptr<objects::Spi> dev,
                       const std::string& name = "spi_bus")
        : dev_(std::move(dev)), name_(name) {}

    std::string name() const override { return name_; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        if (dev_) {
            s.add("oid", JsonValue(static_cast<int64_t>(dev_->oid())));
            s.add("bus", JsonValue(name_));
            s.add("transfer_count", JsonValue(static_cast<int64_t>(transferCount_)));
            s.add("errors", JsonValue(static_cast<int64_t>(errorCount_)));
        }
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override {
        return {"oid", "bus", "transfer_count", "errors"};
    }

    void incrementTransferCount() { transferCount_++; }
    void incrementErrorCount() { errorCount_++; }

private:
    std::shared_ptr<objects::Spi> dev_;
    std::string name_;
    mutable uint64_t transferCount_ = 0;
    mutable uint64_t errorCount_ = 0;
};

/// @brief I2C bus printer object.
class I2cObject : public PrinterObject {
public:
    explicit I2cObject(std::shared_ptr<objects::I2c> dev,
                       const std::string& name = "i2c_bus")
        : dev_(std::move(dev)), name_(name) {}

    std::string name() const override { return name_; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        if (dev_) {
            s.add("oid", JsonValue(static_cast<int64_t>(dev_->oid())));
            s.add("bus", JsonValue(name_));
            s.add("transfer_count", JsonValue(static_cast<int64_t>(transferCount_)));
            s.add("errors", JsonValue(static_cast<int64_t>(errorCount_)));
        }
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override {
        return {"oid", "bus", "transfer_count", "errors"};
    }

    void incrementTransferCount() { transferCount_++; }
    void incrementErrorCount() { errorCount_++; }

private:
    std::shared_ptr<objects::I2c> dev_;
    std::string name_;
    mutable uint64_t transferCount_ = 0;
    mutable uint64_t errorCount_ = 0;
};

/// @brief Endstop printer object (per-endstop).
class EndstopObject : public PrinterObject {
public:
    explicit EndstopObject(std::shared_ptr<objects::Endstop> dev,
                           const std::string& name)
        : dev_(std::move(dev)), name_(name) {}

    std::string name() const override { return name_; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        if (dev_) {
            s.add("state", JsonValue(std::string(dev_->triggered() ? "TRIGGERED" : "open")));
            s.add("state_int", JsonValue(dev_->triggered() ? 1 : 0));
        }
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override {
        return {"state", "state_int"};
    }

private:
    std::shared_ptr<objects::Endstop> dev_;
    std::string name_;
};

/// @brief TRsync printer object.
class TrsyncObject : public PrinterObject {
public:
    explicit TrsyncObject(std::shared_ptr<objects::Trsync> dev,
                          const std::string& name = "trsync")
        : dev_(std::move(dev)), name_(name) {}

    std::string name() const override { return name_; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        if (dev_) {
            s.add("state", JsonValue(std::string(
                dev_->state() == objects::TrsyncState::Armed ? "armed" :
                dev_->state() == objects::TrsyncState::Triggered ? "triggered" :
                dev_->state() == objects::TrsyncState::Sent ? "sent" : "idle")));
        }
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override {
        return {"state"};
    }

private:
    std::shared_ptr<objects::Trsync> dev_;
    std::string name_;
};

} // namespace tether::klipper::klippy
