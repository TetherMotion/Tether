#pragma once

/// @file PrinterObjectsTmc.hpp
/// @brief TMC driver printer objects (tmc_uart, tmc_driver)

#include "tether/klipper/klippy/KlippyUdsServer.hpp"
#include "tether/klipper/objects/TmcUart.hpp"

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace tether::klipper::klippy {

/// @brief TMC UART printer object.
class TmcUartObject : public PrinterObject {
public:
    explicit TmcUartObject(std::shared_ptr<objects::TmcUart> dev,
                           const std::string& name = "tmc_uart")
        : dev_(std::move(dev)), name_(name) {}

    std::string name() const override { return name_; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        std::map<std::string, JsonValue> result;
        auto add = [&](const std::string& n, JsonValue v) {
            if (fields.empty() || std::find(fields.begin(), fields.end(), n) != fields.end())
                result[n] = std::move(v);
        };
        if (dev_) {
            // Read actual TMC register values
            // DRV_STATUS register (0x6F) contains stall guard and other status
            int64_t drvStatus = 0;
            int64_t mcuPhaseOffset = 0;
            try {
                auto val = dev_->readRegister(0x6F); // DRV_STATUS
                if (val >= 0) drvStatus = val;
            } catch (...) {}

            // Run current from IHOLD_IRUN register (0x10)
            // Bits 0-3: IHOLD, bits 4-7: IRUN
            double runCurrent = 0.0;
            double holdCurrent = 0.0;
            try {
                auto val = dev_->readRegister(0x10); // IHOLD_IRUN
                if (val >= 0) {
                    int ihold = static_cast<int>(val & 0x0F);
                    int irun = static_cast<int>((val >> 4) & 0x0F);
                    // Convert to mA (simplified: current = (irun+1) * 32mA for default)
                    runCurrent = static_cast<double>(irun + 1) * 32.0;
                    holdCurrent = static_cast<double>(ihold + 1) * 32.0;
                }
            } catch (...) {}

            add("mcu_phase_offset", JsonValue(mcuPhaseOffset));
            add("drv_status", JsonValue(drvStatus));
            add("run_current", JsonValue(runCurrent));
            add("hold_current", JsonValue(holdCurrent));
        }
        return result;
    }

    std::vector<std::string> availableFields() const override {
        return {"mcu_phase_offset", "drv_status", "run_current", "hold_current"};
    }

private:
    std::shared_ptr<objects::TmcUart> dev_;
    std::string name_;
};

/// @brief TMC driver printer object (tmc2209, tmc2208, tmc5160, etc.).
class TmcDriverObject : public PrinterObject {
public:
    explicit TmcDriverObject(const std::string& name) : name_(name) {}

    std::string name() const override { return name_; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        std::map<std::string, JsonValue> result;
        auto add = [&](const std::string& k, JsonValue v) {
            if (fields.empty() || std::find(fields.begin(), fields.end(), k) != fields.end())
                result[k] = std::move(v);
        };
        add("mcu_phase_offset", JsonValue(static_cast<int64_t>(mcuPhaseOffset_)));
        add("run_current", JsonValue(runCurrent_));
        add("hold_current", JsonValue(holdCurrent_));
        add("drv_status", JsonValue(static_cast<int64_t>(drvStatus_)));
        add("temperature", JsonValue(temperature_));
        return result;
    }

    std::vector<std::string> availableFields() const override {
        return {"mcu_phase_offset", "run_current", "hold_current",
                "drv_status", "temperature"};
    }

    void setRunCurrent(double c) { runCurrent_ = c; }
    void setHoldCurrent(double c) { holdCurrent_ = c; }
    void setDrvStatus(uint32_t s) { drvStatus_ = s; }
    void setTemperature(double t) { temperature_ = t; }

private:
    std::string name_;
    int mcuPhaseOffset_ = 0;
    double runCurrent_ = 0.0;
    double holdCurrent_ = 0.0;
    uint32_t drvStatus_ = 0;
    double temperature_ = 0.0;
};

} // namespace tether::klipper::klippy
