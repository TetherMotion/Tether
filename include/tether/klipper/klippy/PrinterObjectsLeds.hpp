#pragma once

/// @file PrinterObjectsLeds.hpp
/// @brief LED printer objects (led, dotstar, neopixel)

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

/// @brief Neopixel printer object.
class NeopixelObject : public PrinterObject {
public:
    explicit NeopixelObject(std::shared_ptr<objects::Neopixel> dev,
                            const std::string& name = "neopixel")
        : dev_(std::move(dev)), name_(name) {}

    std::string name() const override { return name_; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        if (dev_) {
            s.add("pixel_count", JsonValue(static_cast<int64_t>(dev_->numLeds())));
            // Chain data as a hex string
            std::ostringstream ss;
            for (int i = 0; i < dev_->numLeds(); ++i) {
                auto color = dev_->color(i);
                ss << std::hex << std::setfill('0')
                   << std::setw(2) << static_cast<int>(color.r)
                   << std::setw(2) << static_cast<int>(color.g)
                   << std::setw(2) << static_cast<int>(color.b)
                   << std::setw(2) << static_cast<int>(color.w);
            }
            s.add("color_data", JsonValue(ss.str()));
        }
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override {
        return {"pixel_count", "color_data"};
    }

private:
    std::shared_ptr<objects::Neopixel> dev_;
    std::string name_;
};

/// @brief LED printer object (led).
class LedObject : public PrinterObject {
public:
    explicit LedObject(const std::string& name) : name_(name) {}

    std::string name() const override { return name_; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        std::vector<JsonValue> color;
        for (double c : color_) color.emplace_back(c);
        s.add("color_data", JsonValue(std::vector<JsonValue>{JsonValue(color)}));
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override { return {"color_data"}; }

    void setColor(const std::array<double, 4>& c) { color_ = c; }

private:
    std::string name_;
    std::array<double, 4> color_ = {0, 0, 0, 0};
};

/// @brief Dotstar printer object (dotstar).
class DotstarObject : public PrinterObject {
public:
    explicit DotstarObject(const std::string& name) : name_(name) {}

    std::string name() const override { return name_; }

    std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const override {
        auto s = buildStatus(fields);
        std::vector<JsonValue> color;
        for (double c : color_) color.emplace_back(c);
        s.add("color_data", JsonValue(std::vector<JsonValue>{JsonValue(color)}));
        return std::move(s).take();
    }

    std::vector<std::string> availableFields() const override { return {"color_data"}; }

    void setColor(const std::array<double, 4>& c) { color_ = c; }

private:
    std::string name_;
    std::array<double, 4> color_ = {0, 0, 0, 0};
};

} // namespace tether::klipper::klippy
