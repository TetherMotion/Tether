#pragma once

/// @file FirmwareRetraction.hpp
/// @brief Firmware retraction state machine


namespace tether::klipper::klippy {

/// @brief Firmware retraction parameters.
struct FirmwareRetractionParams {
    double retractLength = 0.0;       ///< Retract length (mm)
    double retractSpeed = 20.0;       ///< Retract speed (mm/s)
    double unretractLength = 0.0;     ///< Additional unretract length (mm)
    double unretractSpeed = 10.0;     ///< Unretract speed (mm/s)
    double zHop = 0.0;                ///< Z hop height (mm)
};

/// @brief Firmware retraction state machine.
class FirmwareRetraction {
public:
    explicit FirmwareRetraction(FirmwareRetractionParams params = {})
        : params_(params) {}

    /// @brief Set the retraction parameters.
    void setParams(FirmwareRetractionParams params) { params_ = params; }

    /// @brief Get the current parameters.
    const FirmwareRetractionParams& params() const { return params_; }

    /// @brief Execute a retract (G10).
    /// @return The E-axis movement to apply (negative = retract).
    double retract() {
        isRetracted_ = true;
        return -params_.retractLength;
    }

    /// @brief Execute an unretract (G11).
    /// @return The E-axis movement to apply (positive = unretract).
    double unretract() {
        isRetracted_ = false;
        return params_.retractLength + params_.unretractLength;
    }

    /// @brief Check if currently retracted.
    bool isRetracted() const { return isRetracted_; }

    /// @brief Get the Z hop amount.
    double zHop() const { return isRetracted_ ? params_.zHop : 0.0; }

private:
    FirmwareRetractionParams params_;
    bool isRetracted_ = false;
};

} // namespace tether::klipper::klippy
