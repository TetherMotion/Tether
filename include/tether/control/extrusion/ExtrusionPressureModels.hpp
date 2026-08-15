/**
 * @file ExtrusionPressureModels.hpp
 * @brief Position-offset pressure-advance models for non-Newtonian extrusion.
 *
 * @details
 * The classic Klipper pressure-advance law is the Newtonian limit:
 *
 *   δe_linear = PA · v_e        (v_e = extruder-axis velocity, mm/s)
 *
 * which corresponds to a linear (Hagen-Poiseuille) pressure/flow relation
 * P = C·Q and a melt compressibility βV_m, giving δe = βV_m·C·Q/A_f = K·Q,
 * with Q = v_e·A_filament. Differentiating gives the familiar δe = PA·v_e
 * with PA = βV_m·C·A_filament/A_f.
 *
 * For a power-law fluid, P = C_n·Q^n, so the equivalent position-offset form is
 *
 *   δe_power_law = K_base · Q^n = K_base · (v_e · A_filament)^n
 *
 * where K_base = βV_m·C_n/A_f has units [filament-mm / (mm³/s)^n]. This is
 * mathematically identical to the flow-rate form K_dynamic·Q^(n-1)·dQ/dt but
 * avoids numerical differentiation and the Q→0 singularity (Q^n → 0 as Q→0
 * for n>0). See jade-x-23-simon-baz.md §0 caveat 2 and §4.
 *
 * For Cross-WLF, the offset reads the pressure from a precomputed LUT:
 *
 *   δe_cross_wlf = (βV_m / A_f) · P_LUT(Q, T_est)
 *
 * Both models share a centered moving-average smoother on the volumetric
 * flow Q so that the offset does not react to per-sample jitter.
 *
 * The models are stateless-per-move: callers feed a sequence of (v_e, T_est)
 * samples and receive the offset-adjusted extruder position series. The
 * Newtonian limit (n=1) of the power-law model reduces exactly to the linear
 * PA offset, which is verified by regression tests.
 *
 * @note The translator uses filament-side flow Q_f = v_e·A_filament so that
 *       no slicer line-width / layer-height knowledge is needed inside the
 *       models. The docs describe the mass-conservation equivalence with the
 *       width·height·v_xy formulation.
 */

#pragma once

#include "tether/control/extrusion/PowerLawRheology.hpp"
#include "tether/control/extrusion/PressureFlowLut.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace tether::control::extrusion {

/// @brief Compensation model selector. `Linear` reproduces classic Klipper PA.
enum class ExtrusionCompensationModel {
    Off,       ///< No compensation (δe = 0)
    Linear,    ///< Newtonian / classic PA: δe = PA · v_e
    PowerLaw,  ///< Power-law: δe = K_base · (v_e·A_f)^n
    CrossWlf   ///< Cross-WLF LUT: δe = (βV_m/A_f) · P_LUT(Q, T)
};

/// @brief Filament cross-section parameters shared by all models.
struct FilamentGeometry {
    double filamentDiameterMm = 1.75; ///< Filament diameter [mm]
    /// @brief Filament cross-sectional area [mm²] = π·d²/4.
    double areaMm2() const {
        return M_PI * filamentDiameterMm * filamentDiameterMm / 4.0;
    }
};

/// @brief Power-law pressure-advance position-offset model.
///
/// δe = K_base · Q^n, Q = v_e · A_filament
/// Newtonian limit n=1 with K_base = PA·A_filament/A_f reduces to δe = PA·v_e.
class PowerLawPressureAdvance {
public:
    /// @brief Model parameters.
    struct Params {
        /// K_base = βV_m·C_n/A_f [filament-mm / (mm³/s)^n].
        double baseGain = 0.0;
        /// Flow index n [-] (1 = Newtonian).
        double flowIndex = 1.0;
        /// Smoothing window in seconds (0 = no smoothing).
        double smoothTime = 0.040;
        /// Maximum absolute compensation [mm] (safety clamp).
        double maxCompensation = 0.5;
    };

    explicit PowerLawPressureAdvance(Params params, FilamentGeometry filament)
        : params_(params), filament_(filament) {}
    explicit PowerLawPressureAdvance(Params params)
        : params_(params), filament_() {}
    PowerLawPressureAdvance()
        : params_(), filament_() {}

    const Params& params() const { return params_; }
    void setParams(Params p) { params_ = p; }
    const FilamentGeometry& filament() const { return filament_; }
    void setFilament(FilamentGeometry f) { filament_ = f; }

    /// @brief Compute the position offset for a single (already-smoothed)
    /// extruder velocity sample. Pure function, no state.
    /// @param velMmPerS Extruder-axis velocity [mm/s].
    /// @return Position offset δe [mm] (clamped to ±maxCompensation).
    double offsetForVelocity(double velMmPerS) const {
        const double Q = velMmPerS * filament_.areaMm2();
        if (Q <= 0.0 || params_.baseGain <= 0.0) return 0.0;
        double off = params_.baseGain * std::pow(Q, params_.flowIndex);
        if (off > params_.maxCompensation) off = params_.maxCompensation;
        if (off < -params_.maxCompensation) off = -params_.maxCompensation;
        return off;
    }

    /// @brief Smooth a velocity series with a centered moving average over
    /// `smoothTime`, using `sampleIntervalSec` as the per-sample spacing.
    /// @returns Smoothed velocity series (same length as input).
    std::vector<double> smoothVelocity(
        const std::vector<double>& velMmPerS,
        double sampleIntervalSec) const {
        std::vector<double> out(velMmPerS.size(), 0.0);
        if (velMmPerS.empty()) return out;
        if (params_.smoothTime <= 0.0 || sampleIntervalSec <= 0.0) {
            return velMmPerS;
        }
        const int n = static_cast<int>(velMmPerS.size());
        int half = std::max(1, static_cast<int>(params_.smoothTime /
                          sampleIntervalSec / 2.0));
        for (int i = 0; i < n; ++i) {
            double sum = 0.0;
            int count = 0;
            for (int j = std::max(0, i - half);
                 j <= std::min(n - 1, i + half); ++j) {
                sum += velMmPerS[static_cast<size_t>(j)];
                ++count;
            }
            out[static_cast<size_t>(i)] = (count > 0) ? sum / count : 0.0;
        }
        return out;
    }

    /// @brief Compute the per-sample position-offset series for a velocity
    /// series. Applies smoothing then the offset law.
    /// @param velMmPerS Raw extruder velocity per sample [mm/s].
    /// @param sampleIntervalSec Per-sample interval [s].
    /// @returns Offset series δe[i] [mm] (same length as input).
    std::vector<double> offsetSeries(const std::vector<double>& velMmPerS,
                                     double sampleIntervalSec) const {
        auto smoothed = smoothVelocity(velMmPerS, sampleIntervalSec);
        std::vector<double> off(smoothed.size(), 0.0);
        for (size_t i = 0; i < smoothed.size(); ++i) {
            off[i] = offsetForVelocity(smoothed[i]);
        }
        return off;
    }

private:
    Params params_;
    FilamentGeometry filament_;
};

/// @brief Cross-WLF pressure-advance position-offset model.
///
/// δe = (βV_m / A_f) · P_LUT(Q, T_est), Q = v_e · A_filament.
class CrossWlfPressureAdvance {
public:
    struct Params {
        /// βV_m / A_f [mm/Pa] — melt compressibility over filament area.
        double compressibilityOverArea = 0.0;
        /// Smoothing window in seconds (0 = no smoothing).
        double smoothTime = 0.040;
        /// Maximum absolute compensation [mm] (safety clamp).
        double maxCompensation = 0.5;
        /// Default melt temperature used when no observer is wired.
        double defaultTempC = 210.0;
    };

    explicit CrossWlfPressureAdvance(std::shared_ptr<PressureFlowLut> lut,
                                     Params params, FilamentGeometry filament)
        : lut_(std::move(lut)), params_(params), filament_(filament) {}
    explicit CrossWlfPressureAdvance(std::shared_ptr<PressureFlowLut> lut,
                                     Params params)
        : lut_(std::move(lut)), params_(params), filament_() {}
    explicit CrossWlfPressureAdvance(std::shared_ptr<PressureFlowLut> lut)
        : lut_(std::move(lut)), params_(), filament_() {}

    const Params& params() const { return params_; }
    void setParams(Params p) { params_ = p; }
    const FilamentGeometry& filament() const { return filament_; }
    void setFilament(FilamentGeometry f) { filament_ = f; }
    void setLut(std::shared_ptr<PressureFlowLut> lut) { lut_ = std::move(lut); }

    /// @brief Compute the position offset for a single (already-smoothed)
    /// extruder velocity and melt-temperature estimate.
    double offsetForVelocity(double velMmPerS, double tempC) const {
        if (!lut_ || lut_->empty() ||
            params_.compressibilityOverArea <= 0.0) {
            return 0.0;
        }
        const double Q = velMmPerS * filament_.areaMm2();
        if (Q <= 0.0) return 0.0;
        const double P = lut_->pressure(Q, tempC);
        double off = params_.compressibilityOverArea * P;
        if (off > params_.maxCompensation) off = params_.maxCompensation;
        if (off < -params_.maxCompensation) off = -params_.maxCompensation;
        return off;
    }

    /// @brief Smooth a velocity series (centered moving average).
    std::vector<double> smoothVelocity(
        const std::vector<double>& velMmPerS,
        double sampleIntervalSec) const {
        std::vector<double> out(velMmPerS.size(), 0.0);
        if (velMmPerS.empty()) return out;
        if (params_.smoothTime <= 0.0 || sampleIntervalSec <= 0.0) {
            return velMmPerS;
        }
        const int n = static_cast<int>(velMmPerS.size());
        int half = std::max(1, static_cast<int>(params_.smoothTime /
                          sampleIntervalSec / 2.0));
        for (int i = 0; i < n; ++i) {
            double sum = 0.0;
            int count = 0;
            for (int j = std::max(0, i - half);
                 j <= std::min(n - 1, i + half); ++j) {
                sum += velMmPerS[static_cast<size_t>(j)];
                ++count;
            }
            out[static_cast<size_t>(i)] = (count > 0) ? sum / count : 0.0;
        }
        return out;
    }

    /// @brief Per-sample offset series given velocity and temperature series.
    /// @param tempC Per-sample melt-temperature estimate [°C] (same length).
    std::vector<double> offsetSeries(const std::vector<double>& velMmPerS,
                                     const std::vector<double>& tempC,
                                     double sampleIntervalSec) const {
        auto smoothed = smoothVelocity(velMmPerS, sampleIntervalSec);
        std::vector<double> off(smoothed.size(), 0.0);
        for (size_t i = 0; i < smoothed.size(); ++i) {
            double T = (i < tempC.size()) ? tempC[i] : params_.defaultTempC;
            off[i] = offsetForVelocity(smoothed[i], T);
        }
        return off;
    }

private:
    std::shared_ptr<PressureFlowLut> lut_;
    Params params_;
    FilamentGeometry filament_;
};

} // namespace tether::control::extrusion
