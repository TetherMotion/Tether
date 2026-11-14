/**
 * @file ThermalCurve.hpp
 * @brief Industrial temperature-curve presets and process-window
 *        validation for the oven control stack.
 *
 * @details
 * Soldering (reflow/wave/selective), brazing and annealing processes
 * specify the thermal cycle as a temperature-vs-time curve plus a
 * process window: peak temperature, time above liquidus (TAL),
 * preheat soak band/time, and maximum ramp rates. This header provides:
 *
 *  - `ThermalCurvePreset` — named, realistic curve templates returned
 *    as `TemperatureProfile` objects (ramp/soak segments; arbitrary
 *    point lists also work via `TemperatureProfile::fromPoints`).
 *  - `ThermalCurveSpec` — the acceptance window a curve must satisfy.
 *  - `analyzeProfile()` — exact piecewise-linear analysis of a
 *    `TemperatureProfile` against a spec (peak, TAL, soak time, max
 *    heat/cool ramps, violations).
 *  - `analyzeSamples()` — the same metrics for a measured/simulated
 *    trajectory given as (t, T) samples.
 *
 * The analysis is reference-side (what the profile *commands*); pair
 * it with `OvenController`/`OvenGradientController` tracking error to
 * judge what the process *experienced* — `analyzeSamples()` accepts a
 * recorded trajectory for that.
 */

#pragma once

#include "tether/control/thermal/TemperatureProfile.hpp"

#include <string>
#include <vector>

namespace tether::control::thermal {

/**@brief Named industrial thermal-curve presets. */
enum class ThermalCurvePreset {
    ReflowSac305,    /**< Lead-free SAC305 reflow (peak ~245 °C) */
    ReflowSnPb,      /**< Sn63Pb37 reflow (peak ~215 °C) */
    WaveSolder,      /**< Wave soldering dwell curve */
    SelectiveSolder, /**< Selective soldering (longer preheat) */
    Brazing,         /**< Silver brazing cycle (~620 °C) */
    Anneal           /**< Stress-relief anneal (~370 °C) */
};

/**@brief Human-readable preset name (UI/logging). */
const char* thermalCurvePresetName(ThermalCurvePreset p);

/**@brief Number of presets (for UI cycling). */
constexpr size_t thermalCurvePresetCount = 6;

/**@brief Build the reference profile for a preset. */
TemperatureProfile presetProfile(ThermalCurvePreset p);

/**@brief Acceptance window for a thermal curve. A field <= 0 disables
 *        that check (except liquidusC, where 0 disables TAL/soak band
 *        logic entirely). */
struct ThermalCurveSpec {
    double liquidusC{0.0};          /**< Solder liquidus [°C]; TAL +
                                         soak band measured against it */
    double soakMinC{0.0};           /**< Preheat soak band lower bound */
    double soakMaxC{0.0};           /**< Preheat soak band upper bound */
    double soakMinS{0.0};           /**< Min time inside the soak band */
    double peakMinC{0.0};           /**< Allowed peak range */
    double peakMaxC{0.0};
    double talMinS{0.0};            /**< Time-above-liquidus window */
    double talMaxS{0.0};
    double maxHeatRampCPerS{0.0};   /**< Max heating slope (<=0 off) */
    double maxCoolRampCPerS{0.0};   /**< Max cooling slope magnitude */
};

/**@brief The process-window spec a preset is designed to satisfy. */
ThermalCurveSpec specForPreset(ThermalCurvePreset p);

/**@brief Analysis result: measured metrics + spec violations. */
struct ThermalCurveReport {
    double peakC{0.0};
    double durationS{0.0};
    double talS{0.0};        /**< Time above liquidusC (0 if disabled) */
    double soakS{0.0};       /**< Time inside [soakMinC, soakMaxC] */
    double heatRampMax{0.0}; /**< Max positive slope [°C/s] */
    double coolRampMax{0.0}; /**< Max negative slope magnitude [°C/s] */
    std::vector<std::string> violations;

    bool pass() const { return violations.empty(); }
};

/**@brief Analyze a reference profile against a spec. Exact for the
 *        piecewise-linear profile (no sampling error). */
ThermalCurveReport analyzeProfile(const TemperatureProfile& profile,
                                  const ThermalCurveSpec& spec);

/**@brief Analyze a sampled trajectory. `times`/`temps` must have equal
 *        size >= 2 with strictly increasing times; returns an empty
 *        report (peak=0, one violation) otherwise. */
ThermalCurveReport analyzeSamples(const std::vector<double>& times,
                                  const std::vector<double>& temps,
                                  const ThermalCurveSpec& spec);

} // namespace tether::control::thermal
