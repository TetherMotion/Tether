/// @file KlippyAutotuningBridge.hpp
/// @brief Bridge between Klipper heater/accelerometer interfaces and the
///        Tether autotuning framework.
///
/// This bridge allows Klipper's PID_CALIBRATE / M303 and resonance testing
/// commands to use the full Tether autotuning framework instead of any
/// inline autotuning code.  All applicable Tether autotuning methods are
/// supported:
///
///   - Relay feedback (Åström-Hägglund) — online
///   - Ziegler-Nichols (step response & ultimate cycle) — offline
///   - Cohen-Coon — offline
///   - Lambda / IMC — offline
///   - SIMC (Skogestad) — offline
///   - AMIGO — offline
///   - Tyreus-Luyben — offline
///   - Chien-Hrones-Reswick — offline
///   - Lopez (ITAE/IAE/ISE) — offline
///
/// For resonance / input-shaper calibration the bridge uses the Tether
/// FrequencyResponseAnalyzer and ChirpGenerator from the identification
/// framework.

#pragma once

#include <tether/klipper/objects/Thermal.hpp>
#include <tether/klipper/objects/Peripherals.hpp>

// Tether autotuning framework (classical methods)
#include <tether/control/autotuning/AutotuningFramework.hpp>
#include <tether/control/autotuning/classical/Common.hpp>
#include <tether/control/autotuning/classical/AstromHagglundRelay.hpp>
#include <tether/control/autotuning/classical/ZieglerNicholsStepResponse.hpp>
#include <tether/control/autotuning/classical/ZieglerNicholsUltimateCycle.hpp>
#include <tether/control/autotuning/classical/CohenCoon.hpp>
#include <tether/control/autotuning/classical/LambdaTuning.hpp>
#include <tether/control/autotuning/classical/SIMCMethod.hpp>
#include <tether/control/autotuning/classical/AMIGOMethod.hpp>
#include <tether/control/autotuning/classical/TyreusLuyben.hpp>
#include <tether/control/autotuning/classical/ChienHronesReswick.hpp>
#include <tether/control/autotuning/classical/LopezMethod.hpp>

// Tether identification framework (for resonance testing)
#include <tether/identification/FrequencyIdentification.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace tether::klipper::klippy {

// ============================================================================
// Type aliases for the Tether autotuning framework
// ============================================================================

using ::Control::Autotuning::PIDGains;
using ::Control::Autotuning::PIDForm;
using ::Control::Autotuning::FOPDTModel;
using ::Control::Autotuning::SOPDTModel;
using ::Control::Autotuning::AstromHagglundRelay;
using ::Control::Autotuning::ZieglerNicholsStepResponse;
using ::Control::Autotuning::ZieglerNicholsUltimateCycle;
using ::Control::Autotuning::CohenCoon;
using ::Control::Autotuning::LambdaTuning;
using ::Control::Autotuning::SIMCMethod;
using ::Control::Autotuning::AMIGOMethod;
using ::Control::Autotuning::TyreusLuyben;
using ::Control::Autotuning::ChienHronesReswick;
using ::Control::Autotuning::LopezMethod;
using ::Control::Autotuning::ProcessIdentification;

// ============================================================================
// Tuning method enumeration (mirrors Tether's available methods)
// ============================================================================

/// @brief PID autotuning method selector.
enum class AutotuneMethod {
    RelayFeedback,          ///< Åström-Hägglund relay feedback (online)
    ZieglerNicholsStep,     ///< Z-N step response (offline, needs step data)
    ZieglerNicholsUltimate, ///< Z-N ultimate cycle (offline, needs Ku/Tu)
    CohenCoon,              ///< Cohen-Coon (offline, needs FOPDT model)
    Lambda,                 ///< Lambda / IMC tuning (offline)
    SIMC,                   ///< Skogestad SIMC (offline)
    AMIGO,                  ///< AMIGO robust tuning (offline)
    TyreusLuyben,           ///< Conservative ultimate-cycle tuning (offline)
    ChienHronesReswick,     ///< CHR with overshoot spec (offline)
    LopezITAE,              ///< ITAE-optimal tuning (offline)
    LopezIAE,               ///< IAE-optimal tuning (offline)
    LopezISE,               ///< ISE-optimal tuning (offline)
};

/// @brief Convert method enum to string name.
inline std::string autotuneMethodName(AutotuneMethod m) {
    switch (m) {
        case AutotuneMethod::RelayFeedback:          return "relay_feedback";
        case AutotuneMethod::ZieglerNicholsStep:     return "ziegler_nichols_step";
        case AutotuneMethod::ZieglerNicholsUltimate: return "ziegler_nichols_ultimate";
        case AutotuneMethod::CohenCoon:              return "cohen_coon";
        case AutotuneMethod::Lambda:                 return "lambda";
        case AutotuneMethod::SIMC:                   return "simc";
        case AutotuneMethod::AMIGO:                  return "amigo";
        case AutotuneMethod::TyreusLuyben:           return "tyreus_luyben";
        case AutotuneMethod::ChienHronesReswick:     return "chien_hrones_reswick";
        case AutotuneMethod::LopezITAE:              return "lopez_itae";
        case AutotuneMethod::LopezIAE:               return "lopez_iae";
        case AutotuneMethod::LopezISE:               return "lopez_ise";
    }
    return "unknown";
}

/// @brief Parse a method name string to enum.
/// @return Method enum, or RelayFeedback if unknown.
inline AutotuneMethod parseAutotuneMethod(const std::string& name) {
    if (name == "relay_feedback" || name == "relay")          return AutotuneMethod::RelayFeedback;
    if (name == "ziegler_nichols_step" || name == "zn_step")  return AutotuneMethod::ZieglerNicholsStep;
    if (name == "ziegler_nichols_ultimate" || name == "zn_ultimate") return AutotuneMethod::ZieglerNicholsUltimate;
    if (name == "cohen_coon")                                  return AutotuneMethod::CohenCoon;
    if (name == "lambda" || name == "imc")                    return AutotuneMethod::Lambda;
    if (name == "simc")                                        return AutotuneMethod::SIMC;
    if (name == "amigo")                                       return AutotuneMethod::AMIGO;
    if (name == "tyreus_luyben")                               return AutotuneMethod::TyreusLuyben;
    if (name == "chien_hrones_reswick" || name == "chr")      return AutotuneMethod::ChienHronesReswick;
    if (name == "lopez_itae" || name == "itae")               return AutotuneMethod::LopezITAE;
    if (name == "lopez_iae" || name == "iae")                 return AutotuneMethod::LopezIAE;
    if (name == "lopez_ise" || name == "ise")                 return AutotuneMethod::LopezISE;
    return AutotuneMethod::RelayFeedback;
}

/// @brief Get a list of all supported autotuning method names.
inline std::vector<std::string> listAutotuneMethods() {
    return {
        "relay_feedback", "ziegler_nichols_step", "ziegler_nichols_ultimate",
        "cohen_coon", "lambda", "simc", "amigo", "tyreus_luyben",
        "chien_hrones_reswick", "lopez_itae", "lopez_iae", "lopez_ise"
    };
}

// ============================================================================
// PID Autotuning Result
// ============================================================================

/// @brief Result of a PID autotuning run.
struct PidAutotuneResult {
    bool success = false;
    double Kp = 0.0;
    double Ki = 0.0;
    double Kd = 0.0;
    double Ku = 0.0;  ///< Ultimate gain (if identified)
    double Tu = 0.0;  ///< Ultimate period (if identified)
    std::string method;
    std::string message;
};

// ============================================================================
// Heater Autotuning Bridge
// ============================================================================

/// @brief Bridge to run Tether autotuning methods on a Klipper Heater.
///
/// The bridge wraps a Heater object and provides:
///   - Online relay-feedback autotuning (drives the heater in real time)
///   - Offline autotuning from collected step-response data
///   - Offline autotuning from a known FOPDT model
///   - Offline autotuning from known ultimate parameters (Ku, Tu)
class HeaterAutotuneBridge {
public:
    /// @brief Construct the bridge for a specific heater.
    /// @param heater The heater to tune (must have sensor + PWM callbacks wired).
    /// @param settingsPid Reference to settings PID params (to store results).
    HeaterAutotuneBridge(objects::Heater& heater,
                          double& settingsKp, double& settingsKi, double& settingsKd)
        : heater_(heater)
        , settingsKp_(settingsKp)
        , settingsKi_(settingsKi)
        , settingsKd_(settingsKd) {}

    // ------------------------------------------------------------------
    // Online relay-feedback autotuning
    // ------------------------------------------------------------------

    /// @brief Run relay-feedback autotuning on the heater.
    /// @param targetTemp Target temperature in °C.
    /// @param cycles Number of oscillation cycles (min 3).
    /// @param relayAmplitude Relay output amplitude (0.0–1.0 PWM).
    /// @param rule Tuning rule (ZN, TyreusLuyben, or AMIGO).
    /// @return Result with PID gains.
    PidAutotuneResult runRelayFeedback(
        double targetTemp,
        int cycles = 5,
        double relayAmplitude = 0.5,
        AstromHagglundRelay::TuningRule rule = AstromHagglundRelay::TuningRule::ZieglerNichols) {

        PidAutotuneResult result;
        result.method = autotuneMethodName(AutotuneMethod::RelayFeedback);

        // Configure the Åström-Hägglund relay autotuner
        AstromHagglundRelay::Config cfg;
        cfg.relayAmplitude = relayAmplitude;
        cfg.hysteresis = 1.0;  // 1°C hysteresis for noise rejection
        cfg.minCycles = std::max(3, cycles);
        cfg.maxCycles = cycles + 10;
        cfg.stabilityTol = 0.1;

        AstromHagglundRelay autotuner;
        autotuner.setConfig(cfg);
        autotuner.setTuningRule(rule);
        autotuner.start();

        // Run the relay feedback loop
        constexpr double dt = 0.1;  // 100ms sample interval
        constexpr int maxIterations = 10000;  // Safety timeout (~16 min)
        int iter = 0;

        heater_.setTarget(targetTemp);

        while (!autotuner.isComplete() && iter < maxIterations) {
            double measured = heater_.currentTemp();
            if (std::isnan(measured)) {
                result.message = "Sensor read failure during autotune";
                return result;
            }
            // The relay output is centered around 0; map to [0, 1] PWM
            double relayOut = autotuner.update(measured, targetTemp, 0.0, dt);
            double pwm = std::clamp(0.5 + relayOut * 0.5, 0.0, 1.0);
            heater_.update(pwm, dt);
            std::this_thread::sleep_for(
                std::chrono::milliseconds(static_cast<int>(dt * 1000)));
            ++iter;
        }

        if (!autotuner.isComplete()) {
            result.message = "Autotune timed out before convergence";
            return result;
        }

        auto intermediate = autotuner.getIntermediateResult();
        if (!intermediate.success || intermediate.parameters.size() < 3) {
            result.message = "Autotune failed: invalid results";
            return result;
        }

        result.success = true;
        result.Kp = intermediate.parameters[0];
        result.Ki = intermediate.parameters[1];
        result.Kd = intermediate.parameters[2];
        result.Ku = autotuner.getUltimateGain();
        result.Tu = autotuner.getUltimatePeriod();

        std::ostringstream ss;
        ss << "Relay feedback autotune complete: Kp=" << result.Kp
           << " Ki=" << result.Ki << " Kd=" << result.Kd
           << " (Ku=" << result.Ku << " Tu=" << result.Tu << "s)";
        result.message = ss.str();

        applyGains(result);
        return result;
    }

    // ------------------------------------------------------------------
    // Offline autotuning from step-response data
    // ------------------------------------------------------------------

    /// @brief Collect step-response data from the heater.
    /// @param stepPower PWM power to apply for the step (0.0–1.0).
    /// @param duration Duration of the step test in seconds.
    /// @param dt Sample interval in seconds.
    /// @return (time, temperature) pairs.
    std::vector<std::pair<double, double>> collectStepResponse(
        double stepPower = 0.5,
        double duration = 60.0,
        double dt = 0.5) {

        std::vector<std::pair<double, double>> data;
        // Start from current temperature (ambient)
        double startTemp = heater_.currentTemp();
        if (std::isnan(startTemp)) startTemp = 25.0;

        heater_.setTarget(startTemp + 100.0);  // Set high target so PID doesn't interfere
        double t = 0.0;
        while (t < duration) {
            heater_.update(stepPower, dt);
            double temp = heater_.currentTemp();
            data.emplace_back(t, std::isnan(temp) ? startTemp : temp);
            std::this_thread::sleep_for(
                std::chrono::milliseconds(static_cast<int>(dt * 1000)));
            t += dt;
        }
        // Turn off heater
        heater_.update(0.0, dt);
        heater_.setTarget(startTemp);
        return data;
    }

    /// @brief Run offline autotuning from step-response data.
    /// @param stepData Collected (time, temperature) pairs.
    /// @param stepPower The power level used for the step.
    /// @param method Tuning method to apply.
    /// @param form PID form (Parallel/Standard/Series).
    /// @param lambda Lambda parameter (for Lambda/SIMC methods, -1 = auto).
    /// @return Result with PID gains.
    PidAutotuneResult runFromStepResponse(
        const std::vector<std::pair<double, double>>& stepData,
        double stepPower,
        AutotuneMethod method,
        PIDForm form = PIDForm::Parallel,
        double lambda = -1.0) {

        PidAutotuneResult result;
        result.method = autotuneMethodName(method);

        if (stepData.size() < 10) {
            result.message = "Insufficient step response data";
            return result;
        }

        // Extract time and response vectors
        std::vector<double> time, response;
        for (const auto& [t, y] : stepData) {
            time.push_back(t);
            response.push_back(y);
        }

        // Identify FOPDT model using Tether's ProcessIdentification
        FOPDTModel model =
            ProcessIdentification::tangentMethod(time, response, stepPower);

        if (!model.isValid()) {
            result.message = "FOPDT model identification failed";
            return result;
        }

        // Apply the requested tuning method
        PIDGains gains = computeGains(model, method, form, lambda);

        if (!gains.isValid()) {
            result.message = "Tuning rule produced invalid gains";
            return result;
        }

        result.success = true;
        result.Kp = gains.Kp;
        result.Ki = gains.Ki;
        result.Kd = gains.Kd;

        // Compute ultimate parameters from the model
        auto [ku, tu] = ProcessIdentification::estimateUltimate(model);
        result.Ku = ku;
        result.Tu = tu;

        std::ostringstream ss;
        ss << autotuneMethodName(method) << " autotune complete: Kp=" << result.Kp
           << " Ki=" << result.Ki << " Kd=" << result.Kd
           << " (model: K=" << model.K << " tau=" << model.tau
           << " L=" << model.L << ")";
        result.message = ss.str();

        applyGains(result);
        return result;
    }

    // ------------------------------------------------------------------
    // Offline autotuning from known ultimate parameters (Ku, Tu)
    // ------------------------------------------------------------------

    /// @brief Run offline autotuning from known ultimate gain and period.
    /// @param Ku Ultimate gain.
    /// @param Tu Ultimate period (seconds).
    /// @param method Tuning method (ZN ultimate, Tyreus-Luyben).
    /// @param form PID form.
    /// @return Result with PID gains.
    PidAutotuneResult runFromUltimateParams(
        double Ku, double Tu,
        AutotuneMethod method,
        PIDForm form = PIDForm::Parallel) {

        PidAutotuneResult result;
        result.method = autotuneMethodName(method);
        result.Ku = Ku;
        result.Tu = Tu;

        PIDGains gains;
        if (method == AutotuneMethod::TyreusLuyben) {
            gains = TyreusLuyben::calculateGains(Ku, Tu, false);
        } else {
            // Default: Ziegler-Nichols ultimate
            gains = ZieglerNicholsUltimateCycle::calculateGains(Ku, Tu, form);
        }

        if (!gains.isValid()) {
            result.message = "Tuning rule produced invalid gains";
            return result;
        }

        result.success = true;
        result.Kp = gains.Kp;
        result.Ki = gains.Ki;
        result.Kd = gains.Kd;

        std::ostringstream ss;
        ss << autotuneMethodName(method) << " from ultimate params: Kp=" << result.Kp
           << " Ki=" << result.Ki << " Kd=" << result.Kd
           << " (Ku=" << Ku << " Tu=" << Tu << "s)";
        result.message = ss.str();

        applyGains(result);
        return result;
    }

    // ------------------------------------------------------------------
    // Offline autotuning from a known FOPDT model
    // ------------------------------------------------------------------

    /// @brief Run offline autotuning from a known FOPDT model.
    /// @param model FOPDT model (K, tau, L).
    /// @param method Tuning method.
    /// @param form PID form.
    /// @param lambda Lambda parameter (for Lambda/SIMC, -1 = auto).
    /// @return Result with PID gains.
    PidAutotuneResult runFromModel(
        const FOPDTModel& model,
        AutotuneMethod method,
        PIDForm form = PIDForm::Parallel,
        double lambda = -1.0) {

        PidAutotuneResult result;
        result.method = autotuneMethodName(method);

        if (!model.isValid()) {
            result.message = "Invalid FOPDT model";
            return result;
        }

        PIDGains gains = computeGains(model, method, form, lambda);

        if (!gains.isValid()) {
            result.message = "Tuning rule produced invalid gains";
            return result;
        }

        result.success = true;
        result.Kp = gains.Kp;
        result.Ki = gains.Ki;
        result.Kd = gains.Kd;

        auto [ku, tu] = ProcessIdentification::estimateUltimate(model);
        result.Ku = ku;
        result.Tu = tu;

        std::ostringstream ss;
        ss << autotuneMethodName(method) << " from model: Kp=" << result.Kp
           << " Ki=" << result.Ki << " Kd=" << result.Kd
           << " (model: K=" << model.K << " tau=" << model.tau
           << " L=" << model.L << ")";
        result.message = ss.str();

        applyGains(result);
        return result;
    }

    // ------------------------------------------------------------------
    // High-level convenience: run autotune with method selection
    // ------------------------------------------------------------------

    /// @brief Run PID autotuning using the specified method.
    ///
    /// For online methods (RelayFeedback), the heater is driven in real time.
    /// For offline methods, step-response data is collected first, then the
    /// tuning rule is applied.
    ///
    /// @param targetTemp Target temperature in °C.
    /// @param method Tuning method to use.
    /// @param cycles Number of cycles (for relay feedback).
    /// @param form PID form.
    /// @param lambda Lambda parameter (for Lambda/SIMC).
    /// @return Result with PID gains.
    PidAutotuneResult autotune(
        double targetTemp,
        AutotuneMethod method = AutotuneMethod::RelayFeedback,
        int cycles = 5,
        PIDForm form = PIDForm::Parallel,
        double lambda = -1.0) {

        switch (method) {
            case AutotuneMethod::RelayFeedback:
                return runRelayFeedback(targetTemp, cycles, 0.5,
                                        AstromHagglundRelay::TuningRule::ZieglerNichols);

            default: {
                // Offline methods: collect step response, identify model, tune
                double stepPower = 0.5;
                auto stepData = collectStepResponse(stepPower, 60.0, 0.5);
                return runFromStepResponse(stepData, stepPower, method, form, lambda);
            }
        }
    }

private:
    objects::Heater& heater_;
    double& settingsKp_;
    double& settingsKi_;
    double& settingsKd_;

    /// @brief Apply computed gains to the heater and settings.
    void applyGains(const PidAutotuneResult& result) {
        if (!result.success) return;
        settingsKp_ = result.Kp;
        settingsKi_ = result.Ki;
        settingsKd_ = result.Kd;
        heater_.setPidParams({result.Kp, result.Ki, result.Kd, 100.0, 0.0, 1.0});
    }

    /// @brief Compute PID gains from a FOPDT model using the specified method.
    static PIDGains computeGains(const FOPDTModel& model,
                                  AutotuneMethod method,
                                  PIDForm form,
                                  double lambda) {
        switch (method) {
            case AutotuneMethod::ZieglerNicholsStep:
                return ZieglerNicholsStepResponse::calculateGains(model, form);
            case AutotuneMethod::ZieglerNicholsUltimate: {
                auto [ku, tu] = ProcessIdentification::estimateUltimate(model);
                return ZieglerNicholsUltimateCycle::calculateGains(ku, tu, form);
            }
            case AutotuneMethod::CohenCoon:
                return CohenCoon::calculateGains(model, form);
            case AutotuneMethod::Lambda: {
                double lam = (lambda > 0) ? lambda : model.tau;
                return LambdaTuning::calculateGains(model, lam, true);
            }
            case AutotuneMethod::SIMC: {
                double tauC = (lambda > 0) ? lambda : model.tau * 0.5;
                return SIMCMethod::calculateGains(model, tauC);
            }
            case AutotuneMethod::AMIGO:
                return AMIGOMethod::calculateGains(model, form);
            case AutotuneMethod::ChienHronesReswick:
                return ChienHronesReswick::calculateGains(
                    model, form, ChienHronesReswick::Mode::SetpointNoOvershoot);
            case AutotuneMethod::LopezITAE:
                return LopezMethod::calculateGains(
                    model, form, LopezMethod::Criterion::ITAE,
                    LopezMethod::ResponseType::Setpoint);
            case AutotuneMethod::LopezIAE:
                return LopezMethod::calculateGains(
                    model, form, LopezMethod::Criterion::IAE,
                    LopezMethod::ResponseType::Setpoint);
            case AutotuneMethod::LopezISE:
                return LopezMethod::calculateGains(
                    model, form, LopezMethod::Criterion::ISE,
                    LopezMethod::ResponseType::Setpoint);
            default:
                return {};
        }
    }
};

// ============================================================================
// Resonance / Input-Shaper Calibration Bridge
// ============================================================================

/// @brief Result of a resonance calibration run.
struct ResonanceCalibrationResult {
    bool success = false;
    double resonantFreqX = 0.0;  ///< Identified X resonant frequency (Hz)
    double resonantFreqY = 0.0;  ///< Identified Y resonant frequency (Hz)
    std::string shaperTypeX;     ///< Recommended X shaper type
    std::string shaperTypeY;     ///< Recommended Y shaper type
    std::vector<Identification::FrequencyResponsePoint> bodeX;
    std::vector<Identification::FrequencyResponsePoint> bodeY;
    std::string message;
};

/// @brief Bridge for resonance testing and input-shaper calibration.
///
/// Uses Tether's FrequencyResponseAnalyzer and ChirpGenerator to perform
/// frequency-sweep measurements on accelerometer data, then recommends
/// input shaper parameters.
class ResonanceCalibrationBridge {
public:
    /// @brief Set the accelerometer data source.
    /// @param accelRead Function that returns current acceleration (x, y, z) in g.
    void setAccelerometerSource(
        std::function<std::array<double, 3>()> accelRead) {
        accelRead_ = std::move(accelRead);
    }

    /// @brief Set the motion callback for generating excitation.
    /// @param moveCallback Function to move the axis (x_mm, y_mm, z_mm, feedrate).
    void setMoveCallback(
        std::function<void(double, double, double, double)> moveCallback) {
        moveCallback_ = std::move(moveCallback);
    }

    /// @brief Run a resonance sweep on the specified axis.
    /// @param axis "X" or "Y".
    /// @param minFreq Start frequency in Hz.
    /// @param maxFreq End frequency in Hz.
    /// @param duration Sweep duration in seconds.
    /// @return Frequency response data.
    std::vector<Identification::FrequencyResponsePoint> runSweep(
        const std::string& axis,
        float minFreq = 5.0f,
        float maxFreq = 100.0f,
        float duration = 30.0f) {

        std::vector<Identification::FrequencyResponsePoint> results;
        if (!accelRead_) return results;

        (void)duration;  // Duration is implicit from freq step + cycles

        // Step through frequencies and use FrequencyResponseAnalyzer
        // at each frequency point
        const float freqStep = 1.0f;  // 1 Hz resolution
        const float cyclesPerFreq = 3.0f;  // 3 cycles per frequency point
        const float dt = 0.001f;  // 1ms sample interval

        for (float freq = minFreq; freq <= maxFreq; freq += freqStep) {
            Identification::FrequencyResponseAnalyzer analyzer;
            analyzer.reset(freq, 1.0f);

            float cyclesRecorded = 0.0f;
            int samples = 0;
            const int samplesNeeded = static_cast<int>(cyclesPerFreq / freq / dt);

            while (cyclesRecorded < cyclesPerFreq && samples < samplesNeeded * 2) {
                // Read accelerometer
                auto accel = accelRead_();
                double accelValue = (axis == "X") ? accel[0] :
                                    (axis == "Y") ? accel[1] : accel[2];

                // Generate excitation (would drive motion in real system)
                float excitation = analyzer.generateExcitation();
                if (moveCallback_) {
                    if (axis == "X") moveCallback_(excitation, 0, 0, 100.0);
                    else if (axis == "Y") moveCallback_(0, excitation, 0, 100.0);
                }

                analyzer.addSample(dt, excitation, static_cast<float>(accelValue));
                cyclesRecorded = analyzer.getCyclesRecorded();
                ++samples;
            }

            auto point = analyzer.computeResponse();
            if (point.coherence > 0.5f) {  // Only keep coherent measurements
                results.push_back(point);
            }
        }

        return results;
    }

    /// @brief Calibrate input shaper from frequency response data.
    /// @param bode Frequency response data for one axis.
    /// @return (resonant_freq, recommended_shaper_type)
    static std::pair<double, std::string> calibrateFromBode(
        const std::vector<Identification::FrequencyResponsePoint>& bode) {

        if (bode.empty()) return {0.0, "none"};

        // Find the peak magnitude (resonance)
        auto maxIt = std::max_element(bode.begin(), bode.end(),
            [](const auto& a, const auto& b) {
                return a.magnitude_dB < b.magnitude_dB;
            });

        double resonantFreq = maxIt->frequency;
        double peakMag = maxIt->magnitude_dB;

        // Recommend shaper type based on resonance characteristics
        std::string shaperType;
        if (peakMag > 20.0f) {
            // Strong resonance — use ZVD for maximum damping
            shaperType = "zvd";
        } else if (peakMag > 10.0f) {
            // Moderate resonance — use EI
            shaperType = "ei";
        } else if (peakMag > 5.0f) {
            // Mild resonance — use MZV
            shaperType = "mzv";
        } else {
            // Very mild — ZV is sufficient
            shaperType = "zv";
        }

        return {resonantFreq, shaperType};
    }

    /// @brief Full calibration: sweep both axes and recommend shaper params.
    /// @param minFreq Minimum frequency to test.
    /// @param maxFreq Maximum frequency to test.
    /// @return Calibration result with recommendations.
    ResonanceCalibrationResult calibrate(
        float minFreq = 5.0f,
        float maxFreq = 100.0f) {

        ResonanceCalibrationResult result;

        if (!accelRead_) {
            result.message = "No accelerometer source configured";
            return result;
        }

        // Sweep X axis
        result.bodeX = runSweep("X", minFreq, maxFreq);
        auto [freqX, typeX] = calibrateFromBode(result.bodeX);
        result.resonantFreqX = freqX;
        result.shaperTypeX = typeX;

        // Sweep Y axis
        result.bodeY = runSweep("Y", minFreq, maxFreq);
        auto [freqY, typeY] = calibrateFromBode(result.bodeY);
        result.resonantFreqY = freqY;
        result.shaperTypeY = typeY;

        result.success = !result.bodeX.empty() || !result.bodeY.empty();

        std::ostringstream ss;
        ss << "Resonance calibration complete: "
           << "X=" << result.resonantFreqX << "Hz (" << result.shaperTypeX << ")"
           << ", Y=" << result.resonantFreqY << "Hz (" << result.shaperTypeY << ")";
        result.message = ss.str();

        return result;
    }

private:
    std::function<std::array<double, 3>()> accelRead_;
    std::function<void(double, double, double, double)> moveCallback_;
};

} // namespace tether::klipper::klippy
