/**
 * @file AstromHagglundRelay.hpp
 * @brief Åström-Hägglund relay-feedback autotuner (online).
 *
 * @details
 * Uses relay feedback to excite sustained oscillations and identify the
 * ultimate gain/period (Ku/Tu). Can be used with different tuning rules
 * (Ziegler-Nichols, Tyreus-Luyben, AMIGO) to compute PID gains.
 */

#pragma once

#include "tether/control/autotuning/classical/Common.hpp"
#include "tether/control/autotuning/AutotuningFramework.hpp"
#include <vector>

namespace tether::control {
namespace Autotuning {

/**
 * @class AstromHagglundRelay
 * @brief Åström–Hägglund relay-feedback autotuner (online).
 *
 * @details
 * Uses relay feedback to induce sustained oscillation; extracts the
 * ultimate gain and period (Ku, Tu) for use with various tuning rules
 * (Ziegler-Nichols, Tyreus-Luyben, AMIGO).
 */
class AstromHagglundRelay : public OnlineAutotuner {
public:
    /** @brief Human-readable name of the tuner.
     *  @return Display name ("Åström–Hägglund Relay")
     */
    std::string getName() const override;

    /** @brief Short description for UI or logs.
     *  @return One-line description of the relay-feedback method
     */
    std::string getDescription() const override;

    /** @brief Whether the given controller is compatible with this online tuner.
     *  @param controller Controller to check
     *  @return `true` if the tuner can drive `controller`
     */
    bool isCompatible(const TunableController& controller) const override;

    /**
     * @brief Run one tuning session; for relay feedback this will perform
     * closed-loop excitation until sufficient cycles are collected.
     * @param controller Controller to tune; gains are applied to it on success
     * @param model Optional known process model; may be nullptr (the relay
     *        method is model-free)
     * @return Tuning outcome including the suggested PID gains and status
     */
    TuningResult tune(TunableController& controller, const ProcessModel* model = nullptr) override;

    /** @brief Called periodically to update internal state from measurements.
     *  @param measured Current process value
     *  @param reference Current setpoint
     *  @param control Last applied control output
     *  @param dt Time since the previous call [s]
     *  @return Control output to apply (relay ±amplitude while running)
     */
    double update(double measured, double reference, double control, double dt) override;

    /** @brief Whether the online autotuning process has completed.
     *  @return `true` once enough stable oscillation cycles were collected
     */
    bool isComplete() const override;

    /** @brief Start the online tuning run. */
    void start() override;

    /** @brief Abort/stop the online tuning run. */
    void stop() override;

    /** @brief Return intermediate tuning result while running.
     *  @return Best result available so far; may be marked in-progress
     */
    TuningResult getIntermediateResult() const override;

    struct Config {
        double relayAmplitude{1.0};
        double hysteresis{0.0};
        int minCycles{3};
        int maxCycles{20};
        double stabilityTol{0.05};
    };

    /** @brief Configure relay amplitude, hysteresis and cycle constraints.
     *  @param config New configuration; takes effect on the next start()
     */
    void setConfig(const Config& config) { m_config = config; }

    enum class TuningRule { ZieglerNichols, TyreusLuyben, AMIGO };
    /** @brief Select which rule to apply to the identified Ku/Tu values.
     *  @param rule Tuning rule used to derive PID gains from Ku/Tu
     */
    void setTuningRule(TuningRule rule) { m_rule = rule; }

    /** @brief Get the current estimated ultimate gain.
     *  @return Ku estimate; 0 until a stable oscillation was measured
     */
    double getUltimateGain() const { return m_Ku; }
    /** @brief Get the current estimated ultimate period.
     *  @return Tu estimate [s]; 0 until a stable oscillation was measured
     */
    double getUltimatePeriod() const { return m_Tu; }

private:
    Config m_config;
    TuningRule m_rule{TuningRule::ZieglerNichols};

    enum class State { Idle, Running, Complete, Failed };
    State m_state{State::Idle};

    double m_setpoint{0.0};
    double m_relayOutput{0.0};
    double m_lastError{0.0};

    std::vector<double> m_peakTimes;
    std::vector<double> m_peakValues;
    std::vector<double> m_valleyValues;
    double m_elapsed{0.0};
    int m_cycles{0};

    double m_Ku{0.0};
    double m_Tu{0.0};
    PIDGains m_gains;
};

} // namespace Autotuning
} // namespace tether::control
