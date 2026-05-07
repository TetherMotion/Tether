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

namespace Control {
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
    /** @brief Human-readable name of the tuner. */
    std::string getName() const override;

    /** @brief Short description for UI or logs. */
    std::string getDescription() const override;

    /** @brief Whether the given controller is compatible with this online tuner. */
    bool isCompatible(const TunableController& controller) const override;

    /**
     * @brief Run one tuning session; for relay feedback this will perform
     * closed-loop excitation until sufficient cycles are collected.
     */
    TuningResult tune(TunableController& controller, const ProcessModel* model = nullptr) override;

    /** @brief Called periodically to update internal state from measurements. */
    double update(double measured, double reference, double control, double dt) override;

    /** @brief Whether the online autotuning process has completed. */
    bool isComplete() const override;

    /** @brief Start the online tuning run. */
    void start() override;

    /** @brief Abort/stop the online tuning run. */
    void stop() override;

    /** @brief Return intermediate tuning result while running. */
    TuningResult getIntermediateResult() const override;

    struct Config {
        double relayAmplitude{1.0};
        double hysteresis{0.0};
        int minCycles{3};
        int maxCycles{20};
        double stabilityTol{0.05};
    };

    /** @brief Configure relay amplitude, hysteresis and cycle constraints. */
    void setConfig(const Config& config) { m_config = config; }

    enum class TuningRule { ZieglerNichols, TyreusLuyben, AMIGO };
    /** @brief Select which rule to apply to the identified Ku/Tu values. */
    void setTuningRule(TuningRule rule) { m_rule = rule; }

    /** @brief Get the current estimated ultimate gain. */
    double getUltimateGain() const { return m_Ku; }
    /** @brief Get the current estimated ultimate period. */
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
} // namespace Control
