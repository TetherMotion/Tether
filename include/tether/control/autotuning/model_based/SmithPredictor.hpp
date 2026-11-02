/**
 * @file SmithPredictor.hpp
 * @brief Smith predictor design autotuner (dead-time compensation).
 *
 * @details
 * Designs a Smith predictor to compensate for process dead-time using
 * an internal process model and a prediction structure.
 */

#pragma once

#include "tether/control/autotuning/AutotuningFramework.hpp"
#include "tether/control/autotuning/ClassicalTuningMethods.hpp"

namespace tether::control {
namespace Autotuning {

/**
 * @class SmithPredictor
 * @brief Smith predictor design autotuner (dead-time compensation).
 *
 * @details
 * Designs a Smith predictor to compensate process dead-time using an
 * internal model and prediction structure.
 */
class SmithPredictor : public AutotunerBase {
public:
    std::string getName() const override { return "Smith Predictor"; }
    std::string getDescription() const override {
        return "Dead-time compensator using process model. "
               "Effectively removes delay from feedback loop.";
    }
    AutotuningMode getMode() const override { return AutotuningMode::Offline; }

    /**
     * @brief Check compatibility for Smith-predictor design.
     * @param controller  Controller whose structure is validated.
     * @return true when the controller can host a predictor-compensated PID.
     */
    bool isCompatible(const TunableController& controller) const override;
    
    /**
     * @brief Design Smith predictor and apply inner controller gains.
     * @param controller  Controller receiving the inner gains.
     * @param model       Optional process model overriding setModel().
     * @return TuningResult with applied gains or an error.
     */
    TuningResult tune(TunableController& controller,
                     const ProcessModel* model = nullptr) override;
    
    struct SmithPredictorStructure {
        PIDGains innerController;
        FOPDTModel processModel;
        double delay;
    };

    /**
     * @brief Compute inner controller and predictor parameters for given model and λ.
     * @param model   FOPDT process model.
     * @param lambda  Closed-loop speed parameter (smaller = faster).
     * @return predictor structure (inner gains + model + delay).
     */
    static SmithPredictorStructure design(const FOPDTModel& model, double lambda);

    /**
     * @brief Set the process model used for Smith predictor design.
     * @param model  FOPDT model (gain, time constant, dead time).
     */
    void setModel(const FOPDTModel& model) { m_model = model; }

    /**
     * @brief Set design filter parameter λ used in Smith predictor design.
     * @param lambda  Closed-loop speed parameter (seconds).
     */
    void setLambda(double lambda) { m_lambda = lambda; }

private:
    FOPDTModel m_model;
    double m_lambda{1.0};
};

} // namespace Autotuning
} // namespace tether::control
