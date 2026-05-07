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

namespace Control {
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

    /** @brief Check compatibility for Smith-predictor design. */
    bool isCompatible(const TunableController& controller) const override;
    
    /** @brief Design Smith predictor and apply inner controller gains. */
    TuningResult tune(TunableController& controller,
                     const ProcessModel* model = nullptr) override;
    
    struct SmithPredictorStructure {
        PIDGains innerController;
        FOPDTModel processModel;
        double delay;
    };

    /** @brief Compute inner controller and predictor parameters for given model and λ. */
    static SmithPredictorStructure design(const FOPDTModel& model, double lambda);

    /** @brief Set the process model used for Smith predictor design. */
    void setModel(const FOPDTModel& model) { m_model = model; }

    /** @brief Set design filter parameter λ used in Smith predictor design. */
    void setLambda(double lambda) { m_lambda = lambda; }

private:
    FOPDTModel m_model;
    double m_lambda{1.0};
};

} // namespace Autotuning
} // namespace Control
