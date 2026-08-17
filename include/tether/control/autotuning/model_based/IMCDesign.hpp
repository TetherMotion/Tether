/**
 * @file IMCDesign.hpp
 * @brief Internal Model Control (IMC) design autotuner.
 *
 * @details
 * Designs controllers by inverting the process model and applying a
 * low-pass filter. The filter time constant (lambda) is used to trade
 * performance for robustness.
 */

#pragma once

#include "tether/control/autotuning/AutotuningFramework.hpp"
#include "tether/control/autotuning/ClassicalTuningMethods.hpp"
#include <algorithm>

namespace tether::control {
namespace Autotuning {

/**
 * @class IMCDesign
 * @brief Internal Model Control (IMC) design autotuner.
 *
 * @details
 * Designs controllers by inverting the process model and applying a
 * low-pass filter; the filter time constant (lambda) trades performance
 * for robustness.
 */
class IMCDesign : public AutotunerBase {
public:
    std::string getName() const override { return "Internal Model Control"; }
    std::string getDescription() const override {
        return "Model-based design using process inverse with filter. "
               "Filter time constant λ trades performance for robustness.";
    }
    AutotuningMode getMode() const override { return AutotuningMode::Offline; }

    /** @brief Check controller compatibility for IMC design. */
    bool isCompatible(const TunableController& controller) const override;
    
    /**
     * @brief Perform IMC design and apply resulting gains to @p controller.
     * @param controller Controller to update.
     * @param model Optional process model (FOPDT/SOPDT/IPDT) used for design.
     */
    TuningResult tune(TunableController& controller,
                     const ProcessModel* model = nullptr) override;
    
    /** @brief Set the IMC filter time constant λ. */
    void setFilterTimeConstant(double lambda) { m_lambda = lambda; }

    /** @brief Set the filter order (1 or 2). */
    void setFilterOrder(int order) { m_filterOrder = std::min(2, std::max(1, order)); }

    /** @brief Design PID for FOPDT using IMC rules. */
    static PIDGains designForFOPDT(const FOPDTModel& model, double lambda, int filterOrder = 1);

    /** @brief Design PID for SOPDT using IMC approximations. */
    static PIDGains designForSOPDT(const SOPDTModel& model, double lambda);

    /** @brief Design PID for IPDT (integrating plus delay) processes. */
    static PIDGains designForIPDT(const IPDTModel& model, double lambda);

    /** @brief Provide a FOPDT model to the tuner. */
    void setModel(const FOPDTModel& model) { m_fopdtModel = model; m_useFOPDT = true; }

    /** @brief Provide a SOPDT model to the tuner. */
    void setModel(const SOPDTModel& model) { m_sopdtModel = model; m_useFOPDT = false; }

private:
    FOPDTModel m_fopdtModel;
    SOPDTModel m_sopdtModel;
    bool m_useFOPDT{true};
    double m_lambda{1.0};
    int m_filterOrder{1};
};

} // namespace Autotuning
} // namespace tether::control
