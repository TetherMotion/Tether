#pragma once
#include "SimulationTypes.hpp"

namespace Simulation {

/// Abstract base class for all dynamical systems
class DynamicalSystem {
public:
    virtual ~DynamicalSystem() = default;

    /// System identification
    virtual const char* name() const = 0;
    virtual const char* description() const = 0;
    virtual SystemCategory category() const = 0;
    virtual int systemId() const = 0;

    /// State space dimensions
    virtual int stateDim() const = 0;
    virtual int inputDim() const = 0;
    virtual int outputDim() const = 0;

    /// Compute state derivatives: dx/dt = f(t, x, u)
    virtual StateVector dynamics(double t, const StateVector& state,
                                  const StateVector& input) const = 0;

    /// Compute output: y = g(t, x, u)
    virtual StateVector output(double t, const StateVector& state,
                                const StateVector& input) const = 0;

    /// Get default initial state
    virtual StateVector defaultInitialState() const = 0;

    /// Get a UI-friendly default initial state that guarantees visible excitation.
    StateVector defaultInitialStateForUi() const;

    /// Get default input
    virtual StateVector defaultInput() const {
        return StateVector(inputDim(), 0.0);
    }

    /// Parameter management
    virtual std::vector<ParamDescriptor> parameterDescriptors() const = 0;
    /// Parameter descriptors enriched for UI consumption.
    std::vector<ParamDescriptor> parameterDescriptorsDetailed() const;
    virtual ParamMap getParameters() const = 0;
    virtual void setParameters(const ParamMap& params) = 0;
    virtual void setParameter(const std::string& name, double value) = 0;
    virtual double getParameter(const std::string& name) const = 0;

    /// Presets
    virtual std::vector<Preset> presets() const = 0;
    virtual void applyPreset(int index) = 0;

    /// Geometry descriptors for each component
    virtual std::vector<GeometryDesc> geometries() const { return {}; }

    /// Linearization at operating point
    virtual void linearize(const StateVector& x0, const StateVector& u0,
                           std::vector<double>& A, std::vector<double>& B,
                           std::vector<double>& C, std::vector<double>& D) const;

    /// Get state variable names
    virtual std::vector<std::string> stateNames() const = 0;

    /// Get output variable names
    virtual std::vector<std::string> outputNames() const = 0;

    /// Get input variable names
    virtual std::vector<std::string> inputNames() const = 0;

    /// Get governing equations as LaTeX strings
    virtual std::vector<std::string> equationStrings() const { return {}; }

    /// Check if system is MIMO
    bool isMIMO() const { return inputDim() > 1 || outputDim() > 1; }

    /// Get sensor configs (one per output)
    virtual std::vector<SensorConfig> defaultSensorConfigs() const {
        return std::vector<SensorConfig>(outputDim());
    }

    /// Get actuator configs (one per input)
    virtual std::vector<ActuatorConfig> defaultActuatorConfigs() const {
        return std::vector<ActuatorConfig>(inputDim());
    }
};

/// Shared base for parametric systems that expose a named parameter map.
///
/// The category-specific system headers are split into one header per system, so
/// the lightweight parameter storage lives at the simulation-core level rather
/// than in one particular module header. This keeps per-system headers
/// self-contained while preserving the existing implementation API used by the
/// monolithic `.cpp` files.
class ParametricSystem : public DynamicalSystem {
public:
    ParamMap getParameters() const override { return params_; }
    void setParameters(const ParamMap& params) override;
    void setParameter(const std::string& name, double value) override;
    double getParameter(const std::string& name) const override;

protected:
    ParamMap params_;

    void initParam(const std::string& name, double value) { params_[name] = value; }
};

} // namespace Simulation
