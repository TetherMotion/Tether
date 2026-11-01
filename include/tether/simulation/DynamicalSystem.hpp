#pragma once
#include "SimulationTypes.hpp"

namespace Simulation {

/// Abstract base class for all dynamical systems.
///
/// A system maps a state vector (size stateDim()), an input vector (size
/// inputDim()) and the current time to a state derivative and an output
/// vector (size outputDim()). Quantities are SI unless a derived system
/// documents otherwise; vector layouts are described by stateNames(),
/// inputNames() and outputNames().
class DynamicalSystem {
public:
    virtual ~DynamicalSystem() = default;

    /// System identification
    /// @return Short display name of the system (e.g. "Acrobot").
    virtual const char* name() const = 0;
    /// @return One-line human-readable description of the modeled plant.
    virtual const char* description() const = 0;
    /// @return Benchmark category the system is grouped under.
    virtual SystemCategory category() const = 0;
    /// @return Stable numeric id used for serialized benchmark references.
    virtual int systemId() const = 0;

    /// State space dimensions
    /// @return Number of state variables in StateVector.
    virtual int stateDim() const = 0;
    /// @return Number of input variables in StateVector.
    virtual int inputDim() const = 0;
    /// @return Number of output variables in StateVector.
    virtual int outputDim() const = 0;

    /// Compute state derivatives: dx/dt = f(t, x, u)
    /// @param t Current simulation time [s].
    /// @param state Current state vector, size stateDim().
    /// @param input Current input vector, size inputDim().
    /// @return State derivative vector, size stateDim().
    virtual StateVector dynamics(double t, const StateVector& state,
                                  const StateVector& input) const = 0;

    /// Compute output: y = g(t, x, u)
    /// @param t Current simulation time [s].
    /// @param state Current state vector, size stateDim().
    /// @param input Current input vector, size inputDim().
    /// @return Output vector, size outputDim().
    virtual StateVector output(double t, const StateVector& state,
                                const StateVector& input) const = 0;

    /// Get default initial state
    /// @return Initial state vector, size stateDim().
    virtual StateVector defaultInitialState() const = 0;

    /// Get a UI-friendly default initial state that guarantees visible excitation.
    /// @return Initial state with all-zero components nudged to a small
    ///         nonzero value so plots show motion.
    StateVector defaultInitialStateForUi() const;

    /// Get default input
    /// @return Zero input vector, size inputDim().
    virtual StateVector defaultInput() const {
        return StateVector(inputDim(), 0.0);
    }

    /// Parameter management
    /// @return One descriptor per tunable parameter (name, bounds, units).
    virtual std::vector<ParamDescriptor> parameterDescriptors() const = 0;
    /// Parameter descriptors enriched for UI consumption.
    /// @return Descriptors with display metadata added on top of
    ///         parameterDescriptors().
    std::vector<ParamDescriptor> parameterDescriptorsDetailed() const;
    /// @return Current parameter values keyed by parameter name.
    virtual ParamMap getParameters() const = 0;
    /// @param params New values; names not in parameterDescriptors() are ignored.
    virtual void setParameters(const ParamMap& params) = 0;
    /// @param name Parameter name; unknown names are added.
    /// @param value New value in the parameter's own unit.
    virtual void setParameter(const std::string& name, double value) = 0;
    /// @param name Parameter name.
    /// @return Current value, or 0.0 when the name is unknown.
    virtual double getParameter(const std::string& name) const = 0;

    /// Presets
    /// @return Named presets; may be empty when the system has none.
    virtual std::vector<Preset> presets() const = 0;
    /// @param index Index into presets(); out-of-range values are ignored.
    virtual void applyPreset(int index) = 0;

    /// Geometry descriptors for each component
    /// @return Display geometry per component; empty when the system has no
    ///         visual model.
    virtual std::vector<GeometryDesc> geometries() const { return {}; }

    /// Linearization at operating point
    ///
    /// Numerical (central-difference) Jacobians of dynamics() and output()
    /// around (x0, u0), written row-major into the standard state-space
    /// matrices. The matrices are resized by this call.
    /// @param x0 Operating-point state, size stateDim().
    /// @param u0 Operating-point input, size inputDim().
    /// @param A dF/dx, stateDim() x stateDim().
    /// @param B dF/du, stateDim() x inputDim().
    /// @param C dG/dx, outputDim() x stateDim().
    /// @param D dG/du, outputDim() x inputDim().
    virtual void linearize(const StateVector& x0, const StateVector& u0,
                           std::vector<double>& A, std::vector<double>& B,
                           std::vector<double>& C, std::vector<double>& D) const;

    /// Get state variable names
    /// @return Symbol per state variable, size stateDim().
    virtual std::vector<std::string> stateNames() const = 0;

    /// Get output variable names
    /// @return Symbol per output variable, size outputDim().
    virtual std::vector<std::string> outputNames() const = 0;

    /// Get input variable names
    /// @return Symbol per input variable, size inputDim().
    virtual std::vector<std::string> inputNames() const = 0;

    /// Get governing equations as LaTeX strings
    /// @return LaTeX equations for documentation; empty when none supplied.
    virtual std::vector<std::string> equationStrings() const { return {}; }

    /// Check if system is MIMO
    /// @return true when the system has more than one input or output.
    bool isMIMO() const { return inputDim() > 1 || outputDim() > 1; }

    /// Get sensor configs (one per output)
    /// @return Default sensor configuration per output channel.
    virtual std::vector<SensorConfig> defaultSensorConfigs() const {
        return std::vector<SensorConfig>(outputDim());
    }

    /// Get actuator configs (one per input)
    /// @return Default actuator configuration per input channel.
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
    /// @return Current parameter values (copy of the stored map).
    ParamMap getParameters() const override { return params_; }
    /// @param params New values; names not already stored are ignored.
    void setParameters(const ParamMap& params) override;
    /// @param name Parameter name; unknown names are added.
    /// @param value New value in the parameter's own unit.
    void setParameter(const std::string& name, double value) override;
    /// @param name Parameter name.
    /// @return Current value, or 0.0 when the name is unknown.
    double getParameter(const std::string& name) const override;

protected:
    ParamMap params_;  ///< Parameter values keyed by name.

    /// Register a parameter with its initial value.
    /// @param name Parameter name.
    /// @param value Initial value in the parameter's own unit.
    void initParam(const std::string& name, double value) { params_[name] = value; }
};

} // namespace Simulation
