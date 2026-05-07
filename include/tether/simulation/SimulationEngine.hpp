#pragma once
#include "DynamicalSystem.hpp"
#include "Integrators.hpp"
#include "SensorActuatorModels.hpp"
#include <memory>
#include <vector>

namespace Simulation {

/// Simulation recording
struct SimulationRecord {
    std::vector<double> times;
    std::vector<StateVector> states;
    std::vector<StateVector> outputs;
    std::vector<StateVector> measuredOutputs;
    std::vector<StateVector> controlInputs;
    std::vector<double> errors;
    std::vector<double> dtHistory;

    void clear() {
        times.clear(); states.clear(); outputs.clear();
        measuredOutputs.clear(); controlInputs.clear();
        errors.clear(); dtHistory.clear();
    }

    size_t size() const { return times.size(); }
};

/// Performance metrics computed from simulation
struct PerformanceMetrics {
    double riseTime = 0.0;
    double settlingTime = 0.0;
    double overshoot = 0.0;
    double undershoot = 0.0;
    double steadyStateError = 0.0;
    double iae = 0.0;    // Integral Absolute Error
    double ise = 0.0;    // Integral Squared Error
    double itae = 0.0;   // Integral Time-weighted Absolute Error
    double maxControl = 0.0;
    double controlEnergy = 0.0;
    double phaseMargin = 0.0;
    double gainMargin = 0.0;
};

/// Controller interface for the simulation engine
class SimController {
public:
    virtual ~SimController() = default;

    /// Compute control input from measurement and reference
    virtual StateVector compute(double t, const StateVector& measured,
                                 const StateVector& reference, double dt) = 0;

    /// Reset controller state
    virtual void reset() = 0;

    /// Get controller name
    virtual const char* name() const = 0;
};

/// Main simulation engine
class SimulationEngine {
public:
    SimulationEngine();
    ~SimulationEngine();

    /// Set the dynamical system to simulate
    void setSystem(std::shared_ptr<DynamicalSystem> system);

    /// Set the controller
    void setController(std::shared_ptr<SimController> controller);

    /// Set the reference/setpoint
    void setReference(const StateVector& ref) { reference_ = ref; }

    /// Set external force callback
    void setExternalForce(ForceCallback force) { externalForce_ = std::move(force); }

    /// Configure simulation
    void setConfig(const SimConfig& config);
    const SimConfig& config() const { return config_; }

    /// Configure sensors (one per output)
    void setSensorConfigs(const std::vector<SensorConfig>& configs);

    /// Configure actuators (one per input)
    void setActuatorConfigs(const std::vector<ActuatorConfig>& configs);

    /// Set initial state (overrides default)
    void setInitialState(const StateVector& state) { initialState_ = state; hasInitialState_ = true; }

    /// Set a constant input vector for open-loop simulation.
    void setInput(const StateVector& input) { input_ = input; }

    /// Run full simulation and return record
    SimulationRecord run();

    /// Step-by-step simulation
    void initialize();
    SimStepResult step();
    bool isFinished() const;

    /// Get current state
    const StateVector& currentState() const { return state_; }
    double currentTime() const { return time_; }

    /// Reset simulation
    void reset();

    /// Compute performance metrics from a simulation record
    static PerformanceMetrics computeMetrics(const SimulationRecord& record,
                                               const StateVector& reference,
                                               int outputIndex = 0);

    /// Compute Bode plot data
    static void computeBodePlot(const SimulationRecord& record,
                                 int inputIndex, int outputIndex,
                                 std::vector<double>& frequencies,
                                 std::vector<double>& magnitudesDb,
                                 std::vector<double>& phasesDeg);

    /// Get system
    std::shared_ptr<DynamicalSystem> system() const { return system_; }

private:
    std::shared_ptr<DynamicalSystem> system_;
    std::shared_ptr<SimController> controller_;
    std::unique_ptr<Integrator> integrator_;
    std::vector<SensorModel> sensors_;
    std::vector<ActuatorModel> actuators_;

    SimConfig config_;
    StateVector state_;
    StateVector reference_;
    StateVector initialState_;
    StateVector input_;
    bool hasInitialState_ = false;
    double time_ = 0.0;
    bool initialized_ = false;

    ForceCallback externalForce_;
};

} // namespace Simulation
