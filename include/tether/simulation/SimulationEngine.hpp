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

    /**
     * @brief Compute control input from measurement and reference
     * @param t Current simulation time [s]
     * @param measured Measured (post-sensor) state vector
     * @param reference Desired state/setpoint vector
     * @param dt Integration step size [s]
     * @return Control input applied to the system's actuators
     */
    virtual StateVector compute(double t, const StateVector& measured,
                                 const StateVector& reference, double dt) = 0;

    /**
     * @brief Reset controller state
     */
    virtual void reset() = 0;

    /**
     * @brief Get controller name
     * @return Human-readable name used in simulation logs/records
     */
    virtual const char* name() const = 0;
};

/// Main simulation engine
class SimulationEngine {
public:
    SimulationEngine();
    ~SimulationEngine();

    /**
     * @brief Set the dynamical system to simulate
     * @param system System model providing derivatives/dynamics
     */
    void setSystem(std::shared_ptr<DynamicalSystem> system);

    /**
     * @brief Set the controller
     * @param controller Feedback controller evaluated each step;
     *        nullptr selects open-loop operation
     */
    void setController(std::shared_ptr<SimController> controller);

    /**
     * @brief Set the reference/setpoint
     * @param ref Constant reference passed to the controller
     */
    void setReference(const StateVector& ref) { reference_ = ref; }

    /**
     * @brief Set external force callback
     * @param force (t, state) -> disturbance force injected into dynamics
     */
    void setExternalForce(ForceCallback force) { externalForce_ = std::move(force); }

    /**
     * @brief Configure simulation
     * @param config Step size, duration, integrator choice
     */
    void setConfig(const SimConfig& config);

    /**
     * @brief Configure sensors (one per output)
     * @param configs Noise/bias/quantization per measured channel
     */
    void setSensorConfigs(const std::vector<SensorConfig>& configs);

    /**
     * @brief Configure actuators (one per input)
     * @param configs Saturation/rate limits per actuated channel
     */
    void setActuatorConfigs(const std::vector<ActuatorConfig>& configs);

    /**
     * @brief Set initial state (overrides default)
     * @param state State at t=0; defaults to the system's initial state
     */
    void setInitialState(const StateVector& state) { initialState_ = state; hasInitialState_ = true; }

    /**
     * @brief Set a constant input vector for open-loop simulation.
     * @param input Input applied each step when no controller is set
     */
    void setInput(const StateVector& input) { input_ = input; }

    /**
     * @brief Run full simulation and return record
     * @return Time series of states, inputs, and measurements
     */
    SimulationRecord run();

    /// Step-by-step simulation
    void initialize();
    SimStepResult step();
    bool isFinished() const;

    /**
     * @brief Get current state
     * @return State vector at the current simulation time
     */
    const StateVector& currentState() const { return state_; }

    /**
     * @brief Reset simulation
     */
    void reset();

    /**
     * @brief Compute performance metrics from a simulation record
     * @param record Simulation output to evaluate
     * @param reference Reference the output was tracking
     * @param outputIndex Which output channel to score
     * @return Rise time, overshoot, settling time, IAE/ISE metrics
     */
    static PerformanceMetrics computeMetrics(const SimulationRecord& record,
                                               const StateVector& reference,
                                               int outputIndex = 0);

    /**
     * @brief Compute Bode plot data
     * @param record Simulation output to transform
     * @param inputIndex Excitation input channel
     * @param outputIndex Response output channel
     * @param[out] frequencies Frequency grid [rad/s]
     * @param[out] magnitudesDb Magnitude per frequency [dB]
     * @param[out] phasesDeg Phase per frequency [deg]
     */
    static void computeBodePlot(const SimulationRecord& record,
                                 int inputIndex, int outputIndex,
                                 std::vector<double>& frequencies,
                                 std::vector<double>& magnitudesDb,
                                 std::vector<double>& phasesDeg);

    /**
     * @brief Get system
     * @return Dynamical system assigned to this engine, or nullptr
     */
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
