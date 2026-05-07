#pragma once
#include <vector>
#include <string>
#include <cstdint>
#include <cmath>
#include <functional>
#include <memory>
#include <algorithm>
#include <numeric>
#include <random>
#include <array>
#include <unordered_map>

namespace Simulation {

/// State vector type
using StateVector = std::vector<double>;

/// Parameter map type
using ParamMap = std::unordered_map<std::string, double>;

/// External force callback: (time, state) -> force vector
using ForceCallback = std::function<StateVector(double, const StateVector&)>;

/// Integration method enumeration
enum class IntegrationMethod {
    EulerForward,
    EulerBackward,
    RungeKutta4,
    DormandPrinceRK45,
    BogackiShampineRK23,
    ImplicitTrapezoidal,
    BDF2,
    SDIRK4
};

/// System category for UI grouping
enum class SystemCategory {
    MechanicalTranslational,
    RotationalAngular,
    AerospaceVehicle,
    Thermal,
    FluidHydraulic,
    ElectricalElectronic,
    OpticalPhotonic,
    ChemicalProcess,
    BiologicalBiomedical,
    ChaoticExtreme,
    DelayDominated
};

/// Geometry shape type for inertia computation
enum class GeometryShape {
    PointMass,
    Sphere,
    SolidCylinder,
    HollowCylinder,
    SolidCuboid,
    ThinRod,
    ThinDisk,
    Cone,
    Custom
};

/// Geometry descriptor
struct GeometryDesc {
    GeometryShape shape = GeometryShape::PointMass;
    double mass = 1.0;
    double length = 1.0;
    double width = 0.1;
    double height = 0.1;
    double radius = 0.05;
    double innerRadius = 0.0;
    double density = 7850.0; // steel

    /// Compute moment of inertia about primary axis
    double computeInertia() const;
    /// Compute cross-sectional area
    double computeArea() const;
};

/// Friction model type
enum class FrictionModel {
    None,
    Coulomb,
    Viscous,
    CoulombViscous,
    Stribeck,
    LuGre,
    Dahl
};

/// Friction parameters
struct FrictionParams {
    FrictionModel model = FrictionModel::None;
    double staticFriction = 0.0;
    double kineticFriction = 0.0;
    double viscousCoeff = 0.0;
    double stribeckVelocity = 0.01;
    double stribeckExponent = 2.0;
    // LuGre model parameters
    double sigma0 = 1e5;     // stiffness
    double sigma1 = 316.0;   // damping
    double sigma2 = 0.0;     // viscous
    double lugreState = 0.0; // internal state z

    /// Compute friction force given velocity
    double compute(double velocity, double normalForce, double dt);
};

/// Noise type enumeration
enum class NoiseType {
    None,
    White,
    Brown,
    Purple,
    Grey,
    PeriodicGSM,
    ActuatorDependent
};

/// Noise parameters
struct NoiseParams {
    NoiseType type = NoiseType::None;
    double amplitude = 0.0;
    double frequency = 0.0;     // for periodic noise
    double bandwidth = 0.0;     // for colored noise
    double actuatorGain = 0.0;  // for actuator-dependent noise
    uint64_t seed = 42;
};

/// Sensor configuration
struct SensorConfig {
    NoiseParams noise;
    double delay = 0.0;            // fixed delay in seconds
    double delayVariance = 0.0;    // random delay variance
    double quantization = 0.0;     // ADC resolution (0 = infinite)
    double sampleRate = 0.0;       // 0 = continuous
    double saturationMin = -1e9;
    double saturationMax = 1e9;
};

/// Actuator nonlinearity type
enum class ActuatorNonlinearity {
    None,
    Saturation,
    DeadZone,
    Backlash,
    RateLimiter,
    Hysteresis,
    ReducedPerformanceNearLimits
};

/// Actuator configuration
struct ActuatorConfig {
    double maxOutput = 1e6;
    double minOutput = -1e6;
    double maxRate = 1e6;
    double backlash = 0.0;
    double deadZone = 0.0;
    double hysteresis = 0.0;
    bool failOnLimit = false;
    bool reducedPerfNearLimits = false;
    double perfReductionZone = 0.1; // fraction of range
    FrictionParams stiction;
};

/// Air resistance configuration
struct AirResistanceConfig {
    bool enabled = false;
    double dragCoefficient = 0.47;  // sphere default
    double airDensity = 1.225;      // kg/m³ at sea level
    double referenceArea = 0.01;    // m²
};

/// Simulation result for one timestep
struct SimStepResult {
    double time;
    StateVector state;
    StateVector output;
    double controlSignal;
    double error;
};

/// System parameter descriptor for UI
struct ParamDescriptor {
    std::string name;
    std::string unit;
    std::string description;
    double defaultValue;
    double minValue;
    double maxValue;
    double step;
    bool logarithmic = false;
};

/// Preset configuration
struct Preset {
    std::string name;
    std::string description;
    ParamMap params;
};

/// Complete simulation configuration
struct SimConfig {
    double dt = 0.001;
    double totalTime = 10.0;
    IntegrationMethod method = IntegrationMethod::RungeKutta4;
    double absTolerance = 1e-6;
    double relTolerance = 1e-6;
    double maxStepSize = 5e-5;
    double minStepSize = 1e-8;
    bool adaptiveStep = false;
    AirResistanceConfig airResistance;
};

} // namespace Simulation
