#include "tether/simulation/AllSystems.hpp"

namespace Simulation {

std::unique_ptr<DynamicalSystem> createSystem(int systemId) {
    switch (systemId) {
        // Mechanical (1–19)
        case 1:  return std::make_unique<MassSpringDamper>();
        case 2:  return std::make_unique<CoupledMassSpringDamper>();
        case 3:  return std::make_unique<InvertedPendulumCart>();
        case 4:  return std::make_unique<DoubleInvertedPendulumCart>();
        case 5:  return std::make_unique<TripleInvertedPendulumCart>();
        case 6:  return std::make_unique<Pendubot>();
        case 7:  return std::make_unique<Acrobot>();
        case 8:  return std::make_unique<FurutaPendulum>();
        case 9:  return std::make_unique<BallOnBeam>();
        case 10: return std::make_unique<BallOnPlate>();
        case 11: return std::make_unique<BouncingBall>();
        case 12: return std::make_unique<SegwayRobot>();
        case 13: return std::make_unique<GantryCrane>();
        case 14: return std::make_unique<DoublePendulumGantryCrane>();
        case 15: return std::make_unique<MagneticLevitation>();
        case 16: return std::make_unique<DualMagneticLevitation>();
        case 17: return std::make_unique<QuarterCarSuspension>();
        case 18: return std::make_unique<HalfCarSuspension>();
        case 19: return std::make_unique<VibrationIsolationPlatform>();

        // Rotational (20–27)
        case 20: return std::make_unique<DCMotorSpeed>();
        case 21: return std::make_unique<DCMotorPosition>();
        case 22: return std::make_unique<FlexibleShaft>();
        case 23: return std::make_unique<DiskDriveHead>();
        case 24: return std::make_unique<ReactionWheelSingleAxis>();
        case 25: return std::make_unique<ReactionWheel2D>();
        case 26: return std::make_unique<ControlMomentGyroscope>();
        case 27: return std::make_unique<FlywheelEnergyStorage>();

        // Aerospace (28–32)
        case 28: return std::make_unique<PlanarQuadrotor>();
        case 29: return std::make_unique<RocketLanding2D>();
        case 30: return std::make_unique<FixedWingAircraft2D>();
        case 31: return std::make_unique<BicycleLean>();
        case 32: return std::make_unique<Hovercraft2D>();

        // Thermal (33–37)
        case 33: return std::make_unique<SingleZoneOven>();
        case 34: return std::make_unique<MultiZoneOven>();
        case 35: return std::make_unique<HeatExchanger>();
        case 36: return std::make_unique<ThermoelectricCooler>();
        case 37: return std::make_unique<RoomHVAC>();

        // Fluid (38–43)
        case 38: return std::make_unique<SingleTankLevel>();
        case 39: return std::make_unique<CoupledTwoTank>();
        case 40: return std::make_unique<FourTankSystem>();
        case 41: return std::make_unique<HydraulicActuator>();
        case 42: return std::make_unique<PneumaticMuscle>();
        case 43: return std::make_unique<PressureVessel>();

        // Electrical (44–49)
        case 44: return std::make_unique<BuckConverter>();
        case 45: return std::make_unique<BoostConverter>();
        case 46: return std::make_unique<BuckBoostConverter>();
        case 47: return std::make_unique<PowerGridFrequency>();
        case 48: return std::make_unique<ActivePowerFilter>();
        case 49: return std::make_unique<PhaseLockLoop>();

        // Chemical (50–53)
        case 50: return std::make_unique<CSTR>();
        case 51: return std::make_unique<pHNeutralization>();
        case 52: return std::make_unique<DistillationColumn>();
        case 53: return std::make_unique<Bioreactor>();

        // Biological (54–56)
        case 54: return std::make_unique<BloodGlucose>();
        case 55: return std::make_unique<AnesthesiaControl>();
        case 56: return std::make_unique<PredatorPrey>();

        // Chaotic (57–62)
        case 57: return std::make_unique<LorenzSystem>();
        case 58: return std::make_unique<RosslerSystem>();
        case 59: return std::make_unique<ChuaCircuit>();
        case 60: return std::make_unique<DuffingOscillator>();
        case 61: return std::make_unique<KapitzaPendulum>();
        case 62: return std::make_unique<TripleLinkGymnast>();

        // Delay (63–65)
        case 63: return std::make_unique<SmithPredictorPlant>();
        case 64: return std::make_unique<NetworkedControlSystem>();
        case 65: return std::make_unique<ConveyorBeltTracking>();

        // Optical (66–70)
        case 66: return std::make_unique<AdaptiveOpticsTelescope>();
        case 67: return std::make_unique<FastSteeringMirror>();
        case 68: return std::make_unique<PoundDreverHallLock>();
        case 69: return std::make_unique<ActiveSeismicIsolationBench>();
        case 70: return std::make_unique<OpticalTweezersTrap>();

        default: return nullptr;
    }
}

std::vector<std::pair<int, const char*>> listSystems() {
    return {
        {1,  "Mass-Spring-Damper"},
        {2,  "Coupled Mass-Spring-Damper"},
        {3,  "Inverted Pendulum on Cart"},
        {4,  "Double Inverted Pendulum on Cart"},
        {5,  "Triple Inverted Pendulum on Cart"},
        {6,  "Pendubot"},
        {7,  "Acrobot"},
        {8,  "Furuta Pendulum"},
        {9,  "Ball on Beam"},
        {10, "Ball on Plate"},
        {11, "Bouncing Ball"},
        {12, "Segway Robot"},
        {13, "Gantry Crane"},
        {14, "Double Pendulum Gantry Crane"},
        {15, "Magnetic Levitation"},
        {16, "Dual Magnetic Levitation"},
        {17, "Quarter-Car Suspension"},
        {18, "Half-Car Suspension"},
        {19, "Vibration Isolation Platform"},
        {20, "DC Motor Speed Control"},
        {21, "DC Motor Position Control"},
        {22, "Flexible Shaft"},
        {23, "Disk Drive Head"},
        {24, "Reaction Wheel (Single Axis)"},
        {25, "Reaction Wheel 2D"},
        {26, "Control Moment Gyroscope"},
        {27, "Flywheel Energy Storage"},
        {28, "Planar Quadrotor"},
        {29, "2D Rocket Landing"},
        {30, "Fixed-Wing Aircraft 2D"},
        {31, "Bicycle Lean"},
        {32, "Hovercraft 2D"},
        {33, "Single Zone Oven"},
        {34, "Multi-Zone Oven"},
        {35, "Heat Exchanger"},
        {36, "Thermoelectric Cooler"},
        {37, "Room HVAC"},
        {38, "Single Tank Level"},
        {39, "Coupled Two-Tank"},
        {40, "Four-Tank System"},
        {41, "Hydraulic Actuator"},
        {42, "Pneumatic Muscle"},
        {43, "Pressure Vessel"},
        {44, "Buck Converter"},
        {45, "Boost Converter"},
        {46, "Buck-Boost Converter"},
        {47, "Power Grid Frequency"},
        {48, "Active Power Filter"},
        {49, "Phase-Locked Loop"},
        {50, "CSTR"},
        {51, "pH Neutralization"},
        {52, "Distillation Column"},
        {53, "Bioreactor"},
        {54, "Blood Glucose"},
        {55, "Anesthesia Control"},
        {56, "Predator-Prey"},
        {57, "Lorenz System"},
        {58, "Rössler System"},
        {59, "Chua Circuit"},
        {60, "Duffing Oscillator"},
        {61, "Kapitza Pendulum"},
        {62, "Triple-Link Gymnast"},
        {63, "Smith Predictor Plant"},
        {64, "Networked Control System"},
        {65, "Conveyor Belt Tracking"},
        {66, "Adaptive Optics Telescope"},
        {67, "Fast Steering Mirror"},
        {68, "Pound-Drever-Hall Lock"},
        {69, "Active Seismic Isolation Bench"},
        {70, "Optical Tweezers Trap"},
    };
}

int systemCount() {
    return 70;
}

} // namespace Simulation
