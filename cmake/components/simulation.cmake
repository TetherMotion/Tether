# Component: tether_simulation
# Dynamical system simulation: 70 physical systems, integrators, sensor/actuator models, simulation engine

set(TETHER_SIMULATION_SOURCES
    ${TETHER_ROOT}/src/simulation/SimulationTypes.cpp
    ${TETHER_ROOT}/src/simulation/DynamicalSystem.cpp
    ${TETHER_ROOT}/src/simulation/Integrators.cpp
    ${TETHER_ROOT}/src/simulation/SensorActuatorModels.cpp
    ${TETHER_ROOT}/src/simulation/SimulationEngine.cpp
    ${TETHER_ROOT}/src/simulation/AllSystems.cpp
    ${TETHER_ROOT}/src/simulation/ParametricSystem.cpp
    # Mechanical systems (1-19)
    ${TETHER_ROOT}/src/simulation/systems/mechanical/MassSpringDamper.cpp
    ${TETHER_ROOT}/src/simulation/systems/mechanical/CoupledMassSpringDamper.cpp
    ${TETHER_ROOT}/src/simulation/systems/mechanical/InvertedPendulumCart.cpp
    ${TETHER_ROOT}/src/simulation/systems/mechanical/DoubleInvertedPendulumCart.cpp
    ${TETHER_ROOT}/src/simulation/systems/mechanical/TripleInvertedPendulumCart.cpp
    ${TETHER_ROOT}/src/simulation/systems/mechanical/Pendubot.cpp
    ${TETHER_ROOT}/src/simulation/systems/mechanical/Acrobot.cpp
    ${TETHER_ROOT}/src/simulation/systems/mechanical/FurutaPendulum.cpp
    ${TETHER_ROOT}/src/simulation/systems/mechanical/BallOnBeam.cpp
    ${TETHER_ROOT}/src/simulation/systems/mechanical/BallOnPlate.cpp
    ${TETHER_ROOT}/src/simulation/systems/mechanical/BouncingBall.cpp
    ${TETHER_ROOT}/src/simulation/systems/mechanical/SegwayRobot.cpp
    ${TETHER_ROOT}/src/simulation/systems/mechanical/GantryCrane.cpp
    ${TETHER_ROOT}/src/simulation/systems/mechanical/DoublePendulumGantryCrane.cpp
    ${TETHER_ROOT}/src/simulation/systems/mechanical/MagneticLevitation.cpp
    ${TETHER_ROOT}/src/simulation/systems/mechanical/DualMagneticLevitation.cpp
    ${TETHER_ROOT}/src/simulation/systems/mechanical/QuarterCarSuspension.cpp
    ${TETHER_ROOT}/src/simulation/systems/mechanical/HalfCarSuspension.cpp
    ${TETHER_ROOT}/src/simulation/systems/mechanical/VibrationIsolationPlatform.cpp
    # Rotational systems (20-27)
    ${TETHER_ROOT}/src/simulation/systems/rotational/DCMotorSpeed.cpp
    ${TETHER_ROOT}/src/simulation/systems/rotational/DCMotorPosition.cpp
    ${TETHER_ROOT}/src/simulation/systems/rotational/FlexibleShaft.cpp
    ${TETHER_ROOT}/src/simulation/systems/rotational/DiskDriveHead.cpp
    ${TETHER_ROOT}/src/simulation/systems/rotational/ReactionWheelSingleAxis.cpp
    ${TETHER_ROOT}/src/simulation/systems/rotational/ReactionWheel2D.cpp
    ${TETHER_ROOT}/src/simulation/systems/rotational/ControlMomentGyroscope.cpp
    ${TETHER_ROOT}/src/simulation/systems/rotational/FlywheelEnergyStorage.cpp
    # Aerospace systems (28-32)
    ${TETHER_ROOT}/src/simulation/systems/aerospace/PlanarQuadrotor.cpp
    ${TETHER_ROOT}/src/simulation/systems/aerospace/RocketLanding2D.cpp
    ${TETHER_ROOT}/src/simulation/systems/aerospace/FixedWingAircraft2D.cpp
    ${TETHER_ROOT}/src/simulation/systems/aerospace/BicycleLean.cpp
    ${TETHER_ROOT}/src/simulation/systems/aerospace/Hovercraft2D.cpp
    # Thermal systems (33-37)
    ${TETHER_ROOT}/src/simulation/systems/thermal/SingleZoneOven.cpp
    ${TETHER_ROOT}/src/simulation/systems/thermal/MultiZoneOven.cpp
    ${TETHER_ROOT}/src/simulation/systems/thermal/HeatExchanger.cpp
    ${TETHER_ROOT}/src/simulation/systems/thermal/ThermoelectricCooler.cpp
    ${TETHER_ROOT}/src/simulation/systems/thermal/RoomHVAC.cpp
    # Fluid systems (38-43)
    ${TETHER_ROOT}/src/simulation/systems/fluid/SingleTankLevel.cpp
    ${TETHER_ROOT}/src/simulation/systems/fluid/CoupledTwoTank.cpp
    ${TETHER_ROOT}/src/simulation/systems/fluid/FourTankSystem.cpp
    ${TETHER_ROOT}/src/simulation/systems/fluid/HydraulicActuator.cpp
    ${TETHER_ROOT}/src/simulation/systems/fluid/PneumaticMuscle.cpp
    ${TETHER_ROOT}/src/simulation/systems/fluid/PressureVessel.cpp
    # Electrical systems (44-49)
    ${TETHER_ROOT}/src/simulation/systems/electrical/BuckConverter.cpp
    ${TETHER_ROOT}/src/simulation/systems/electrical/BoostConverter.cpp
    ${TETHER_ROOT}/src/simulation/systems/electrical/BuckBoostConverter.cpp
    ${TETHER_ROOT}/src/simulation/systems/electrical/PowerGridFrequency.cpp
    ${TETHER_ROOT}/src/simulation/systems/electrical/ActivePowerFilter.cpp
    ${TETHER_ROOT}/src/simulation/systems/electrical/PhaseLockLoop.cpp
    # Chemical systems (50-53)
    ${TETHER_ROOT}/src/simulation/systems/chemical/CSTR.cpp
    ${TETHER_ROOT}/src/simulation/systems/chemical/pHNeutralization.cpp
    ${TETHER_ROOT}/src/simulation/systems/chemical/DistillationColumn.cpp
    ${TETHER_ROOT}/src/simulation/systems/chemical/Bioreactor.cpp
    # Biological systems (54-56)
    ${TETHER_ROOT}/src/simulation/systems/biological/BloodGlucose.cpp
    ${TETHER_ROOT}/src/simulation/systems/biological/AnesthesiaControl.cpp
    ${TETHER_ROOT}/src/simulation/systems/biological/PredatorPrey.cpp
    # Chaotic systems (57-62)
    ${TETHER_ROOT}/src/simulation/systems/chaotic/LorenzSystem.cpp
    ${TETHER_ROOT}/src/simulation/systems/chaotic/RosslerSystem.cpp
    ${TETHER_ROOT}/src/simulation/systems/chaotic/ChuaCircuit.cpp
    ${TETHER_ROOT}/src/simulation/systems/chaotic/DuffingOscillator.cpp
    ${TETHER_ROOT}/src/simulation/systems/chaotic/KapitzaPendulum.cpp
    ${TETHER_ROOT}/src/simulation/systems/chaotic/TripleLinkGymnast.cpp
    # Delay systems (63-65)
    ${TETHER_ROOT}/src/simulation/systems/delay/SmithPredictorPlant.cpp
    ${TETHER_ROOT}/src/simulation/systems/delay/NetworkedControlSystem.cpp
    ${TETHER_ROOT}/src/simulation/systems/delay/ConveyorBeltTracking.cpp
    # Optical systems (66-70)
    ${TETHER_ROOT}/src/simulation/systems/optical/AdaptiveOpticsTelescope.cpp
    ${TETHER_ROOT}/src/simulation/systems/optical/FastSteeringMirror.cpp
    ${TETHER_ROOT}/src/simulation/systems/optical/PoundDreverHallLock.cpp
    ${TETHER_ROOT}/src/simulation/systems/optical/ActiveSeismicIsolationBench.cpp
    ${TETHER_ROOT}/src/simulation/systems/optical/OpticalTweezersTrap.cpp
)

# Filter to only existing files
set(TETHER_SIMULATION_SOURCES_FILTERED "")
foreach(src ${TETHER_SIMULATION_SOURCES})
    if(EXISTS ${src})
        list(APPEND TETHER_SIMULATION_SOURCES_FILTERED ${src})
    endif()
endforeach()

# Create the simulation library
add_library(tether_simulation STATIC ${TETHER_SIMULATION_SOURCES_FILTERED})
add_library(tether::simulation ALIAS tether_simulation)

target_include_directories(tether_simulation
    PUBLIC
        $<BUILD_INTERFACE:${TETHER_ROOT}/include>
        $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether>
        $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/simulation>
        $<INSTALL_INTERFACE:include>
        $<INSTALL_INTERFACE:include/tether>
        $<INSTALL_INTERFACE:include/tether/simulation>
    PRIVATE
        ${TETHER_ROOT}/src
)

target_link_libraries(tether_simulation
    PUBLIC tether_common
)

set_target_properties(tether_simulation PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
)

set(TETHER_SIMULATION_LIBRARY tether_simulation PARENT_SCOPE)
