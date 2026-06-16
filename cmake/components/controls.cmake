# Component: tether_controls
# Control algorithms: PID, state-space, LQR, LQG, robust controllers, learning controllers

set(TETHER_CONTROLS_SOURCES
    ${TETHER_ROOT}/src/control/ControllerBase.cpp
    ${TETHER_ROOT}/src/control/PIDControllers.cpp
    ${TETHER_ROOT}/src/control/FractionalPID.cpp
    ${TETHER_ROOT}/src/control/KalmanFilter.cpp
    ${TETHER_ROOT}/src/control/LQRController.cpp
    ${TETHER_ROOT}/src/control/LQGLQIControllers.cpp
    ${TETHER_ROOT}/src/control/RobustControllers.cpp
    ${TETHER_ROOT}/src/control/LearningControllers.cpp
    ${TETHER_ROOT}/src/control/CompositeControllers.cpp
    ${TETHER_ROOT}/src/control/MatrixUtils.cpp
    # Autotuning sources
    ${TETHER_ROOT}/src/control/autotuning/AdaptiveMethods.cpp
    ${TETHER_ROOT}/src/control/autotuning/AutotuningFramework.cpp
    ${TETHER_ROOT}/src/control/autotuning/classical/Common.cpp
    ${TETHER_ROOT}/src/control/autotuning/classical/ZieglerNicholsStepResponse.cpp
    ${TETHER_ROOT}/src/control/autotuning/classical/ZieglerNicholsUltimateCycle.cpp
    ${TETHER_ROOT}/src/control/autotuning/classical/TyreusLuyben.cpp
    ${TETHER_ROOT}/src/control/autotuning/classical/CohenCoon.cpp
    ${TETHER_ROOT}/src/control/autotuning/classical/ChienHronesReswick.cpp
    ${TETHER_ROOT}/src/control/autotuning/classical/AstromHagglundRelay.cpp
    ${TETHER_ROOT}/src/control/autotuning/classical/LopezMethod.cpp
    ${TETHER_ROOT}/src/control/autotuning/classical/LambdaTuning.cpp
    ${TETHER_ROOT}/src/control/autotuning/classical/SIMCMethod.cpp
    ${TETHER_ROOT}/src/control/autotuning/classical/AMIGOMethod.cpp
    ${TETHER_ROOT}/src/control/autotuning/classical/ClassicalTuningFactory.cpp
    ${TETHER_ROOT}/src/control/autotuning/HybridMethods.cpp
    ${TETHER_ROOT}/src/control/autotuning/IndustrialAutotuners.cpp
    ${TETHER_ROOT}/src/control/autotuning/LQRTuning.cpp
    ${TETHER_ROOT}/src/control/autotuning/model_based/IMCDesign.cpp
    ${TETHER_ROOT}/src/control/autotuning/model_based/PolePlacement.cpp
    ${TETHER_ROOT}/src/control/autotuning/model_based/LoopShaping.cpp
    ${TETHER_ROOT}/src/control/autotuning/model_based/DirectSynthesis.cpp
    ${TETHER_ROOT}/src/control/autotuning/model_based/SmithPredictor.cpp
    ${TETHER_ROOT}/src/control/autotuning/model_based/DahlinAlgorithm.cpp
    ${TETHER_ROOT}/src/control/autotuning/model_based/DeadbeatControl.cpp
    ${TETHER_ROOT}/src/control/autotuning/model_based/MinimumVarianceControl.cpp
    ${TETHER_ROOT}/src/control/autotuning/MuSynthesis.cpp
    ${TETHER_ROOT}/src/control/autotuning/OptimizationAlgorithms.cpp
    ${TETHER_ROOT}/src/control/autotuning/QFT.cpp
    ${TETHER_ROOT}/src/control/autotuning/SlidingModeControl.cpp
)

# Filter to only existing files
set(TETHER_CONTROLS_SOURCES_FILTERED "")
foreach(src ${TETHER_CONTROLS_SOURCES})
    if(EXISTS ${src})
        list(APPEND TETHER_CONTROLS_SOURCES_FILTERED ${src})
    endif()
endforeach()

# Get all source files from control directory as fallback
if(NOT TETHER_CONTROLS_SOURCES_FILTERED)
    file(GLOB_RECURSE TETHER_CONTROLS_SOURCES_FILTERED "${TETHER_ROOT}/src/control/*.cpp")
endif()

# Create the controls library
add_library(tether_controls STATIC ${TETHER_CONTROLS_SOURCES_FILTERED})
add_library(tether::controls ALIAS tether_controls)

target_include_directories(tether_controls
    PUBLIC
        $<BUILD_INTERFACE:${TETHER_ROOT}/include>
        $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether>
        $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/control>
        $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/control/autotuning>
        $<INSTALL_INTERFACE:include>
        $<INSTALL_INTERFACE:include/tether>
        $<INSTALL_INTERFACE:include/tether/control>
        $<INSTALL_INTERFACE:include/tether/control/autotuning>
    PRIVATE
        ${TETHER_ROOT}/src
)

target_link_libraries(tether_controls
    PUBLIC tether_common
)

set_target_properties(tether_controls PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
)

# Export for other components
set(TETHER_CONTROLS_LIBRARY tether_controls)
