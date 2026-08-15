# Component: tether_controls
# Core control algorithms: PID, state-space, LQR, LQG, robust controllers, learning
# controllers, composite controllers, matrix utilities.
#
# Autotuning methods live in the separate tether_autotuning component (opt-in).

set(TETHER_CONTROLS_SOURCES
    ${TETHER_ROOT}/src/control/ControllerBase.cpp
    ${TETHER_ROOT}/src/control/PIDControllers.cpp
    ${TETHER_ROOT}/src/control/FractionalPID.cpp
    ${TETHER_ROOT}/src/control/KalmanFilter.cpp
    ${TETHER_ROOT}/src/control/ExtendedKalmanFilter.cpp
    ${TETHER_ROOT}/src/control/LQRController.cpp
    ${TETHER_ROOT}/src/control/LQGLQIControllers.cpp
    ${TETHER_ROOT}/src/control/RobustControllers.cpp
    ${TETHER_ROOT}/src/control/LearningControllers.cpp
    ${TETHER_ROOT}/src/control/CompositeControllers.cpp
    ${TETHER_ROOT}/src/control/MatrixUtils.cpp
    # Non-Newtonian extrusion + flow-adaptive temperature control
    ${TETHER_ROOT}/src/control/extrusion/CrossWlfRheology.cpp
    ${TETHER_ROOT}/src/control/extrusion/PressureFlowLut.cpp
    ${TETHER_ROOT}/src/control/extrusion/MeltZoneThermalObserver.cpp
    ${TETHER_ROOT}/src/control/extrusion/FlowAdaptiveHeaterController.cpp
    # LPV/LTI deconvolution controllers
    ${TETHER_ROOT}/src/control/extrusion/LTIFrequencyDomainDeconvolver.cpp
    ${TETHER_ROOT}/src/control/extrusion/OverlapAddLPVDeconvolver.cpp
    ${TETHER_ROOT}/src/control/extrusion/ARXLPVInverseFilter.cpp
    ${TETHER_ROOT}/src/control/extrusion/StateSpaceLPVInputEstimator.cpp
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

# Create variant targets
set(_variants "")
if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_controls_shared SHARED ${TETHER_CONTROLS_SOURCES_FILTERED})
    list(APPEND _variants tether_controls_shared)
endif()
if(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_controls_static STATIC ${TETHER_CONTROLS_SOURCES_FILTERED})
    list(APPEND _variants tether_controls_static)
endif()

foreach(_tgt IN LISTS _variants)
    target_include_directories(${_tgt}
        PUBLIC
            $<BUILD_INTERFACE:${TETHER_ROOT}/include>
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether>
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/control>
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/control/autotuning>
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/control/extrusion>
            $<INSTALL_INTERFACE:include>
            $<INSTALL_INTERFACE:include/tether>
            $<INSTALL_INTERFACE:include/tether/control>
            $<INSTALL_INTERFACE:include/tether/control/autotuning>
            $<INSTALL_INTERFACE:include/tether/control/extrusion>
        PRIVATE
            ${TETHER_ROOT}/src
    )

    target_link_libraries(${_tgt} PUBLIC tether_common)

    set_target_properties(${_tgt} PROPERTIES
        POSITION_INDEPENDENT_CODE ON
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED ON
    )
endforeach()

if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_controls ALIAS tether_controls_shared)
    add_library(tether::controls ALIAS tether_controls_shared)
elseif(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_controls ALIAS tether_controls_static)
    add_library(tether::controls ALIAS tether_controls_static)
endif()

set(TETHER_CONTROLS_LIBRARY tether_controls)
set(TETHER_CONTROLS_TARGETS ${_variants})
