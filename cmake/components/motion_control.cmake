# Component: tether_motion_control
# Real-time motion control, profiles, motor models, and kinematics

set(TETHER_MOTION_CONTROL_SOURCES
    # Motion generators and profiles
    ${TETHER_ROOT}/src/motion/MotionGenerator.cpp
    ${TETHER_ROOT}/src/identification/DenseLinearAlgebra.cpp
    ${TETHER_ROOT}/src/identification/StepResponse.cpp
    ${TETHER_ROOT}/src/identification/LeastSquares.cpp
    ${TETHER_ROOT}/src/identification/FrictionIdentification.cpp
    ${TETHER_ROOT}/src/identification/FrequencyIdentification.cpp
    ${TETHER_ROOT}/src/identification/PolynomialModels.cpp
    ${TETHER_ROOT}/src/identification/SubspaceIdentification.cpp
    ${TETHER_ROOT}/src/identification/AdvancedFrictionModels.cpp
    ${TETHER_ROOT}/src/identification/RigidBodyIdentification.cpp
    ${TETHER_ROOT}/src/identification/AdaptiveObservers.cpp
    ${TETHER_ROOT}/src/identification/NonlinearIdentification.cpp
    
    # CiA 402 motion control (not drive protocol, but motion logic)
    ${TETHER_ROOT}/src/profiles/cia402/MotionController.cpp
    ${TETHER_ROOT}/src/profiles/cia402/AxisMotion.cpp
    ${TETHER_ROOT}/src/profiles/cia402/PathMotion.cpp
    ${TETHER_ROOT}/src/profiles/cia402/ProfileGenerator.cpp
    ${TETHER_ROOT}/src/profiles/cia402/MultiAxisPath.cpp
    ${TETHER_ROOT}/src/profiles/cia402/MotorModel.cpp
    ${TETHER_ROOT}/src/profiles/cia402/AdvancedMotorModelCore.cpp
    ${TETHER_ROOT}/src/profiles/cia402/AdvancedMotorModelPhysics.cpp
    ${TETHER_ROOT}/src/profiles/cia402/AdvancedMotorModelFactory.cpp
    ${TETHER_ROOT}/src/profiles/cia402/MotorModelFactory.cpp
    ${TETHER_ROOT}/src/profiles/cia402/CiA402PIDController.cpp
)

# Filter to only existing files
set(TETHER_MOTION_CONTROL_SOURCES_FILTERED "")
foreach(src ${TETHER_MOTION_CONTROL_SOURCES})
    if(EXISTS ${src})
        list(APPEND TETHER_MOTION_CONTROL_SOURCES_FILTERED ${src})
    endif()
endforeach()

# Fallback
if(NOT TETHER_MOTION_CONTROL_SOURCES_FILTERED)
    file(GLOB_RECURSE TETHER_MOTION_CONTROL_SOURCES_FILTERED "${TETHER_ROOT}/src/motion/*.cpp")
endif()

# Create variant targets
set(_variants "")
if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_motion_control_shared SHARED ${TETHER_MOTION_CONTROL_SOURCES_FILTERED})
    list(APPEND _variants tether_motion_control_shared)
endif()
if(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_motion_control_static STATIC ${TETHER_MOTION_CONTROL_SOURCES_FILTERED})
    list(APPEND _variants tether_motion_control_static)
endif()

foreach(_tgt IN LISTS _variants)
    target_include_directories(${_tgt}
        PUBLIC
            $<BUILD_INTERFACE:${TETHER_ROOT}/include>
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether>
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/motion>
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/profiles/cia402>
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/kinematics>
            $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/identification>
            $<INSTALL_INTERFACE:include>
            $<INSTALL_INTERFACE:include/tether>
        PRIVATE
            ${TETHER_ROOT}/src
    )

    target_link_libraries(${_tgt} PUBLIC tether_common tether_controls)

    set_target_properties(${_tgt} PROPERTIES
        POSITION_INDEPENDENT_CODE ON
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED ON
    )
endforeach()

if(TETHER_BUILD_SHARED_LIBS)
    add_library(tether_motion_control ALIAS tether_motion_control_shared)
    add_library(tether::motion_control ALIAS tether_motion_control_shared)
elseif(TETHER_BUILD_STATIC_LIBS)
    add_library(tether_motion_control ALIAS tether_motion_control_static)
    add_library(tether::motion_control ALIAS tether_motion_control_static)
endif()

set(TETHER_MOTION_CONTROL_LIBRARY tether_motion_control)
set(TETHER_MOTION_CONTROL_TARGETS ${_variants})
