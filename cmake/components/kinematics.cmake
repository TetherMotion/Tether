# Component: tether_kinematics
# Header-only forward/inverse kinematics and dynamics models for robotics
# and 3D-printer kinematics.
#
# This module consolidates kinematics models that previously lived in
# tether_motion_control (generic robotics: ForwardKinematics/ForwardDynamics)
# and tether_klipper (printer kinematics: DeltaPrinter, RotaryDeltaPrinter,
# KinematicsTransform, PrinterKinematics enum).
#
# All code is header-only — no .cpp sources are compiled. The component
# provides an INTERFACE library that exports the include directories and
# links tether_common for any shared type dependencies.
#
# Dependencies: common

add_library(tether_kinematics INTERFACE)
add_library(tether::kinematics ALIAS tether_kinematics)

target_include_directories(tether_kinematics INTERFACE
    $<BUILD_INTERFACE:${TETHER_ROOT}/include>
    $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether>
    $<BUILD_INTERFACE:${TETHER_ROOT}/include/tether/kinematics>
    $<INSTALL_INTERFACE:include>
    $<INSTALL_INTERFACE:include/tether>
    $<INSTALL_INTERFACE:include/tether/kinematics>
)

# Link tether_common if it exists (for shared types); otherwise this is a
# no-op since kinematics headers are self-contained.
if(TARGET tether_common)
    target_link_libraries(tether_kinematics INTERFACE tether_common)
endif()

# Export for other components and the install machinery.
set(TETHER_KINEMATICS_LIBRARY tether_kinematics)
set(TETHER_KINEMATICS_TARGETS tether_kinematics)
