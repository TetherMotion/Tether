#pragma once

/**
 * @file Tether.hpp
 * @brief Main include header for the Tether library
 * 
 * This header includes all the major components of the Tether library.
 * Users can include this single header or include specific components directly.
 */

// Configuration
#if __has_include("tether/TetherConfig.hpp")
#include "tether/TetherConfig.hpp"
#endif

// GCode Interpreter
#ifdef TETHER_ENABLE_GCODE
#include "tether/gcode/GCodeParser.hpp"
#include "tether/gcode/GCodeInterpreter.hpp"
#include "tether/gcode/motion/InterpolationStrategy.hpp"
#include "tether/gcode/motion/G64CornerMode.hpp"
#endif

// CiA 402 Drive Profile
#ifdef TETHER_ENABLE_CIA402
#include "tether/cia402/CiA402Drive.hpp"
#include "tether/cia402/CiA402StateMachine.hpp"
#include "tether/cia402/MotorModel.hpp"
#include "tether/cia402/AdvancedMotorModel.hpp"
#include "tether/cia402/MotionProfile.hpp"
#endif

// Motion Control
#ifdef TETHER_ENABLE_MOTION
#include "tether/motion/MotionGenerator.hpp"
#endif

// Control Algorithms
#ifdef TETHER_ENABLE_CONTROL
#include "tether/control/PIDControllers.hpp"
#include "tether/control/Controllers.hpp"
#endif

// FSoE (Functional Safety over EtherCAT)
#ifdef TETHER_ENABLE_FSOE
#include "tether/fsoe/FSoESlave.hpp"
#include "tether/fsoe/FSoEConnection.hpp"
#include "tether/fsoe/TypedProcessData.hpp"
#include "tether/drives/Synapticon/SafeMotionFSoE.hpp"
#endif

// HAL (Hardware Abstraction Layer)
#ifdef TETHER_ENABLE_HAL
#include "tether/hal/HAL.hpp"
#include "tether/hal/IClock.hpp"
#include "tether/hal/IEthernet.hpp"
#include "tether/hal/IPeriodicTimer.hpp"
#endif

// EtherCAT
#ifdef TETHER_ENABLE_ETHERCAT
#include "tether/ethercat/Types.hpp"
#include "tether/ethercat/SDOManager.hpp"
#include "tether/ethercat/PDOManager.hpp"
#endif
