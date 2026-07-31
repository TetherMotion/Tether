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
#include "tether/fsoe/FSoEMasterConnection.hpp"
#include "tether/fsoe/FSoEMaster.hpp"
#include "tether/fsoe/TypedProcessData.hpp"
#include "tether/fsoe/Synapticon/SafeMotionFSoE.hpp"
#endif

// HAL (Hardware Abstraction Layer)
#ifdef TETHER_ENABLE_HAL
#include "tether/hal/HAL.hpp"
#include "tether/hal/IClock.hpp"
#include "tether/hal/IEthernet.hpp"
#include "tether/hal/IPeriodicTimer.hpp"
#endif

// Kinematics (robotics + printer kinematics models)
#ifdef TETHER_ENABLE_KINEMATICS
#include "tether/kinematics/ForwardKinematics.hpp"
#include "tether/kinematics/ForwardDynamics.hpp"
#include "tether/kinematics/PrinterKinematics.hpp"
#include "tether/kinematics/DeltaPrinter.hpp"
#include "tether/kinematics/RotaryDeltaPrinter.hpp"
#include "tether/kinematics/KinematicsTransform.hpp"
#endif

// EtherCAT
#ifdef TETHER_ENABLE_ETHERCAT
#include "tether/ethercat/Types.hpp"
#include "tether/ethercat/SDOManager.hpp"
#include "tether/ethercat/PDOManager.hpp"
#endif

// Klipper protocol
#ifdef TETHER_ENABLE_KLIPPER
#include "tether/klipper/protocol/Crc16.hpp"
#include "tether/klipper/protocol/Vlq.hpp"
#include "tether/klipper/protocol/MessageBlock.hpp"
#include "tether/klipper/protocol/Constants.hpp"
#include "tether/klipper/protocol/ParameterFormat.hpp"
#include "tether/klipper/protocol/DataDictionary.hpp"
#include "tether/klipper/protocol/IdentifyProtocol.hpp"
#include "tether/klipper/protocol/CommandTable.hpp"
#include "tether/klipper/transport/IByteStreamTransport.hpp"
#include "tether/klipper/transport/LoopbackTransport.hpp"
#include "tether/klipper/transport/PipeTransport.hpp"
#include "tether/klipper/transport/TcpStreamTransport.hpp"
#ifdef TETHER_ENABLE_KLIPPER_CAN
#include "tether/klipper/transport/CanTransport.hpp"
#include "tether/hal/ICan.hpp"
#endif
#include "tether/klipper/reliability/SequenceCounter.hpp"
#include "tether/klipper/reliability/RtoEstimator.hpp"
#include "tether/klipper/reliability/SerialQueue.hpp"
#include "tether/klipper/reliability/AckMessage.hpp"
#include "tether/klipper/clock/McuClock.hpp"
#include "tether/klipper/clock/ClockSync.hpp"
#include "tether/klipper/objects/OidAllocator.hpp"
#include "tether/klipper/objects/Stepper.hpp"
#include "tether/klipper/objects/Peripherals.hpp"
#include "tether/klipper/motion/MotionBlock.hpp"
#include "tether/klipper/motion/MotionTranslator.hpp"
#include "tether/klipper/motion/MotionReconstructor.hpp"
#include "tether/klipper/motion/MotionBlockSink.hpp"
#include "tether/klipper/klippy/KlippyHost.hpp"
#include "tether/klipper/device/KlipperDevice.hpp"
#include "tether/klipper/config/KlipperConfig.hpp"
#include "tether/klipper/config/StandardCommands.hpp"
#endif
