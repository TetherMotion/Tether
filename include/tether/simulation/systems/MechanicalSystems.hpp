#pragma once

#include "../DynamicalSystem.hpp"

/**
 * @file MechanicalSystems.hpp
 * @brief Aggregated include for translational mechanical benchmark systems.
 *
 * Include this header when you want the complete mechanical benchmark catalog.
 * Include the per-system headers under `systems/mechanical/` for narrower
 * compile dependencies and module-focused tests.
 */

#include "mechanical/MassSpringDamper.hpp"
#include "mechanical/CoupledMassSpringDamper.hpp"
#include "mechanical/InvertedPendulumCart.hpp"
#include "mechanical/DoubleInvertedPendulumCart.hpp"
#include "mechanical/TripleInvertedPendulumCart.hpp"
#include "mechanical/Pendubot.hpp"
#include "mechanical/Acrobot.hpp"
#include "mechanical/FurutaPendulum.hpp"
#include "mechanical/BallOnBeam.hpp"
#include "mechanical/BallOnPlate.hpp"
#include "mechanical/BouncingBall.hpp"
#include "mechanical/SegwayRobot.hpp"
#include "mechanical/GantryCrane.hpp"
#include "mechanical/DoublePendulumGantryCrane.hpp"
#include "mechanical/MagneticLevitation.hpp"
#include "mechanical/DualMagneticLevitation.hpp"
#include "mechanical/QuarterCarSuspension.hpp"
#include "mechanical/HalfCarSuspension.hpp"
#include "mechanical/VibrationIsolationPlatform.hpp"
