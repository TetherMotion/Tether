/**
 * @file PrinterObjectRegistry.hpp
 * @brief Printer object storage for KlippyInstance.
 *
 * @details
 * Extracted from KlippyInstance.hpp to reduce the god-object problem.
 * All printer object shared_ptrs (toolhead, extruder, heater_bed, fan,
 * bed_mesh, etc.) are grouped here in a single struct.
 *
 * KlippyInstance inherits privately from PrinterObjectRegistry so that
 * the .ipp callback files can reference the object fields by name without
 * changes. The setupObjects() method is also moved here to separate the
 * object creation/registration logic from the rest of KlippyInstance.
 */

#pragma once

#include "tether/klipper/klippy/PrinterObjects.hpp"
#include "tether/klipper/klippy/PrinterObjectsE2.hpp"
#include "tether/klipper/klippy/VirtualSdcard.hpp"
#include "tether/klipper/klippy/FirmwareRetraction.hpp"
#if TETHER_ENABLE_PRESSURE_ADVANCE
#include "tether/klipper/klippy/PressureAdvance.hpp"
#endif
#include "tether/klipper/klippy/InputShaper.hpp"
#include "tether/klipper/objects/BedLevel.hpp"
#include "tether/klipper/objects/Peripherals.hpp"
#include "tether/klipper/objects/Homing.hpp"

#include <memory>
#include <string>
#include <map>

namespace tether::klipper::klippy {

// Forward declarations
class KlippyUdsServer;
class KlippyInstanceConfig;

/// @brief Printer object storage and setup for KlippyInstance.
///
/// Holds all printer object shared_ptrs and provides the setupObjects()
/// method that creates and registers them with the UDS server.
struct PrinterObjectRegistry {
    // --- Core printer objects ---
    std::shared_ptr<ToolheadObject> toolheadObj_;
    std::shared_ptr<DisplayStatusObject> displayStatusObj_;
    std::shared_ptr<PauseResumeObject> pauseResumeObj_;
    std::shared_ptr<PrintStatsObject> printStatsObj_;
    std::shared_ptr<MotionReportObject> motionReportObj_;
    std::shared_ptr<ExtruderObject> extruderObj_;
    std::shared_ptr<HeaterBedObject> heaterBedObj_;
    std::shared_ptr<FanObject> fanObj_;
    std::shared_ptr<ProbeObject> probeObj_;
    std::shared_ptr<BedMeshObject> bedMeshObj_;
    std::shared_ptr<QueryEndstopsObject> queryEndstopsObj_;
    std::shared_ptr<Adxl345Object> adxl345Obj_;
    std::shared_ptr<McuObject> mcuObj_;
    std::shared_ptr<SystemStatsObject> systemStatsObj_;
    std::shared_ptr<IdleTimeoutObject> idleTimeoutObj_;
    std::shared_ptr<StepperEnableObject> stepperEnableObj_;
    std::shared_ptr<GcodeMoveObject> gcodeMoveObj_;
    std::shared_ptr<ConfigfileObject> configfileObj_;
    std::shared_ptr<WebhooksObject> webhooksObj_;
    std::shared_ptr<FirmwareRetractionObject> firmwareRetractionObj_;

    // --- B4: Extended printer objects ---
    std::shared_ptr<OutputPinObject> outputPinObj_;
    std::shared_ptr<PWMToolObject> pwmToolObj_;
    std::shared_ptr<TemperatureFanObject> temperatureFanObj_;
    std::shared_ptr<ControllerFanObject> controllerFanObj_;
    std::shared_ptr<HeaterFanObject> heaterFanObj_;
    std::shared_ptr<FanGenericObject> fanGenericObj_;
    std::shared_ptr<LedObject> ledObj_;
    std::shared_ptr<DotstarObject> dotstarObj_;
    std::shared_ptr<ServoObject> servoObj_;
    std::shared_ptr<BltouchObject> bltouchObj_;
    std::shared_ptr<ZTiltObject> zTiltObj_;
    std::shared_ptr<QuadGantryLevelObject> quadGantryLevelObj_;
    std::shared_ptr<ScrewsTiltAdjustObject> screwsTiltAdjustObj_;
    std::shared_ptr<BedScrewsObject> bedScrewsObj_;
    std::shared_ptr<DeltaCalibrateObject> deltaCalibrateObj_;
    std::shared_ptr<SkewCorrectionObject> skewCorrectionObj_;
    std::shared_ptr<InputShaperObject> inputShaperObj_;
#if TETHER_ENABLE_PRESSURE_ADVANCE
    std::shared_ptr<PressureAdvanceObject> pressureAdvanceObj_;
#endif
    std::shared_ptr<ExcludeObjectObject> excludeObjectObj_;
    std::shared_ptr<ZThermalAdjustObject> zThermalAdjustObj_;
    std::shared_ptr<HeaterGenericObject> heaterGenericObj_;
    std::shared_ptr<TemperatureProbeObject> temperatureProbeObj_;
    std::shared_ptr<ForceMoveObject> forceMoveObj_;
    std::shared_ptr<DualCarriageObject> dualCarriageObj_;
    std::shared_ptr<ExtruderStepperObject> extruderStepperObj_;
    std::shared_ptr<ManualStepperObject> manualStepperObj_;
    std::shared_ptr<EndstopPhaseObject> endstopPhaseObj_;
    std::shared_ptr<SafeZHomeObject> safeZHomeObj_;
    std::shared_ptr<BedTiltObject> bedTiltObj_;
    std::shared_ptr<MultiPinObject> multiPinObj_;
    std::shared_ptr<ButtonObject> buttonObj_;
    std::shared_ptr<SmartEffectorObject> smartEffectorObj_;
    std::shared_ptr<TmcDriverObject> tmcDriverObj_;

    // --- D2: Additional printer objects ---
    std::shared_ptr<class ManualProbeObject> manualProbeObj_;
    std::shared_ptr<class FilamentMotionSensorObject> filamentMotionSensorObj_;
    std::shared_ptr<class LoadCellObject> loadCellObj_;
    std::shared_ptr<class CanbusStatsObject> canbusStatsObj_;
    std::shared_ptr<class PWMCycleTimeObject> pwmCycleTimeObj_;
    std::shared_ptr<class ResonanceTesterObject> resonanceTesterObj_;
    std::shared_ptr<class AngleObject> angleObj_;
    std::shared_ptr<class Palette2Object> palette2Obj_;
    std::shared_ptr<class MenuObject> menuObj_;
    std::shared_ptr<class GcodeObject> gcodeObj_;

    // --- E4: Additional printer objects from PrinterObjectsE2.hpp ---
    std::shared_ptr<DelayedGcodeObject> delayedGcodeObj_;
    std::shared_ptr<SaveVariablesObject> saveVariablesObj_;
    std::shared_ptr<BoardPinsObject> boardPinsObj_;
};

} // namespace tether::klipper::klippy
