#pragma once

/// @file KlippyInstanceConfig.hpp
/// @brief Configuration structures for KlippyInstance.

#include "tether/kinematics/PrinterKinematics.hpp"
#include "tether/kinematics/DeltaPrinter.hpp"
#include "tether/kinematics/RotaryDeltaPrinter.hpp"
#include "tether/klipper/klippy/SkewCorrection.hpp"
#include "tether/klipper/klippy/UdsTypes.hpp"
#include "tether/klipper/transport/IByteStreamTransport.hpp"
#include "tether/klipper/protocol/DataDictionary.hpp"

#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace tether::klipper::klippy {

// The Kinematics enum, string conversion helpers, and geometry config structs
// (DeltaGeometry, DeltaEndstopAdjust, RotaryDeltaGeometry,
// RotaryDeltaEndstopAdjust, PolarConfig, WinchConfig) have been moved to the
// tether_kinematics module (tether/kinematics/). These using-declarations keep
// existing klippy code compiling with the old unqualified names.
using ::tether::kinematics::PrinterKinematics;
using ::tether::kinematics::printerKinematicsFromString;
using ::tether::kinematics::printerKinematicsToString;
using ::tether::kinematics::DeltaGeometry;
using ::tether::kinematics::DeltaEndstopAdjust;
using ::tether::kinematics::RotaryDeltaGeometry;
using ::tether::kinematics::RotaryDeltaEndstopAdjust;
using ::tether::kinematics::PolarConfig;
using ::tether::kinematics::WinchConfig;
using ::tether::kinematics::DeltaPrinter;
using ::tether::kinematics::RotaryDeltaPrinter;

/// @brief Backward-compatible alias for the kinematics enum name.
///   Existing klippy code uses `klippy::Kinematics::Cartesian` etc.
using Kinematics = PrinterKinematics;

/// @brief Backward-compatible alias for the string-to-enum converter.
inline Kinematics kinematicsFromString(const std::string& s) {
    return printerKinematicsFromString(s);
}

/// @brief Backward-compatible alias for the enum-to-string converter.
inline std::string kinematicsToString(Kinematics k) {
    return printerKinematicsToString(k);
}


/// @brief Persistent printer settings (M500-M503).
struct KlippySettings {
    // Printer kinematics and motion limits
    Kinematics kinematics = Kinematics::Cartesian;
    double maxVelocity = 3000.0;       ///< Max velocity (mm/s)
    double maxAccel = 3000.0;          ///< Max acceleration (mm/s^2)
    double maxAccelToDecel = 1500.0;   ///< Max accel-to-decel (mm/s^2)
    double squareCornerVelocity = 5.0; ///< Square corner velocity (mm/s)
    double maxZVelocity = 200.0;       ///< Max Z velocity (mm/s)
    double maxZAccel = 100.0;          ///< Max Z acceleration (mm/s^2)
    // Motion
    std::map<std::string, double> stepsPerMm = {
        {"x", 80.0}, {"y", 80.0}, {"z", 400.0}, {"e", 500.0}
    };
    std::map<std::string, double> maxFeedrate = {
        {"x", 500.0}, {"y", 500.0}, {"z", 10.0}, {"e", 25.0}
    };
    std::map<std::string, int> microstepping = {
        {"x", 16}, {"y", 16}, {"z", 16}, {"e", 16}
    };
    std::map<std::string, double> stepperCurrent = {
        {"x", 800.0}, {"y", 800.0}, {"z", 600.0}, {"e", 600.0}
    };
    std::map<std::string, int> stepperDirection = {
        {"x", 0}, {"y", 0}, {"z", 0}, {"e", 0}
    };
    double acceleration = 3000.0;
    double travelAcceleration = 3000.0;
    double jerk = 20.0;
    double startAccel = 1000.0;

    // Offsets
    std::map<std::string, double> homeOffset = {
        {"x", 0.0}, {"y", 0.0}, {"z", 0.0}
    };
    std::map<std::string, double> toolOffset[8]; // per-tool offsets
    double probeOffset = 0.0;

    // Backlash
    std::map<std::string, double> backlash = {
        {"x", 0.0}, {"y", 0.0}, {"z", 0.0}
    };

    // Filament
    double filamentDiameter = 1.75;

    // Retract
    double retractLength = 0.0;
    double retractSpeed = 0.0;
    double retractZLift = 0.0;
    double unretractLength = 0.0;
    double unretractSpeed = 0.0;

    // PID
    double hotendKp = 0.0, hotendKi = 0.0, hotendKd = 0.0;
    double bedKp = 0.0, bedKi = 0.0, bedKd = 0.0;

    // Autotuning configuration (delegated to Tether autotuning framework)
    std::string pidAutotuneMethod = "relay_feedback";  ///< Method name (see AutotuneMethod enum)
    std::string pidAutotuneForm = "pid";               ///< Controller form: "pi", "pid", "pid_filtered"
    double pidAutotuneLambda = -1.0;                   ///< Lambda for IMC/SIMC (-1 = auto)

    // Bed mesh
    bool bedMeshEnabled = false;
    std::string bedMeshMin;
    std::string bedMeshMax;
    std::string bedMeshProbeCount = "3,3";
    double bedMeshSpeed = 50.0;
    double bedMeshFadeStart = 1.0;
    double bedMeshFadeEnd = 10.0;
    double bedMeshFadeTarget = 0.0;
    std::string bedMeshAlgorithm = "lagrange";

    // Extruder
    double nozzleDiameter = 0.4;
#if TETHER_ENABLE_PRESSURE_ADVANCE
    double extruderPressureAdvance = 0.0;
    double extruderSmoothTime = 0.040;
    // Non-Newtonian extrusion compensation (extends PA; default Linear = classic PA)
    std::string extrusionCompensationModel = "linear"; ///< linear|power_law|cross_wlf
    double paFlowIndex = 1.0;              ///< n [-] (power-law flow index)
    double paConsistency = 0.0;            ///< K_base [filament-mm / (mm³/s)^n]
    double paMaxCompensation = 0.5;        ///< [mm] safety clamp
    // Cross-WLF parameters
    double crossWlfTauStar = 1.0e5;        ///< τ* [Pa]
    double crossWlfFlowIndex = 0.4;        ///< n [-] (Cross index)
    double crossWlfC1 = 17.44;             ///< WLF C1
    double crossWlfC2 = 51.6;              ///< WLF C2 [K]
    double crossWlfRefTempC = 200.0;       ///< T_ref [°C]
    double crossWlfZeroShearViscosityRef = 1000.0; ///< η_ref [Pa·s]
    double crossWlfCompressibilityOverArea = 0.0;  ///< βV_m/A_f [mm/Pa]
    std::string crossWlfLutPath;           ///< optional serialized LUT path
    // Flow-adaptive heater compensation (three-state thermal model)
    bool heaterFlowPreEmphasis = false;    ///< enable flow-adaptive heater ctrl
    double filamentHeatCapacity = 2.1;     ///< ρ·c_p [J/(mm³·K)]
    // Three-state thermal model capacitances
    double heaterBlockCapacitance = 8.0;   ///< C_h [J/K]
    double sensorCapacitance = 1.0;        ///< C_s [J/K]
    double meltZoneCapacitance = 2.0;      ///< C_m [J/K]
    // Three-state thermal model conductances
    double heaterSensorConductance = 2.0;  ///< G_hs [W/K]
    double sensorMeltConductance = 1.5;    ///< G_sm [W/K]
    // Luenberger observer gains
    double luenbergerGainHeater = 0.5;     ///< L_h [1/s]
    double luenbergerGainSensor = 2.0;     ///< L_s [1/s]
    double luenbergerGainMelt = 0.3;       ///< L_m [1/s]
    // Feed-forward parameters
    double debtTimeConstant = 2.0;         ///< τ [s]
    double maxPreEmphasisPower = 0.4;      ///< [0-1 PWM]
    double maxPostEmphasisPower = 0.2;     ///< [0-1 PWM]
    double maxHeaterOvershoot = 10.0;      ///< [°C]
    // Deconvolution feedforward controller
    std::string deconvolutionController = "none"; ///< none|lti_freq|overlap_add_lpv|arx_lpv|statespace_lpv
    double deconvolutionLambda = 1e-6;     ///< Tikhonov λ (LTI & state-space)
    bool deconvolutionEnabled = false;     ///< runtime enable/disable
    // LTI frequency-domain deconvolver
    bool ltiPadToPowerOfTwo = true;        ///< FFT padding
    // Overlap-add LPV deconvolver
    int overlapAddBlockSize = 256;         ///< Block size B (samples)
    double overlapAddOverlapRatio = 0.5;   ///< Overlap fraction [0,1)
    // ARX LPV inverse filter
    int arxNa = 2;                         ///< Order of A(z)
    int arxNb = 1;                         ///< Order of B'(z)
    // State-space LPV input estimator
    int stateSpaceStateDim = 2;            ///< State vector dimension
    int stateSpaceInputDim = 1;            ///< Input dimension
    int stateSpaceOutputDim = 1;           ///< Output dimension
#endif

    // Heater bed
    double bedMinTemp = 0.0;
    double bedMaxTemp = 300.0;

    // Fan
    double fanMaxPower = 1.0;
    double fanCycleTime = 0.010;
    double fanKickStartTime = 0.100;
    double fanOffBelow = 0.0;

    // Probe
    double probeXOffset = 0.0;
    double probeYOffset = 0.0;
    double probeSpeed = 5.0;
    int probeSampleCount = 3;
    std::string probeSamplesResult = "average";

    // MCU
    std::string mcuSerial;
    int mcuBaud = 250000;
    std::string mcuRestartMethod = "command";
    std::map<std::string, std::string> secondaryMcuSerials;

    // Virtual SD card
    std::string virtualSdcardOnErrorGcode;

    // Safe Z home
    std::string safeZHomeXYPosition = "0,0";
    double safeZHomeZHop = 0.0;
    double safeZHomeZHopSpeed = 0.0;
    double safeZHomeXYHomeSpeed = 0.0;
    bool safeZHomeMoveToPrevious = false;

    // Idle timeout
    double idleTimeout = 600.0;
    std::string idleTimeoutGcode = "TURN_OFF_HEATERS";

    // Pause/resume
    bool pauseResumeEnabled = false;
    bool pauseResumeRecoverFromSubtract = false;

    // Display status
    bool displayStatusEnabled = false;

    // Output pins
    struct OutputPinConfig {
        std::string pin;
        double value = 0.0;
        bool pwm = false;
        double cycleTime = 0.100;
        double scale = 1.0;
    };
    std::map<std::string, OutputPinConfig> outputPins;

    // Servos
    struct ServoConfig {
        std::string pin;
        double minPulseWidth = 0.001;
        double maxPulseWidth = 0.002;
        double minAngle = 0.0;
        double maxAngle = 180.0;
        double initialAngle = 0.0;
    };
    std::map<std::string, ServoConfig> servos;

    // Temperature sensors
    struct TemperatureSensorConfig {
        std::string sensorType = "NTC 100K";
        std::string sensorPin;
        double minTemp = 0.0;
        double maxTemp = 100.0;
    };
    std::map<std::string, TemperatureSensorConfig> temperatureSensors;

    // [thermistor <name>] definitions — reusable thermistor parameter sets
    // referenced by [temperature_sensor] via sensor_type.
    struct ThermistorConfig {
        double pullupResistor = 4700.0;
        double referenceVoltage = 3.3;
        double adcMax = 4095.0;
        double resistanceAt25C = 100000.0;
        double beta = 3950.0;
        /// Multi-point calibration table (temperature, resistance pairs).
        /// When non-empty, used instead of beta model.
        std::vector<std::pair<double, double>> calibrationTable;
    };
    std::map<std::string, ThermistorConfig> thermistors;

    // [thermocouple <name>] definitions — reusable thermocouple parameter sets
    struct ThermocoupleConfig {
        std::string type = "K"; // K, J, T, E, N, R, S, B
        std::string spiBus;
        std::string csPin;
    };
    std::map<std::string, ThermocoupleConfig> thermocouples;

    // [rtd <name>] definitions — reusable RTD parameter sets
    struct RtdConfig {
        double nominalResistance = 100.0;   ///< 100 for PT100, 1000 for PT1000
        double alpha = 0.003851;            ///< Temperature coefficient (PT385)
        double referenceResistor = 430.0;   ///< Reference resistor (ohms)
        double adcMax = 4095.0;
        double referenceVoltage = 3.3;
    };
    std::map<std::string, RtdConfig> rtds;

    // Temperature fans
    struct TemperatureFanConfig {
        std::string pin;
        std::string sensorType = "NTC 100K";
        std::string sensorPin;
        double maxPower = 1.0;
        double targetTemp = 0.0;
        double minTemp = 0.0;
        double maxTemp = 100.0;
    };
    std::map<std::string, TemperatureFanConfig> temperatureFans;

    // Heater fans
    struct HeaterFanConfig {
        std::string pin;
        double maxPower = 1.0;
        std::string heater = "extruder";
        double heaterTemp = 50.0;
    };
    std::map<std::string, HeaterFanConfig> heaterFans;

    // Controller fans
    struct ControllerFanConfig {
        std::string pin;
        double maxPower = 1.0;
        double idleSpeed = 0.0;
        double idleTimeout = 30.0;
    };
    std::map<std::string, ControllerFanConfig> controllerFans;

    // TMC drivers
    struct TmcSectionConfig {
        std::string driverType;  ///< tmc2209, tmc2208, tmc5160, tmc2130, tmc2660, tmc2240
        std::string stepper;
        std::string uartPin;
        std::string spiBus;
        std::string csPin;
        double runCurrent = 0.0;
        double holdCurrent = 0.0;
        double stealthchopThreshold = 0.0;
        bool interpolate = false;
        int uartAddress = 0;
        // Advanced TMC features
        bool stealthchop = false;           ///< Enable StealthChop mode
        double spreadCycleThreshold = 0.0;  ///< Speed threshold for SpreadCycle (mm/s)
        int chopperTiming = 0;             ///< Chopper timing parameter (0=auto)
        double coolstepThreshold = 0.0;     ///< CoolStep threshold speed (mm/s)
        bool stallguard = false;            ///< Enable StallGuard
        double stallguardThreshold = 0.0;   ///< StallGuard threshold
        int microsteps = 0;                 ///< Microstep resolution (0=use stepper default)
        bool multiHoming = false;           ///< Use multi-mcu homing
        double homeCurrent = 0.0;           ///< Run current during homing (A, 0=use run_current)
    };
    std::map<std::string, TmcSectionConfig> tmcDrivers;

    // ADXL345 accelerometer
    bool adxl345Configured = false;
    std::string adxl345SpiBus;
    std::string adxl345CsPin;
    int adxl345Rate = 3200;
    std::string adxl345AxesMap = "xyz";

    // TSL1401CL filament width sensor
    struct Tsl1401clConfig {
        bool configured = false;
        std::string sensorPin;
        double nominalWidth = 1.75;
        double tolerance = 0.1;
        double minWidth = 1.5;
        double maxWidth = 2.0;
        int pixelCount = 128;
        double pixelSpacing = 0.1;
    };
    Tsl1401clConfig tsl1401clConfig;

    // Input shaper
    double inputShaperFreqX = 0.0;
    double inputShaperFreqY = 0.0;
    std::string inputShaperTypeX = "ei";
    std::string inputShaperTypeY = "ei";
    double inputShaperDampingX = 0.1;
    double inputShaperDampingY = 0.1;

    // Z tilt
    bool zTiltEnabled = false;
    std::string zTiltPositions;
    int zTiltRetries = 0;
    double zTiltRetryTolerance = 0.0;
    std::vector<double> zTiltAdjustments; ///< Computed Z tilt adjustments per stepper

    // Quad gantry level
    bool qglEnabled = false;
    std::string qglPositions;
    int qglRetries = 0;

    // Bed screws
    bool bedScrewsEnabled = false;
    std::string bedScrewsList;
    double bedScrewsProbeSpeed = 0.0;

    // Screws tilt adjust
    bool screwsTiltEnabled = false;
    std::string screwsTiltList;
    std::string screwsTiltThread = "CW-M3";
    double screwsTiltHorizontalZ = 0.0;

    // Delta printer
    DeltaGeometry deltaGeometry;
    DeltaEndstopAdjust deltaEndstopAdjust;

    // Rotary delta printer
    RotaryDeltaGeometry rotaryDeltaGeometry;
    RotaryDeltaEndstopAdjust rotaryDeltaEndstopAdjust;

    // Polar printer (PolarConfig is defined in tether::kinematics)
    PolarConfig polarConfig;

    // Winch printer (WinchConfig is defined in tether::kinematics)
    WinchConfig winchConfig;

    // Skew correction
    SkewParams skewParams;

    // Case light
    bool caseLightOn = false;
    double caseLightBrightness = 1.0;
};

/// @brief Configuration for the optional motion backend.
///
/// When provided, KlippyInstance creates a KlippyHost (+ optional in-process
/// KlipperDevice) and routes G-code moves through the real Klipper wire
/// protocol (MotionPlan → MotionTranslator → queue_step) instead of just
/// updating the printer object model.
struct MotionBackendConfig {
    /// Host-side transport (required). Connects to the device.
    std::shared_ptr<transport::IByteStreamTransport> hostTransport;
    /// Device-side transport (optional). If provided, an in-process
    /// KlipperDevice is created and pumped automatically during
    /// connect/sync/move. If null, the user is responsible for pumping
    /// the device side externally.
    std::shared_ptr<transport::IByteStreamTransport> deviceTransport;
    /// Pre-built data dictionary. If empty (no messages), it is built
    /// from withStandardCommands() at construction time.
    protocol::DataDictionary dict;
    /// MCU clock frequency in Hz.
    uint32_t clockFreqHz = 180000000;
    /// Per-axis steps per millimeter (X, Y, Z, E).
    std::array<double, 4> stepsPerMm = {80.0, 80.0, 400.0, 500.0};
    /// Per-axis direction invert (true = reverse direction).
    std::array<bool, 4> invertDirection = {false, false, false, false};
    /// Per-axis OIDs (must match the device's registered stepper OIDs).
    std::array<uint8_t, 4> axisOids = {0, 1, 2, 3};
    /// Sample interval for motion discretization (seconds).
    double sampleIntervalSec = 0.0002;
    /// If true, connect + download dict + sync clock on construction.
    bool autoConnect = true;
    /// If true and deviceTransport is set, register 4 steppers on the
    /// device and enable default queue_step motion handlers.
    bool registerDeviceSteppers = true;
#if TETHER_ENABLE_PRESSURE_ADVANCE
    /// Runtime enable flag for pressure advance (default: off).
    /// Must be true for PA to be applied in the motion pipeline.
    bool pressureAdvanceEnabled = false;
    /// Pressure advance amount for the extruder (seconds).
    /// Set to 0 to disable PA in the motion pipeline.
    double pressureAdvance = 0.0;
    /// Pressure advance smoothing window (seconds, 0 = no smoothing).
    double smoothTime = 0.0;
    /// Extrusion compensation model (default Linear = classic PA).
    std::string extrusionCompensationModel = "linear";
    /// Power-law flow index n (1 = Newtonian limit).
    double paFlowIndex = 1.0;
    /// Power-law K_base [filament-mm / (mm³/s)^n].
    double paConsistency = 0.0;
    /// Maximum absolute compensation [mm] (safety clamp).
    double paMaxCompensation = 0.5;
    /// Cross-WLF βV_m/A_f [mm/Pa].
    double crossWlfCompressibilityOverArea = 0.0;
    /// Melt-temperature estimate [°C] for the Cross-WLF LUT lookup.
    double meltTempC = 210.0;
#endif
};

/// @brief Configuration for a KlippyInstance.
struct KlippyInstanceConfig {
    UdsServerConfig udsConfig;
    std::string sdcardDir = "/tmp/tether_sdcard";
    std::string settingsPath = "/tmp/tether_klippy_settings.cfg";
    std::string configPath = "/etc/tether/printer.cfg";
    double minExtrudeTemp = 170.0;
    std::string firmwareVersion = "tether-klipper-1.0.0";

    /// Optional motion backend. If set, G-code moves are routed through
    /// the real Klipper wire protocol instead of just updating the object
    /// model. See MotionBackendConfig for details.
    std::shared_ptr<MotionBackendConfig> motionBackend;
};

} // namespace tether::klipper::klippy
