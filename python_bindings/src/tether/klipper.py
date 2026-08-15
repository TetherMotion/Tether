"""
Klipper module for Tether.

Provides Python access to the tether_klipper C++ library, including:
- Thermal control (heaters, sensors, PID)
- Homing and probing
- Bed leveling / mesh compensation
- Peripherals (fans, LEDs, filament sensors, pulse counters)
- TMC UART stepper driver interface
- G-code executor
- Printer state machine
- Config file parser (printer.cfg) and validator
- Debug flags and diagnostic gate
- Delta printer kinematics
- TMC driver configuration
- Filament load/unload
- Multi-MCU coordination
- Skew correction
- Case light
- Printer object wrappers (UDS status)
"""

from ._klipper import (
    # Thermal
    ThermistorParams,
    Thermistor,
    HeaterPIDParams,
    Heater,
    # Homing
    HomingAxisConfig,
    HomingResult,
    HomingSequence,
    Probe,
    # Bed leveling
    BedMesh,
    # Peripherals
    Fan,
    LedColor,
    Neopixel,
    FilamentSensor,
    PulseCounter,
    # TMC UART
    TmcUart,
    # G-code executor
    GcodeLine,
    PrinterMotionState,
    GCodeExecutor,
    parse_gcode_line,
    # G-code macros
    GcodeMacro,
    GcodeMacroRegistry,
    # Printer state machine
    PrinterState,
    PrinterStateMachine,
    printer_state_to_string,
    # Config parser
    ConfigSection,
    ConfigParser,
    # Debug
    DebugFlag,
    DebugManager,
    # Advanced objects
    VirtualSdcard,
    ShaperType,
    InputShaperParams,
    InputShaper,
    shaper_type_to_string,
    FirmwareRetractionParams,
    FirmwareRetraction,
    # KlippyInstance
    KlippyInstanceConfig,
    KlippyInstance,
    # JSON value (UDS protocol)
    JsonValue,
    JsonValueType,
    # Printer object base
    PrinterObject,
    # Delta printer
    DeltaGeometry,
    DeltaEndstopAdjust,
    DeltaPrinter,
    # TMC driver configuration
    TmcDriverParams,
    TmcDriverConfig,
    # Filament loader
    FilamentLoader,
    # Multi-MCU coordination
    SecondaryMcuConfig,
    MultiMcuManager,
    # Skew correction
    SkewParams,
    SkewCorrection,
    # Case light
    CaseLight,
    # Config validator
    ConfigValidationResult,
    ConfigValidator,
    # Peripheral device classes
    DigitalOut,
    PWMOut,
    AnalogIn,
    Spi,
    I2c,
    Endstop,
    TrsyncState,
    Trsync,
    HallFilamentSensor,
    # PrinterObject wrapper classes
    DigitalOutObject,
    PWMOutObject,
    AnalogInObject,
    SpiObject,
    I2cObject,
    EndstopObject,
    TrsyncObject,
    HallFilamentSensorObject,
    PulseCounterObject,
    NeopixelObject,
    TmcUartObject,
)

# Pressure advance is an opt-in feature (TETHER_ENABLE_PRESSURE_ADVANCE).
# When compiled out, the C++ extension does not export these classes.
try:
    from ._klipper import PressureAdvanceParams, PressureAdvance  # noqa: F401
    _HAS_PRESSURE_ADVANCE = True
except ImportError:
    _HAS_PRESSURE_ADVANCE = False

__all__ = [
    "ThermistorParams", "Thermistor", "HeaterPIDParams", "Heater",
    "HomingAxisConfig", "HomingResult", "HomingSequence", "Probe",
    "BedMesh",
    "Fan", "LedColor", "Neopixel", "FilamentSensor", "PulseCounter",
    "TmcUart",
    "GcodeLine", "PrinterMotionState", "GCodeExecutor", "parse_gcode_line",
    "GcodeMacro", "GcodeMacroRegistry",
    "PrinterState", "PrinterStateMachine", "printer_state_to_string",
    "ConfigSection", "ConfigParser",
    "DebugFlag", "DebugManager",
    "VirtualSdcard",
    "ShaperType", "InputShaperParams", "InputShaper", "shaper_type_to_string",
    "FirmwareRetractionParams", "FirmwareRetraction",
    "KlippyInstanceConfig", "KlippyInstance",
    "JsonValue", "JsonValueType",
    "PrinterObject",
    "DeltaGeometry", "DeltaEndstopAdjust", "DeltaPrinter",
    "TmcDriverParams", "TmcDriverConfig",
    "FilamentLoader",
    "SecondaryMcuConfig", "MultiMcuManager",
    "SkewParams", "SkewCorrection",
    "CaseLight",
    "ConfigValidationResult", "ConfigValidator",
    "DigitalOut", "PWMOut", "AnalogIn", "Spi", "I2c",
    "Endstop", "TrsyncState", "Trsync", "HallFilamentSensor",
    "DigitalOutObject", "PWMOutObject", "AnalogInObject",
    "SpiObject", "I2cObject",
    "EndstopObject", "TrsyncObject", "HallFilamentSensorObject",
    "PulseCounterObject", "NeopixelObject", "TmcUartObject",
]

if _HAS_PRESSURE_ADVANCE:
    __all__ += ["PressureAdvanceParams", "PressureAdvance"]
