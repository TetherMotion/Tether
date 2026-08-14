/**
 * @file klipper_bindings.cpp
 * @brief Python bindings for the tether_klipper module.
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include "tether/klipper/objects/Thermal.hpp"
#include "tether/klipper/objects/Homing.hpp"
#include "tether/klipper/objects/BedLevel.hpp"
#include "tether/klipper/objects/Peripherals.hpp"
#include "tether/klipper/objects/TmcUart.hpp"
#include "tether/klipper/klippy/GCodeExecutor.hpp"
#include "tether/klipper/klippy/AdvancedObjects.hpp"
#include "tether/klipper/klippy/KlippyInstance.hpp"
#include "tether/klipper/klippy/PrinterObjects.hpp"
#include "tether/klipper/klippy/PrinterStateMachine.hpp"
#include "tether/klipper/config/ConfigParser.hpp"
#include "tether/klipper/debug/Debug.hpp"

namespace py = pybind11;
using namespace tether::klipper::objects;
using namespace tether::klipper::klippy;
using namespace tether::klipper::config;
using namespace tether::klipper::debug;
namespace objects = tether::klipper::objects;

PYBIND11_MODULE(_klipper, m) {
    m.doc() = "Tether Klipper module bindings";

    // ----------------------------------------------------------------
    // Thermal
    // ----------------------------------------------------------------
    py::class_<Thermistor::Params>(m, "ThermistorParams")
        .def(py::init<>())
        .def_readwrite("pullup_resistor", &Thermistor::Params::pullupResistor)
        .def_readwrite("reference_voltage", &Thermistor::Params::referenceVoltage)
        .def_readwrite("adc_max", &Thermistor::Params::adcMax)
        .def_readwrite("resistance_at_25c", &Thermistor::Params::resistanceAt25C)
        .def_readwrite("beta", &Thermistor::Params::beta)
        .def_readwrite("min_temp", &Thermistor::Params::minTemp)
        .def_readwrite("max_temp", &Thermistor::Params::maxTemp);

    py::class_<Thermistor, std::shared_ptr<Thermistor>>(m, "Thermistor")
        .def(py::init<uint8_t, Thermistor::Params, Thermistor::AdcReadFunc>())
        .def("read", &Thermistor::read)
        .def("update", &Thermistor::update)
        .def("type", &Thermistor::type)
        .def_property_readonly("oid", &Thermistor::oid)
        .def_property_readonly("last_temperature", &Thermistor::lastTemperature);

    py::class_<HeaterPidParams>(m, "HeaterPidParams")
        .def(py::init<>())
        .def_readwrite("kp", &HeaterPidParams::Kp)
        .def_readwrite("ki", &HeaterPidParams::Ki)
        .def_readwrite("kd", &HeaterPidParams::Kd)
        .def_readwrite("imax", &HeaterPidParams::imax)
        .def_readwrite("pwm_min", &HeaterPidParams::pwmMin)
        .def_readwrite("pwm_max", &HeaterPidParams::pwmMax);

    py::class_<Heater, std::shared_ptr<Heater>>(m, "Heater")
        .def(py::init<uint8_t, Heater::PwmWriteFunc, Heater::SensorReadFunc>())
        .def("set_target", &Heater::setTarget)
        .def("control", &Heater::control)
        .def("reset", &Heater::reset)
        .def("at_target", &Heater::atTarget, py::arg("tolerance") = 2.0)
        .def("set_pid_params", &Heater::setPidParams)
        .def("set_safety_limits", &Heater::setSafetyLimits)
        .def("set_shutdown_callback", &Heater::setShutdownCallback)
        .def("set_control_interval", &Heater::setControlInterval)
        .def_property_readonly("oid", &Heater::oid)
        .def_property_readonly("target", &Heater::target)
        .def_property_readonly("current_temp", &Heater::currentTemp);

    // ----------------------------------------------------------------
    // Homing
    // ----------------------------------------------------------------
    py::class_<HomingAxisConfig>(m, "HomingAxisConfig")
        .def(py::init<>())
        .def_readwrite("name", &HomingAxisConfig::name)
        .def_readwrite("stepper_index", &HomingAxisConfig::stepperIndex)
        .def_readwrite("search_speed", &HomingAxisConfig::searchSpeed)
        .def_readwrite("bounce_speed", &HomingAxisConfig::bounceSpeed)
        .def_readwrite("bounce_distance", &HomingAxisConfig::bounceDistance)
        .def_readwrite("home_position", &HomingAxisConfig::homePosition)
        .def_readwrite("positive_direction", &HomingAxisConfig::positiveDirection);

    py::class_<HomingResult>(m, "HomingResult")
        .def_readonly("success", &HomingResult::success)
        .def_readonly("axis", &HomingResult::axis)
        .def_readonly("final_position", &HomingResult::finalPosition)
        .def_readonly("trigger_position", &HomingResult::triggerPosition)
        .def_readonly("error_message", &HomingResult::errorMessage);

    py::class_<HomingSequence, std::shared_ptr<HomingSequence>>(m, "HomingSequence")
        .def(py::init<HomingSequence::EndstopCheckFunc,
                       HomingSequence::MoveFunc,
                       HomingSequence::SetPositionFunc,
                       HomingSequence::GetPositionFunc,
                       HomingSequence::WaitFunc>())
        .def("home_axis", &HomingSequence::homeAxis)
        .def("home_axes", &HomingSequence::homeAxes);

    py::class_<Probe, std::shared_ptr<Probe>>(m, "Probe")
        .def(py::init<uint8_t, Probe::PinReadFunc>())
        .def("triggered", &Probe::triggered)
        .def("set_z_offset", &Probe::setZOffset)
        .def("set_virtual_endstop", &Probe::setVirtualEndstop)
        .def_property_readonly("oid", &Probe::oid)
        .def_property_readonly("z_offset", &Probe::zOffset);

    // ----------------------------------------------------------------
    // Bed leveling
    // ----------------------------------------------------------------
    py::class_<BedMesh>(m, "BedMesh")
        .def(py::init<>())
        .def("configure", &BedMesh::configure)
        .def("set_point", &BedMesh::setPoint)
        .def("compensation_at", &BedMesh::compensationAt)
        .def("is_complete", &BedMesh::isComplete)
        .def("clear", &BedMesh::clear)
        .def_property_readonly("x_points", &BedMesh::xPoints)
        .def_property_readonly("y_points", &BedMesh::yPoints);

    // ----------------------------------------------------------------
    // Peripherals
    // ----------------------------------------------------------------
    py::class_<Fan, std::shared_ptr<Fan>>(m, "Fan")
        .def(py::init<uint8_t, Fan::PwmWriteFunc>())
        .def("set_speed", &Fan::setSpeed)
        .def("set_off_time", &Fan::setOffTime)
        .def_property_readonly("oid", &Fan::oid)
        .def_property_readonly("speed", &Fan::speed);

    py::class_<LedColor>(m, "LedColor")
        .def(py::init<>())
        .def(py::init<uint8_t, uint8_t, uint8_t, uint8_t>(),
             py::arg("r") = 0, py::arg("g") = 0, py::arg("b") = 0, py::arg("w") = 0)
        .def_readwrite("r", &LedColor::r)
        .def_readwrite("g", &LedColor::g)
        .def_readwrite("b", &LedColor::b)
        .def_readwrite("w", &LedColor::w);

    py::class_<Neopixel, std::shared_ptr<Neopixel>>(m, "Neopixel")
        .def(py::init<uint8_t, int, Neopixel::SpiWriteFunc, bool>(),
             py::arg("oid"), py::arg("num_leds"), py::arg("spi_write"), py::arg("has_white") = false)
        .def("set_color", &Neopixel::setColor)
        .def("set_all", &Neopixel::setAll)
        .def("update", &Neopixel::update)
        .def("clear", &Neopixel::clear)
        .def_property_readonly("oid", &Neopixel::oid)
        .def_property_readonly("num_leds", &Neopixel::numLeds);

    py::class_<FilamentSensor, std::shared_ptr<FilamentSensor>>(m, "FilamentSensor")
        .def(py::init<uint8_t, FilamentSensor::PinReadFunc>())
        .def("filament_present", &FilamentSensor::filamentPresent)
        .def("runout", &FilamentSensor::runout)
        .def("update", &FilamentSensor::update)
        .def("consume_runout_event", &FilamentSensor::consumeRunoutEvent)
        .def_property_readonly("oid", &FilamentSensor::oid);

    py::class_<PulseCounter, std::shared_ptr<PulseCounter>>(m, "PulseCounter")
        .def(py::init<uint8_t>())
        .def("on_edge", &PulseCounter::onEdge)
        .def("reset", &PulseCounter::reset)
        .def_property_readonly("oid", &PulseCounter::oid)
        .def_property_readonly("count", &PulseCounter::count);

    // ----------------------------------------------------------------
    // TMC UART
    // ----------------------------------------------------------------
    py::class_<TmcUart, std::shared_ptr<TmcUart>>(m, "TmcUart")
        .def(py::init<uint8_t, uint8_t, TmcUart::UartTransferFunc>())
        .def("read_register", &TmcUart::readRegister)
        .def("write_register", &TmcUart::writeRegister)
        .def("set_field", &TmcUart::setField)
        .def("get_field", &TmcUart::getField)
        .def_property_readonly("oid", &TmcUart::oid)
        .def_property_readonly("slave_address", &TmcUart::slaveAddress);

    // ----------------------------------------------------------------
    // G-code executor
    // ----------------------------------------------------------------
    py::class_<GcodeLine>(m, "GcodeLine")
        .def_readonly("code", &GcodeLine::code)
        .def_readonly("comment", &GcodeLine::comment)
        .def("has", &GcodeLine::has)
        .def("get", &GcodeLine::get, py::arg("key"), py::arg("default_val") = 0.0);

    m.def("parse_gcode_line", &parseGcodeLine);

    py::class_<PrinterMotionState>(m, "PrinterMotionState")
        .def(py::init<>())
        .def_property("absolute_coordinates",
                       &PrinterMotionState::absoluteCoordinates,
                       &PrinterMotionState::setAbsoluteCoordinates)
        .def_readwrite("absolute_extrude", &PrinterMotionState::absoluteExtrude)
        .def_readwrite("feedrate", &PrinterMotionState::feedrate)
        .def_readwrite("homed_axes", &PrinterMotionState::homedAxes);

    py::class_<GCodeExecutor, std::shared_ptr<GCodeExecutor>>(m, "GCodeExecutor")
        .def(py::init<GcodeCallbacks, PrinterMotionState*>(),
             py::arg("callbacks"), py::arg("state") = nullptr)
        .def("execute", &GCodeExecutor::execute)
        .def("execute_line", &GCodeExecutor::executeLine)
        .def_property_readonly("state", [](const GCodeExecutor& e) {
            return e.state();
        });

    // ----------------------------------------------------------------
    // Printer state machine
    // ----------------------------------------------------------------
    py::enum_<PrinterState>(m, "PrinterState")
        .value("Startup", PrinterState::Startup)
        .value("Ready", PrinterState::Ready)
        .value("Printing", PrinterState::Printing)
        .value("Paused", PrinterState::Paused)
        .value("Error", PrinterState::Error)
        .value("Shutdown", PrinterState::Shutdown);

    py::class_<PrinterStateMachine, std::shared_ptr<PrinterStateMachine>>(m, "PrinterStateMachine")
        .def(py::init<>())
        .def("transition", &PrinterStateMachine::transition,
             py::arg("new_state"), py::arg("message") = "")
        .def_property_readonly("state", &PrinterStateMachine::state)
        .def_property_readonly("message", &PrinterStateMachine::message)
        .def("is_operational", &PrinterStateMachine::isOperational)
        .def("is_printing", &PrinterStateMachine::isPrinting)
        .def("is_terminal", &PrinterStateMachine::isTerminal);

    m.def("printer_state_to_string", &printerStateToString);

    // ----------------------------------------------------------------
    // Config parser
    // ----------------------------------------------------------------
    py::class_<ConfigSection>(m, "ConfigSection")
        .def(py::init<>())
        .def("has", &ConfigSection::has)
        .def("get", &ConfigSection::get, py::arg("key"), py::arg("default_val") = "")
        .def("get_int", &ConfigSection::getInt, py::arg("key"), py::arg("default_val") = 0)
        .def("get_double", &ConfigSection::getDouble, py::arg("key"), py::arg("default_val") = 0.0)
        .def("get_bool", &ConfigSection::getBool, py::arg("key"), py::arg("default_val") = false)
        .def_readonly("name", &ConfigSection::name);

    py::class_<ConfigParser, std::shared_ptr<ConfigParser>>(m, "ConfigParser")
        .def(py::init<>())
        .def("parse", &ConfigParser::parse)
        .def("parse_file", &ConfigParser::parseFile)
        .def("has_section", &ConfigParser::hasSection)
        .def("get_section", &ConfigParser::getSection, py::return_value_policy::reference)
        .def("section_names", &ConfigParser::sectionNames);

    // ----------------------------------------------------------------
    // Debug
    // ----------------------------------------------------------------
    py::enum_<DebugFlag>(m, "DebugFlag")
        .value("None", DebugFlag::None)
        .value("Commands", DebugFlag::Commands)
        .value("Responses", DebugFlag::Responses)
        .value("Motion", DebugFlag::Motion)
        .value("Clock", DebugFlag::Clock)
        .value("Objects", DebugFlag::Objects)
        .value("Transports", DebugFlag::Transports)
        .value("Thermal", DebugFlag::Thermal)
        .value("Homing", DebugFlag::Homing)
        .value("Probing", DebugFlag::Probing)
        .value("Config", DebugFlag::Config)
        .value("All", DebugFlag::All);

    py::class_<DebugManager, std::shared_ptr<DebugManager>>(m, "DebugManager")
        .def(py::init<>())
        .def("set_flags", &DebugManager::setFlags)
        .def("enable", &DebugManager::enable)
        .def("disable", &DebugManager::disable)
        .def("is_enabled", &DebugManager::isEnabled)
        .def("log", &DebugManager::log)
        .def("set_log_callback", &DebugManager::setLogCallback);

    // ----------------------------------------------------------------
    // Advanced objects
    // ----------------------------------------------------------------

    // Virtual SD card
    py::class_<VirtualSdcard, std::shared_ptr<VirtualSdcard>>(m, "VirtualSdcard")
        .def(py::init<std::string>())
        .def("list_files", &VirtualSdcard::listFiles)
        .def("select_file", &VirtualSdcard::selectFile)
        .def("start_print", &VirtualSdcard::startPrint)
        .def("pause_print", &VirtualSdcard::pausePrint)
        .def("resume_print", &VirtualSdcard::resumePrint)
        .def("cancel_print", &VirtualSdcard::cancelPrint)
        .def("read_chunk", &VirtualSdcard::readChunk,
             py::arg("max_lines") = 32)
        .def("reset_position", &VirtualSdcard::resetPosition)
        .def("seek", &VirtualSdcard::seek)
        .def_property_readonly("is_active", &VirtualSdcard::isActive)
        .def_property_readonly("is_paused", &VirtualSdcard::isPaused)
        .def_property_readonly("file_path", &VirtualSdcard::filePath)
        .def_property_readonly("file_size", &VirtualSdcard::fileSize)
        .def_property_readonly("file_position", &VirtualSdcard::filePosition)
        .def_property_readonly("progress", &VirtualSdcard::progress);

    // Pressure advance
    py::class_<PressureAdvanceParams>(m, "PressureAdvanceParams")
        .def(py::init<>())
        .def_readwrite("pressure_advance", &PressureAdvanceParams::pressureAdvance)
        .def_readwrite("smooth_time", &PressureAdvanceParams::smoothTime);

    py::class_<PressureAdvance, std::shared_ptr<PressureAdvance>>(m, "PressureAdvance")
        .def(py::init<PressureAdvanceParams>())
        .def("set_params", &PressureAdvance::setParams)
        .def("is_active", &PressureAdvance::isActive)
        .def("compute_extrusion", &PressureAdvance::computeExtrusion)
        .def("smooth_extrusion_rate", &PressureAdvance::smoothExtrusionRate)
        .def("reset", &PressureAdvance::reset);

    // Input shaper
    py::enum_<ShaperType>(m, "ShaperType")
        .value("None", ShaperType::None)
        .value("ZV", ShaperType::ZV)
        .value("ZVD", ShaperType::ZVD)
        .value("MZV", ShaperType::MZV)
        .value("EI", ShaperType::EI)
        .value("DampedEI", ShaperType::DampedEI);

    py::class_<InputShaperParams>(m, "InputShaperParams")
        .def(py::init<>())
        .def_readwrite("type", &InputShaperParams::type)
        .def_readwrite("freq", &InputShaperParams::freq)
        .def_readwrite("damping", &InputShaperParams::damping);

    py::class_<InputShaper, std::shared_ptr<InputShaper>>(m, "InputShaper")
        .def(py::init<InputShaperParams>())
        .def("set_params", &InputShaper::setParams)
        .def("is_active", &InputShaper::isActive)
        .def("shape_acceleration", &InputShaper::shapeAcceleration)
        .def("shaping_delay", &InputShaper::shapingDelay);

    m.def("shaper_type_to_string", &shaperTypeToString);

    // Firmware retraction
    py::class_<FirmwareRetractionParams>(m, "FirmwareRetractionParams")
        .def(py::init<>())
        .def_readwrite("retract_length", &FirmwareRetractionParams::retractLength)
        .def_readwrite("retract_speed", &FirmwareRetractionParams::retractSpeed)
        .def_readwrite("unretract_length", &FirmwareRetractionParams::unretractLength)
        .def_readwrite("unretract_speed", &FirmwareRetractionParams::unretractSpeed)
        .def_readwrite("z_hop", &FirmwareRetractionParams::zHop);

    py::class_<FirmwareRetraction, std::shared_ptr<FirmwareRetraction>>(m, "FirmwareRetraction")
        .def(py::init<FirmwareRetractionParams>())
        .def("set_params", &FirmwareRetraction::setParams)
        .def("retract", &FirmwareRetraction::retract)
        .def("unretract", &FirmwareRetraction::unretract)
        .def("is_retracted", &FirmwareRetraction::isRetracted)
        .def("z_hop", &FirmwareRetraction::zHop);

    // G-code macros
    py::class_<GcodeMacro>(m, "GcodeMacro")
        .def(py::init<>())
        .def_readwrite("name", &GcodeMacro::name)
        .def_readwrite("gcode", &GcodeMacro::gcode)
        .def_readwrite("description", &GcodeMacro::description);

    py::class_<GcodeMacroRegistry, std::shared_ptr<GcodeMacroRegistry>>(m, "GcodeMacroRegistry")
        .def(py::init<>())
        .def("register_macro", &GcodeMacroRegistry::registerMacro)
        .def("unregister_macro", &GcodeMacroRegistry::unregisterMacro)
        .def("has_macro", &GcodeMacroRegistry::hasMacro)
        .def("list_macros", &GcodeMacroRegistry::listMacros)
        .def("expand_macro", &GcodeMacroRegistry::expandMacro);

    // ----------------------------------------------------------------
    // KlippyInstance
    // ----------------------------------------------------------------
    py::class_<KlippyInstanceConfig>(m, "KlippyInstanceConfig")
        .def(py::init<>())
        .def_readwrite("sdcard_dir", &KlippyInstanceConfig::sdcardDir)
        .def_readwrite("min_extrude_temp", &KlippyInstanceConfig::minExtrudeTemp);

    py::class_<KlippyInstance, std::shared_ptr<KlippyInstance>>(m, "KlippyInstance")
        .def(py::init<KlippyInstanceConfig>())
        .def("start", &KlippyInstance::start)
        .def("stop", &KlippyInstance::stop)
        .def("execute_gcode", &KlippyInstance::executeGcode)
        .def("register_macro", &KlippyInstance::registerMacro)
        .def("load_config", &KlippyInstance::loadConfig)
        .def_property_readonly("server", [](KlippyInstance& i) -> KlippyServer& { return i.server(); }, py::return_value_policy::reference)
        .def_property_readonly("sdcard", [](KlippyInstance& i) -> VirtualSdcard& { return i.sdcard(); }, py::return_value_policy::reference)
        .def_property_readonly("macros", [](KlippyInstance& i) -> GcodeMacroRegistry& { return i.macros(); }, py::return_value_policy::reference);

    // ----------------------------------------------------------------
    // JsonValue (needed for PrinterObject status() returns)
    // ----------------------------------------------------------------
    py::enum_<JsonValue::Type>(m, "JsonValueType")
        .value("Null", JsonValue::Type::Null)
        .value("Bool", JsonValue::Type::Bool)
        .value("Int", JsonValue::Type::Int)
        .value("Double", JsonValue::Type::Double)
        .value("String", JsonValue::Type::String)
        .value("Array", JsonValue::Type::Array)
        .value("Object", JsonValue::Type::Object);

    py::class_<JsonValue, std::shared_ptr<JsonValue>>(m, "JsonValue",
        "Lightweight JSON value for the UDS protocol.")
        .def(py::init<>())
        .def(py::init<bool>())
        .def(py::init<int64_t>())
        .def(py::init<double>())
        .def(py::init<std::string>())
        .def("type", &JsonValue::type)
        .def("is_null", &JsonValue::isNull)
        .def("is_bool", &JsonValue::isBool)
        .def("is_int", &JsonValue::isInt)
        .def("is_double", &JsonValue::isDouble)
        .def("is_string", &JsonValue::isString)
        .def("is_array", &JsonValue::isArray)
        .def("is_object", &JsonValue::isObject)
        .def("as_bool", &JsonValue::asBool)
        .def("as_int", &JsonValue::asInt)
        .def("as_double", &JsonValue::asDouble)
        .def("as_string", &JsonValue::asString, py::return_value_policy::reference)
        .def("dump", &JsonValue::dump)
        .def_static("parse", [](const std::string& s) {
            auto r = JsonValue::parse(s);
            return r ? std::optional<JsonValue>(*r) : std::optional<JsonValue>();
        });

    // ----------------------------------------------------------------
    // PrinterObject base class
    // ----------------------------------------------------------------
    py::class_<PrinterObject, std::shared_ptr<PrinterObject>>(m, "PrinterObject",
        "Base class for printer objects exposed via the UDS.")
        .def("name", &PrinterObject::name)
        .def("status", &PrinterObject::status, py::arg("fields"))
        .def("available_fields", &PrinterObject::availableFields);

    // ----------------------------------------------------------------
    // Delta printer (M665/M666)
    // ----------------------------------------------------------------
    py::class_<DeltaGeometry>(m, "DeltaGeometry",
        "Delta printer geometry parameters (M665).")
        .def(py::init<>())
        .def_readwrite("arm_length", &DeltaGeometry::armLength)
        .def_readwrite("delta_radius", &DeltaGeometry::deltaRadius)
        .def_readwrite("tower_angle_a", &DeltaGeometry::towerAngleA)
        .def_readwrite("tower_angle_b", &DeltaGeometry::towerAngleB)
        .def_readwrite("tower_angle_c", &DeltaGeometry::towerAngleC);

    py::class_<DeltaEndstopAdjust>(m, "DeltaEndstopAdjust",
        "Delta endstop adjustments (M666).")
        .def(py::init<>())
        .def_readwrite("adj_x", &DeltaEndstopAdjust::adjX)
        .def_readwrite("adj_y", &DeltaEndstopAdjust::adjY)
        .def_readwrite("adj_z", &DeltaEndstopAdjust::adjZ);

    py::class_<DeltaPrinter, std::shared_ptr<DeltaPrinter>>(m, "DeltaPrinter",
        "Delta printer configuration and kinematics.")
        .def(py::init<>())
        .def("set_geometry", &DeltaPrinter::setGeometry)
        .def("set_endstop_adjust", &DeltaPrinter::setEndstopAdjust)
        .def_property_readonly("geometry", &DeltaPrinter::geometry,
            py::return_value_policy::reference)
        .def_property_readonly("endstop_adjust", &DeltaPrinter::endstopAdjust,
            py::return_value_policy::reference)
        .def("forward_actuator_kinematics", &DeltaPrinter::forwardActuatorKinematics,
            py::arg("x"), py::arg("y"), py::arg("z"))
        .def("inverse_actuator_kinematics", &DeltaPrinter::inverseActuatorKinematics,
            py::arg("tower_a"), py::arg("tower_b"), py::arg("tower_c"));

    // ----------------------------------------------------------------
    // TMC driver configuration (M907-M914)
    // ----------------------------------------------------------------
    py::class_<TmcDriverParams>(m, "TmcDriverParams",
        "TMC driver parameters for a single axis.")
        .def(py::init<>())
        .def_readwrite("run_current", &TmcDriverParams::runCurrent)
        .def_readwrite("hold_current", &TmcDriverParams::holdCurrent)
        .def_readwrite("stealth_chop", &TmcDriverParams::stealthChop)
        .def_readwrite("spread_threshold", &TmcDriverParams::spreadThreshold)
        .def_readwrite("bump_sensitivity", &TmcDriverParams::bumpSensitivity)
        .def_readwrite("diag_pin", &TmcDriverParams::diagPin);

    py::class_<TmcDriverConfig, std::shared_ptr<TmcDriverConfig>>(m, "TmcDriverConfig",
        "TMC driver configuration manager (M907-M914).")
        .def(py::init<>())
        .def("set_run_current", &TmcDriverConfig::setRunCurrent)
        .def("set_hold_current", &TmcDriverConfig::setHoldCurrent)
        .def("set_stealth_chop", &TmcDriverConfig::setStealthChop)
        .def("set_spread_threshold", &TmcDriverConfig::setSpreadThreshold)
        .def("set_bump_sensitivity", &TmcDriverConfig::setBumpSensitivity)
        .def("set_diag_pin", &TmcDriverConfig::setDiagPin)
        .def("params", &TmcDriverConfig::params,
            py::return_value_policy::reference)
        .def("axes", &TmcDriverConfig::axes);

    // ----------------------------------------------------------------
    // Filament loader (M701-M708)
    // ----------------------------------------------------------------
    py::class_<FilamentLoader, std::shared_ptr<FilamentLoader>>(m, "FilamentLoader",
        "Filament load/unload state and operations (M701-M708).")
        .def(py::init<>())
        .def("set_load_callback", &FilamentLoader::setLoadCallback)
        .def("set_unload_callback", &FilamentLoader::setUnloadCallback)
        .def("set_purge_callback", &FilamentLoader::setPurgeCallback)
        .def("set_retract_callback", &FilamentLoader::setRetractCallback)
        .def("load_filament", &FilamentLoader::loadFilament)
        .def("unload_filament", &FilamentLoader::unloadFilament)
        .def("load_to_tool", &FilamentLoader::loadToTool)
        .def("unload_from_tool", &FilamentLoader::unloadFromTool)
        .def("purge", &FilamentLoader::purge)
        .def("retract", &FilamentLoader::retract)
        .def("set_sensor_state", &FilamentLoader::setSensorState)
        .def("report_sensor_state", &FilamentLoader::reportSensorState)
        .def("is_loaded", &FilamentLoader::isLoaded)
        .def("set_load_length", &FilamentLoader::setLoadLength)
        .def("set_tool_load_length", &FilamentLoader::setToolLoadLength)
        .def("set_load_speed", &FilamentLoader::setLoadSpeed)
        .def("set_purge_length", &FilamentLoader::setPurgeLength)
        .def("set_purge_speed", &FilamentLoader::setPurgeSpeed)
        .def("set_retract_length", &FilamentLoader::setRetractLength)
        .def("set_retract_speed", &FilamentLoader::setRetractSpeed);

    // ----------------------------------------------------------------
    // Multi-MCU coordination (M860-M876)
    // ----------------------------------------------------------------
    py::class_<SecondaryMcuConfig>(m, "SecondaryMcuConfig",
        "Secondary MCU configuration.")
        .def(py::init<>())
        .def_readwrite("id", &SecondaryMcuConfig::id)
        .def_readwrite("serial_path", &SecondaryMcuConfig::serialPath)
        .def_readwrite("baud_rate", &SecondaryMcuConfig::baudRate)
        .def_readwrite("enabled", &SecondaryMcuConfig::enabled)
        .def_readwrite("clock_freq", &SecondaryMcuConfig::clockFreq)
        .def_readwrite("connected", &SecondaryMcuConfig::connected)
        .def_readwrite("firmware_version", &SecondaryMcuConfig::firmwareVersion)
        .def_readwrite("bytes_read", &SecondaryMcuConfig::bytesRead)
        .def_readwrite("bytes_write", &SecondaryMcuConfig::bytesWrite)
        .def_readwrite("retransmits", &SecondaryMcuConfig::retransmits);

    py::class_<MultiMcuManager, std::shared_ptr<MultiMcuManager>>(m, "MultiMcuManager",
        "Multi-MCU coordination manager (M860-M876).")
        .def(py::init<>())
        .def("set_serial_path", &MultiMcuManager::setSerialPath)
        .def("set_baud_rate", &MultiMcuManager::setBaudRate)
        .def("set_enabled", &MultiMcuManager::setEnabled)
        .def("set_clock_freq", &MultiMcuManager::setClockFreq)
        .def("get_status", &MultiMcuManager::getStatus)
        .def("update_stats", &MultiMcuManager::updateStats)
        .def("set_firmware_version", &MultiMcuManager::setFirmwareVersion)
        .def("mcu_ids", &MultiMcuManager::mcuIds)
        .def("get_mcu", [](const MultiMcuManager& mgr, int id) {
            const auto* mcu = mgr.getMcu(id);
            return mcu ? std::optional<SecondaryMcuConfig>(*mcu)
                       : std::optional<SecondaryMcuConfig>();
        }, py::arg("id"));

    // ----------------------------------------------------------------
    // Skew correction (M852)
    // ----------------------------------------------------------------
    py::class_<SkewParams>(m, "SkewParams",
        "Skew correction parameters.")
        .def(py::init<>())
        .def_readwrite("xy", &SkewParams::xy)
        .def_readwrite("xz", &SkewParams::xz)
        .def_readwrite("yz", &SkewParams::yz);

    py::class_<SkewCorrection, std::shared_ptr<SkewCorrection>>(m, "SkewCorrection",
        "Skew correction manager (M852).")
        .def(py::init<>())
        .def("set_params", &SkewCorrection::setParams)
        .def_property_readonly("params", &SkewCorrection::params,
            py::return_value_policy::reference)
        .def("correct", &SkewCorrection::correct,
            py::arg("x"), py::arg("y"), py::arg("z"))
        .def("is_active", &SkewCorrection::isActive);

    // ----------------------------------------------------------------
    // Case light (M355)
    // ----------------------------------------------------------------
    py::class_<CaseLight, std::shared_ptr<CaseLight>>(m, "CaseLight",
        "Case light controller (M355).")
        .def(py::init<>())
        .def("set_state", &CaseLight::setState,
            py::arg("on"), py::arg("brightness"))
        .def_property_readonly("is_on", &CaseLight::isOn)
        .def_property_readonly("brightness", &CaseLight::brightness);

    // ----------------------------------------------------------------
    // Config validator
    // ----------------------------------------------------------------
    py::class_<ConfigValidationResult>(m, "ConfigValidationResult",
        "Validation result for a single config section.")
        .def(py::init<>())
        .def_readwrite("section_name", &ConfigValidationResult::sectionName)
        .def_readwrite("valid", &ConfigValidationResult::valid)
        .def_readwrite("errors", &ConfigValidationResult::errors)
        .def_readwrite("warnings", &ConfigValidationResult::warnings);

    py::class_<ConfigValidator, std::shared_ptr<ConfigValidator>>(m, "ConfigValidator",
        "Validates Klipper configuration sections.")
        .def(py::init<>())
        .def("validate", &ConfigValidator::validate)
        .def("validate_section", &ConfigValidator::validateSection)
        .def("all_valid", &ConfigValidator::allValid)
        .def("format_errors", &ConfigValidator::formatErrors);

    // ----------------------------------------------------------------
    // Peripheral device classes (underlying objects for wrappers)
    // ----------------------------------------------------------------
    py::class_<objects::DigitalOut, std::shared_ptr<objects::DigitalOut>>(m, "DigitalOut",
        "Digital output pin object.")
        .def(py::init<uint8_t>())
        .def(py::init<uint8_t, objects::DigitalOut::WriteFunc>())
        .def("set_value", &objects::DigitalOut::setValue)
        .def_property_readonly("oid", &objects::DigitalOut::oid)
        .def_property_readonly("value", &objects::DigitalOut::value);

    py::class_<objects::PwmOut, std::shared_ptr<objects::PwmOut>>(m, "PwmOut",
        "PWM output pin object.")
        .def(py::init<uint8_t>())
        .def(py::init<uint8_t, objects::PwmOut::WriteFunc>())
        .def("set_duty", py::overload_cast<double>(&objects::PwmOut::setDuty))
        .def("set_duty_raw", py::overload_cast<uint32_t, uint32_t>(&objects::PwmOut::setDuty))
        .def("set_cycle_time", &objects::PwmOut::setCycleTime)
        .def("set_write_func", &objects::PwmOut::setWriteFunc)
        .def_property_readonly("oid", &objects::PwmOut::oid)
        .def_property_readonly("duty", &objects::PwmOut::duty)
        .def_property_readonly("duty_double", &objects::PwmOut::dutyDouble)
        .def_property_readonly("cycle_time", &objects::PwmOut::cycleTime);

    py::class_<objects::AnalogIn, std::shared_ptr<objects::AnalogIn>>(m, "AnalogIn",
        "Analog input pin object.")
        .def(py::init<uint8_t>())
        .def(py::init<uint8_t, objects::AnalogIn::ReadFunc>())
        .def("read", &objects::AnalogIn::read)
        .def("set_read_func", &objects::AnalogIn::setReadFunc)
        .def("set_sample", &objects::AnalogIn::setSample)
        .def("update", &objects::AnalogIn::update)
        .def_property_readonly("oid", &objects::AnalogIn::oid)
        .def_property_readonly("last_sample", &objects::AnalogIn::lastSample)
        .def_property_readonly("last_value", &objects::AnalogIn::lastValue);

    py::class_<objects::Spi, std::shared_ptr<objects::Spi>>(m, "Spi",
        "SPI peripheral.")
        .def(py::init<uint8_t>())
        .def(py::init<uint8_t, objects::Spi::TransferFunc>())
        .def("transfer", py::overload_cast<const std::vector<uint8_t>&>(&objects::Spi::transfer))
        .def("set_transfer_func", &objects::Spi::setTransferFunc)
        .def_property_readonly("oid", &objects::Spi::oid);

    py::class_<objects::I2c, std::shared_ptr<objects::I2c>>(m, "I2c",
        "I2C peripheral.")
        .def(py::init<uint8_t>())
        .def(py::init<uint8_t, objects::I2c::ReadFunc, objects::I2c::WriteFunc>())
        .def("read", py::overload_cast<uint8_t, uint8_t, size_t>(&objects::I2c::read),
             py::arg("addr"), py::arg("reg"), py::arg("len"))
        .def("read_no_register", &objects::I2c::readNoRegister)
        .def("read16", &objects::I2c::read16)
        .def("write", py::overload_cast<uint8_t, const std::vector<uint8_t>&>(&objects::I2c::write),
             py::arg("addr"), py::arg("data"))
        .def("set_read_func", &objects::I2c::setReadFunc)
        .def("set_write_func", &objects::I2c::setWriteFunc)
        .def_property_readonly("oid", &objects::I2c::oid);

    py::class_<objects::Endstop, std::shared_ptr<objects::Endstop>>(m, "Endstop",
        "Endstop peripheral.")
        .def(py::init<uint8_t>())
        .def(py::init<uint8_t, objects::Endstop::PinReadFunc>())
        .def("triggered", &objects::Endstop::triggered)
        .def("set_pin_read_func", &objects::Endstop::setPinReadFunc)
        .def("set_sample_count", &objects::Endstop::setSampleCount)
        .def("set_state", &objects::Endstop::setState)
        .def_property_readonly("oid", &objects::Endstop::oid)
        .def_property_readonly("sample_count", &objects::Endstop::sampleCount)
        .def_property_readonly("state", &objects::Endstop::state);

    py::enum_<objects::TrsyncState>(m, "TrsyncState")
        .value("Idle", objects::TrsyncState::Idle)
        .value("Armed", objects::TrsyncState::Armed)
        .value("Triggered", objects::TrsyncState::Triggered)
        .value("Sent", objects::TrsyncState::Sent);

    py::class_<objects::Trsync, std::shared_ptr<objects::Trsync>>(m, "Trsync",
        "Trsync peripheral for homing synchronization.")
        .def(py::init<uint8_t>())
        .def("arm", py::overload_cast<uint32_t>(&objects::Trsync::arm))
        .def("arm_with_report", py::overload_cast<uint32_t, uint32_t>(&objects::Trsync::arm))
        .def("tick", &objects::Trsync::tick)
        .def("trigger", &objects::Trsync::trigger)
        .def("expire", &objects::Trsync::expire)
        .def("mark_sent", &objects::Trsync::markSent)
        .def("reset", &objects::Trsync::reset)
        .def_property_readonly("oid", &objects::Trsync::oid)
        .def_property_readonly("state", &objects::Trsync::state)
        .def_property_readonly("trigger_clock", &objects::Trsync::triggerClock)
        .def_property_readonly("timeout_clock", &objects::Trsync::timeoutClock);

    py::class_<objects::HallFilamentSensor, std::shared_ptr<objects::HallFilamentSensor>>(
        m, "HallFilamentSensor",
        "Hall effect filament width sensor.")
        .def(py::init<uint8_t, objects::HallFilamentSensor::AdcReadFunc>())
        .def("diameter", &objects::HallFilamentSensor::diameter)
        .def("within_tolerance", &objects::HallFilamentSensor::withinTolerance,
            py::arg("nominal") = 1.75, py::arg("tolerance") = 0.1)
        .def_property_readonly("oid", &objects::HallFilamentSensor::oid);

    // ----------------------------------------------------------------
    // PrinterObject wrapper classes
    // ----------------------------------------------------------------
    py::class_<DigitalOutObject, PrinterObject, std::shared_ptr<DigitalOutObject>>(
        m, "DigitalOutObject",
        "Digital output pin printer object (UDS wrapper).")
        .def(py::init<std::shared_ptr<objects::DigitalOut>, std::string>(),
             py::arg("dev"), py::arg("name") = "output_pin");

    py::class_<PwmOutObject, PrinterObject, std::shared_ptr<PwmOutObject>>(
        m, "PwmOutObject",
        "PWM output pin printer object (UDS wrapper).")
        .def(py::init<std::shared_ptr<objects::PwmOut>, std::string>(),
             py::arg("dev"), py::arg("name") = "pwm_tool");

    py::class_<AnalogInObject, PrinterObject, std::shared_ptr<AnalogInObject>>(
        m, "AnalogInObject",
        "Analog input printer object (UDS wrapper).")
        .def(py::init<std::shared_ptr<objects::AnalogIn>, std::string>(),
             py::arg("dev"), py::arg("name") = "analog_pin");

    py::class_<SpiObject, PrinterObject, std::shared_ptr<SpiObject>>(
        m, "SpiObject",
        "SPI bus printer object (UDS wrapper).")
        .def(py::init<std::shared_ptr<objects::Spi>, std::string>(),
             py::arg("dev"), py::arg("name") = "spi_bus");

    py::class_<I2cObject, PrinterObject, std::shared_ptr<I2cObject>>(
        m, "I2cObject",
        "I2C bus printer object (UDS wrapper).")
        .def(py::init<std::shared_ptr<objects::I2c>, std::string>(),
             py::arg("dev"), py::arg("name") = "i2c_bus");

    py::class_<EndstopObject, PrinterObject, std::shared_ptr<EndstopObject>>(
        m, "EndstopObject",
        "Endstop printer object (UDS wrapper).")
        .def(py::init<std::shared_ptr<objects::Endstop>, std::string>(),
             py::arg("dev"), py::arg("name"));

    py::class_<TrsyncObject, PrinterObject, std::shared_ptr<TrsyncObject>>(
        m, "TrsyncObject",
        "TRsync printer object (UDS wrapper).")
        .def(py::init<std::shared_ptr<objects::Trsync>, std::string>(),
             py::arg("dev"), py::arg("name") = "trsync");

    py::class_<HallFilamentSensorObject, PrinterObject,
               std::shared_ptr<HallFilamentSensorObject>>(
        m, "HallFilamentSensorObject",
        "Hall filament sensor printer object (UDS wrapper).")
        .def(py::init<std::shared_ptr<objects::HallFilamentSensor>, std::string>(),
             py::arg("dev"), py::arg("name") = "hall_filament_sensor");

    py::class_<PulseCounterObject, PrinterObject, std::shared_ptr<PulseCounterObject>>(
        m, "PulseCounterObject",
        "Pulse counter printer object (UDS wrapper).")
        .def(py::init<std::shared_ptr<objects::PulseCounter>, std::string>(),
             py::arg("dev"), py::arg("name") = "pulse_counter");

    py::class_<NeopixelObject, PrinterObject, std::shared_ptr<NeopixelObject>>(
        m, "NeopixelObject",
        "Neopixel printer object (UDS wrapper).")
        .def(py::init<std::shared_ptr<objects::Neopixel>, std::string>(),
             py::arg("dev"), py::arg("name") = "neopixel");

    py::class_<TmcUartObject, PrinterObject, std::shared_ptr<TmcUartObject>>(
        m, "TmcUartObject",
        "TMC UART printer object (UDS wrapper).")
        .def(py::init<std::shared_ptr<objects::TmcUart>, std::string>(),
             py::arg("dev"), py::arg("name") = "tmc_uart");
}
