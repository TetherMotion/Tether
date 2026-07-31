/**
 * @file KlippyInstance.hpp
 * @brief Integrated Klippy instance that auto-wires all components.
 *
 * The KlippyInstance is a top-level facade that connects:
 *   - KlippyUdsServer (Moonraker-facing UDS API)
 *   - GCodeExecutor (G-code parsing and dispatch)
 *   - VirtualSdcard (G-code file management)
 *   - Printer objects (extruder, heater_bed, fan, toolhead, etc.)
 *   - GcodeMacroRegistry (custom macro expansion)
 *   - FirmwareRetraction (G10/G11)
 *   - PressureAdvance / InputShaper state
 *   - BedMesh (bed leveling)
 *   - Endstops (for M119/query_endstops)
 *
 * This eliminates the need for users to manually wire callbacks.
 */

#pragma once

#include "tether/klipper/klippy/KlippyUdsServer.hpp"
#include "tether/klipper/klippy/GCodeExecutor.hpp"
#include "tether/klipper/klippy/AdvancedObjects.hpp"
#include "tether/klipper/klippy/PrinterObjects.hpp"
#include "tether/klipper/klippy/PrinterObjectsE2.hpp"
#include "tether/klipper/klippy/KlippyInstanceConfig.hpp"
#include "tether/klipper/klippy/KlippyHost.hpp"
#include "tether/klipper/device/KlipperDevice.hpp"
#include "tether/klipper/motion/MotionTranslator.hpp"
#include "tether/klipper/motion/MotionDispatcher.hpp"
#include "tether/klipper/config/StandardCommands.hpp"
#include "tether/klipper/objects/Thermal.hpp"
#include "tether/klipper/objects/Peripherals.hpp"
#include "tether/klipper/objects/Homing.hpp"
#include "tether/klipper/objects/BedLevel.hpp"
#include "tether/klipper/klippy/KlippyAutotuningBridge.hpp"
#include "tether/io/SpiDriver.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <cstdlib>

namespace tether::klipper::klippy {


/// @brief Config-based temperature sensor stub.
/// Reports a configurable temperature until a real hardware backend is set.
/// Used when [temperature_sensor <name>] is parsed from config but no
/// hardware sensor is registered via registerTemperatureSensor().
class ConfigTemperatureSensor : public objects::TemperatureSensor {
public:
    ConfigTemperatureSensor(const std::string& name,
                             const std::string& sensorType,
                             double minTemp, double maxTemp)
        : objects::TemperatureSensor(0)
        , name_(name)
        , sensorType_(sensorType)
        , minTemp_(minTemp)
        , maxTemp_(maxTemp) {}

    double read() override { return 25.0; } // Room temperature default
    std::string type() const override { return sensorType_; }

    const std::string& sensorName() const { return name_; }
    double minTemp() const { return minTemp_; }
    double maxTemp() const { return maxTemp_; }

private:
    std::string name_;
    std::string sensorType_;
    double minTemp_ = 0.0;
    double maxTemp_ = 200.0;
};


/// @brief Integrated Klippy instance.
///
/// Wires together the UDS server, G-code executor, virtual SD card,
/// and all printer objects. Provides a single entry point for running
/// a complete Klipper-compatible host.
class KlippyInstance {
public:
    explicit KlippyInstance(KlippyInstanceConfig cfg = {})
        : config_(std::move(cfg))
        , server_(config_.udsConfig)
        , sdcard_(std::make_shared<VirtualSdcard>(config_.sdcardDir))
        , macros_(std::make_shared<GcodeMacroRegistry>())
        , firmwareRetraction_(std::make_shared<FirmwareRetraction>())
        , pressureAdvance_(std::make_shared<PressureAdvance>())
        , inputShaper_(std::make_shared<InputShaper>())
        , bedMesh_(std::make_shared<objects::BedMesh>())
        , deltaPrinter_(std::make_shared<DeltaPrinter>())
        , rotaryDeltaPrinter_(std::make_shared<RotaryDeltaPrinter>())
        , tmcConfig_(std::make_shared<TmcDriverConfig>())
        , filamentLoader_(std::make_shared<FilamentLoader>())
        , multiMcuManager_(std::make_shared<MultiMcuManager>())
        , skewCorrection_(std::make_shared<SkewCorrection>())
        , caseLight_(std::make_shared<CaseLight>())
    {
        setupObjects();
        setupCallbacks();
        if (config_.motionBackend) {
            setupMotionBackend();
        }
    }

    // ------------------------------------------------------------------
    // Accessors
    // ------------------------------------------------------------------

    KlippyUdsServer& server() { return server_; }
    GCodeExecutor& gcode() { return gcode_; }
    VirtualSdcard& sdcard() { return *sdcard_; }
    GcodeMacroRegistry& macros() { return *macros_; }
    FirmwareRetraction& firmwareRetraction() { return *firmwareRetraction_; }
    PressureAdvance& pressureAdvance() { return *pressureAdvance_; }
    InputShaper& inputShaper() { return *inputShaper_; }
    objects::BedMesh& bedMesh() { return *bedMesh_; }
    KlippySettings& settings() { return settings_; }
    DeltaPrinter& deltaPrinter() { return *deltaPrinter_; }
    RotaryDeltaPrinter& rotaryDeltaPrinter() { return *rotaryDeltaPrinter_; }
    TmcDriverConfig& tmcConfig() { return *tmcConfig_; }
    FilamentLoader& filamentLoader() { return *filamentLoader_; }
    MultiMcuManager& multiMcuManager() { return *multiMcuManager_; }
    SkewCorrection& skewCorrection() { return *skewCorrection_; }
    CaseLight& caseLight() { return *caseLight_; }

    // Printer object accessors
    std::shared_ptr<ExtruderObject>& extruderObject() { return extruderObj_; }
    std::shared_ptr<HeaterBedObject>& heaterBedObject() { return heaterBedObj_; }
    std::shared_ptr<FanObject>& fanObject() { return fanObj_; }
    std::shared_ptr<PrintStatsObject>& printStatsObject() { return printStatsObj_; }
    std::shared_ptr<ToolheadObject>& toolheadObject() { return toolheadObj_; }
    std::shared_ptr<DisplayStatusObject>& displayStatusObject() { return displayStatusObj_; }
    std::shared_ptr<PauseResumeObject>& pauseResumeObject() { return pauseResumeObj_; }
    std::shared_ptr<MotionReportObject>& motionReportObject() { return motionReportObj_; }
    std::shared_ptr<BedMeshObject>& bedMeshObject() { return bedMeshObj_; }
    std::shared_ptr<QueryEndstopsObject>& queryEndstopsObject() { return queryEndstopsObj_; }
    std::shared_ptr<McuObject>& mcuObject() { return mcuObj_; }
    std::shared_ptr<SystemStatsObject>& systemStatsObject() { return systemStatsObj_; }
    std::shared_ptr<IdleTimeoutObject>& idleTimeoutObject() { return idleTimeoutObj_; }
    std::shared_ptr<StepperEnableObject>& stepperEnableObject() { return stepperEnableObj_; }

    // Heater/Fan/Probe backends
    /// @brief Set the heater for extruder index 0 (single-extruder convenience).
    void setExtruderHeater(std::shared_ptr<objects::Heater> h) {
        setExtruderHeater(0, h);
    }
    /// @brief Set the heater for a specific extruder index (multi-extruder).
    /// Updates the active extruder object if this index is the active one.
    void setExtruderHeater(int index, std::shared_ptr<objects::Heater> h) {
        extruderHeaters_[index] = h;
        if (index == activeExtruder_) {
            extruderHeater_ = h;
            refreshExtruderObject();
        }
    }
    /// @brief Select the active extruder by index (T-command).
    /// Updates extruderHeater_ and extruderObj_ to point at this extruder.
    void setActiveExtruder(int index) {
        activeExtruder_ = index;
        auto it = extruderHeaters_.find(index);
        extruderHeater_ = (it != extruderHeaters_.end()) ? it->second : nullptr;
        refreshExtruderObject();
    }
    /// @return The active extruder index.
    int activeExtruder() const { return activeExtruder_; }
    /// @return The heater for a given extruder index, or nullptr.
    std::shared_ptr<objects::Heater> extruderHeater(int index) const {
        auto it = extruderHeaters_.find(index);
        return (it != extruderHeaters_.end()) ? it->second : nullptr;
    }
    void setHeaterBed(std::shared_ptr<objects::Heater> h) {
        heaterBed_ = h;
    }
    void setFan(std::shared_ptr<objects::Fan> f) {
        fan_ = f;
    }
    void setProbe(std::shared_ptr<objects::Probe> p) {
        probe_ = p;
        if (probeObj_) probeObj_ = std::make_shared<ProbeObject>(p);
    }

    /// @brief Register an ADC read callback for a given pin name.
    /// Used by auto-wired thermistor sensors to read real ADC values.
    /// @param pin Pin name (e.g. "PA4", matching sensor_pin in config).
    /// @param func Callback returning the current ADC value (0..adcMax).
    void registerAdcCallback(const std::string& pin, std::function<double()> func) {
        adcCallbacks_[pin] = std::move(func);
    }

    /// @brief Get the ADC callback for a pin, or nullptr if not registered.
    std::function<double()> adcCallback(const std::string& pin) const {
        auto it = adcCallbacks_.find(pin);
        return (it != adcCallbacks_.end()) ? it->second : nullptr;
    }

    /// @brief Register an SPI transfer callback for a given SPI bus name.
    /// Used by auto-wired thermocouple and ADXL345 sensors.
    /// @param bus SPI bus name (e.g. "spi0", matching spi_bus in config).
    /// @param func Callback performing a full-duplex SPI transfer.
    void registerSpiCallback(const std::string& bus,
                             std::function<std::vector<uint8_t>(std::span<const uint8_t>)> func) {
        spiCallbacks_[bus] = std::move(func);
    }

    /// @brief Get the SPI callback for a bus, or nullptr if not registered.
    std::function<std::vector<uint8_t>(std::span<const uint8_t>)>
    spiCallback(const std::string& bus) const {
        auto it = spiCallbacks_.find(bus);
        return (it != spiCallbacks_.end()) ? it->second : nullptr;
    }

    /// @brief Register an endstop for an axis (for M119/query_endstops).
    void setEndstop(const std::string& axis, std::shared_ptr<objects::Endstop> e) {
        endstops_[axis] = e;
    }

    /// @brief Set the ADXL345 accelerometer (for input shaper calibration).
    void setAdxl345(std::shared_ptr<objects::Adxl345> a) {
        adxl345_ = a;
        adxl345Obj_ = std::make_shared<Adxl345Object>(a);
        server_.unregisterObject("adxl345");
        server_.registerObject(adxl345Obj_);
    }

    /// @brief Set the PID autotuning method to use for M303/PID_CALIBRATE.
    /// Default is RelayFeedback (Åström-Hägglund).  All Tether autotuning
    /// methods are supported (see AutotuneMethod enum).
    void setPidAutotuneMethod(AutotuneMethod method) {
        pidAutotuneMethod_ = method;
    }

    /// @brief Get the currently configured PID autotuning method.
    AutotuneMethod pidAutotuneMethod() const { return pidAutotuneMethod_; }

    /// @brief Set the I2C bus (for M260/M261 I2C commands).
    void setI2cBus(std::shared_ptr<objects::I2c> i2c) {
        i2cBus_ = i2c;
        server_.registerObject(std::make_shared<I2cObject>(i2c));
    }

    /// @brief Register a temperature sensor (e.g. "temperature_sensor chamber").
    void registerTemperatureSensor(const std::string& name,
                                    std::shared_ptr<objects::TemperatureSensor> sensor) {
        auto obj = std::make_shared<TemperatureSensorObject>(name, sensor);
        server_.registerObject(obj);
    }

    /// @brief Register a filament switch sensor.
    void registerFilamentSwitchSensor(const std::string& name,
                                       std::shared_ptr<objects::FilamentSensor> sensor) {
        auto obj = std::make_shared<FilamentSwitchSensorObject>(name, sensor);
        server_.registerObject(obj);
    }

    /// @brief Register a digital output pin.
    void registerDigitalOut(const std::string& name,
                             std::shared_ptr<objects::DigitalOut> dev) {
        server_.registerObject(std::make_shared<DigitalOutObject>(dev, name));
    }

    /// @brief Register a PWM output pin.
    void registerPwmOut(const std::string& name,
                         std::shared_ptr<objects::PwmOut> dev) {
        server_.registerObject(std::make_shared<PwmOutObject>(dev, name));
    }

    /// @brief Register an analog input pin.
    void registerAnalogIn(const std::string& name,
                           std::shared_ptr<objects::AnalogIn> dev) {
        server_.registerObject(std::make_shared<AnalogInObject>(dev, name));
    }

    /// @brief Register a Neopixel LED strip.
    void registerNeopixel(const std::string& name,
                           std::shared_ptr<objects::Neopixel> dev) {
        server_.registerObject(std::make_shared<NeopixelObject>(dev, name));
    }

    /// @brief Register a pulse counter.
    void registerPulseCounter(const std::string& name,
                               std::shared_ptr<objects::PulseCounter> dev) {
        server_.registerObject(std::make_shared<PulseCounterObject>(dev, name));
    }

    /// @brief Register a Hall filament sensor.
    void registerHallFilamentSensor(const std::string& name,
                                     std::shared_ptr<objects::HallFilamentSensor> dev) {
        server_.registerObject(std::make_shared<HallFilamentSensorObject>(dev, name));
    }

    /// @brief Register a TMC UART driver.
    void registerTmcUart(const std::string& name,
                          std::shared_ptr<objects::TmcUart> dev) {
        server_.registerObject(std::make_shared<TmcUartObject>(dev, name));
    }

    // ------------------------------------------------------------------
    // Lifecycle
    // ------------------------------------------------------------------

    bool start() {
        return server_.start();
    }

    void stop() {
        server_.stop();
    }

    /// @return The motion backend's KlippyHost, or nullptr if not configured.
    klippy::KlippyHost* motionHost() { return motionHost_.get(); }

    /// @return The motion backend's KlipperDevice, or nullptr if not configured
    ///         or no in-process device was created.
    device::KlipperDevice* motionDevice() { return motionDevice_.get(); }

    /// @return The motion backend's MotionDispatcher, or nullptr if not configured.
    motion::MotionDispatcher* motionDispatcher() { return motionDispatcher_.get(); }

    /// @return True if the motion backend is wired and the host is ready
    ///         (connected + dict downloaded + clock synced).
    bool motionBackendReady() const {
        return motionHost_ && motionHost_->isReady();
    }

    /// @brief Pump the motion backend: pump the device (if in-process) and
    ///        the host so protocol messages flow. Call this periodically
    ///        (e.g. from tick()) when using the motion backend.
    void pumpMotionBackend() {
        if (motionDevice_) motionDevice_->pump();
        if (motionHost_) motionHost_->pump();
    }

    /// @brief Execute a G-code script.
    bool executeGcode(const std::string& script) {
        return gcode_.execute(script);
    }

    /// @brief Register a G-code macro.
    void registerMacro(const GcodeMacro& macro) {
        macros_->registerMacro(macro);
        // Register a printer object for the macro
        auto obj = std::make_shared<GcodeMacroObject>(macro.name, macros_);
        server_.registerObject(obj);
    }

    // === Config processing (extracted to .ipp) ===
    #include "tether/klipper/klippy/KlippyInstanceConfigProcessing.ipp"

    /// @brief Set up the motion backend: create KlippyHost (+ optional
    /// KlipperDevice), connect, download dict, sync clock, create
    /// MotionDispatcher, and override the G-code move callback to route
    /// moves through the real Klipper wire protocol.
    void setupMotionBackend() {
        auto& mb = *config_.motionBackend;
        motionBackendCfg_ = config_.motionBackend;

        // Build the data dictionary if not pre-provided.
        protocol::DataDictionary dict = mb.dict;
        if (dict.messages().empty()) {
            config::KlipperConfig cfg;
            config::withStandardCommands(cfg, mb.clockFreqHz);
            dict = cfg.build();
        }

        // Create the in-process device if a device transport is provided.
        if (mb.deviceTransport) {
            device::KlipperDeviceConfig dcfg;
            dcfg.clockFreqHz = mb.clockFreqHz;
            motionDevice_ = std::make_unique<device::KlipperDevice>(
                mb.deviceTransport, dict, dcfg);
            motionDevice_->start();
            if (mb.registerDeviceSteppers) {
                for (uint8_t i = 0; i < 4; ++i) {
                    auto s = std::make_shared<objects::Stepper>(i);
                    motionDevice_->registerStepper(s);
                    deviceSteppers_.push_back(s);
                }
                motionDevice_->enableStepperMotion();
            }
        }

        // Create the host and connect.
        motionHost_ = std::make_unique<klippy::KlippyHost>(mb.hostTransport);
        if (mb.autoConnect) {
            motionHost_->connect();
            auto devicePump = [this]() {
                if (motionDevice_) motionDevice_->pump();
            };
            if (!motionHost_->downloadDictionary(devicePump)) return;
            if (!motionHost_->syncClock(devicePump)) return;
        }

        // Create the motion dispatcher.
        motion::MotionDispatcher::Config dcfg;
        for (size_t i = 0; i < 4; ++i) {
            dcfg.axisConfigs[i] = {mb.stepsPerMm[i], mb.invertDirection[i]};
        }
        dcfg.axisOids = mb.axisOids;
        dcfg.clockFreqHz = mb.clockFreqHz;
        dcfg.sampleIntervalSec = mb.sampleIntervalSec;
        motionDispatcher_ = std::make_unique<motion::MotionDispatcher>(dcfg);
        motionDispatcher_->setKinematicsTransform(kinematicsTransform_);

        // Wire the dispatcher's send callback to the host's step sender.
        // Pump both device and host when the serial window is full.
        motionDispatcher_->setSendCallback(
            [this](const std::vector<motion::AxisStepSequence>& seqs) {
                return motionHost_->sendStepSequences(seqs, [this]() {
                    if (motionDevice_) motionDevice_->pump();
                    motionHost_->pump();
                });
            });

        // Wire the clock provider to the host's clock sync.
        motionDispatcher_->setClockProvider([this]() {
            return motionHost_->clockSync().isSynchronised()
                ? motionHost_->clockSync().hostToMcu(clock::HostClock::now())
                : 0u;
        });

        // Override the G-code move callback to route through the dispatcher
        // in addition to updating the printer object model.
        gcode_.callbacks().move = [this](double x, double y, double z,
                                          double e, double speed) {
            // Route through the wire protocol (G-code space, no offset).
            if (motionDispatcher_) {
                motionDispatcher_->move(x, y, z, e, speed);
            }
            // Update the printer object model (with G-code offset for display).
            std::array<double, 4> pos = {x + gcodeOffset_[0], y + gcodeOffset_[1],
                                         z + gcodeOffset_[2], e + gcodeOffset_[3]};
            toolheadObj_->setPosition(pos);
            motionReportObj_->setPosition(pos);
            motionReportObj_->setVelocity(speed);
            moveQueueDepth_++;
            noteActivity();
        };
    }

    /// @brief Update system statistics (call periodically).
    void updateSystemStats() {
        if (systemStatsObj_) {
            // Read /proc/loadavg for sysload
            std::ifstream loadavg("/proc/loadavg");
            if (loadavg) {
                double load;
                loadavg >> load;
                systemStatsObj_->setSysload(load);
            }
            // Read /proc/meminfo for memavail
            std::ifstream meminfo("/proc/meminfo");
            if (meminfo) {
                std::string line;
                while (std::getline(meminfo, line)) {
                    if (line.substr(0, 9) == "MemAvaila") {
                        size_t pos = line.find(':');
                        if (pos != std::string::npos) {
                            size_t val = 0;
                            std::istringstream iss(line.substr(pos + 1));
                            iss >> val;
                            systemStatsObj_->setMemavail(val);
                        }
                        break;
                    }
                }
            }
        }
    }

    /// @brief Update MCU statistics (call periodically with real MCU data).
    void updateMcuStats(uint32_t bytesRead, uint32_t bytesWrite,
                        uint32_t retransmits, double mcuAwake) {
        if (mcuObj_) {
            mcuObj_->setStats(mcuAwake, bytesRead, bytesWrite, retransmits);
        }
    }

    /// @brief Process due delayed G-codes (call periodically).
    /// Executes any delayed G-codes whose scheduled time has passed.
    /// Also checks idle timeout and executes timeout G-code if expired.
    void tick() {
        auto now = std::chrono::steady_clock::now();

        // Pump the motion backend so protocol messages flow.
        pumpMotionBackend();

        // Process delayed G-codes
        std::vector<std::string> toExecute;
        for (auto& [id, dg] : delayedGcodes_) {
            if (dg.enabled && now >= dg.scheduledTime) {
                toExecute.push_back(dg.gcode);
                dg.enabled = false;
            }
        }
        for (const auto& gcode : toExecute) {
            executeGcode(gcode);
        }

        // Check idle timeout
        checkIdleTimeout(now);
    }

    /// @brief Check if idle timeout has expired and execute timeout G-code.
    void checkIdleTimeout(std::chrono::steady_clock::time_point now) {
        if (idleTimeout_ <= 0.0) return;
        if (settings_.idleTimeoutGcode.empty()) return;
        if (idleTimeoutState_ == "Printing") return;

        double idleSeconds = std::chrono::duration<double>(
            now - lastActivityTime_).count();
        if (idleSeconds >= idleTimeout_ && idleTimeoutState_ != "Idle") {
            // Transition to idle
            idleTimeoutState_ = "Idle";
            if (idleTimeoutObj_) {
                idleTimeoutObj_->setState("Idle");
            }
            executeGcode(settings_.idleTimeoutGcode);
        }
    }

    /// @brief Mark activity (resets idle timeout timer).
    void noteActivity() {
        lastActivityTime_ = std::chrono::steady_clock::now();
        if (idleTimeoutState_ != "Printing") {
            idleTimeoutState_ = "Ready";
            if (idleTimeoutObj_) {
                idleTimeoutObj_->setState("Ready");
            }
        }
    }

    /// @brief Set fan RPM from tachometer.
    void setFanRpm(double rpm) {
        if (fanObj_) fanObj_->setRpm(rpm);
    }

    /// @brief Set an ADC value (for QUERY_ADC to return).
    void setAdcValue(const std::string& name, double value) {
        adcValues_[name] = value;
    }

private:
    /// @brief Rebuild extruderObj_ from the current active extruder heater.
    /// Called after setExtruderHeater/setActiveExtruder so the printer
    /// object always reflects the active extruder's heater.
    void refreshExtruderObject() {
        auto newObj = std::make_shared<ExtruderObject>(extruderHeater_);
        newObj->setName("extruder");
        // Preserve settings from the old object if it existed.
        if (extruderObj_) {
            newObj->setPressureAdvance(extruderObj_->pressureAdvance());
            newObj->setMinExtrudeTemp(extruderObj_->minExtrudeTemp());
            newObj->setMotionQueue(extruderObj_->motionQueue());
        }
        extruderObj_ = newObj;
        server_.unregisterObject("extruder");
        server_.registerObject(extruderObj_);
    }

    void setupObjects() {
        // Create printer objects
        toolheadObj_ = std::make_shared<ToolheadObject>();
        displayStatusObj_ = std::make_shared<DisplayStatusObject>();
        pauseResumeObj_ = std::make_shared<PauseResumeObject>();
        printStatsObj_ = std::make_shared<PrintStatsObject>(sdcard_);
        motionReportObj_ = std::make_shared<MotionReportObject>();
        extruderObj_ = std::make_shared<ExtruderObject>(extruderHeater_);
        heaterBedObj_ = std::make_shared<HeaterBedObject>(heaterBed_);
        fanObj_ = std::make_shared<FanObject>(fan_);
        probeObj_ = std::make_shared<ProbeObject>(probe_);
        bedMeshObj_ = std::make_shared<BedMeshObject>(bedMesh_);
        mcuObj_ = std::make_shared<McuObject>();
        mcuObj_->setMcuVersion(config_.firmwareVersion);
        stepperEnableObj_ = std::make_shared<StepperEnableObject>();
        stepperEnableObj_->setStepperEnabled("stepper_x", true);
        stepperEnableObj_->setStepperEnabled("stepper_y", true);
        stepperEnableObj_->setStepperEnabled("stepper_z", true);
        stepperEnableObj_->setStepperEnabled("extruder", true);
        idleTimeoutObj_ = std::make_shared<IdleTimeoutObject>();
        systemStatsObj_ = std::make_shared<SystemStatsObject>();

        // Query endstops — wired to endstops_ map
        queryEndstopsObj_ = std::make_shared<QueryEndstopsObject>(
            [this]() {
                std::map<std::string, bool> states;
                for (const auto& [axis, es] : endstops_) {
                    states[axis] = es ? es->triggered() : false;
                }
                return states;
            });

        // ADXL345 (optional)
        if (adxl345_) {
            adxl345Obj_ = std::make_shared<Adxl345Object>(adxl345_);
        }

        auto heatersObj = std::make_shared<HeatersObject>();
        heatersObj->addHeater("extruder");
        heatersObj->addHeater("heater_bed");

        // gcode_move, configfile, webhooks, firmware_retraction
        gcodeMoveObj_ = std::make_shared<GcodeMoveObject>();
        configfileObj_ = std::make_shared<ConfigfileObject>();
        configfileObj_->setPath(config_.configPath);
        webhooksObj_ = std::make_shared<WebhooksObject>(server_);
        firmwareRetractionObj_ = std::make_shared<FirmwareRetractionObject>(firmwareRetraction_);

        // Replace the default stub objects with wired ones
        server_.unregisterObject("toolhead");
        server_.unregisterObject("pause_resume");
        server_.unregisterObject("virtual_sdcard");
        server_.unregisterObject("display_status");
        server_.unregisterObject("gcode_move");
        server_.unregisterObject("configfile");
        server_.unregisterObject("webhooks");

        // Register all objects
        server_.registerObject(toolheadObj_);
        server_.registerObject(displayStatusObj_);
        server_.registerObject(pauseResumeObj_);
        server_.registerObject(std::make_shared<VirtualSdcardObject>(sdcard_));
        server_.registerObject(printStatsObj_);
        server_.registerObject(motionReportObj_);
        server_.registerObject(extruderObj_);
        server_.registerObject(heaterBedObj_);
        server_.registerObject(fanObj_);
        server_.registerObject(heatersObj);
        server_.registerObject(mcuObj_);
        server_.registerObject(stepperEnableObj_);
        server_.registerObject(idleTimeoutObj_);
        server_.registerObject(systemStatsObj_);
        server_.registerObject(bedMeshObj_);
        server_.registerObject(queryEndstopsObj_);
        server_.registerObject(gcodeMoveObj_);
        server_.registerObject(configfileObj_);
        server_.registerObject(webhooksObj_);
        server_.registerObject(firmwareRetractionObj_);
        if (probe_) server_.registerObject(probeObj_);
        if (adxl345Obj_) server_.registerObject(adxl345Obj_);

        // B4: Register new printer objects
        outputPinObj_ = std::make_shared<OutputPinObject>("output_pin");
        pwmToolObj_ = std::make_shared<PwmToolObject>("pwm_tool");
        temperatureFanObj_ = std::make_shared<TemperatureFanObject>("temperature_fan");
        controllerFanObj_ = std::make_shared<ControllerFanObject>("controller_fan");
        heaterFanObj_ = std::make_shared<HeaterFanObject>("heater_fan");
        fanGenericObj_ = std::make_shared<FanGenericObject>("fan_generic");
        ledObj_ = std::make_shared<LedObject>("led");
        dotstarObj_ = std::make_shared<DotstarObject>("dotstar");
        servoObj_ = std::make_shared<ServoObject>("servo");
        bltouchObj_ = std::make_shared<BltouchObject>();
        zTiltObj_ = std::make_shared<ZTiltObject>();
        quadGantryLevelObj_ = std::make_shared<QuadGantryLevelObject>();
        screwsTiltAdjustObj_ = std::make_shared<ScrewsTiltAdjustObject>();
        bedScrewsObj_ = std::make_shared<BedScrewsObject>();
        deltaCalibrateObj_ = std::make_shared<DeltaCalibrateObject>();
        skewCorrectionObj_ = std::make_shared<SkewCorrectionObject>();
        inputShaperObj_ = std::make_shared<InputShaperObject>();
        pressureAdvanceObj_ = std::make_shared<PressureAdvanceObject>();
        excludeObjectObj_ = std::make_shared<ExcludeObjectObject>();
        zThermalAdjustObj_ = std::make_shared<ZThermalAdjustObject>();
        heaterGenericObj_ = std::make_shared<HeaterGenericObject>("heater_generic");
        temperatureProbeObj_ = std::make_shared<TemperatureProbeObject>("temperature_probe");
        forceMoveObj_ = std::make_shared<ForceMoveObject>();
        dualCarriageObj_ = std::make_shared<DualCarriageObject>();
        extruderStepperObj_ = std::make_shared<ExtruderStepperObject>("extruder_stepper");
        manualStepperObj_ = std::make_shared<ManualStepperObject>("manual_stepper");
        endstopPhaseObj_ = std::make_shared<EndstopPhaseObject>("endstop_phase");
        safeZHomeObj_ = std::make_shared<SafeZHomeObject>();
        bedTiltObj_ = std::make_shared<BedTiltObject>();
        multiPinObj_ = std::make_shared<MultiPinObject>("multi_pin");
        buttonObj_ = std::make_shared<ButtonObject>("button");
        smartEffectorObj_ = std::make_shared<SmartEffectorObject>();
        tmcDriverObj_ = std::make_shared<TmcDriverObject>("tmc2209");

        server_.registerObject(outputPinObj_);
        server_.registerObject(pwmToolObj_);
        server_.registerObject(temperatureFanObj_);
        server_.registerObject(controllerFanObj_);
        server_.registerObject(heaterFanObj_);
        server_.registerObject(fanGenericObj_);
        server_.registerObject(ledObj_);
        server_.registerObject(dotstarObj_);
        server_.registerObject(servoObj_);
        server_.registerObject(bltouchObj_);
        server_.registerObject(zTiltObj_);
        server_.registerObject(quadGantryLevelObj_);
        server_.registerObject(screwsTiltAdjustObj_);
        server_.registerObject(bedScrewsObj_);
        server_.registerObject(deltaCalibrateObj_);
        server_.registerObject(skewCorrectionObj_);
        server_.registerObject(inputShaperObj_);
        server_.registerObject(pressureAdvanceObj_);
        server_.registerObject(excludeObjectObj_);
        server_.registerObject(zThermalAdjustObj_);
        server_.registerObject(heaterGenericObj_);
        server_.registerObject(temperatureProbeObj_);
        server_.registerObject(forceMoveObj_);
        server_.registerObject(dualCarriageObj_);
        server_.registerObject(extruderStepperObj_);
        server_.registerObject(manualStepperObj_);
        server_.registerObject(endstopPhaseObj_);
        server_.registerObject(safeZHomeObj_);
        server_.registerObject(bedTiltObj_);
        server_.registerObject(multiPinObj_);
        server_.registerObject(buttonObj_);
        server_.registerObject(smartEffectorObj_);
        server_.registerObject(tmcDriverObj_);

        // D2: Register new printer objects
        manualProbeObj_ = std::make_shared<ManualProbeObject>();
        server_.registerObject(manualProbeObj_);
        filamentMotionSensorObj_ = std::make_shared<FilamentMotionSensorObject>("filament_motion_sensor");
        server_.registerObject(filamentMotionSensorObj_);
        loadCellObj_ = std::make_shared<LoadCellObject>("load_cell");
        server_.registerObject(loadCellObj_);
        canbusStatsObj_ = std::make_shared<CanbusStatsObject>("canbus_stats");
        server_.registerObject(canbusStatsObj_);
        pwmCycleTimeObj_ = std::make_shared<PwmCycleTimeObject>("pwm_cycle_time");
        server_.registerObject(pwmCycleTimeObj_);
        resonanceTesterObj_ = std::make_shared<ResonanceTesterObject>();
        server_.registerObject(resonanceTesterObj_);
        angleObj_ = std::make_shared<AngleObject>("angle");
        server_.registerObject(angleObj_);
        palette2Obj_ = std::make_shared<Palette2Object>();
        server_.registerObject(palette2Obj_);
        menuObj_ = std::make_shared<MenuObject>();
        server_.registerObject(menuObj_);
        gcodeObj_ = std::make_shared<GcodeObject>();
        server_.registerObject(gcodeObj_);

        // E4: Register new printer objects from PrinterObjectsE2.hpp
        delayedGcodeObj_ = std::make_shared<DelayedGcodeObject>("delayed_gcode");
        server_.registerObject(delayedGcodeObj_);
        saveVariablesObj_ = std::make_shared<SaveVariablesObject>();
        server_.registerObject(saveVariablesObj_);
        boardPinsObj_ = std::make_shared<BoardPinsObject>();
        server_.registerObject(boardPinsObj_);
    }


    // === G-code callbacks (extracted to .ipp) ===
    #include "tether/klipper/klippy/KlippyInstanceCallbacks.ipp"

    // === Extended command handlers (extracted to .ipp) ===
    #include "tether/klipper/klippy/KlippyInstanceExtendedCommands.ipp"


    // ------------------------------------------------------------------
    // Internal state
    // ------------------------------------------------------------------

    KlippyInstanceConfig config_;
    KlippyUdsServer server_;
    std::shared_ptr<VirtualSdcard> sdcard_;
    std::shared_ptr<GcodeMacroRegistry> macros_;
    std::shared_ptr<FirmwareRetraction> firmwareRetraction_;
    std::shared_ptr<PressureAdvance> pressureAdvance_;
    std::shared_ptr<InputShaper> inputShaper_;
    std::shared_ptr<objects::BedMesh> bedMesh_;
    std::shared_ptr<objects::Adxl345> adxl345_;

    // Motion backend (optional, wired when config_.motionBackend is set)
    std::shared_ptr<MotionBackendConfig> motionBackendCfg_;
    std::unique_ptr<klippy::KlippyHost> motionHost_;
    std::unique_ptr<device::KlipperDevice> motionDevice_;
    std::unique_ptr<motion::MotionDispatcher> motionDispatcher_;
    std::vector<std::shared_ptr<objects::Stepper>> deviceSteppers_;

    // Heater/Fan/Probe backends (set by user)
    std::shared_ptr<objects::Heater> extruderHeater_;  ///< active extruder's heater
    std::map<int, std::shared_ptr<objects::Heater>> extruderHeaters_; ///< per-index heaters
    int activeExtruder_ = 0; ///< active extruder index (T-command)
    std::shared_ptr<objects::Heater> heaterBed_;
    std::shared_ptr<objects::Fan> fan_;
    std::shared_ptr<objects::Probe> probe_;

    // Endstops (set by user)
    std::map<std::string, std::shared_ptr<objects::Endstop>> endstops_;

    // ADC/SPI callbacks (set by user for auto-wired sensors)
    std::map<std::string, std::function<double()>> adcCallbacks_;
    std::map<std::string, std::function<std::vector<uint8_t>(std::span<const uint8_t>)>> spiCallbacks_;

    // Printer objects
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

    // B4: New printer objects
    std::shared_ptr<OutputPinObject> outputPinObj_;
    std::shared_ptr<PwmToolObject> pwmToolObj_;
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
    std::shared_ptr<PressureAdvanceObject> pressureAdvanceObj_;
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

    // D2: New printer objects
    std::shared_ptr<class ManualProbeObject> manualProbeObj_;
    std::shared_ptr<class FilamentMotionSensorObject> filamentMotionSensorObj_;
    std::shared_ptr<class LoadCellObject> loadCellObj_;
    std::shared_ptr<class CanbusStatsObject> canbusStatsObj_;
    std::shared_ptr<class PwmCycleTimeObject> pwmCycleTimeObj_;
    std::shared_ptr<class ResonanceTesterObject> resonanceTesterObj_;
    std::shared_ptr<class AngleObject> angleObj_;
    std::shared_ptr<class Palette2Object> palette2Obj_;
    std::shared_ptr<class MenuObject> menuObj_;
    std::shared_ptr<class GcodeObject> gcodeObj_;

    // Advanced feature objects
    std::shared_ptr<DeltaPrinter> deltaPrinter_;
    std::shared_ptr<RotaryDeltaPrinter> rotaryDeltaPrinter_;
    std::shared_ptr<TmcDriverConfig> tmcConfig_;
    std::shared_ptr<FilamentLoader> filamentLoader_;
    std::shared_ptr<MultiMcuManager> multiMcuManager_;
    std::shared_ptr<SkewCorrection> skewCorrection_;
    std::shared_ptr<CaseLight> caseLight_;
    std::shared_ptr<objects::I2c> i2cBus_;

    // E4: Kinematics transform (wired from config)
    motion::KinematicsTransform kinematicsTransform_;

    // E4: New printer objects from PrinterObjectsE2.hpp
    std::shared_ptr<DelayedGcodeObject> delayedGcodeObj_;
    std::shared_ptr<SaveVariablesObject> saveVariablesObj_;
    std::shared_ptr<BoardPinsObject> boardPinsObj_;

    // E4: Tracking which config-derived temp sensors have been registered
    std::set<std::string> configTempSensorsRegistered_;

    // G-code state
    PrinterMotionState motionState_;
    GCodeExecutor gcode_ = GCodeExecutor(GcodeCallbacks{});
    KlippySettings settings_;

    // Runtime state
    double autoTempInterval_ = 0.0;
    uint32_t moveQueueDepth_ = 0;

    // Peripheral state (for M42/M300/M280/M150)
    std::map<int, double> outputPins_;
    double lastBeepFreq_ = 0.0;
    double lastBeepDuration_ = 0.0;
    std::map<int, double> servos_;
    std::array<int, 4> ledColor_ = {0, 0, 0, 0};
    int lcdContrast_ = 100;
    std::array<double, 4> g92Offsets_ = {0, 0, 0, 0};
    std::array<double, 4> savedG92Offsets_ = {0, 0, 0, 0};
    std::chrono::steady_clock::time_point lastCameraTrigger_;

    // CNC state
    double spindleRpm_ = 0.0;
    int activeTool_ = 0;
    bool coolantFlood_ = false;
    bool coolantMist_ = false;

    // Thermistor parameters (M305)
    std::map<int, std::array<double, 4>> thermistorParams_;

    // Motion callback (for G53 machine moves and canned cycles)
    std::function<void(double, double, double, double, double)> moveCallback_;

    // --- Extended command state ---

    // Servo state (SET_SERVO): name -> {angle, pulse_width}
    struct ServoState {
        double angle = 0.0;
        double pulseWidth = 0.0;
    };
    std::map<std::string, ServoState> servoStates_;

    // Bed mesh profiles (BED_MESH_PROFILE): name -> serialized mesh
    struct BedMeshProfile {
        std::string name;
        bool loaded = false;
    };
    std::map<std::string, BedMeshProfile> bedMeshProfiles_;

    // Save config pending items (SAVE_CONFIG)
    std::map<std::string, std::string> saveConfigPendingItems_;

    // G-code offset (SET_GCODE_OFFSET)
    std::array<double, 4> gcodeOffset_ = {0, 0, 0, 0}; // X, Y, Z, E

    // Extruder rotation distance (SET_EXTRUDER_ROTATION_DISTANCE)
    std::map<std::string, double> extruderRotationDistance_;

    // Named output pins (SET_PIN / SET_PWM_PIN)
    std::map<std::string, double> namedOutputPins_;
    std::map<std::string, double> namedPwmPins_;

    // Generic fan speeds (SET_FAN_SPEED)
    std::map<std::string, double> genericFanSpeeds_;

    // Temperature fan targets (SET_TEMPERATURE_FAN)
    std::map<std::string, double> temperatureFanTargets_;

    // ADC query results (QUERY_ADC)
    std::map<std::string, double> adcValues_;

    // LED state (SET_LED)
    struct LedState {
        std::array<double, 4> color = {0, 0, 0, 0}; // R, G, B, W
        bool white = false;
        int index = 0;
    };
    std::map<std::string, LedState> ledStates_;

    // Neopixel state (SET_NEOPIXEL)
    std::map<std::string, std::vector<std::array<double, 4>>> neopixelStates_;

    // Probe calibration state (PROBE_CALIBRATE)
    struct ProbeCalibState {
        bool active = false;
        double zPosition = 0.0;
        double zOffset = 0.0;
    };
    ProbeCalibState probeCalibState_;

    // Stepper enable overrides (SET_STEPPER_ENABLE)
    std::map<std::string, bool> stepperEnableOverrides_;

    // Idle timeout (SET_IDLE_TIMEOUT)
    double idleTimeout_ = 600.0; // 10 minutes default
    std::chrono::steady_clock::time_point lastActivityTime_ =
        std::chrono::steady_clock::now();
    std::string idleTimeoutState_ = "Ready";

    // Delayed G-code (SET_DELAYED_GCODE / UPDATE_DELAYED_GCODE)
    struct DelayedGcode {
        std::string gcode;
        double delay = 0.0;
        std::chrono::steady_clock::time_point scheduledTime;
        bool enabled = false;
    };
    std::map<std::string, DelayedGcode> delayedGcodes_;

    // Dual carriage (SET_DUAL_CARRIAGE)
    struct DualCarriageState {
        int mode = 0;
        double offset = 0.0;
    };
    std::map<std::string, DualCarriageState> dualCarriageStates_;

    // G-code variables (SET_GCODE_VARIABLE / SAVE_VARIABLE)
    std::map<std::string, std::map<std::string, std::string>> gcodeVariables_;
    std::map<std::string, std::string> savedVariables_;

    // TMC field overrides (SET_TMC_FIELD)
    std::map<std::string, std::map<std::string, uint32_t>> tmcFieldOverrides_;

    // Exclude object state (EXCLUDE_OBJECT_*)
    struct ExcludeObject {
        std::string name;
        std::vector<std::array<double, 2>> polygon; // XY outline
        bool excluded = false;
        bool started = false;
        bool finished = false;
    };
    std::vector<ExcludeObject> excludeObjects_;
    int currentExcludeObject_ = -1;

    // Manual probe state (MANUAL_PROBE / ABORT / ACCEPT / ADJUSTED)
    struct ManualProbeState {
        bool active = false;
        double zPosition = 0.0;
        double zOffset = 0.0;
    };
    ManualProbeState manualProbeState_;

    // Sync extruder stepper (SYNC_EXTRUDER_STEPPER)
    std::map<std::string, std::string> extruderStepperSync_; // stepper -> motion_queue

    // Print stats info (SET_PRINT_STATS_INFO)
    std::string printStatsInfoTotalLayer_;
    std::string printStatsInfoCurrentLayer_;

    // Display group (SET_DISPLAY_GROUP)
    std::string displayGroup_;

    // D4: Config section state
    bool excludeObjectEnabled_ = false;
    bool saveVariablesEnabled_ = false;
    bool forceMoveEnabled_ = false;
    bool palette2Configured_ = false;
    std::map<std::string, std::string> homingOverrides_;
    struct EndstopPhaseState {
        int endstopAlignTolerance = 0;
    };
    std::map<std::string, EndstopPhaseState> endstopPhases_;
    std::map<std::string, std::string> menuDefinitions_;

    // G-code offset application mode for SET_GCODE_OFFSET
    // RESET=0, ADJUST=1
    int gcodeOffsetMode_ = 0;

    // D3: Accelerometer state (TEST_RESONANCES / SHAPER_CALIBRATE)
    bool accelerometerMeasuring_ = false;
    std::vector<std::array<double, 3>> accelerometerData_;

    // Autotuning bridges (delegate to Tether autotuning framework)
    std::unique_ptr<HeaterAutotuneBridge> extruderAutotuneBridge_;
    std::unique_ptr<HeaterAutotuneBridge> bedAutotuneBridge_;
    std::unique_ptr<ResonanceCalibrationBridge> resonanceBridge_;
    AutotuneMethod pidAutotuneMethod_ = AutotuneMethod::RelayFeedback;

    // E3: G-code state save/restore (SAVE_GCODE_STATE / RESTORE_GCODE_STATE)
    struct GcodeState {
        std::array<double, 4> position = {0, 0, 0, 0};
        std::array<double, 4> gcodeOffset = {0, 0, 0, 0};
        bool absoluteCoords = true;
        bool absoluteExtrude = false;
        double feedrate = 1500.0;
        double speedFactor = 1.0;
        double extrudeFactor = 1.0;
    };
    std::map<std::string, GcodeState> gcodeStates_;

    // E3: Active extruder name (ACTIVATE_EXTRUDER)
    std::string activeExtruderName_ = "extruder";

    // E3: Extruder step distance (SET_EXTRUDER_STEP_DISTANCE)
    std::map<std::string, double> extruderStepDistance_;

    // E3: Digital pin states (SET_DIGITAL_PIN)
    std::map<std::string, bool> digitalPinStates_;

    // E3: Dotstar LED states (SET_DOTSTAR)
    std::map<std::string, std::vector<std::array<double, 4>>> dotstarStates_;

    // E3: Bed mesh offset (BED_MESH_OFFSET)
    double bedMeshOffsetX_ = 0.0;
    double bedMeshOffsetY_ = 0.0;

    // E3: Excluded objects (EXCLUDE_OBJECT_RESET)
    std::set<std::string> excludedObjects_;
    bool excludeObjectStarted_ = false;

    // E3: Stepper current (SET_CURRENT)
    std::map<std::string, double> stepperCurrents_;
    std::map<std::string, double> stepperHoldCurrents_;

    // E3: Home position (SET_HOME_POSITION)
    std::array<double, 3> homePosition_ = {0, 0, 0};

    // E3: Endstop home positions (ENDSTOP_HOME)
    std::map<std::string, double> endstopHomePositions_;
};

} // namespace tether::klipper::klippy
