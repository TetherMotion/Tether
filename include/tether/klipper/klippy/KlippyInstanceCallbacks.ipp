/// @file KlippyInstanceCallbacks.ipp
/// @brief G-code callback setup for KlippyInstance (included into class body).
///
/// This file is included inside the KlippyInstance class definition.
/// It contains the setupCallbacks() method which wires all G-code
/// executor callbacks to the appropriate hardware/software backends.

    void setupCallbacks() {
        GcodeCallbacks cb;

        // Motion
        // cb.move is overridden later in KlippyInstance.hpp to route through
        // the motion dispatcher with coordinate transform applied. Here we
        // set up the stored callbacks used by G53 and canned cycles.
        cb.move = [this](double x, double y, double z, double e, double speed) {
            // Apply G-code offset (program-space additive) then coordinate
            // transform (WCS + G52 + G68 rotation + G51 scale).
            double px = x + gcodeOffset_[0];
            double py = y + gcodeOffset_[1];
            double pz = z + gcodeOffset_[2];
            auto m = motionState_.coordTransform.toMachineXYZ(px, py, pz);
            std::array<double, 4> pos = {m[0], m[1], m[2], e + gcodeOffset_[3]};
            toolheadObj_->setPosition(pos);
            motionReportObj_->setPosition(pos);
            motionReportObj_->setVelocity(speed);
            moveQueueDepth_++;
            noteActivity();
        };
        // Stored move callback (applies transform) — for canned cycles.
        moveCallback_ = [this](double x, double y, double z, double e, double speed) {
            double px = x + gcodeOffset_[0];
            double py = y + gcodeOffset_[1];
            double pz = z + gcodeOffset_[2];
            auto m = motionState_.coordTransform.toMachineXYZ(px, py, pz);
            std::array<double, 4> pos = {m[0], m[1], m[2], e + gcodeOffset_[3]};
            toolheadObj_->setPosition(pos);
            motionReportObj_->setPosition(pos);
            motionReportObj_->setVelocity(speed);
            moveQueueDepth_++;
        };
        // Raw move callback (bypasses transform) — for G53 machine coords.
        moveCallbackRaw_ = [this](double x, double y, double z, double e, double speed) {
            std::array<double, 4> pos = {x, y, z, e};
            toolheadObj_->setPosition(pos);
            motionReportObj_->setPosition(pos);
            motionReportObj_->setVelocity(speed);
            moveQueueDepth_++;
        };

        // Homing
        cb.home = [this](const std::string& axes) {
            // Safe Z home: if Z is being homed and safe_z_home is configured,
            // perform Z hop and move to safe XY position first.
            if (axes.find('z') != std::string::npos &&
                settings_.safeZHomeZHop > 0.0) {
                // Z hop: move Z up by z_hop before homing
                if (moveCallback_) {
                    moveCallback_(0, 0, settings_.safeZHomeZHop,
                                  settings_.safeZHomeZHopSpeed, 0);
                }
                // Move to safe XY position
                if (moveCallback_ && settings_.safeZHomeXYPosition != "0,0") {
                    // Parse XY position string "x, y"
                    double sx = 0, sy = 0;
                    {
                        std::string pos = settings_.safeZHomeXYPosition;
                        for (auto& c : pos) if (c == ',') c = ' ';
                        std::istringstream iss(pos);
                        iss >> sx >> sy;
                    }
                    moveCallback_(sx, sy, settings_.safeZHomeZHop,
                                  settings_.safeZHomeXYHomeSpeed, 0);
                }
            }

            // Set position to NaN (unknown) for axes being homed.
            // In real Klipper, unhomed axes have no known position.
            {
                std::array<double, 4> pos = motionState_.position;
                for (char c : axes) {
                    int idx = (c == 'x') ? 0 : (c == 'y') ? 1 : (c == 'z') ? 2 : -1;
                    if (idx >= 0) pos[idx] = std::numeric_limits<double>::quiet_NaN();
                }
                motionState_.position = pos;
                toolheadObj_->setPosition(pos);
                motionReportObj_->setPosition(pos);
            }

            // Home each axis sequentially using the kinematic simulator.
            // This moves the axis toward its endstop position at homing_speed,
            // producing realistic position updates over time.
            for (char c : axes) {
                std::string axisName(1, c);
                int idx = (c == 'x') ? 0 : (c == 'y') ? 1 : (c == 'z') ? 2 : -1;
                if (idx < 0) continue;

                // Get homing config for this axis
                auto homingIt = settings_.stepperHoming.find(axisName);
                if (homingIt == settings_.stepperHoming.end()) {
                    // No config — just mark as homed at current position (0)
                    motionState_.position[idx] = 0.0;
                    continue;
                }
                const auto& hcfg = homingIt->second;

                // Build simulator config
                Simulation::HomingAxisConfig simCfg;
                simCfg.name = axisName;
                simCfg.endstopPosition = hcfg.positionEndstop;
                simCfg.homingSpeed = hcfg.homingSpeed;
                simCfg.secondHomingSpeed = hcfg.secondHomingSpeed;
                simCfg.acceleration = settings_.acceleration;
                simCfg.positiveDirection = hcfg.positiveDirection;

                // Run the homing simulation
                Simulation::HomingAxisSimulator sim(simCfg);
                double initPos = motionState_.position[idx];
                if (std::isnan(initPos)) initPos = hcfg.positiveDirection ? -100.0 : 100.0;
                double finalPos = sim.run(initPos, 0.001);

                // Update position
                motionState_.position[idx] = finalPos;
            }

            // Update printer objects with final positions
            {
                std::array<double, 4> pos = motionState_.position;
                toolheadObj_->setPosition(pos);
                motionReportObj_->setPosition(pos);
                gcodeMoveObj_->setGcodePosition(pos);
            }

            toolheadObj_->setHomedAxes(axes);
            // Track activity for idle timeout
            lastActivityTime_ = std::chrono::steady_clock::now();
        };

        // Temperature
        cb.setHotendTemp = [this](int extruder, double temp, bool wait) {
            // extruder == -1 means no T parameter was specified in the G-code;
            // use the active extruder's heater in that case.
            int idx = (extruder >= 0) ? extruder : activeExtruder_;
            auto it = extruderHeaters_.find(idx);
            if (it != extruderHeaters_.end() && it->second) {
                it->second->setTarget(temp);
            } else if (extruderHeater_) {
                extruderHeater_->setTarget(temp);
            }
        };
        cb.setBedTemp = [this](double temp, bool wait) {
            if (heaterBed_) heaterBed_->setTarget(temp);
        };

        // Fan
        cb.setFanSpeed = [this](double speed) {
            if (fan_) fan_->setSpeed(speed);
        };

        // Motors — wire to StepperEnableObject
        cb.setMotorEnable = [this](const std::string& axes, bool enable) {
            for (char c : axes) {
                std::string axis(1, c);
                stepperEnableObj_->setStepperEnabled(
                    c == 'x' ? "stepper_x" :
                    c == 'y' ? "stepper_y" :
                    c == 'z' ? "stepper_z" :
                    c == 'e' ? "extruder" : axis, enable);
            }
            stepperEnableObj_->setEnabled(enable);
        };

        // Emergency stop
        cb.emergencyStop = [this]() {
            server_.setState(PrinterState::Shutdown, "Emergency stop");
        };

        // Probe
        cb.probe = [this]() -> double {
            if (probe_ && probe_->triggered()) {
                return motionState_.position[2];
            }
            return NAN;
        };

        // Set position
        cb.setPosition = [this](double x, double y, double z, double e) {
            if (!std::isnan(x)) motionState_.position[0] = x;
            if (!std::isnan(y)) motionState_.position[1] = y;
            if (!std::isnan(z)) motionState_.position[2] = z;
            if (!std::isnan(e)) motionState_.position[3] = e;
        };

        // Dwell — blocks the G-code executor for the specified duration.
        // This is intentional: the G-code executor is single-threaded and processes
        // commands sequentially. A dwell is supposed to pause execution.
        // For very long dwells, consider yielding to allow UDS event processing.
        cb.dwell = [](double seconds) {
            if (seconds > 0) {
                std::this_thread::sleep_for(
                    std::chrono::microseconds(static_cast<int64_t>(seconds * 1e6)));
            }
        };

        // Output
        cb.output = [this](const std::string& msg) {
            server_.emitGcodeResponse(msg);
        };

        // Firmware retraction
        cb.retract = [this]() { return firmwareRetraction_->retract(); };
        cb.unretract = [this]() { return firmwareRetraction_->unretract(); };

        // SD card
        cb.selectSdFile = [this](const std::string& filename) {
            return sdcard_->selectFile(filename);
        };
        cb.startSdPrint = [this]() {
            sdcard_->startPrint();
            printStatsObj_->setState("printing");
            idleTimeoutObj_->setState("Printing");
        };
        cb.pauseSdPrint = [this]() {
            sdcard_->pausePrint();
            printStatsObj_->setState("paused");
            pauseResumeObj_->setPaused(true);
            idleTimeoutObj_->setState("Idle");
        };
        cb.sdStatus = [this]() {
            std::ostringstream ss;
            ss << "SD printing byte " << sdcard_->filePosition()
               << "/" << sdcard_->fileSize();
            return ss.str();
        };

        // Display
        cb.setDisplayProgress = [this](double p) {
            displayStatusObj_->setProgress(p);
        };
        cb.setDisplayMessage = [this](const std::string& msg) {
            displayStatusObj_->setMessage(msg);
        };

        // Status queries
        cb.getPositionStatus = [this]() {
            // Report program coordinates (inverse-transformed from machine).
            auto p = motionState_.coordTransform.toProgramXYZ(
                motionState_.position[0], motionState_.position[1],
                motionState_.position[2]);
            std::ostringstream ss;
            ss << "X:" << p[0]
               << " Y:" << p[1]
               << " Z:" << p[2]
               << " E:" << motionState_.position[3]
               << " Count A:0 B:0 C:0";
            return ss.str();
        };
        cb.getEndstopStatus = [this]() {
            std::ostringstream ss;
            for (const auto& [axis, es] : endstops_) {
                ss << axis << ":" << (es && es->triggered() ? "TRIGGERED" : "open") << " ";
            }
            if (endstops_.empty()) {
                ss << "x:open y:open z:open";
            }
            return ss.str();
        };
        cb.getTempStatus = [this]() {
            std::ostringstream ss;
            ss << "T:" << (extruderHeater_ ? extruderHeater_->currentTemp() : 0.0)
               << " /" << (extruderHeater_ ? extruderHeater_->target() : 0.0)
               << " B:" << (heaterBed_ ? heaterBed_->currentTemp() : 0.0)
               << " /" << (heaterBed_ ? heaterBed_->target() : 0.0)
               << " @:0 B@:0";
            return ss.str();
        };
        cb.setAutoTempReport = [this](double interval) {
            autoTempInterval_ = interval;
        };

        // Sync — flush the move queue
        cb.waitForMoves = [this]() {
            moveQueueDepth_ = 0;
        };

        // Bed leveling — wire to BedMesh
        cb.bedLevel = [this]() {
            bedMesh_->clear();
            if (probe_) {
                probeBedGrid();
            }
            settings_.bedMeshEnabled = true;
        };

        // Pressure advance / input shaper
#if TETHER_ENABLE_PRESSURE_ADVANCE
        cb.setPressureAdvance = [this](int extruder, double pa) {
            pressureAdvance_->setEnabled(true);
            pressureAdvance_->setParams({pa, pressureAdvance_->params().smoothTime});
            extruderObj_->setPressureAdvance(pa);
            // Sync to the motion dispatcher for step generation.
            if (motionDispatcher_) {
                auto paCfg = motionDispatcher_->pressureAdvanceConfig();
                paCfg.enabled = true;
                paCfg.pressureAdvance = pa;
                motionDispatcher_->setPressureAdvanceConfig(paCfg);
            }
        };
#endif
        cb.setInputShaperParams = [this](const std::string&, double freq, const std::string& type) {
            InputShaperParams params = inputShaper_->params();
            params.freq = freq;
            if (type == "ZV") params.type = ShaperType::ZV;
            else if (type == "ZVD") params.type = ShaperType::ZVD;
            else if (type == "MZV") params.type = ShaperType::MZV;
            else if (type == "EI") params.type = ShaperType::EI;
            else if (type == "damped_ei") params.type = ShaperType::DampedEI;
            else params.type = ShaperType::None;
            inputShaper_->setParams(params);
        };

        // Nozzle clean — execute a cleaning pattern
        cb.cleanNozzle = [this](double iterations, double radius, double speed) {
            // Execute a circular cleaning pattern via the move callback
            double centerX = motionState_.position[0];
            double centerY = motionState_.position[1];
            int iters = static_cast<int>(iterations);
            if (iters <= 0) iters = 3;
            if (radius <= 0) radius = 5.0;
            double rps = speed > 0 ? speed / 60.0 : 1.0;
            int steps = 32;
            for (int it = 0; it < iters; ++it) {
                for (int i = 0; i <= steps; ++i) {
                    double angle = 2.0 * M_PI * i / steps;
                    double x = centerX + radius * std::cos(angle);
                    double y = centerY + radius * std::sin(angle);
                    // Update position and notify via toolhead/motion report
                    std::array<double, 4> pos = {x, y, motionState_.position[2], motionState_.position[3]};
                    toolheadObj_->setPosition(pos);
                    motionReportObj_->setPosition(pos);
                    motionReportObj_->setVelocity(rps * radius);
                }
            }
        };

        // Settings — persist to file
        cb.saveSettings = [this]() {
            saveSettingsToFile();
        };
        cb.loadSettings = [this]() {
            loadSettingsFromFile();
        };
        cb.resetSettings = [this]() {
            settings_ = KlippySettings{};
        };
        cb.reportSettings = [this]() {
            return reportSettings();
        };

        // Firmware info
        cb.getFirmwareInfo = [this]() {
            std::ostringstream ss;
            ss << "FIRMWARE_NAME:TetherKlipper " << config_.firmwareVersion
               << " FIRMWARE_URL:https://tether.dev Capabilities:AUTOREPORT_TEMP";
            return ss.str();
        };

        // Wait for temperatures — poll heaters until they reach target
        cb.waitForTemperatures = [this]() {
            const double tolerance = 1.0; // 1 degree C tolerance
            const int maxIterations = 300; // 30 second max wait (100ms per poll)
            for (int i = 0; i < maxIterations; ++i) {
                bool allAtTarget = true;
                if (extruderHeater_ && extruderHeater_->target() > 0) {
                    if (std::abs(extruderHeater_->currentTemp() -
                                 extruderHeater_->target()) > tolerance) {
                        allAtTarget = false;
                    }
                }
                if (heaterBed_ && heaterBed_->target() > 0) {
                    if (std::abs(heaterBed_->currentTemp() -
                                 heaterBed_->target()) > tolerance) {
                        allAtTarget = false;
                    }
                }
                if (allAtTarget) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        };

        // Stepper configuration
        cb.setStepsPerMm = [this](const std::string& axis, double steps) {
            settings_.stepsPerMm[axis] = steps;
        };
        cb.setMicrostepping = [this](const std::string& axis, int ms) {
            settings_.microstepping[axis] = ms;
        };
        cb.setStepperCurrent = [this](const std::string& axis, double currentMa) {
            settings_.stepperCurrent[axis] = currentMa;
        };
        cb.setStepperDirection = [this](const std::string& axis, int dir) {
            settings_.stepperDirection[axis] = dir;
        };

        // Motion limits
        cb.setMaxFeedrate = [this](const std::string& axis, double feedrate) {
            settings_.maxFeedrate[axis] = feedrate;
        };
        cb.setAcceleration = [this](double accel, double travelAccel) {
            settings_.acceleration = accel;
            settings_.travelAcceleration = travelAccel > 0 ? travelAccel : accel;
        };
        cb.setAdvancedMotion = [this](double jerk, double startAccel) {
            settings_.jerk = jerk;
            settings_.startAccel = startAccel;
        };

        // Offsets
        cb.setHomeOffset = [this](const std::string& axis, double offset) {
            settings_.homeOffset[axis] = offset;
        };
        cb.setToolOffset = [this](int tool, const std::string& axis, double offset) {
            if (tool >= 0 && tool < 8) settings_.toolOffset[tool][axis] = offset;
        };
        cb.setProbeOffset = [this](double offset) {
            settings_.probeOffset = offset;
            if (probeObj_) probeObj_->setZOffset(offset);
        };

        // Retract settings
        cb.setRetractParams = [this](double length, double speed, double zLift) {
            settings_.retractLength = length;
            settings_.retractSpeed = speed;
            settings_.retractZLift = zLift;
        };
        cb.setUnretractParams = [this](double length, double speed) {
            settings_.unretractLength = length;
            settings_.unretractSpeed = speed;
        };

        // PID
        cb.setHotendPID = [this](double kp, double ki, double kd) {
            settings_.hotendKp = kp; settings_.hotendKi = ki; settings_.hotendKd = kd;
            if (extruderHeater_) {
                extruderHeater_->setPIDParams({kp, ki, kd, 100.0, 0.0, 1.0});
            }
        };
        cb.setBedPID = [this](double kp, double ki, double kd) {
            settings_.bedKp = kp; settings_.bedKi = ki; settings_.bedKd = kd;
            if (heaterBed_) {
                heaterBed_->setPIDParams({kp, ki, kd, 100.0, 0.0, 1.0});
            }
        };
        cb.runPIDAutotune = [this](double temp, int cycles) {
            // PID autotune is delegated to the Tether autotuning framework
            // via the HeaterAutotuneBridge.  No inline autotuning code here.
            std::ostringstream ss;
            ss << "PID autotune starting at " << temp << "C for " << cycles
               << " cycles (method=" << autotuneMethodName(pidAutotuneMethod_) << ")\n";

            if (!extruderHeater_) {
                ss << "PID autotune failed: no heater connected";
                return ss.str();
            }

            // Lazily create the bridge for the extruder heater
            if (!extruderAutotuneBridge_) {
                extruderAutotuneBridge_ = std::make_unique<HeaterAutotuneBridge>(
                    *extruderHeater_,
                    settings_.hotendKp, settings_.hotendKi, settings_.hotendKd);
            }

            auto result = extruderAutotuneBridge_->autotune(
                temp, pidAutotuneMethod_, cycles);
            ss << result.message;
            return ss.str();
        };

        // Probe control
        cb.deployProbe = [this]() {
            // Deploy probe — hardware-specific
            if (probeObj_) probeObj_->setLastQuery(true);
        };
        cb.stowProbe = [this]() {
            // Stow probe — hardware-specific
            if (probeObj_) probeObj_->setLastQuery(false);
        };

        // Bed mesh management
        cb.setBedMeshEnabled = [this](bool enable) {
            settings_.bedMeshEnabled = enable;
        };
        cb.setBedMeshPoint = [this](int xIdx, int yIdx, double z) {
            bedMesh_->setPoint(xIdx, yIdx, z);
        };

        // Backlash
        cb.setBacklash = [this](const std::string& axis, double compensation) {
            settings_.backlash[axis] = compensation;
        };

        // Filament
        cb.setFilamentDiameter = [this](double diameter) {
            settings_.filamentDiameter = diameter;
        };
        cb.filamentChange = [this]() {
            // Pause print and wait for filament change
            sdcard_->pausePrint();
            printStatsObj_->setState("paused");
            pauseResumeObj_->setPaused(true);
        };

        // Misc — wire to peripheral backends
        cb.setPinState = [this](int pin, double value) {
            // Set output pin state — store in output pin map
            outputPins_[pin] = value;
        };
        cb.beep = [this](double freq, double duration) {
            // Beep — store last beep parameters
            lastBeepFreq_ = freq;
            lastBeepDuration_ = duration;
        };
        cb.setServoAngle = [this](int servo, double angle) {
            // Set servo angle — store in servo map
            servos_[servo] = angle;
        };
        cb.setLedColor = [this](int r, int g, int b, int w) {
            // Set LED color — store last color
            ledColor_ = {r, g, b, w};
        };

        // --- Delta printer support (M665/M666) ---
        cb.setDeltaGeometry = [this](double armLength, double deltaRadius,
                                      double angleA, double angleB, double angleC) {
            DeltaGeometry geo;
            if (armLength > 0) geo.armLength = armLength;
            if (deltaRadius > 0) geo.deltaRadius = deltaRadius;
            geo.towerAngleA = angleA;
            geo.towerAngleB = angleB;
            geo.towerAngleC = angleC;
            settings_.deltaGeometry = geo;
            deltaPrinter_->setGeometry(geo);
        };
        cb.setDeltaEndstopAdjust = [this](double adjX, double adjY, double adjZ) {
            DeltaEndstopAdjust adj;
            adj.adjX = adjX;
            adj.adjY = adjY;
            adj.adjZ = adjZ;
            settings_.deltaEndstopAdjust = adj;
            deltaPrinter_->setEndstopAdjust(adj);
        };

        // --- TMC driver configuration (M907-M914) ---
        cb.setTmcCurrent = [this](const std::string& axis, double currentMa) {
            tmcConfig_->setRunCurrent(axis, currentMa);
        };
        cb.setTmcRunCurrent = [this](const std::string& axis, double currentMa) {
            tmcConfig_->setRunCurrent(axis, currentMa);
        };
        cb.setTmcHoldCurrent = [this](const std::string& axis, double currentMa) {
            tmcConfig_->setHoldCurrent(axis, currentMa);
        };
        cb.setTmcStealthChop = [this](const std::string& axis, bool enable) {
            tmcConfig_->setStealthChop(axis, enable);
        };
        cb.setTmcSpreadThreshold = [this](const std::string& axis, double threshold) {
            tmcConfig_->setSpreadThreshold(axis, threshold);
        };
        cb.setTmcBumpSensitivity = [this](const std::string& axis, int sensitivity) {
            tmcConfig_->setBumpSensitivity(axis, sensitivity);
        };
        cb.setTmcDiagPin = [this](const std::string& axis, int diag) {
            tmcConfig_->setDiagPin(axis, diag);
        };

        // --- Filament load/unload (M701-M708) ---
        cb.loadFilament = [this](int extruder) {
            filamentLoader_->loadFilament(extruder);
        };
        cb.unloadFilament = [this](int extruder) {
            filamentLoader_->unloadFilament(extruder);
        };
        cb.loadFilamentToTool = [this](int tool) {
            filamentLoader_->loadToTool(tool);
        };
        cb.unloadFilamentFromTool = [this](int tool) {
            filamentLoader_->unloadFromTool(tool);
        };
        cb.purgeFilament = [this](int extruder) {
            filamentLoader_->purge(extruder);
        };
        cb.retractFilament = [this](int extruder) {
            filamentLoader_->retract(extruder);
        };
        cb.setFilamentSensorState = [this](int sensor, bool enabled) {
            filamentLoader_->setSensorState(sensor, enabled);
        };
        cb.reportFilamentSensorState = [this]() {
            return filamentLoader_->reportSensorState();
        };

        // --- Multi-MCU coordination (M860-M876) ---
        cb.setSecondaryMcuSerial = [this](int id, const std::string& path) {
            multiMcuManager_->setSerialPath(id, path);
        };
        cb.setSecondaryMcuBaud = [this](int id, int baud) {
            multiMcuManager_->setBaudRate(id, baud);
        };
        cb.setSecondaryMcuEnabled = [this](int id, bool enable) {
            multiMcuManager_->setEnabled(id, enable);
        };
        cb.setSecondaryMcuFreq = [this](int id, uint32_t freq) {
            multiMcuManager_->setClockFreq(id, freq);
        };
        cb.getSecondaryMcuStatus = [this](int id) {
            return multiMcuManager_->getStatus(id);
        };

        // --- Additional missing G-codes ---
        cb.resetG92Offsets = [this](int mode) {
            // G92.1: reset G92 offsets and homing offsets
            // G92.2: reset G92 offsets only (save first)
            // G92.3: restore G92 offsets from saved
            if (mode == 1) {
                savedG92Offsets_ = g92Offsets_;
                g92Offsets_.fill(0);
                for (auto& [k, v] : settings_.homeOffset) v = 0.0;
            } else if (mode == 2) {
                savedG92Offsets_ = g92Offsets_;
                g92Offsets_.fill(0);
            } else if (mode == 3) {
                g92Offsets_ = savedG92Offsets_;
            }
        };
        cb.waitForPinState = [this](int pin, int state) {
            // Check output pin map for state
            auto it = outputPins_.find(pin);
            if (it != outputPins_.end()) {
                (void)state; // Can't really poll output pins
            }
        };
        cb.waitForPin = [this](int pin, int state, double timeout) -> bool {
            // Poll output pin map for desired state
            auto deadline = std::chrono::steady_clock::now() +
                std::chrono::milliseconds(static_cast<int>(timeout * 1000));
            while (std::chrono::steady_clock::now() < deadline) {
                auto it = outputPins_.find(pin);
                if (it != outputPins_.end()) {
                    int curState = static_cast<int>(it->second);
                    if (curState == state) return true;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            return false;
        };
        cb.triggerCamera = [this]() {
            // Trigger camera — store timestamp
            lastCameraTrigger_ = std::chrono::steady_clock::now();
        };
        cb.setLcdContrast = [this](int contrast) {
            lcdContrast_ = contrast;
        };
        cb.sendI2cData = [this](uint8_t addr, const std::vector<uint8_t>& data) {
            if (i2cBus_) i2cBus_->write(addr, data);
        };
        cb.requestI2cData = [this](uint8_t addr, size_t len) -> std::vector<uint8_t> {
            if (i2cBus_) return i2cBus_->readNoRegister(addr, len);
            return std::vector<uint8_t>(len, 0);
        };
        cb.setCaseLight = [this](bool on, double brightness) {
            caseLight_->setState(on, brightness);
            settings_.caseLightOn = on;
            settings_.caseLightBrightness = brightness;
        };
        cb.setHomeOffsetFromPosition = [this]() {
            // M428: set home offset so current position becomes 0,0,0
            settings_.homeOffset["x"] = motionState_.position[0];
            settings_.homeOffset["y"] = motionState_.position[1];
            settings_.homeOffset["z"] = motionState_.position[2];
        };
        cb.abortSdPrint = [this]() {
            sdcard_->cancelPrint();
            printStatsObj_->setState("cancelled");
            idleTimeoutObj_->setState("Idle");
        };
        cb.clearBedMesh = [this]() {
            bedMesh_->clear();
            settings_.bedMeshEnabled = false;
        };
        cb.setSkewCorrection = [this](double xy, double xz, double yz) {
            SkewParams params;
            params.xy = xy;
            params.xz = xz;
            params.yz = yz;
            settings_.skewParams = params;
            skewCorrection_->setParams(params);
        };
        cb.setProbeCalibration = [this](double zOffset) {
            settings_.probeOffset = zOffset;
            if (probeObj_) probeObj_->setZOffset(zOffset);
        };

        // --- Spindle / tool / coolant (CNC) ---
        cb.setSpindleSpeed = [this](double rpm) {
            spindleRpm_ = rpm;
        };
        cb.toolChange = [this](int tool) {
            motionState_.activeExtruder = "extruder" + (tool > 0 ? std::to_string(tool) : "");
            activeTool_ = tool;
            // Switch the active extruder heater so M104 targets the right tool.
            setActiveExtruder(tool);
        };
        cb.setCoolant = [this](bool flood, bool mist) {
            coolantFlood_ = flood;
            coolantMist_ = mist;
        };

        // --- Coordinate systems (G54-G59.3) ---
        cb.selectCoordinateSystem = [this](int system) {
            motionState_.activeCoordSystem = system;
            motionState_.rebuildCoordTransform();
        };
        cb.setCoordinateSystemOffset = [this](int system, double x, double y, double z) {
            if (system >= 0 && system < 9) {
                motionState_.coordSystemOffsets[system] = {x, y, z};
                motionState_.rebuildCoordTransform();
            }
        };

        // --- Local offset (G52) ---
        cb.setLocalOffset = [this](double x, double y, double z) {
            if (!std::isnan(x)) motionState_.g52Offset[0] = x;
            if (!std::isnan(y)) motionState_.g52Offset[1] = y;
            if (!std::isnan(z)) motionState_.g52Offset[2] = z;
            // If all three are NaN (no axis words), reset to zero.
            if (std::isnan(x) && std::isnan(y) && std::isnan(z))
                motionState_.g52Offset = {0, 0, 0};
            motionState_.rebuildCoordTransform();
        };

        // --- Coordinate rotation (G68/G69) ---
        cb.setCoordinateRotation2D = [this](double angleDeg, double px, double py) {
            motionState_.g68Active = true;
            motionState_.g68Mode = 0;
            motionState_.coordRotation = angleDeg;
            motionState_.g68Pivot = {px, py, 0};
            motionState_.rebuildCoordTransform();
        };
        cb.setCoordinateRotation3D = [this](double a, double b, double c,
                                             double px, double py, double pz) {
            motionState_.g68Active = true;
            motionState_.g68Mode = 1;
            motionState_.g68Euler = {a, b, c};
            motionState_.g68Pivot = {px, py, pz};
            motionState_.rebuildCoordTransform();
        };
        cb.setCoordinateRotationAxis = [this](double ix, double iy, double iz,
                                               double angleDeg,
                                               double px, double py, double pz) {
            motionState_.g68Active = true;
            motionState_.g68Mode = 2;
            motionState_.g68Axis = {ix, iy, iz};
            motionState_.g68AxisAngle = angleDeg;
            motionState_.g68Pivot = {px, py, pz};
            motionState_.rebuildCoordTransform();
        };
        cb.cancelCoordinateRotation = [this]() {
            motionState_.g68Active = false;
            motionState_.g68Mode = 0;
            motionState_.coordRotation = 0.0;
            motionState_.g68Euler = {0, 0, 0};
            motionState_.g68Axis = {0, 0, 0};
            motionState_.g68AxisAngle = 0.0;
            motionState_.g68Pivot = {0, 0, 0};
            motionState_.rebuildCoordTransform();
        };

        // --- Scaling (G51/G50) ---
        cb.setScaling = [this](double sx, double sy, double sz) {
            motionState_.g51Active = true;
            motionState_.scaleFactors = {sx, sy, sz};
            motionState_.rebuildCoordTransform();
        };
        cb.cancelScaling = [this]() {
            motionState_.g51Active = false;
            motionState_.scaleFactors = {1, 1, 1};
            motionState_.rebuildCoordTransform();
        };

        // --- Machine coordinates (G53) ---
        cb.moveMachine = [this](double x, double y, double z, double speed) {
            // Move in machine coordinates (bypassing coordinate transforms).
            // We set the position directly in machine space and call the
            // raw move callback without applying coordTransform.
            if (!std::isnan(x)) motionState_.position[0] = x;
            if (!std::isnan(y)) motionState_.position[1] = y;
            if (!std::isnan(z)) motionState_.position[2] = z;
            if (moveCallbackRaw_) {
                moveCallbackRaw_(motionState_.position[0],
                                 motionState_.position[1],
                                 motionState_.position[2],
                                 motionState_.position[3], speed);
            }
        };

        // --- Path control (G61/G61.1/G64) ---
        cb.setPathControl = [this](int mode, double tolerance) {
            motionState_.pathControlMode = mode;
            motionState_.pathBlendingTolerance = tolerance;
        };

        // --- Program flow (M0/M1/M2/M30) ---
        cb.programStop = [this](const std::string& message) {
            if (pauseResumeObj_) pauseResumeObj_->setPaused(true);
            printStatsObj_->setState("paused");
            if (!message.empty() && displayStatusObj_) {
                displayStatusObj_->setMessage(message);
            }
        };
        cb.programEnd = [this](const std::string& message) {
            sdcard_->cancelPrint();
            printStatsObj_->setState("complete");
            idleTimeoutObj_->setState("Idle");
            if (!message.empty() && displayStatusObj_) {
                displayStatusObj_->setMessage(message);
            }
        };

        // --- Software endstops (M208/M211) ---
        cb.setSoftwareEndstops = [this](const std::string& axis, double min, double max, bool enable) {
            motionState_.softwareEndstopLimits[axis] = {min, max};
            motionState_.softwareEndstopsEnabled = enable;
        };
        cb.setSoftwareEndstopEnable = [this](bool enable) {
            motionState_.softwareEndstopsEnabled = enable;
        };

        // --- Thermistor parameters (M305) ---
        cb.setThermistorParams = [this](int sensor, double rPullup, double beta,
                                         double rNominal, double tNominal) {
            thermistorParams_[sensor] = {rPullup, beta, rNominal, tNominal};
        };

        // --- Filament width sensor (M405/M406/M407) ---
        cb.setFilamentWidthSensor = [this](bool enable) {
            motionState_.filamentWidthSensorEnabled = enable;
        };
        cb.getFilamentWidth = [this]() -> std::string {
            std::ostringstream ss;
            ss << "Filament width: " << motionState_.filamentWidthMeasured << "mm";
            return ss.str();
        };

        // --- Canned cycles (G81-G89) ---
        cb.executeCannedCycle = [this](int type, double x, double y, double z,
                                        double retractHeight, double feedRate) {
            // Execute a canned cycle: move to XY, plunge to Z, retract to R
            if (moveCallback_) {
                double speed = feedRate / 60.0;
                // Rapid to XY at retract height
                moveCallback_(x, y, retractHeight, motionState_.position[3], speed);
                // Plunge to Z
                moveCallback_(x, y, z, motionState_.position[3], speed);
                // Retract to R
                moveCallback_(x, y, retractHeight, motionState_.position[3], speed);
            }
            motionState_.position[0] = x;
            motionState_.position[1] = y;
            motionState_.position[2] = retractHeight;
        };
        cb.cancelCannedCycle = [this]() {
            motionState_.cannedCycleActive = false;
        };

        // --- Bed probing (G29) ---
        cb.probeBed = [this]() -> int {
            // Even without a probe, mark bed mesh as enabled
            settings_.bedMeshEnabled = true;
            if (!probe_) {
                // No probe available — just mark mesh as needing probing
                return 0;
            }
            // Probe a grid of points and fill the bed mesh
            int probed = 0;
            // Default grid: 3x3 over 200x200mm area
            int gridX = 3, gridY = 3;
            double minX = 10, maxX = 190;
            double minY = 10, maxY = 190;
            double stepX = (maxX - minX) / (gridX - 1);
            double stepY = (maxY - minY) / (gridY - 1);

            bedMesh_->configure(minX, maxX, minY, maxY, gridX, gridY);
            for (int j = 0; j < gridY; ++j) {
                for (int i = 0; i < gridX; ++i) {
                    double x = minX + i * stepX;
                    double y = minY + j * stepY;
                    // Move to XY position
                    if (moveCallback_) {
                        moveCallback_(x, y, 5.0, motionState_.position[3], 10.0);
                    }
                    motionState_.position[0] = x;
                    motionState_.position[1] = y;
                    // Probe down
                    double z = 0.0;
                    if (probe_) {
                        // Use the probe to find Z height
                        // In a real implementation, we'd wait for trigger
                        // Here we simulate finding the bed at z=0
                        z = 0.0 - (probe_ ? probe_->zOffset() : 0.0);
                    }
                    // Set mesh point
                    bedMesh_->setPoint(i, j, z);
                    ++probed;
                    // Retract
                    if (moveCallback_) {
                        moveCallback_(x, y, 5.0, motionState_.position[3], 10.0);
                    }
                }
            }
            settings_.bedMeshEnabled = true;
            return probed;
        };

        // Auto bed level (G32) — similar to probeBed but also applies correction
        cb.autoBedLevel = [this]() -> int {
            // Reuse the bed probing logic
            settings_.bedMeshEnabled = true;
            if (!probe_) return 0;
            int probed = 0;
            int gridX = 3, gridY = 3;
            double minX = 10, maxX = 190, minY = 10, maxY = 190;
            double stepX = (maxX - minX) / (gridX - 1);
            double stepY = (maxY - minY) / (gridY - 1);
            bedMesh_->configure(minX, maxX, minY, maxY, gridX, gridY);
            for (int j = 0; j < gridY; ++j) {
                for (int i = 0; i < gridX; ++i) {
                    double x = minX + i * stepX;
                    double y = minY + j * stepY;
                    if (moveCallback_) {
                        moveCallback_(x, y, 5.0, motionState_.position[3], 10.0);
                    }
                    motionState_.position[0] = x;
                    motionState_.position[1] = y;
                    double z = 0.0 - (probe_ ? probe_->zOffset() : 0.0);
                    bedMesh_->setPoint(i, j, z);
                    ++probed;
                    if (moveCallback_) {
                        moveCallback_(x, y, 5.0, motionState_.position[3], 10.0);
                    }
                }
            }
            return probed;
        };

        // Delta calibration (G33)
        cb.deltaCalibrate = [this]() -> int {
            if (settings_.kinematics != Kinematics::Delta) {
                if (gcode_.callbacks().output) {
                    gcode_.callbacks().output("G33 requires delta kinematics");
                }
                return -1;
            }
            // Delta calibration: probe tower endstops and adjust
            // In a real implementation, this would probe each tower and
            // adjust endstop positions. Here we mark calibration as done.
            int iterations = 1;
            if (deltaCalibrateObj_) {
                // Mark as calibrated
            }
            return iterations;
        };

        // Z tilt leveling (G34)
        cb.zTiltLevel = [this]() -> bool {
            if (!settings_.zTiltEnabled) {
                if (gcode_.callbacks().output) {
                    gcode_.callbacks().output("G34 requires [z_tilt] config");
                }
                return false;
            }
            // Z tilt: probe each Z stepper and adjust
            // In a real implementation, this would probe each Z point and
            // adjust stepper positions. Here we mark as leveled.
            if (zTiltObj_) {
                // Mark as leveled
            }
            return true;
        };

        // --- Print control (Moonraker-style) ---
        cb.startPrint = [this]() {
            sdcard_->startPrint();
            printStatsObj_->setState("printing");
            idleTimeoutObj_->setState("Printing");
        };
        cb.cancelPrint = [this]() {
            sdcard_->cancelPrint();
            printStatsObj_->setState("cancelled");
            idleTimeoutObj_->setState("Idle");
            if (pauseResumeObj_) pauseResumeObj_->setPaused(false);
        };
        cb.pausePrint = [this]() {
            sdcard_->pausePrint();
            printStatsObj_->setState("paused");
            if (pauseResumeObj_) pauseResumeObj_->setPaused(true);
        };
        cb.resumePrint = [this]() {
            sdcard_->resumePrint();
            printStatsObj_->setState("printing");
            if (pauseResumeObj_) pauseResumeObj_->setPaused(false);
        };

        // Extended command handler (SET_SERVO, BED_MESH_CALIBRATE, etc.)
        cb.extendedCommand = [this](const GcodeLine& g) -> bool {
            return handleExtendedCommand(g);
        };

        // Create the executor with our state
        gcode_ = GCodeExecutor(std::move(cb), &motionState_);
        gcode_.setMacroRegistry(macros_.get());

        // Wire the UDS server to use our G-code executor.
        // All callbacks acquire instanceMutex_ to prevent races with the
        // tick() thread (see threading model in KlippyInstance.hpp).
        server_.setGcodeScriptHandler([this](const std::string& script) {
            std::lock_guard<std::recursive_mutex> lock(instanceMutex_);
            gcode_.execute(script);
        });
        server_.setEmergencyStopHandler([this]() {
            std::lock_guard<std::recursive_mutex> lock(instanceMutex_);
            server_.setState(PrinterState::Shutdown, "Emergency stop");
        });
        // Wire VirtualSdcard and file root for file operations
        server_.setVirtualSdcard(sdcard_);
        server_.setFileRoot(config_.sdcardDir);
        // Wire print control endpoints to G-code callbacks
        server_.setPrintStartHandler([this]() {
            std::lock_guard<std::recursive_mutex> lock(instanceMutex_);
            sdcard_->startPrint();
            printStatsObj_->setState("printing");
            idleTimeoutObj_->setState("Printing");
        });
        server_.setPrintCancelHandler([this]() {
            std::lock_guard<std::recursive_mutex> lock(instanceMutex_);
            sdcard_->cancelPrint();
            printStatsObj_->setState("cancelled");
            idleTimeoutObj_->setState("Idle");
            if (pauseResumeObj_) pauseResumeObj_->setPaused(false);
        });
        server_.setPrintPauseHandler([this]() {
            std::lock_guard<std::recursive_mutex> lock(instanceMutex_);
            sdcard_->pausePrint();
            printStatsObj_->setState("paused");
            if (pauseResumeObj_) pauseResumeObj_->setPaused(true);
        });
        server_.setPrintResumeHandler([this]() {
            std::lock_guard<std::recursive_mutex> lock(instanceMutex_);
            sdcard_->resumePrint();
            printStatsObj_->setState("printing");
            if (pauseResumeObj_) pauseResumeObj_->setPaused(false);
        });
    }
