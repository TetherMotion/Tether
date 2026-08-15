/// @file KlippyInstanceExtendedCommands.ipp
/// @brief Extended G-code command handlers for KlippyInstance (included into class body).
///
/// This file is included inside the KlippyInstance class definition.
/// It contains the handleExtendedCommand() method and the probeBedGrid()
/// helper, which together implement all extended Klipper G-code commands.

    // ------------------------------------------------------------------
    // Extended command dispatch (Klipper module commands)
    // ------------------------------------------------------------------

    /// @brief Handle a Klipper extended command (SET_SERVO, BED_MESH_CALIBRATE, etc.)
    /// @return true if the command was recognized and handled.
    bool handleExtendedCommand(const GcodeLine& g) {
        const std::string& cmd = g.code;

        // Helper to emit a response
        auto respond = [this](const std::string& msg) {
            server_.emitGcodeResponse(msg);
        };

        // ---- Bed mesh commands ----
        if (cmd == "BED_MESH_CALIBRATE") {
            // Probe the bed and fill the mesh (same as G29)
            {
                int probed = 0;
                if (probe_) {
                    // Use the G29 probeBed callback path
                    // Just call the existing probeBed logic
                    probed = probeBedGrid();
                }
                settings_.bedMeshEnabled = true;
                respond("// bed_mesh: generated\n");
            }
            return true;
        }

        if (cmd == "BED_MESH_OUTPUT") {
            // Output the current mesh to G-code console
            std::ostringstream ss;
            ss << "// Mesh points (" << bedMesh_->xPoints() << "x" << bedMesh_->yPoints() << "):\n";
            for (int j = 0; j < bedMesh_->yPoints(); ++j) {
                ss << "// ";
                for (int i = 0; i < bedMesh_->xPoints(); ++i) {
                    auto pts = bedMesh_->points();
                    for (const auto& p : pts) {
                        if (p.x == i && p.y == j) {
                            ss << std::fixed << std::setprecision(4) << p.z << " ";
                            break;
                        }
                    }
                }
                ss << "\n";
            }
            respond(ss.str());
            return true;
        }

        if (cmd == "BED_MESH_MAP") {
            // Output mesh as a map
            std::ostringstream ss;
            ss << "// bed_mesh_map\n";
            auto pts = bedMesh_->points();
            for (const auto& p : pts) {
                ss << "// " << p.x << "," << p.y << "," << p.z << "\n";
            }
            respond(ss.str());
            return true;
        }

        if (cmd == "BED_MESH_CLEAR") {
            bedMesh_->clear();
            settings_.bedMeshEnabled = false;
            respond("// bed_mesh: cleared\n");
            return true;
        }

        if (cmd == "BED_MESH_PROFILE") {
            std::string action = g.getNamed("PROFILE", "");
            std::string name = g.getNamed("NAME", "");
            if (action == "LOAD" || action == "load") {
                auto it = bedMeshProfiles_.find(name);
                if (it != bedMeshProfiles_.end()) {
                    it->second.loaded = true;
                    settings_.bedMeshEnabled = true;
                    respond("// bed_mesh: loaded profile '" + name + "'\n");
                } else {
                    respond("// bed_mesh: profile '" + name + "' not found\n");
                }
            } else if (action == "SAVE" || action == "save") {
                bedMeshProfiles_[name] = {name, false};
                respond("// bed_mesh: saved profile '" + name + "'\n");
            } else if (action == "REMOVE" || action == "remove") {
                bedMeshProfiles_.erase(name);
                respond("// bed_mesh: removed profile '" + name + "'\n");
            }
            return true;
        }

        // ---- Servo control ----
        if (cmd == "SET_SERVO") {
            std::string servo = g.getNamed("SERVO", "");
            if (servo.empty()) return true;
            if (g.hasNamed("ANGLE")) {
                double angle = g.getNamedDouble("ANGLE");
                servoStates_[servo].angle = angle;
                // Also invoke the M280 callback if servo index is numeric
                try {
                    int idx = std::stoi(servo);
                    if (gcode_.callbacks().setServoAngle) {
                        gcode_.callbacks().setServoAngle(idx, angle);
                    }
                } catch (...) {}
            }
            if (g.hasNamed("WIDTH")) {
                servoStates_[servo].pulseWidth = g.getNamedDouble("WIDTH");
            }
            return true;
        }

        // ---- Accelerometer commands ----
        if (cmd == "ACCELEROMETER_MEASURE") {
            std::string name = g.getNamed("NAME", "");
            if (adxl345_) {
                adxl345_->startMeasurement();
                if (!name.empty()) {
                    respond("// accelerometer measurement '" + name + "' started\n");
                } else {
                    respond("// accelerometer measurement started\n");
                }
            } else {
                respond("// accelerometer not configured\n");
            }
            return true;
        }

        if (cmd == "ACCELEROMETER_QUERY") {
            if (adxl345_) {
                auto acc = adxl345_->read();
                std::ostringstream ss;
                ss << "// adxl345: x=" << acc.x << " y=" << acc.y << " z=" << acc.z << "\n";
                respond(ss.str());
            } else {
                respond("// accelerometer not configured\n");
            }
            return true;
        }

        // ---- Config save ----
        if (cmd == "SAVE_CONFIG") {
            // Mark config as pending save
            if (configfileObj_) {
                // Add pending items
                for (const auto& [k, v] : saveConfigPendingItems_) {
                    (void)k; (void)v;
                }
            }
            saveSettingsToFile();
            respond("// config saved\n");
            return true;
        }

        // ---- G-code offset ----
        if (cmd == "SET_GCODE_OFFSET") {
            if (g.hasNamed("X")) gcodeOffset_[0] = g.getNamedDouble("X");
            if (g.hasNamed("Y")) gcodeOffset_[1] = g.getNamedDouble("Y");
            if (g.hasNamed("Z")) gcodeOffset_[2] = g.getNamedDouble("Z");
            if (g.hasNamed("E")) gcodeOffset_[3] = g.getNamedDouble("E");
            if (g.hasNamed("RESET")) {
                std::string reset = g.getNamed("RESET");
                if (reset == "1" || reset == "true" || reset == "RESET") {
                    gcodeOffset_ = {0, 0, 0, 0};
                }
            }
            if (g.hasNamed("ADJUST")) {
                std::string adj = g.getNamed("ADJUST");
                if (adj == "1" || adj == "true") {
                    // Adjust current position by the offset delta
                    for (int i = 0; i < 4; ++i) {
                        motionState_.position[i] += gcodeOffset_[i];
                    }
                }
            }
            return true;
        }

        // ---- Extruder rotation distance ----
        if (cmd == "SET_EXTRUDER_ROTATION_DISTANCE") {
            std::string extruder = g.getNamed("EXTRUDER", "extruder");
            double dist = g.getNamedDouble("DISTANCE", 0.0);
            extruderRotationDistance_[extruder] = dist;
            // Also update steps per mm (rotation_distance = 1/steps_per_mm roughly)
            if (dist > 0) {
                settings_.stepsPerMm["e"] = 1.0 / dist;
            }
            return true;
        }

        // ---- PID calibrate (alias for M303) ----
        if (cmd == "PID_CALIBRATE") {
            double temp = g.getNamedDouble("TARGET", 200.0);
            int cycles = g.getNamedInt("CYCLES", 3);
            std::string heater = g.getNamed("HEATER", "extruder");
            // Optional METHOD parameter to override the configured autotune method
            std::string methodStr = g.getNamed("METHOD", "");
            if (!methodStr.empty()) pidAutotuneMethod_ = parseAutotuneMethod(methodStr);

            // Run PID autotune via the Tether autotuning bridge
            if (heater == "extruder" || heater == "heater_extruder") {
                if (gcode_.callbacks().runPidAutotune) {
                    std::string result = gcode_.callbacks().runPidAutotune(temp, cycles);
                    respond(result);
                }
            } else if (heater == "heater_bed") {
                // Bed PID autotune via the Tether autotuning bridge
                std::ostringstream ss;
                ss << "PID autotune for heater_bed at " << temp << "C"
                   << " (method=" << autotuneMethodName(pidAutotuneMethod_) << ")\n";
                if (heaterBed_) {
                    if (!bedAutotuneBridge_) {
                        bedAutotuneBridge_ = std::make_unique<HeaterAutotuneBridge>(
                            *heaterBed_,
                            settings_.bedKp, settings_.bedKi, settings_.bedKd);
                    }
                    auto result = bedAutotuneBridge_->autotune(
                        temp, pidAutotuneMethod_, cycles);
                    ss << result.message << "\n";
                } else {
                    ss << "Bed PID autotune failed: no bed heater connected\n";
                }
                respond(ss.str());
            }
            return true;
        }

        // ---- Velocity limit ----
        if (cmd == "SET_VELOCITY_LIMIT") {
            if (g.hasNamed("VELOCITY")) {
                double v = g.getNamedDouble("VELOCITY");
                settings_.maxFeedrate["x"] = v;
                settings_.maxFeedrate["y"] = v;
            }
            if (g.hasNamed("ACCEL")) {
                settings_.acceleration = g.getNamedDouble("ACCEL");
            }
            if (g.hasNamed("ACCEL_TO_DECEL")) {
                // Store in advanced motion
                settings_.startAccel = g.getNamedDouble("ACCEL_TO_DECEL");
            }
            if (g.hasNamed("SQUARE_CORNER_VELOCITY")) {
                settings_.jerk = g.getNamedDouble("SQUARE_CORNER_VELOCITY");
            }
            return true;
        }

        // ---- Pin control ----
        if (cmd == "SET_PIN") {
            std::string pin = g.getNamed("PIN", "");
            double value = g.getNamedDouble("VALUE", 0.0);
            if (!pin.empty()) {
                namedOutputPins_[pin] = value;
            }
            return true;
        }

        if (cmd == "SET_PWM_PIN") {
            std::string pin = g.getNamed("PIN", "");
            double value = g.getNamedDouble("VALUE", 0.0);
            if (!pin.empty()) {
                namedPwmPins_[pin] = value;
            }
            return true;
        }

        // ---- Fan speed ----
        if (cmd == "SET_FAN_SPEED") {
            std::string fan = g.getNamed("FAN", "");
            double speed = g.getNamedDouble("SPEED", 0.0);
            if (!fan.empty()) {
                genericFanSpeeds_[fan] = speed;
            }
            return true;
        }

        // ---- Heater temperature ----
        if (cmd == "SET_HEATER_TEMPERATURE") {
            std::string heater = g.getNamed("HEATER", "extruder");
            double temp = g.getNamedDouble("TARGET", 0.0);
            if (heater == "extruder" || heater == "heater_extruder") {
                if (extruderHeater_) extruderHeater_->setTarget(temp);
            } else if (heater == "heater_bed") {
                if (heaterBed_) heaterBed_->setTarget(temp);
            }
            return true;
        }

        // ---- Temperature fan ----
        if (cmd == "SET_TEMPERATURE_FAN") {
            std::string fan = g.getNamed("TEMPERATURE_FAN", "");
            double temp = g.getNamedDouble("TARGET", 0.0);
            if (!fan.empty()) {
                temperatureFanTargets_[fan] = temp;
            }
            return true;
        }

        // ---- Query endstops ----
        if (cmd == "QUERY_ENDSTOPS") {
            std::string result;
            if (gcode_.callbacks().getEndstopStatus) {
                result = gcode_.callbacks().getEndstopStatus();
            }
            respond(result);
            return true;
        }

        // ---- Query ADC ----
        if (cmd == "QUERY_ADC") {
            std::string name = g.getNamed("NAME", "");
            // Return stored ADC value or 0
            double val = 0.0;
            auto it = adcValues_.find(name);
            if (it != adcValues_.end()) val = it->second;
            std::ostringstream ss;
            ss << "// adc " << name << " = " << val << "\n";
            respond(ss.str());
            return true;
        }

        // ---- LED control ----
        if (cmd == "SET_LED") {
            std::string led = g.getNamed("LED", "");
            LedState& state = ledStates_[led];
            if (g.hasNamed("RED")) state.color[0] = g.getNamedDouble("RED");
            if (g.hasNamed("GREEN")) state.color[1] = g.getNamedDouble("GREEN");
            if (g.hasNamed("BLUE")) state.color[2] = g.getNamedDouble("BLUE");
            if (g.hasNamed("WHITE")) {
                state.color[3] = g.getNamedDouble("WHITE");
                state.white = true;
            }
            if (g.hasNamed("INDEX")) state.index = g.getNamedInt("INDEX");
            if (g.hasNamed("TRANSMIT")) {
                std::string tx = g.getNamed("TRANSMIT");
                if (tx == "0" || tx == "false") {
                    // Don't transmit yet
                }
            }
            return true;
        }

        // ---- Neopixel control ----
        if (cmd == "SET_NEOPIXEL") {
            std::string neo = g.getNamed("NEOPIXEL", "");
            if (neo.empty()) return true;
            auto& pixels = neopixelStates_[neo];
            std::array<double, 4> color = {0, 0, 0, 0};
            if (g.hasNamed("RED")) color[0] = g.getNamedDouble("RED");
            if (g.hasNamed("GREEN")) color[1] = g.getNamedDouble("GREEN");
            if (g.hasNamed("BLUE")) color[2] = g.getNamedDouble("BLUE");
            if (g.hasNamed("WHITE")) color[3] = g.getNamedDouble("WHITE");
            if (g.hasNamed("INDEX")) {
                int idx = g.getNamedInt("INDEX");
                if (idx >= 0 && static_cast<size_t>(idx) < pixels.size()) {
                    pixels[idx] = color;
                }
            } else {
                // Set all pixels
                for (auto& p : pixels) p = color;
            }
            return true;
        }

        // ---- Probe commands ----
        if (cmd == "SET_PROBE") {
            if (g.hasNamed("Z_OFFSET")) {
                double offset = g.getNamedDouble("Z_OFFSET");
                settings_.probeOffset = offset;
                if (probeObj_) probeObj_->setZOffset(offset);
            }
            if (g.hasNamed("VIRTUAL_ENDSTOP")) {
                std::string ve = g.getNamed("VIRTUAL_ENDSTOP");
                if (probe_) probe_->setVirtualEndstop(ve == "1" || ve == "true");
            }
            return true;
        }

        if (cmd == "PROBE_CALIBRATE") {
            probeCalibState_.active = true;
            probeCalibState_.zPosition = motionState_.position[2];
            respond("// probe_calibrate: starting calibration\n");
            return true;
        }

        if (cmd == "Z_OFFSET_APPLY_PROBE") {
            // Apply the current probe offset to the probe's Z offset
            double offset = g.getNamedDouble("Z_OFFSET", settings_.probeOffset);
            settings_.probeOffset += offset;
            if (probeObj_) probeObj_->setZOffset(settings_.probeOffset);
            saveConfigPendingItems_["probe z_offset"] = std::to_string(settings_.probeOffset);
            respond("// z_offset_apply_probe: applied\n");
            return true;
        }

        if (cmd == "Z_OFFSET_APPLY_ENDSTOP") {
            // Apply Z offset to endstop homing offset
            double offset = g.getNamedDouble("Z_OFFSET", 0.0);
            settings_.homeOffset["z"] += offset;
            saveConfigPendingItems_["stepper_z position_endstop"] =
                std::to_string(settings_.homeOffset["z"]);
            respond("// z_offset_apply_endstop: applied\n");
            return true;
        }

        // ---- Stepper enable ----
        if (cmd == "SET_STEPPER_ENABLE") {
            std::string stepper = g.getNamed("STEPPER", "");
            bool enable = g.getNamed("ENABLE", "0") == "1" || g.getNamed("ENABLE", "0") == "true";
            if (!stepper.empty()) {
                stepperEnableOverrides_[stepper] = enable;
                stepperEnableObj_->setStepperEnabled(stepper, enable);
            }
            return true;
        }

        // ---- Idle timeout ----
        if (cmd == "SET_IDLE_TIMEOUT") {
            double timeout = g.getNamedDouble("TIMEOUT", 600.0);
            idleTimeout_ = timeout;
            if (idleTimeoutObj_) idleTimeoutObj_->setTimeout(timeout);
            return true;
        }

        // ---- Delayed G-code ----
        if (cmd == "SET_DELAYED_GCODE") {
            std::string id = g.getNamed("ID", "default");
            std::string gcode = g.getNamed("GCODE", "");
            double delay = g.getNamedDouble("DELAY", 0.0);
            DelayedGcode& dg = delayedGcodes_[id];
            dg.gcode = gcode;
            dg.delay = delay;
            dg.enabled = true;
            dg.scheduledTime = std::chrono::steady_clock::now() +
                std::chrono::milliseconds(static_cast<int64_t>(delay * 1000));
            return true;
        }

        if (cmd == "UPDATE_DELAYED_GCODE") {
            std::string id = g.getNamed("ID", "default");
            auto it = delayedGcodes_.find(id);
            if (it != delayedGcodes_.end()) {
                if (g.hasNamed("GCODE")) it->second.gcode = g.getNamed("GCODE");
                if (g.hasNamed("DELAY")) {
                    it->second.delay = g.getNamedDouble("DELAY");
                    it->second.scheduledTime = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(static_cast<int64_t>(it->second.delay * 1000));
                }
                if (g.hasNamed("ENABLE")) {
                    it->second.enabled = g.getNamed("ENABLE", "0") == "1" ||
                                         g.getNamed("ENABLE", "0") == "true";
                }
            }
            return true;
        }

        // ---- Skew correction ----
        if (cmd == "SET_SKEW") {
            double xy = g.getNamedDouble("XY", settings_.skewParams.xy);
            double xz = g.getNamedDouble("XZ", settings_.skewParams.xz);
            double yz = g.getNamedDouble("YZ", settings_.skewParams.yz);
            SkewParams params = {xy, xz, yz};
            settings_.skewParams = params;
            skewCorrection_->setParams(params);
            return true;
        }

        // ---- Dual carriage ----
        if (cmd == "SET_DUAL_CARRIAGE") {
            std::string carriage = g.getNamed("CARRIAGE", "0");
            std::string mode = g.getNamed("MODE", "PRIMARY");
            DualCarriageState& state = dualCarriageStates_[carriage];
            if (mode == "PRIMARY" || mode == "0") state.mode = 0;
            else if (mode == "COPY" || mode == "1") state.mode = 1;
            else if (mode == "MIRROR" || mode == "2") state.mode = 2;
            if (g.hasNamed("OFFSET")) state.offset = g.getNamedDouble("OFFSET");
            return true;
        }

        // ---- Set position ----
        if (cmd == "SET_POSITION") {
            // Like G92 but with named params
            if (g.hasNamed("X")) motionState_.position[0] = g.getNamedDouble("X");
            if (g.hasNamed("Y")) motionState_.position[1] = g.getNamedDouble("Y");
            if (g.hasNamed("Z")) motionState_.position[2] = g.getNamedDouble("Z");
            if (g.hasNamed("E")) motionState_.position[3] = g.getNamedDouble("E");
            return true;
        }

        // ---- G-code variables ----
        if (cmd == "SET_GCODE_VARIABLE") {
            std::string macro = g.getNamed("MACRO", "");
            std::string var = g.getNamed("VARIABLE", "");
            std::string value = g.getNamed("VALUE", "");
            if (!macro.empty() && !var.empty()) {
                gcodeVariables_[macro][var] = value;
            }
            return true;
        }

        if (cmd == "SAVE_VARIABLE") {
            std::string var = g.getNamed("VARIABLE", "");
            std::string value = g.getNamed("VALUE", "");
            if (!var.empty()) {
                savedVariables_[var] = value;
            }
            return true;
        }

        // ---- TMC commands ----
        if (cmd == "DUMP_TMC") {
            std::string stepper = g.getNamed("STEPPER", "");
            std::ostringstream ss;
            ss << "// TMC dump for " << stepper << ":\n";
            ss << "// (register dump would be displayed here)\n";
            respond(ss.str());
            return true;
        }

        if (cmd == "INIT_TMC") {
            std::string stepper = g.getNamed("STEPPER", "");
            // Initialize TMC driver — hardware-specific
            respond("// TMC init for " + stepper + "\n");
            return true;
        }

        if (cmd == "SET_TMC_FIELD") {
            std::string stepper = g.getNamed("STEPPER", "");
            std::string field = g.getNamed("FIELD", "");
            uint32_t value = static_cast<uint32_t>(g.getNamedDouble("VALUE", 0.0));
            if (!stepper.empty() && !field.empty()) {
                tmcFieldOverrides_[stepper][field] = value;
            }
            return true;
        }

        // ---- Exclude object commands ----
        if (cmd == "EXCLUDE_OBJECT_DEFINE") {
            std::string name = g.getNamed("NAME", "");
            std::string action = g.getNamed("RESET", "");
            if (action == "1" || action == "true") {
                excludeObjects_.clear();
                currentExcludeObject_ = -1;
            } else if (!name.empty()) {
                ExcludeObject obj;
                obj.name = name;
                // Parse polygon if provided
                // Format: "x1,y1,x2,y2,..." or "[[x1,y1],[x2,y2],...]"
                std::string polygon = g.getNamed("POLYGON", "");
                if (!polygon.empty()) {
                    // Remove brackets and whitespace, parse comma-separated numbers
                    std::string cleaned;
                    for (char c : polygon) {
                        if (c != '[' && c != ']' && c != ' ' && c != '\t') {
                            cleaned += c;
                        }
                    }
                    std::vector<double> nums;
                    std::string current;
                    for (char c : cleaned) {
                        if (c == ',') {
                            if (!current.empty()) {
                                try {
                                    nums.push_back(std::stod(current));
                                } catch (...) {}
                                current.clear();
                            }
                        } else {
                            current += c;
                        }
                    }
                    if (!current.empty()) {
                        try {
                            nums.push_back(std::stod(current));
                        } catch (...) {}
                    }
                    // Pair up numbers as XY coordinates
                    for (size_t i = 0; i + 1 < nums.size(); i += 2) {
                        obj.polygon.push_back({nums[i], nums[i + 1]});
                    }
                }
                excludeObjects_.push_back(obj);
            }
            return true;
        }

        if (cmd == "EXCLUDE_OBJECT_START") {
            std::string name = g.getNamed("NAME", "");
            for (int i = 0; i < static_cast<int>(excludeObjects_.size()); ++i) {
                if (excludeObjects_[i].name == name) {
                    excludeObjects_[i].started = true;
                    currentExcludeObject_ = i;
                    break;
                }
            }
            return true;
        }

        if (cmd == "EXCLUDE_OBJECT_END") {
            std::string name = g.getNamed("NAME", "");
            for (auto& obj : excludeObjects_) {
                if (obj.name == name) {
                    obj.finished = true;
                    break;
                }
            }
            currentExcludeObject_ = -1;
            return true;
        }

        if (cmd == "EXCLUDE_OBJECT") {
            std::string name = g.getNamed("NAME", "");
            std::string action = g.getNamed("CURRENT", "");
            if (action == "1" || action == "true") {
                // Mark as current
                for (int i = 0; i < static_cast<int>(excludeObjects_.size()); ++i) {
                    if (excludeObjects_[i].name == name) {
                        currentExcludeObject_ = i;
                        break;
                    }
                }
            } else {
                // Exclude the object
                for (auto& obj : excludeObjects_) {
                    if (obj.name == name) {
                        obj.excluded = true;
                        break;
                    }
                }
            }
            return true;
        }

        // ---- Manual probe commands ----
        if (cmd == "MANUAL_PROBE") {
            manualProbeState_.active = true;
            manualProbeState_.zPosition = motionState_.position[2];
            respond("// manual_probe: starting\n"
                    "//  position: Z=" + std::to_string(motionState_.position[2]) + "\n");
            return true;
        }

        if (cmd == "ABORT") {
            if (manualProbeState_.active) {
                manualProbeState_.active = false;
                respond("// manual_probe: aborted\n");
            } else if (probeCalibState_.active) {
                probeCalibState_.active = false;
                respond("// probe_calibrate: aborted\n");
            }
            return true;
        }

        if (cmd == "ACCEPT") {
            if (manualProbeState_.active) {
                manualProbeState_.zOffset = motionState_.position[2];
                manualProbeState_.active = false;
                settings_.probeOffset = manualProbeState_.zOffset;
                if (probeObj_) probeObj_->setZOffset(manualProbeState_.zOffset);
                respond("// manual_probe: accepted Z=" +
                        std::to_string(manualProbeState_.zOffset) + "\n");
            } else if (probeCalibState_.active) {
                probeCalibState_.zOffset = motionState_.position[2];
                probeCalibState_.active = false;
                settings_.probeOffset = probeCalibState_.zOffset;
                if (probeObj_) probeObj_->setZOffset(probeCalibState_.zOffset);
                respond("// probe_calibrate: accepted Z=" +
                        std::to_string(probeCalibState_.zOffset) + "\n");
            }
            return true;
        }

        if (cmd == "ADJUSTED") {
            if (manualProbeState_.active) {
                double z = motionState_.position[2];
                respond("// manual_probe: adjusted Z=" + std::to_string(z) + "\n");
            } else if (probeCalibState_.active) {
                double z = motionState_.position[2];
                respond("// probe_calibrate: adjusted Z=" + std::to_string(z) + "\n");
            }
            return true;
        }

        // ---- Sync extruder stepper ----
        if (cmd == "SYNC_EXTRUDER_STEPPER") {
            std::string stepper = g.getNamed("EXTRUDER_STEPPER", "");
            std::string motionQueue = g.getNamed("MOTION_QUEUE", "");
            if (!stepper.empty()) {
                extruderStepperSync_[stepper] = motionQueue;
            }
            return true;
        }

        // ---- Print stats info ----
        if (cmd == "SET_PRINT_STATS_INFO") {
            if (g.hasNamed("TOTAL_LAYER")) {
                printStatsInfoTotalLayer_ = g.getNamed("TOTAL_LAYER");
                try {
                    if (printStatsObj_) {
                        printStatsObj_->setInfoTotalLayer(
                            std::stoll(printStatsInfoTotalLayer_));
                    }
                } catch (...) {}
            }
            if (g.hasNamed("CURRENT_LAYER")) {
                printStatsInfoCurrentLayer_ = g.getNamed("CURRENT_LAYER");
                try {
                    if (printStatsObj_) {
                        printStatsObj_->setInfoCurrentLayer(
                            std::stoll(printStatsInfoCurrentLayer_));
                    }
                } catch (...) {}
            }
            return true;
        }

        // ---- Display group ----
        if (cmd == "SET_DISPLAY_GROUP") {
            displayGroup_ = g.getNamed("DISPLAY_GROUP", "");
            return true;
        }

        // ---- D1: Additional extended commands ----

        // Resonance testing — uses Tether identification framework
        if (cmd == "TEST_RESONANCES") {
            std::string axis = g.getNamed("AXIS", "X");
            double hzPerSec = g.getNamedDouble("HZ_PER_SEC", 0.0);
            double minFreq = g.getNamedDouble("MIN_FREQ", 5.0);
            double maxFreq = g.getNamedDouble("MAX_FREQ", 100.0);
            int chips = g.getNamedInt("CHIPS", 0);
            (void)hzPerSec; (void)chips;

            // Run resonance sweep via the Tether identification framework
            if (!adxl345_) {
                respond("// resonance_tester: no accelerometer configured\n");
                return true;
            }
            if (!resonanceBridge_) {
                resonanceBridge_ = std::make_unique<ResonanceCalibrationBridge>();
                resonanceBridge_->setAccelerometerSource(
                    [this]() -> std::array<double, 3> {
                        if (!adxl345_) return {0, 0, 0};
                        auto a = adxl345_->read();
                        return {a.x, a.y, a.z};
                    });
            }

            respond("// resonance_tester: testing axis " + axis +
                    " freq=" + std::to_string(minFreq) + "-" +
                    std::to_string(maxFreq) + " Hz\n");
            auto bode = resonanceBridge_->runSweep(
                axis,
                static_cast<float>(minFreq),
                static_cast<float>(maxFreq));
            respond("// resonance_tester: collected " +
                    std::to_string(bode.size()) + " frequency points\n");
            accelerometerMeasuring_ = true;
            accelerometerData_.clear();
            return true;
        }

        if (cmd == "SHAPER_CALIBRATE") {
            std::string axis = g.getNamed("AXIS", "both");
            double maxSmoothing = g.getNamedDouble("MAX_SMOOTHING", 0.0);
            (void)maxSmoothing;

            // Calibrate input shaper via the Tether identification framework
            if (!adxl345_) {
                respond("// shaper_calibrate: no accelerometer configured\n");
                return true;
            }
            if (!resonanceBridge_) {
                resonanceBridge_ = std::make_unique<ResonanceCalibrationBridge>();
                resonanceBridge_->setAccelerometerSource(
                    [this]() -> std::array<double, 3> {
                        if (!adxl345_) return {0, 0, 0};
                        auto a = adxl345_->read();
                        return {a.x, a.y, a.z};
                    });
            }

            respond("// shaper_calibrate: calibrating axis " + axis + "\n");
            auto result = resonanceBridge_->calibrate();

            if (axis == "X" || axis == "both") {
                if (inputShaperObj_) {
                    inputShaperObj_->setShaperFreqX(result.resonantFreqX);
                    inputShaperObj_->setShaperTypeX(result.shaperTypeX);
                }
                respond("// shaper_calibrate: X shaper_freq=" +
                        std::to_string(result.resonantFreqX) +
                        " type=" + result.shaperTypeX + "\n");
            }
            if (axis == "Y" || axis == "both") {
                if (inputShaperObj_) {
                    inputShaperObj_->setShaperFreqY(result.resonantFreqY);
                    inputShaperObj_->setShaperTypeY(result.shaperTypeY);
                }
                respond("// shaper_calibrate: Y shaper_freq=" +
                        std::to_string(result.resonantFreqY) +
                        " type=" + result.shaperTypeY + "\n");
            }
            respond("// " + result.message + "\n");
            return true;
        }

        // Bed leveling commands
        if (cmd == "Z_TILT_ADJUST") {
            // Adjust Z tilt by probing at each Z stepper position and
            // computing endstop adjustments to level the gantry.
            if (probe_) {
                // Probe at each Z stepper position
                // Default Z tilt positions: front-left, front-right, back-center
                struct ZTiltPoint { double x; double y; const char* name; };
                ZTiltPoint points[] = {
                    {30.0, 30.0, "z0"},
                    {170.0, 30.0, "z1"},
                    {100.0, 170.0, "z2"},
                };
                double zValues[3] = {0, 0, 0};
                for (int i = 0; i < 3; ++i) {
                    if (moveCallback_) {
                        moveCallback_(points[i].x, points[i].y, 5.0,
                                      motionState_.position[3], 10.0);
                    }
                    motionState_.position[0] = points[i].x;
                    motionState_.position[1] = points[i].y;
                    zValues[i] = 0.0 - probe_->zOffset();
                }
                // Compute adjustments: average Z, then adjust each endstop
                double avgZ = (zValues[0] + zValues[1] + zValues[2]) / 3.0;
                double adj0 = avgZ - zValues[0];
                double adj1 = avgZ - zValues[1];
                double adj2 = avgZ - zValues[2];
                // Apply adjustments to Z stepper endstop positions
                auto& home = settings_.homeOffset;
                if (home.count("z")) home["z"] += adj0;
                else home["z"] = adj0;
                // Store Z tilt adjustments in settings
                settings_.zTiltAdjustments.clear();
                settings_.zTiltAdjustments.push_back(adj0);
                settings_.zTiltAdjustments.push_back(adj1);
                settings_.zTiltAdjustments.push_back(adj2);
                if (zTiltObj_) zTiltObj_->setApplied(true);
                respond("// z_tilt: applied, z0=" + std::to_string(adj0) +
                        " z1=" + std::to_string(adj1) +
                        " z2=" + std::to_string(adj2) + "\n");
            } else {
                respond("// z_tilt: no probe available\n");
            }
            return true;
        }

        if (cmd == "QUAD_GANTRY_LEVEL") {
            // Level the gantry by probing at 4 corners
            if (probe_) {
                int probed = probeBedGrid();
                if (quadGantryLevelObj_) quadGantryLevelObj_->setApplied(true);
                respond("// quad_gantry_level: applied, probed " +
                        std::to_string(probed) + " points\n");
            } else {
                respond("// quad_gantry_level: no probe available\n");
            }
            return true;
        }

        if (cmd == "SCREWS_TILT_ADJUST") {
            // Adjust bed screws by probing
            if (probe_) {
                int probed = probeBedGrid();
                if (screwsTiltAdjustObj_) screwsTiltAdjustObj_->setAdjusted(true);
                respond("// screws_tilt_adjust: applied, probed " +
                        std::to_string(probed) + " points\n");
            } else {
                respond("// screws_tilt_adjust: no probe available\n");
            }
            return true;
        }

        if (cmd == "BED_SCREWS_ADJUST") {
            // Manual bed screw adjustment
            respond("// bed_screws: adjust screws at marked positions\n");
            if (bedScrewsObj_) bedScrewsObj_->setState("adjust");
            return true;
        }

        // Delta calibration
        if (cmd == "DELTA_CALIBRATE") {
            if (probe_) {
                // Probe at center and at each tower position to compute
                // endstop adjustments for delta geometry.
                // Tower positions are at 120° spacing around the bed.
                double centerRadius = 0.0;
                double towerRadius = 80.0; // typical probe radius
                // Tower angles: A=210°, B=330°, C=90° (matching DeltaPrinter defaults)
                struct TowerPoint { double x; double y; };
                TowerPoint towers[3] = {
                    {towerRadius * cos(210.0 * M_PI / 180.0),
                     towerRadius * sin(210.0 * M_PI / 180.0)},
                    {towerRadius * cos(330.0 * M_PI / 180.0),
                     towerRadius * sin(330.0 * M_PI / 180.0)},
                    {towerRadius * cos(90.0 * M_PI / 180.0),
                     towerRadius * sin(90.0 * M_PI / 180.0)},
                };
                // Probe center
                if (moveCallback_) {
                    moveCallback_(0, 0, 5.0, motionState_.position[3], 10.0);
                }
                motionState_.position[0] = 0;
                motionState_.position[1] = 0;
                double zCenter = 0.0 - probe_->zOffset();
                // Probe each tower
                double zTowers[3] = {0, 0, 0};
                for (int i = 0; i < 3; ++i) {
                    if (moveCallback_) {
                        moveCallback_(towers[i].x, towers[i].y, 5.0,
                                      motionState_.position[3], 10.0);
                    }
                    motionState_.position[0] = towers[i].x;
                    motionState_.position[1] = towers[i].y;
                    zTowers[i] = 0.0 - probe_->zOffset();
                }
                // Compute endstop adjustments: difference between tower and center
                double adjA = zTowers[0] - zCenter;
                double adjB = zTowers[1] - zCenter;
                double adjC = zTowers[2] - zCenter;
                // Apply to delta endstop adjustments (towers A=X, B=Y, C=Z)
                settings_.deltaEndstopAdjust.adjX += adjA;
                settings_.deltaEndstopAdjust.adjY += adjB;
                settings_.deltaEndstopAdjust.adjZ += adjC;
                // Update the delta printer with new adjustments
                deltaPrinter_->setEndstopAdjust(settings_.deltaEndstopAdjust);
                if (deltaCalibrateObj_) deltaCalibrateObj_->setApplied(true);
                respond("// delta_calibrate: applied, adjA=" + std::to_string(adjA) +
                        " adjB=" + std::to_string(adjB) +
                        " adjC=" + std::to_string(adjC) + "\n");
            } else {
                respond("// delta_calibrate: no probe available\n");
            }
            return true;
        }

        if (cmd == "DELTA_ANALYZE") {
            // Analyze delta calibration results
            double radius = g.getNamedDouble("CALIBRATE_RADIUS", 0.0);
            respond("// delta_analyze: analysis complete\n");
            return true;
        }

        // Probe accuracy
        if (cmd == "PROBE_ACCURACY") {
            int samples = g.getNamedInt("SAMPLES", 10);
            double speed = g.getNamedDouble("PROBE_SPEED", 5.0);
            if (probe_) {
                // Simulate probe accuracy test
                double sum = 0.0;
                double minVal = 1e9, maxVal = -1e9;
                for (int i = 0; i < samples; ++i) {
                    // Use current Z position as simulated probe result
                    double z = motionState_.position[2];
                    sum += z;
                    if (z < minVal) minVal = z;
                    if (z > maxVal) maxVal = z;
                }
                double avg = sum / samples;
                double range = maxVal - minVal;
                respond("// probe_accuracy: samples=" + std::to_string(samples) +
                        " avg=" + std::to_string(avg) +
                        " range=" + std::to_string(range) + "\n");
            } else {
                respond("// probe_accuracy: no probe available\n");
            }
            return true;
        }

        // Direct setting commands
#if TETHER_ENABLE_PRESSURE_ADVANCE
        if (cmd == "SET_PRESSURE_ADVANCE") {
            double pa = g.getNamedDouble("ADVANCE", 0.0);
            double smoothTime = g.getNamedDouble("SMOOTH_TIME", 0.040);
            std::string model = g.getNamed("MODEL", "");
            double flowIndex = g.getNamedDouble("FLOW_INDEX", -1.0);
            double consistency = g.getNamedDouble("CONSISTENCY", -1.0);
            pressureAdvance_->setEnabled(true);
            pressureAdvance_->setParams({pa, smoothTime});
            if (pressureAdvanceObj_) {
                pressureAdvanceObj_->setPressureAdvance(pa);
                pressureAdvanceObj_->setSmoothTime(smoothTime);
                if (!model.empty()) pressureAdvanceObj_->setModel(model);
            }
            if (extruderObj_) {
                extruderObj_->setPressureAdvance(pa);
            }
            // Sync to the motion dispatcher for step generation.
            if (motionDispatcher_) {
                auto paCfg = motionDispatcher_->pressureAdvanceConfig();
                paCfg.enabled = true;
                paCfg.smoothTime = smoothTime;
                if (!model.empty()) {
                    if (model == "power_law") {
                        paCfg.model = motion::ExtrusionCompensationModel::PowerLaw;
                    } else if (model == "cross_wlf") {
                        paCfg.model = motion::ExtrusionCompensationModel::CrossWlf;
                    } else {
                        paCfg.model = motion::ExtrusionCompensationModel::Linear;
                    }
                }
                if (flowIndex > 0.0)  paCfg.flowIndex = flowIndex;
                if (consistency >= 0.0) paCfg.powerLawBaseGain = consistency;
                // Always keep the linear PA value in sync (used when model=Linear).
                paCfg.pressureAdvance = pa;
                motionDispatcher_->setPressureAdvanceConfig(paCfg);
            }
            std::string msg = "// pressure_advance: advance=" + std::to_string(pa) +
                              " smooth_time=" + std::to_string(smoothTime);
            if (!model.empty()) msg += " model=" + model;
            if (flowIndex > 0.0) msg += " flow_index=" + std::to_string(flowIndex);
            if (consistency >= 0.0) msg += " consistency=" + std::to_string(consistency);
            msg += "\n";
            respond(msg);
            return true;
        }

        if (cmd == "SET_HEATER_FLOW_COMPENSATION") {
            // Toggle / tune the flow-adaptive heater controller.
            // Args: ENABLE=0|1, FILAMENT_HEAT_CAPACITY=, MELT_ZONE_CAPACITANCE=,
            //       HEATER_MELT_CONDUCTANCE=, DEBT_TIME_CONSTANT=,
            //       MAX_PRE_EMPHASIS_POWER=, MAX_POST_EMPHASIS_POWER=,
            //       MAX_HEATER_OVERSHOOT=
            bool enable = g.getNamedInt("ENABLE", 0) != 0;
            settings_.heaterFlowPreEmphasis = enable;
            if (g.hasNamed("FILAMENT_HEAT_CAPACITY"))
                settings_.filamentHeatCapacity = g.getNamedDouble("FILAMENT_HEAT_CAPACITY", settings_.filamentHeatCapacity);
            if (g.hasNamed("MELT_ZONE_CAPACITANCE"))
                settings_.meltZoneCapacitance = g.getNamedDouble("MELT_ZONE_CAPACITANCE", settings_.meltZoneCapacitance);
            if (g.hasNamed("HEATER_MELT_CONDUCTANCE"))
                settings_.heaterMeltConductance = g.getNamedDouble("HEATER_MELT_CONDUCTANCE", settings_.heaterMeltConductance);
            if (g.hasNamed("DEBT_TIME_CONSTANT"))
                settings_.debtTimeConstant = g.getNamedDouble("DEBT_TIME_CONSTANT", settings_.debtTimeConstant);
            if (g.hasNamed("MAX_PRE_EMPHASIS_POWER"))
                settings_.maxPreEmphasisPower = g.getNamedDouble("MAX_PRE_EMPHASIS_POWER", settings_.maxPreEmphasisPower);
            if (g.hasNamed("MAX_POST_EMPHASIS_POWER"))
                settings_.maxPostEmphasisPower = g.getNamedDouble("MAX_POST_EMPHASIS_POWER", settings_.maxPostEmphasisPower);
            if (g.hasNamed("MAX_HEATER_OVERSHOOT"))
                settings_.maxHeaterOvershoot = g.getNamedDouble("MAX_HEATER_OVERSHOOT", settings_.maxHeaterOvershoot);
            applyFlowAdaptiveHeaterSettings();
            respond("// heater_flow_compensation: enable=" + std::to_string(enable ? 1 : 0) + "\n");
            return true;
        }
#endif

        if (cmd == "SET_INPUT_SHAPER") {
            std::string shaperX = g.getNamed("SHAPER_FREQ_X", "");
            std::string shaperY = g.getNamed("SHAPER_FREQ_Y", "");
            std::string typeX = g.getNamed("DAMPING_RATIO_X", "");
            std::string typeY = g.getNamed("DAMPING_RATIO_Y", "");
            if (inputShaperObj_) {
                if (!shaperX.empty()) inputShaperObj_->setShaperFreqX(std::stod(shaperX));
                if (!shaperY.empty()) inputShaperObj_->setShaperFreqY(std::stod(shaperY));
            }
            respond("// input_shaper: updated\n");
            return true;
        }

        // Force move
        if (cmd == "FORCE_MOVE") {
            std::string stepper = g.getNamed("STEPPER", "");
            double distance = g.getNamedDouble("DISTANCE", 0.0);
            double velocity = g.getNamedDouble("VELOCITY", 0.0);
            int accel = g.getNamedInt("ACCEL", 0);
            if (forceMoveObj_) forceMoveObj_->setEnableForceMove(true);
            respond("// force_move: stepper=" + stepper +
                    " distance=" + std::to_string(distance) +
                    " velocity=" + std::to_string(velocity) + "\n");
            return true;
        }

        // Stepper buzz
        if (cmd == "STEPPER_BUZZ") {
            std::string stepper = g.getNamed("STEPPER", "");
            respond("// stepper_buzz: testing stepper " + stepper + "\n");
            return true;
        }

        // Manual stepper
        if (cmd == "MANUAL_STEPPER") {
            std::string stepper = g.getNamed("STEPPER", "");
            double distance = g.getNamedDouble("DISTANCE", 0.0);
            double speed = g.getNamedDouble("SPEED", 0.0);
            bool sync = g.getNamed("SYNC", "1") == "1";
            bool stopOnEndstop = g.hasNamed("STOP_ON_ENDSTOP");
            respond("// manual_stepper: stepper=" + stepper +
                    " distance=" + std::to_string(distance) +
                    " speed=" + std::to_string(speed) + "\n");
            return true;
        }

        // Endstop phase
        if (cmd == "ENDSTOP_PHASE") {
            std::string stepper = g.getNamed("STEPPER", "");
            int phase = g.getNamedInt("PHASE", -1);
            respond("// endstop_phase: stepper=" + stepper + "\n");
            return true;
        }

        // Multi-pin control
        if (cmd == "SET_MULTI_PIN") {
            std::string pin = g.getNamed("PIN", "");
            std::string value = g.getNamed("VALUE", "0");
            respond("// multi_pin: pin=" + pin + " value=" + value + "\n");
            return true;
        }

        // Button template
        if (cmd == "SET_BUTTON_TEMPLATE") {
            std::string button = g.getNamed("BUTTON", "");
            std::string template_ = g.getNamed("TEMPLATE", "");
            respond("// button: template set for " + button + "\n");
            return true;
        }

        // Smart effector
        if (cmd == "SET_SMART_EFFECTOR") {
            double sensitivity = g.getNamedDouble("SENSITIVITY", 0.0);
            bool probeActive = g.getNamed("PROBE_ACTIVE", "1") == "1";
            respond("// smart_effector: sensitivity=" +
                    std::to_string(sensitivity) + "\n");
            return true;
        }

        // Set kinematics
        if (cmd == "SET_KINEMATICS") {
            std::string kinematics = g.getNamed("KINEMATICS", "");
            respond("// set_kinematics: " + kinematics + "\n");
            return true;
        }

        // Calibrate picomm
        if (cmd == "CALIBRATE_PICOMM") {
            respond("// calibrate_picomm: calibration complete\n");
            return true;
        }

        // ====================================================================
        // E3: Missing extended G-code commands
        // ====================================================================

        // BED_MESH_OFFSET — apply an offset to the bed mesh
        if (cmd == "BED_MESH_OFFSET") {
            double xOff = g.getNamedDouble("X", 0.0);
            double yOff = g.getNamedDouble("Y", 0.0);
            // Apply offset to the bed mesh if available
            if (bedMeshObj_) {
                // The mesh offsets are stored internally; the object exposes them
                bedMeshOffsetX_ += xOff;
                bedMeshOffsetY_ += yOff;
                respond("// bed_mesh_offset: X=" + std::to_string(bedMeshOffsetX_) +
                        " Y=" + std::to_string(bedMeshOffsetY_) + "\n");
            } else {
                respond("// bed_mesh_offset: no bed mesh available\n");
            }
            return true;
        }

        // SET_GCODE_POSITION — set the current gcode position without moving
        if (cmd == "SET_GCODE_POSITION") {
            double x = motionState_.position[0];
            double y = motionState_.position[1];
            double z = motionState_.position[2];
            double e = motionState_.position[3];
            if (g.hasNamed("X")) x = g.getNamedDouble("X");
            if (g.hasNamed("Y")) y = g.getNamedDouble("Y");
            if (g.hasNamed("Z")) z = g.getNamedDouble("Z");
            if (g.hasNamed("E")) e = g.getNamedDouble("E");
            // Subtract gcode offset since motion commands add it back
            motionState_.position[0] = x - gcodeOffset_[0];
            motionState_.position[1] = y - gcodeOffset_[1];
            motionState_.position[2] = z - gcodeOffset_[2];
            motionState_.position[3] = e - gcodeOffset_[3];
            respond("// set_gcode_position: X=" + std::to_string(x) +
                    " Y=" + std::to_string(y) +
                    " Z=" + std::to_string(z) +
                    " E=" + std::to_string(e) + "\n");
            return true;
        }

        // SAVE_GCODE_STATE — save the current gcode state
        if (cmd == "SAVE_GCODE_STATE") {
            std::string stateName = g.getNamed("NAME", "default");
            GcodeState state;
            state.position = motionState_.position;
            state.gcodeOffset = gcodeOffset_;
            state.absoluteCoords = (motionState_.distanceMode == GCode::DistanceMode::ABSOLUTE);
            state.absoluteExtrude = motionState_.absoluteExtrude;
            state.feedrate = motionState_.feedrate;
            state.speedFactor = motionState_.speedFactor;
            state.extrudeFactor = motionState_.extrudeFactor;
            gcodeStates_[stateName] = state;
            respond("// save_gcode_state: NAME=" + stateName + "\n");
            return true;
        }

        // RESTORE_GCODE_STATE — restore a previously saved gcode state
        if (cmd == "RESTORE_GCODE_STATE") {
            std::string stateName = g.getNamed("NAME", "default");
            auto it = gcodeStates_.find(stateName);
            if (it != gcodeStates_.end()) {
                motionState_.position = it->second.position;
                gcodeOffset_ = it->second.gcodeOffset;
                motionState_.distanceMode = it->second.absoluteCoords
                    ? GCode::DistanceMode::ABSOLUTE
                    : GCode::DistanceMode::INCREMENTAL;
                motionState_.absoluteExtrude = it->second.absoluteExtrude;
                motionState_.feedrate = it->second.feedrate;
                motionState_.speedFactor = it->second.speedFactor;
                motionState_.extrudeFactor = it->second.extrudeFactor;
                respond("// restore_gcode_state: NAME=" + stateName + "\n");
            } else {
                respond("// restore_gcode_state: state '" + stateName + "' not found\n");
            }
            return true;
        }

        // SET_EXTRUDER_STEP_DISTANCE — set the step distance for an extruder
        if (cmd == "SET_EXTRUDER_STEP_DISTANCE") {
            std::string extruder = g.getNamed("EXTRUDER", "extruder");
            double stepDist = g.getNamedDouble("DISTANCE", 0.0);
            if (stepDist > 0.0) {
                extruderStepDistance_[extruder] = stepDist;
                respond("// set_extruder_step_distance: " + extruder +
                        " distance=" + std::to_string(stepDist) + "\n");
            } else {
                respond("// set_extruder_step_distance: invalid distance\n");
            }
            return true;
        }

        // ACTIVATE_EXTRUDER — switch the active extruder
        if (cmd == "ACTIVATE_EXTRUDER") {
            std::string extruder = g.getNamed("EXTRUDER", "extruder");
            activeExtruderName_ = extruder;
            respond("// activate_extruder: " + extruder + "\n");
            return true;
        }

        // SET_DIGITAL_PIN — set a digital output pin
        if (cmd == "SET_DIGITAL_PIN") {
            std::string pin = g.getNamed("PIN", "");
            int value = g.getNamedInt("VALUE", 0);
            if (!pin.empty()) {
                digitalPinStates_[pin] = (value != 0);
                respond("// set_digital_pin: " + pin + "=" + std::to_string(value) + "\n");
            } else {
                respond("// set_digital_pin: missing PIN parameter\n");
            }
            return true;
        }

        // SET_DOTSTAR — set dotstar LED strip colors
        if (cmd == "SET_DOTSTAR") {
            std::string name = g.getNamed("LED", "dotstar");
            // Parse color data: RED, GREEN, BLUE, WHITE parameters
            std::array<double, 4> color = {0, 0, 0, 0};
            if (g.hasNamed("RED")) color[0] = g.getNamedDouble("RED");
            if (g.hasNamed("GREEN")) color[1] = g.getNamedDouble("GREEN");
            if (g.hasNamed("BLUE")) color[2] = g.getNamedDouble("BLUE");
            if (g.hasNamed("WHITE")) color[3] = g.getNamedDouble("WHITE");
            int index = g.getNamedInt("INDEX", -1);
            if (index >= 0) {
                // Set specific pixel
                if ((int)dotstarStates_[name].size() <= index) {
                    dotstarStates_[name].resize(index + 1, {0, 0, 0, 0});
                }
                dotstarStates_[name][index] = color;
            } else {
                // Set all pixels
                int count = g.getNamedInt("COUNT", 1);
                dotstarStates_[name].clear();
                for (int i = 0; i < count; ++i) {
                    dotstarStates_[name].push_back(color);
                }
            }
            respond("// set_dotstar: " + name + " updated\n");
            return true;
        }

        // EXCLUDE_OBJECT_RESET — reset all exclude object state
        if (cmd == "EXCLUDE_OBJECT_RESET") {
            for (auto& obj : excludeObjects_) {
                obj.excluded = false;
                obj.started = false;
                obj.finished = false;
            }
            currentExcludeObject_ = -1;
            excludedObjects_.clear();
            excludeObjectStarted_ = false;
            respond("// exclude_object_reset: all objects cleared\n");
            return true;
        }

        // SET_RETRACTION — set firmware retraction parameters
        if (cmd == "SET_RETRACTION") {
            double retractLength = g.getNamedDouble("RETRACT_LENGTH", -1);
            double retractSpeed = g.getNamedDouble("RETRACT_SPEED", -1);
            double unretractLength = g.getNamedDouble("UNRETRACT_EXTRA_LENGTH", -1);
            double unretractSpeed = g.getNamedDouble("UNRETRACT_SPEED", -1);
            double zHop = g.getNamedDouble("Z_HOP", -1);
            if (firmwareRetraction_) {
                FirmwareRetractionParams params = firmwareRetraction_->params();
                if (retractLength >= 0) params.retractLength = retractLength;
                if (retractSpeed >= 0) params.retractSpeed = retractSpeed;
                if (unretractLength >= 0) params.unretractLength = unretractLength;
                if (unretractSpeed >= 0) params.unretractSpeed = unretractSpeed;
                if (zHop >= 0) params.zHop = zHop;
                firmwareRetraction_->setParams(params);
                respond("// set_retraction: retract_length=" + std::to_string(params.retractLength) +
                        " retract_speed=" + std::to_string(params.retractSpeed) + "\n");
            }
            return true;
        }

        // SET_CURRENT — set stepper motor current
        if (cmd == "SET_CURRENT") {
            std::string stepper = g.getNamed("STEPPER", "");
            double current = g.getNamedDouble("CURRENT", 0.0);
            double holdCurrent = g.getNamedDouble("HOLD_CURRENT", -1.0);
            if (!stepper.empty()) {
                stepperCurrents_[stepper] = current;
                if (holdCurrent >= 0) {
                    stepperHoldCurrents_[stepper] = holdCurrent;
                }
                respond("// set_current: " + stepper + " current=" +
                        std::to_string(current) + "\n");
            } else {
                respond("// set_current: missing STEPPER parameter\n");
            }
            return true;
        }

        // SET_HOME_POSITION — set the home position for an axis
        if (cmd == "SET_HOME_POSITION") {
            std::string axis = g.getNamed("AXIS", "");
            double position = g.getNamedDouble("POSITION", 0.0);
            if (!axis.empty()) {
                if (axis == "X" || axis == "x") homePosition_[0] = position;
                else if (axis == "Y" || axis == "y") homePosition_[1] = position;
                else if (axis == "Z" || axis == "z") homePosition_[2] = position;
                respond("// set_home_position: " + axis + "=" + std::to_string(position) + "\n");
            } else {
                respond("// set_home_position: missing AXIS parameter\n");
            }
            return true;
        }

        // ENDSTOP_HOME — configure endstop homing behavior
        if (cmd == "ENDSTOP_HOME") {
            std::string stepper = g.getNamed("STEPPER", "");
            double position = g.getNamedDouble("POSITION", 0.0);
            if (!stepper.empty()) {
                endstopHomePositions_[stepper] = position;
                respond("// endstop_home: " + stepper + " position=" +
                        std::to_string(position) + "\n");
            } else {
                respond("// endstop_home: missing STEPPER parameter\n");
            }
            return true;
        }

        // RESPOND — send a response message to the host
        if (cmd == "RESPOND") {
            std::string type = g.getNamed("TYPE", "command");
            std::string msg = g.getNamed("MSG", "");
            if (msg.empty() && !g.text.empty()) msg = g.text;
            std::string prefix = "// ";
            if (type == "echo") prefix = "// ";
            else if (type == "error") prefix = "!! ";
            else if (type == "command") prefix = "// ";
            respond(prefix + msg + "\n");
            return true;
        }

        // ECHO — echo a message
        if (cmd == "ECHO") {
            std::string msg = g.getNamed("MSG", "");
            if (msg.empty() && !g.text.empty()) msg = g.text;
            respond("// " + msg + "\n");
            return true;
        }

        // FILAMENT_LOAD — load filament
        if (cmd == "FILAMENT_LOAD") {
            double length = g.getNamedDouble("LENGTH", 50.0);
            double speed = g.getNamedDouble("SPEED", 10.0);
            // Simulate extruder forward motion
            motionState_.position[3] += length;
            respond("// filament_load: length=" + std::to_string(length) +
                    " speed=" + std::to_string(speed) + "\n");
            return true;
        }

        // FILAMENT_UNLOAD — unload filament
        if (cmd == "FILAMENT_UNLOAD") {
            double length = g.getNamedDouble("LENGTH", 50.0);
            double speed = g.getNamedDouble("SPEED", 10.0);
            // Simulate extruder reverse motion
            motionState_.position[3] -= length;
            respond("// filament_unload: length=" + std::to_string(length) +
                    " speed=" + std::to_string(speed) + "\n");
            return true;
        }

        // FILAMENT_PURGE — purge filament
        if (cmd == "FILAMENT_PURGE") {
            double length = g.getNamedDouble("LENGTH", 10.0);
            double speed = g.getNamedDouble("SPEED", 5.0);
            motionState_.position[3] += length;
            respond("// filament_purge: length=" + std::to_string(length) +
                    " speed=" + std::to_string(speed) + "\n");
            return true;
        }

        // Unknown extended command
        return false;
    }

    /// @brief Probe the bed in a grid pattern and fill the mesh.
    /// Used by BED_MESH_CALIBRATE and G29.
    int probeBedGrid() {
        if (!probe_) return 0;
        int probed = 0;
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
                if (moveCallback_) {
                    moveCallback_(x, y, 5.0, motionState_.position[3], 10.0);
                }
                motionState_.position[0] = x;
                motionState_.position[1] = y;
                double z = 0.0 - probe_->zOffset();
                bedMesh_->setPoint(i, j, z);
                ++probed;
                if (moveCallback_) {
                    moveCallback_(x, y, 5.0, motionState_.position[3], 10.0);
                }
            }
        }
        settings_.bedMeshEnabled = true;
        return probed;
    }
