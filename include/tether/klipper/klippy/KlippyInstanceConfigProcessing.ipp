/// @file KlippyInstanceConfigProcessing.ipp
/// @brief Config processing and settings persistence for KlippyInstance (included into class body).
///
/// This file is included inside the KlippyInstance class definition.
/// It contains processConfigSections() and the settings persistence methods
/// (saveSettingsToFile, loadSettingsFromFile, reportSettings).

    /// @brief Load config from file and process sections.
    bool loadConfig(const std::string& path) {
        if (!server_.loadConfigFile(path)) return false;
        processConfigSections();
        applySettings();
        return true;
    }

    /// @brief Apply parsed settings to runtime objects.
    /// Called after processConfigSections() to wire config values
    /// into the toolhead, motion translator, printer objects, etc.
    void applySettings() {
        // --- Motion limits ---
        if (toolheadObj_) {
            toolheadObj_->setMaxVelocity(settings_.maxVelocity);
            toolheadObj_->setMaxAccel(settings_.maxAccel);
            toolheadObj_->setMaxAccelToDecel(settings_.maxAccelToDecel);
        }

        // --- Kinematics ---
        kinematicsTransform_.setKinematics(settings_.kinematics);
        if (settings_.kinematics == Kinematics::Delta) {
            deltaPrinter_->setGeometry(settings_.deltaGeometry);
            deltaPrinter_->setEndstopAdjust(settings_.deltaEndstopAdjust);
            kinematicsTransform_.setDeltaPrinter(deltaPrinter_.get());
        }
        if (settings_.kinematics == Kinematics::RotaryDelta) {
            rotaryDeltaPrinter_->setGeometry(settings_.rotaryDeltaGeometry);
            rotaryDeltaPrinter_->setEndstopAdjust(settings_.rotaryDeltaEndstopAdjust);
            kinematicsTransform_.setRotaryDeltaPrinter(rotaryDeltaPrinter_.get());
        }
        if (settings_.kinematics == Kinematics::Winch) {
            kinematicsTransform_.setWinchParams(
                settings_.winchConfig.anchorRadius,
                settings_.winchConfig.anchorHeight);
        }

        // --- Idle timeout ---
        idleTimeout_ = settings_.idleTimeout;
        if (idleTimeoutObj_) {
            idleTimeoutObj_->setTimeout(settings_.idleTimeout);
        }

        // --- Autotuning method (delegated to Tether autotuning framework) ---
        pidAutotuneMethod_ = parseAutotuneMethod(settings_.pidAutotuneMethod);

        // --- Input shaper ---
        if (settings_.inputShaperFreqX > 0.0 || settings_.inputShaperFreqY > 0.0) {
            InputShaperParams paramsX;
            paramsX.freq = settings_.inputShaperFreqX;
            paramsX.damping = settings_.inputShaperDampingX;
            if (settings_.inputShaperTypeX == "ei") paramsX.type = ShaperType::EI;
            else if (settings_.inputShaperTypeX == "zv") paramsX.type = ShaperType::ZV;
            else if (settings_.inputShaperTypeX == "zvd") paramsX.type = ShaperType::ZVD;
            else if (settings_.inputShaperTypeX == "mzv") paramsX.type = ShaperType::MZV;
            else if (settings_.inputShaperTypeX == "damped_ei") paramsX.type = ShaperType::DampedEI;
            // Apply X shaper (inputShaper_ is shared; for separate X/Y we'd need two)
            inputShaper_->setParams(paramsX);
        }

#if TETHER_ENABLE_PRESSURE_ADVANCE
        // --- Pressure advance / extrusion compensation (runtime opt-in) ---
        // The classic linear PA is enabled when pressure_advance > 0 and the
        // model is "linear" (the default). For power_law/cross_wlf models the
        // dispatcher is configured with the model + its parameters; the
        // classic pressure_advance value is ignored for those models.
        const bool wantsPA = settings_.extruderPressureAdvance > 0.0 ||
                             settings_.paConsistency > 0.0 ||
                             settings_.crossWlfCompressibilityOverArea > 0.0;
        if (wantsPA) {
            pressureAdvance_->setEnabled(true);
            pressureAdvance_->setParams({settings_.extruderPressureAdvance,
                                         settings_.extruderSmoothTime});
            if (extruderObj_) {
                extruderObj_->setPressureAdvance(settings_.extruderPressureAdvance);
            }
            if (pressureAdvanceObj_) {
                pressureAdvanceObj_->setPressureAdvance(settings_.extruderPressureAdvance);
                pressureAdvanceObj_->setSmoothTime(settings_.extruderSmoothTime);
                pressureAdvanceObj_->setModel(settings_.extrusionCompensationModel);
            }
            if (motionDispatcher_) {
                auto paCfg = motionDispatcher_->pressureAdvanceConfig();
                paCfg.enabled = true;
                paCfg.smoothTime = settings_.extruderSmoothTime;
                paCfg.filamentDiameterMm = settings_.filamentDiameter;
                paCfg.maxCompensation = settings_.paMaxCompensation;
                if (settings_.extrusionCompensationModel == "power_law") {
                    paCfg.model = motion::ExtrusionCompensationModel::PowerLaw;
                    paCfg.powerLawBaseGain = settings_.paConsistency;
                    paCfg.flowIndex = settings_.paFlowIndex;
                } else if (settings_.extrusionCompensationModel == "cross_wlf") {
                    paCfg.model = motion::ExtrusionCompensationModel::CrossWlf;
                    paCfg.crossWlfCompressibilityOverArea =
                        settings_.crossWlfCompressibilityOverArea;
                    // LUT is built lazily on first use if not already set.
                } else {
                    paCfg.model = motion::ExtrusionCompensationModel::Linear;
                    paCfg.pressureAdvance = settings_.extruderPressureAdvance;
                }
                motionDispatcher_->setPressureAdvanceConfig(paCfg);
            }
        }
        // Flow-adaptive heater compensation (built/wired if enabled).
        applyFlowAdaptiveHeaterSettings();
#endif

        // --- Skew correction ---
        skewCorrection_->setParams(settings_.skewParams);

        // --- TMC drivers ---
        for (const auto& [stepper, cfg] : settings_.tmcDrivers) {
            tmcConfig_->setRunCurrent(stepper, cfg.runCurrent);
            tmcConfig_->setHoldCurrent(stepper, cfg.holdCurrent);
            tmcConfig_->setStealthChop(stepper, cfg.interpolate);
        }

        // --- Output pins: register named pin state ---
        for (const auto& [name, cfg] : settings_.outputPins) {
            namedOutputPins_[name] = cfg.value;
            if (cfg.pwm) {
                namedPwmPins_[name] = cfg.value;
            }
        }

        // --- Servos: initialize servo state from config ---
        for (const auto& [name, cfg] : settings_.servos) {
            servoStates_[name] = ServoState{cfg.initialAngle, 0.0};
        }

        // --- Temperature fan targets ---
        for (const auto& [name, cfg] : settings_.temperatureFans) {
            temperatureFanTargets_[name] = cfg.targetTemp;
        }

        // --- Generic fan speeds from config ---
        for (const auto& [name, cfg] : settings_.heaterFans) {
            genericFanSpeeds_[name] = cfg.maxPower;
        }
        for (const auto& [name, cfg] : settings_.controllerFans) {
            genericFanSpeeds_[name] = cfg.maxPower;
        }

        // --- Save variables ---
        if (saveVariablesObj_) {
            // Load any saved variables into the object
            for (const auto& [k, v] : savedVariables_) {
                saveVariablesObj_->setVariable(k, v);
            }
        }

        // --- Board pins ---
        if (boardPinsObj_) {
            boardPinsObj_->setMcuName("mcu");
        }

        // --- Safe Z home ---
        if (safeZHomeObj_) {
            safeZHomeObj_->setHomeXyPosition(settings_.safeZHomeXYPosition);
            safeZHomeObj_->setZHop(settings_.safeZHomeZHop);
            safeZHomeObj_->setZHopSpeed(settings_.safeZHomeZHopSpeed);
            safeZHomeObj_->setXyHomeSpeed(settings_.safeZHomeXYHomeSpeed);
            safeZHomeObj_->setMoveToPrevious(settings_.safeZHomeMoveToPrevious);
        }

        // --- Temperature sensors: create real or stub sensors for each configured sensor ---
        // When a [thermistor <name>] or [thermocouple <name>] definition matches
        // the sensor_type, instantiate the real sensor class. If a real
        // ADC/SPI callback has been registered via registerAdcCallback()/
        // registerSpiCallback(), use it; otherwise use a placeholder.
        // If no matching thermistor/thermocouple definition exists, fall
        // back to the ConfigTemperatureSensor stub.
        for (const auto& [name, cfg] : settings_.temperatureSensors) {
            // Only register if not already registered (avoid duplicates on re-apply)
            if (configTempSensorsRegistered_.find(name) !=
                configTempSensorsRegistered_.end()) {
                continue;
            }

            std::shared_ptr<objects::TemperatureSensor> sensor;

            // Check if sensor_type references a [thermistor <name>] definition
            auto thermIt = settings_.thermistors.find(cfg.sensorType);
            if (thermIt != settings_.thermistors.end()) {
                objects::Thermistor::Params params;
                params.pullupResistor = thermIt->second.pullupResistor;
                params.referenceVoltage = thermIt->second.referenceVoltage;
                params.adcMax = thermIt->second.adcMax;
                params.resistanceAt25C = thermIt->second.resistanceAt25C;
                params.beta = thermIt->second.beta;
                params.minTemp = cfg.minTemp;
                params.maxTemp = cfg.maxTemp;
                // Copy calibration table if present
                for (const auto& [temp, res] : thermIt->second.calibrationTable) {
                    params.calibrationTable.push_back({temp, res});
                }
                // Use registered ADC callback if available, else placeholder
                auto registeredAdc = adcCallback(cfg.sensorPin);
                std::function<double()> adcRead = registeredAdc
                    ? registeredAdc
                    : std::function<double()>([]() { return 2048.0; });
                sensor = std::make_shared<objects::Thermistor>(0, params, adcRead);
            }

            // Check if sensor_type references a [thermocouple <name>] definition
            if (!sensor) {
                auto tcIt = settings_.thermocouples.find(cfg.sensorType);
                if (tcIt != settings_.thermocouples.end()) {
                    objects::Thermocouple::Type tcType = objects::Thermocouple::Type::K;
                    const std::string& t = tcIt->second.type;
                    if (t == "J") tcType = objects::Thermocouple::Type::J;
                    else if (t == "T") tcType = objects::Thermocouple::Type::T;
                    else if (t == "E") tcType = objects::Thermocouple::Type::E;
                    else if (t == "N") tcType = objects::Thermocouple::Type::N;
                    else if (t == "R") tcType = objects::Thermocouple::Type::R;
                    else if (t == "S") tcType = objects::Thermocouple::Type::S;
                    else if (t == "B") tcType = objects::Thermocouple::Type::B;
                    // Use registered SPI callback if available, else placeholder
                    auto registeredSpi = spiCallback(tcIt->second.spiBus);
                    std::function<std::vector<uint8_t>(std::span<const uint8_t>)> spiTransfer =
                        registeredSpi
                            ? registeredSpi
                            : std::function<std::vector<uint8_t>(std::span<const uint8_t>)>(
                                  [](std::span<const uint8_t>) {
                                      return std::vector<uint8_t>(4, 0);
                                  });
                    sensor = std::make_shared<objects::Thermocouple>(0, tcType, spiTransfer);
                }
            }

            // Check if sensor_type references a [rtd <name>] definition
            if (!sensor) {
                auto rtdIt = settings_.rtds.find(cfg.sensorType);
                if (rtdIt != settings_.rtds.end()) {
                    objects::RtdSensor::Params params;
                    params.nominalResistance = rtdIt->second.nominalResistance;
                    params.alpha = rtdIt->second.alpha;
                    params.referenceResistor = rtdIt->second.referenceResistor;
                    params.adcMax = rtdIt->second.adcMax;
                    params.referenceVoltage = rtdIt->second.referenceVoltage;
                    params.minTemp = cfg.minTemp;
                    params.maxTemp = cfg.maxTemp;
                    auto registeredAdc = adcCallback(cfg.sensorPin);
                    std::function<double()> adcRead = registeredAdc
                        ? registeredAdc
                        : std::function<double()>([]() { return 2048.0; });
                    sensor = std::make_shared<objects::RtdSensor>(0, params, adcRead);
                }
            }

            // Fall back to stub if no matching thermistor/thermocouple/RTD definition
            if (!sensor) {
                sensor = std::make_shared<ConfigTemperatureSensor>(
                    name, cfg.sensorType, cfg.minTemp, cfg.maxTemp);
            }

            registerTemperatureSensor("temperature_sensor " + name, sensor);
            configTempSensorsRegistered_.insert(name);
        }

        // --- ADXL345: create a real or stub ADXL345 if configured but not set ---
        if (settings_.adxl345Configured && !adxl345_) {
            std::shared_ptr<objects::Adxl345> adxl;

            // First, try to use a user-registered SPI callback for the bus
            if (!settings_.adxl345SpiBus.empty()) {
                auto registeredSpi = spiCallback(settings_.adxl345SpiBus);
                if (registeredSpi) {
                    adxl = std::make_shared<objects::Adxl345>(0, registeredSpi);
                }
            }

            // Next, try to create a real SPI-backed ADXL345 via PosixSpiDriver
            #if !defined(ESP_PLATFORM)
            if (!adxl && !settings_.adxl345SpiBus.empty()) {
                // Parse bus number from "spiN" format
                std::string bus = settings_.adxl345SpiBus;
                int busNum = 0;
                if (bus.size() > 3 && bus.substr(0, 3) == "spi") {
                    busNum = std::atoi(bus.c_str() + 3);
                }
                // Default chip select 0; could parse from cs_pin in future
                std::string devPath = "/dev/spidev" + std::to_string(busNum) + ".0";

                auto spiDriver = std::make_shared<tether::io::PosixSpiDriver>();
                if (spiDriver->open(devPath.c_str(), 0, 5000000, 8)) {
                    // Create SPI transfer function backed by the real driver.
                    // The driver is captured by shared_ptr to keep it alive.
                    auto spiTransfer = [spiDriver](std::span<const uint8_t> tx) {
                        return spiDriver->transfer(tx);
                    };
                    adxl = std::make_shared<objects::Adxl345>(0, spiTransfer);
                }
            }
            #endif

            // Fall back to stub if real SPI not available
            if (!adxl) {
                auto stubSpi = [](std::span<const uint8_t>) {
                    return std::vector<uint8_t>(8, 0);
                };
                adxl = std::make_shared<objects::Adxl345>(0, stubSpi);
            }
            setAdxl345(adxl);
        }

        // --- Z tilt: configure from settings ---
        if (zTiltObj_ && settings_.zTiltEnabled) {
            // Parse z_positions string into JSON values
            std::vector<JsonValue> positions;
            std::istringstream iss(settings_.zTiltPositions);
            std::string token;
            while (std::getline(iss, token, ',')) {
                try {
                    positions.push_back(JsonValue(std::stod(token)));
                } catch (...) {}
            }
            if (!positions.empty()) {
                zTiltObj_->setZPositions(positions);
            }
        }
    }

    /// @brief Process parsed config sections into KlippyInstance state.
    /// Handles: printer, stepper, extruder, heater_bed, fan, probe, bed_mesh,
    /// mcu, virtual_sdcard, safe_z_home, idle_timeout, pause_resume,
    /// display_status, output_pin, servo, temperature_sensor, temperature_fan,
    /// heater_fan, controller_fan, tmc*, adxl345, input_shaper, skew_correction,
    /// z_tilt, quad_gantry_level, bed_screws, screws_tilt_adjust,
    /// gcode_macro, delayed_gcode, firmware_retraction, exclude_object,
    /// save_variables, force_move, homing_override, endstop_phase, menu, palette2.
    void processConfigSections() {
        const auto& parser = server_.configParser();
        for (const auto& section : parser.sections()) {
            const std::string& name = section.name;

            // [printer] — main printer configuration
            if (name == "printer") {
                if (section.has("kinematics")) {
                    settings_.kinematics = kinematicsFromString(section.get("kinematics", "cartesian"));
                }
                if (section.has("max_velocity")) {
                    try { settings_.maxVelocity = section.getDouble("max_velocity"); } catch (...) {}
                }
                if (section.has("max_accel")) {
                    try { settings_.maxAccel = section.getDouble("max_accel"); } catch (...) {}
                }
                if (section.has("max_accel_to_decel")) {
                    try { settings_.maxAccelToDecel = section.getDouble("max_accel_to_decel"); } catch (...) {}
                }
                if (section.has("square_corner_velocity")) {
                    try { settings_.squareCornerVelocity = section.getDouble("square_corner_velocity"); } catch (...) {}
                }
                if (section.has("max_z_velocity")) {
                    try { settings_.maxZVelocity = section.getDouble("max_z_velocity"); } catch (...) {}
                }
                if (section.has("max_z_accel")) {
                    try { settings_.maxZAccel = section.getDouble("max_z_accel"); } catch (...) {}
                }
            }

            // [stepper_x], [stepper_y], [stepper_z], [stepper_z1], etc.
            if (name.substr(0, 8) == "stepper_") {
                std::string axis = name.substr(8);
                // Normalize axis name: x, y, z, z1, z2, z3 -> use first char lower
                std::string key = axis;
                if (axis.size() > 1) key = axis.substr(0, 1) + axis.substr(1);

                if (section.has("rotation_distance")) {
                    try {
                        double rd = section.getDouble("rotation_distance");
                        if (rd > 0) {
                            settings_.stepsPerMm[key] = 1.0 / rd;
                        }
                    } catch (...) {}
                }
                if (section.has("microsteps")) {
                    try {
                        settings_.microstepping[key] = static_cast<int>(section.getInt("microsteps"));
                    } catch (...) {}
                }
                if (section.has("run_current")) {
                    try {
                        settings_.stepperCurrent[key] = section.getDouble("run_current");
                    } catch (...) {}
                }
                if (section.has("dir_pin")) {
                    // Invert direction if pin name starts with '!'
                    std::string pin = section.get("dir_pin", "");
                    settings_.stepperDirection[key] = (pin.size() > 0 && pin[0] == '!') ? 1 : 0;
                }
                if (section.has("max_velocity")) {
                    try {
                        settings_.maxFeedrate[key] = section.getDouble("max_velocity");
                    } catch (...) {}
                }
                if (section.has("position_endstop")) {
                    try {
                        settings_.homeOffset[key] = section.getDouble("position_endstop");
                    } catch (...) {}
                }
            }

            // [extruder]
            else if (name == "extruder") {
                if (section.has("nozzle_diameter")) {
                    try { settings_.nozzleDiameter = section.getDouble("nozzle_diameter"); } catch (...) {}
                }
                if (section.has("filament_diameter")) {
                    try { settings_.filamentDiameter = section.getDouble("filament_diameter"); } catch (...) {}
                }
                if (section.has("rotation_distance")) {
                    try {
                        double rd = section.getDouble("rotation_distance");
                        if (rd > 0) settings_.stepsPerMm["e"] = 1.0 / rd;
                    } catch (...) {}
                }
                if (section.has("microsteps")) {
                    try { settings_.microstepping["e"] = static_cast<int>(section.getInt("microsteps")); } catch (...) {}
                }
                if (section.has("run_current")) {
                    try { settings_.stepperCurrent["e"] = section.getDouble("run_current"); } catch (...) {}
                }
                if (section.has("max_extrude_only_velocity")) {
                    try { settings_.maxFeedrate["e"] = section.getDouble("max_extrude_only_velocity"); } catch (...) {}
                }
#if TETHER_ENABLE_PRESSURE_ADVANCE
                if (section.has("pressure_advance")) {
                    try { settings_.extruderPressureAdvance = section.getDouble("pressure_advance"); } catch (...) {}
                }
                if (section.has("smooth_time")) {
                    try { settings_.extruderSmoothTime = section.getDouble("smooth_time"); } catch (...) {}
                }
                // Non-Newtonian extrusion compensation (extends PA)
                if (section.has("pressure_advance_model")) {
                    try { settings_.extrusionCompensationModel = section.get("pressure_advance_model"); } catch (...) {}
                }
                if (section.has("pa_flow_index")) {
                    try { settings_.paFlowIndex = section.getDouble("pa_flow_index"); } catch (...) {}
                }
                if (section.has("pa_consistency")) {
                    try { settings_.paConsistency = section.getDouble("pa_consistency"); } catch (...) {}
                }
                if (section.has("pa_max_compensation")) {
                    try { settings_.paMaxCompensation = section.getDouble("pa_max_compensation"); } catch (...) {}
                }
                // Cross-WLF parameters
                if (section.has("cross_wlf_tau_star")) {
                    try { settings_.crossWlfTauStar = section.getDouble("cross_wlf_tau_star"); } catch (...) {}
                }
                if (section.has("cross_wlf_flow_index")) {
                    try { settings_.crossWlfFlowIndex = section.getDouble("cross_wlf_flow_index"); } catch (...) {}
                }
                if (section.has("cross_wlf_c1")) {
                    try { settings_.crossWlfC1 = section.getDouble("cross_wlf_c1"); } catch (...) {}
                }
                if (section.has("cross_wlf_c2")) {
                    try { settings_.crossWlfC2 = section.getDouble("cross_wlf_c2"); } catch (...) {}
                }
                if (section.has("cross_wlf_ref_temp")) {
                    try { settings_.crossWlfRefTempC = section.getDouble("cross_wlf_ref_temp"); } catch (...) {}
                }
                if (section.has("cross_wlf_zero_shear_viscosity")) {
                    try { settings_.crossWlfZeroShearViscosityRef = section.getDouble("cross_wlf_zero_shear_viscosity"); } catch (...) {}
                }
                if (section.has("cross_wlf_compressibility_over_area")) {
                    try { settings_.crossWlfCompressibilityOverArea = section.getDouble("cross_wlf_compressibility_over_area"); } catch (...) {}
                }
                if (section.has("cross_wlf_lut_path")) {
                    try { settings_.crossWlfLutPath = section.get("cross_wlf_lut_path"); } catch (...) {}
                }
                // Flow-adaptive heater compensation
                if (section.has("heater_flow_pre_emphasis")) {
                    try { settings_.heaterFlowPreEmphasis = section.getBool("heater_flow_pre_emphasis"); } catch (...) {}
                }
                if (section.has("filament_heat_capacity")) {
                    try { settings_.filamentHeatCapacity = section.getDouble("filament_heat_capacity"); } catch (...) {}
                }
                if (section.has("melt_zone_capacitance")) {
                    try { settings_.meltZoneCapacitance = section.getDouble("melt_zone_capacitance"); } catch (...) {}
                }
                if (section.has("heater_melt_conductance")) {
                    try { settings_.heaterMeltConductance = section.getDouble("heater_melt_conductance"); } catch (...) {}
                }
                if (section.has("debt_time_constant")) {
                    try { settings_.debtTimeConstant = section.getDouble("debt_time_constant"); } catch (...) {}
                }
                if (section.has("max_pre_emphasis_power")) {
                    try { settings_.maxPreEmphasisPower = section.getDouble("max_pre_emphasis_power"); } catch (...) {}
                }
                if (section.has("max_post_emphasis_power")) {
                    try { settings_.maxPostEmphasisPower = section.getDouble("max_post_emphasis_power"); } catch (...) {}
                }
                if (section.has("max_heater_overshoot")) {
                    try { settings_.maxHeaterOvershoot = section.getDouble("max_heater_overshoot"); } catch (...) {}
                }
#endif
                if (section.has("min_extrude_temp")) {
                    try { config_.minExtrudeTemp = section.getDouble("min_extrude_temp"); } catch (...) {}
                }
                // PID parameters
                if (section.has("pid_Kp")) {
                    try { settings_.hotendKp = section.getDouble("pid_Kp"); } catch (...) {}
                }
                if (section.has("pid_Ki")) {
                    try { settings_.hotendKi = section.getDouble("pid_Ki"); } catch (...) {}
                }
                if (section.has("pid_Kd")) {
                    try { settings_.hotendKd = section.getDouble("pid_Kd"); } catch (...) {}
                }
            }

            // [heater_bed]
            else if (name == "heater_bed") {
                if (section.has("min_temp")) {
                    try { settings_.bedMinTemp = section.getDouble("min_temp"); } catch (...) {}
                }
                if (section.has("max_temp")) {
                    try { settings_.bedMaxTemp = section.getDouble("max_temp"); } catch (...) {}
                }
                // PID parameters
                if (section.has("pid_Kp")) {
                    try { settings_.bedKp = section.getDouble("pid_Kp"); } catch (...) {}
                }
                if (section.has("pid_Ki")) {
                    try { settings_.bedKi = section.getDouble("pid_Ki"); } catch (...) {}
                }
                if (section.has("pid_Kd")) {
                    try { settings_.bedKd = section.getDouble("pid_Kd"); } catch (...) {}
                }
            }

            // [fan]
            else if (name == "fan") {
                if (section.has("max_power")) {
                    try { settings_.fanMaxPower = section.getDouble("max_power"); } catch (...) {}
                }
                if (section.has("cycle_time")) {
                    try { settings_.fanCycleTime = section.getDouble("cycle_time"); } catch (...) {}
                }
                if (section.has("kick_start_time")) {
                    try { settings_.fanKickStartTime = section.getDouble("kick_start_time"); } catch (...) {}
                }
                if (section.has("off_below")) {
                    try { settings_.fanOffBelow = section.getDouble("off_below"); } catch (...) {}
                }
            }

            // [probe]
            else if (name == "probe") {
                if (section.has("z_offset")) {
                    try {
                        settings_.probeOffset = section.getDouble("z_offset");
                        if (probeObj_) probeObj_->setZOffset(settings_.probeOffset);
                    } catch (...) {}
                }
                if (section.has("x_offset")) {
                    try { settings_.probeXOffset = section.getDouble("x_offset"); } catch (...) {}
                }
                if (section.has("y_offset")) {
                    try { settings_.probeYOffset = section.getDouble("y_offset"); } catch (...) {}
                }
                if (section.has("speed")) {
                    try { settings_.probeSpeed = section.getDouble("speed"); } catch (...) {}
                }
                if (section.has("sample_count")) {
                    try { settings_.probeSampleCount = static_cast<int>(section.getInt("sample_count")); } catch (...) {}
                }
                if (section.has("samples_result")) {
                    settings_.probeSamplesResult = section.get("samples_result", "average");
                }
            }

            // [bed_mesh]
            else if (name == "bed_mesh") {
                settings_.bedMeshEnabled = true;
                if (section.has("mesh_min")) {
                    settings_.bedMeshMin = section.get("mesh_min", "");
                }
                if (section.has("mesh_max")) {
                    settings_.bedMeshMax = section.get("mesh_max", "");
                }
                if (section.has("probe_count")) {
                    settings_.bedMeshProbeCount = section.get("probe_count", "3,3");
                }
                if (section.has("mesh_speed")) {
                    try { settings_.bedMeshSpeed = section.getDouble("mesh_speed"); } catch (...) {}
                }
                if (section.has("fade_start")) {
                    try { settings_.bedMeshFadeStart = section.getDouble("fade_start"); } catch (...) {}
                }
                if (section.has("fade_end")) {
                    try { settings_.bedMeshFadeEnd = section.getDouble("fade_end"); } catch (...) {}
                }
                if (section.has("fade_target")) {
                    try { settings_.bedMeshFadeTarget = section.getDouble("fade_target"); } catch (...) {}
                }
                if (section.has("algorithm")) {
                    settings_.bedMeshAlgorithm = section.get("algorithm", "lagrange");
                }
            }

            // [mcu]
            else if (name == "mcu" || name.substr(0, 4) == "mcu ") {
                if (section.has("serial")) {
                    settings_.mcuSerial = section.get("serial", "");
                }
                if (section.has("baud")) {
                    try { settings_.mcuBaud = static_cast<int>(section.getInt("baud")); } catch (...) {}
                }
                if (section.has("restart_method")) {
                    settings_.mcuRestartMethod = section.get("restart_method", "command");
                }
                // For secondary MCUs [mcu <name>], store the serial path
                if (name.size() > 4 && section.has("serial")) {
                    settings_.secondaryMcuSerials[section.get("serial", "")] = name.substr(4);
                }
            }

            // [virtual_sdcard]
            else if (name == "virtual_sdcard") {
                if (section.has("path")) {
                    config_.sdcardDir = section.get("path", config_.sdcardDir);
                    // Recreate the sdcard with the new path
                    sdcard_ = std::make_shared<VirtualSdcard>(config_.sdcardDir);
                }
                if (section.has("on_error_gcode")) {
                    settings_.virtualSdcardOnErrorGcode = section.get("on_error_gcode", "");
                }
            }

            // [safe_z_home]
            else if (name == "safe_z_home") {
                if (section.has("home_xy_position")) {
                    settings_.safeZHomeXYPosition = section.get("home_xy_position", "0,0");
                }
                if (section.has("z_hop")) {
                    try { settings_.safeZHomeZHop = section.getDouble("z_hop"); } catch (...) {}
                }
                if (section.has("z_hop_speed")) {
                    try { settings_.safeZHomeZHopSpeed = section.getDouble("z_hop_speed"); } catch (...) {}
                }
                if (section.has("xy_home_speed")) {
                    try { settings_.safeZHomeXYHomeSpeed = section.getDouble("xy_home_speed"); } catch (...) {}
                }
                if (section.has("move_to_previous")) {
                    settings_.safeZHomeMoveToPrevious = section.get("move_to_previous", "False") == "True";
                }
            }

            // [idle_timeout]
            else if (name == "idle_timeout") {
                if (section.has("timeout")) {
                    try { settings_.idleTimeout = section.getDouble("timeout"); } catch (...) {}
                }
                if (section.has("gcode")) {
                    settings_.idleTimeoutGcode = section.get("gcode", "TURN_OFF_HEATERS");
                }
            }

            // [pause_resume]
            else if (name == "pause_resume") {
                settings_.pauseResumeEnabled = true;
                if (section.has("recover_from_subtract")) {
                    settings_.pauseResumeRecoverFromSubtract =
                        section.get("recover_from_subtract", "False") == "True";
                }
            }

            // [display_status]
            else if (name == "display_status") {
                settings_.displayStatusEnabled = true;
            }

            // [spoolman]
            else if (name == "spoolman") {
                std::string url = section.get("server", "");
                if (!url.empty()) {
                    config_.udsConfig.spoolmanUrl = url;
                    server_.setSpoolmanUrl(url);
                }
            }
            // [output_pin <name>]
            else if (name.substr(0, 11) == "output_pin ") {
                std::string pinName = name.substr(11);
                auto start = pinName.find_first_not_of(" \t");
                auto end = pinName.find_last_not_of(" \t");
                if (start != std::string::npos) {
                    pinName = pinName.substr(start, end - start + 1);
                }
                KlippySettings::OutputPinConfig pinCfg;
                pinCfg.pin = section.get("pin", "");
                pinCfg.value = section.getDouble("value", 0.0);
                if (section.has("pwm")) {
                    pinCfg.pwm = section.get("pwm", "False") == "True";
                }
                if (section.has("cycle_time")) {
                    try { pinCfg.cycleTime = section.getDouble("cycle_time"); } catch (...) {}
                }
                if (section.has("scale")) {
                    try { pinCfg.scale = section.getDouble("scale"); } catch (...) {}
                }
                settings_.outputPins[pinName] = pinCfg;
            }

            // [servo <name>]
            else if (name.substr(0, 6) == "servo ") {
                std::string servoName = name.substr(6);
                auto start = servoName.find_first_not_of(" \t");
                auto end = servoName.find_last_not_of(" \t");
                if (start != std::string::npos) {
                    servoName = servoName.substr(start, end - start + 1);
                }
                KlippySettings::ServoConfig servoCfg;
                servoCfg.pin = section.get("pin", "");
                if (section.has("minimum_pulse_width")) {
                    try { servoCfg.minPulseWidth = section.getDouble("minimum_pulse_width"); } catch (...) {}
                }
                if (section.has("maximum_pulse_width")) {
                    try { servoCfg.maxPulseWidth = section.getDouble("maximum_pulse_width"); } catch (...) {}
                }
                if (section.has("minimum_angle")) {
                    try { servoCfg.minAngle = section.getDouble("minimum_angle"); } catch (...) {}
                }
                if (section.has("maximum_angle")) {
                    try { servoCfg.maxAngle = section.getDouble("maximum_angle"); } catch (...) {}
                }
                if (section.has("initial_angle")) {
                    try { servoCfg.initialAngle = section.getDouble("initial_angle"); } catch (...) {}
                }
                settings_.servos[servoName] = servoCfg;
            }

            // [temperature_sensor <name>]
            else if (name.substr(0, 19) == "temperature_sensor ") {
                std::string sensorName = name.substr(19);
                auto start = sensorName.find_first_not_of(" \t");
                auto end = sensorName.find_last_not_of(" \t");
                if (start != std::string::npos) {
                    sensorName = sensorName.substr(start, end - start + 1);
                }
                KlippySettings::TemperatureSensorConfig sensorCfg;
                sensorCfg.sensorType = section.get("sensor_type", "NTC 100K");
                sensorCfg.sensorPin = section.get("sensor_pin", "");
                if (section.has("min_temp")) {
                    try { sensorCfg.minTemp = section.getDouble("min_temp"); } catch (...) {}
                }
                if (section.has("max_temp")) {
                    try { sensorCfg.maxTemp = section.getDouble("max_temp"); } catch (...) {}
                }
                settings_.temperatureSensors[sensorName] = sensorCfg;
            }

            // [thermistor <name>] — defines reusable thermistor parameters
            else if (name.substr(0, 11) == "thermistor ") {
                std::string thermName = name.substr(11);
                auto start = thermName.find_first_not_of(" \t");
                auto end = thermName.find_last_not_of(" \t");
                if (start != std::string::npos) {
                    thermName = thermName.substr(start, end - start + 1);
                }
                KlippySettings::ThermistorConfig thermCfg;
                if (section.has("pullup_resistor")) {
                    try { thermCfg.pullupResistor = section.getDouble("pullup_resistor"); } catch (...) {}
                }
                if (section.has("reference_voltage")) {
                    try { thermCfg.referenceVoltage = section.getDouble("reference_voltage"); } catch (...) {}
                }
                if (section.has("adc_max")) {
                    try { thermCfg.adcMax = section.getDouble("adc_max"); } catch (...) {}
                }
                if (section.has("resistance_at_25c")) {
                    try { thermCfg.resistanceAt25C = section.getDouble("resistance_at_25c"); } catch (...) {}
                }
                if (section.has("beta")) {
                    try { thermCfg.beta = section.getDouble("beta"); } catch (...) {}
                }
                // Parse calibration table: "temp1,res1;temp2,res2;..."
                if (section.has("calibration_points")) {
                    std::string calStr = section.get("calibration_points", "");
                    std::stringstream ss(calStr);
                    std::string pair;
                    while (std::getline(ss, pair, ';')) {
                        auto comma = pair.find(',');
                        if (comma != std::string::npos) {
                            try {
                                double temp = std::stod(pair.substr(0, comma));
                                double res = std::stod(pair.substr(comma + 1));
                                thermCfg.calibrationTable.emplace_back(temp, res);
                            } catch (...) {}
                        }
                    }
                }
                settings_.thermistors[thermName] = thermCfg;
            }

            // [thermocouple <name>] — defines reusable thermocouple parameters
            else if (name.substr(0, 13) == "thermocouple ") {
                std::string tcName = name.substr(13);
                auto start = tcName.find_first_not_of(" \t");
                auto end = tcName.find_last_not_of(" \t");
                if (start != std::string::npos) {
                    tcName = tcName.substr(start, end - start + 1);
                }
                KlippySettings::ThermocoupleConfig tcCfg;
                tcCfg.type = section.get("type", "K");
                tcCfg.spiBus = section.get("spi_bus", "");
                tcCfg.csPin = section.get("cs_pin", "");
                settings_.thermocouples[tcName] = tcCfg;
            }

            // [rtd <name>] — defines reusable RTD parameters
            else if (name.substr(0, 4) == "rtd ") {
                std::string rtdName = name.substr(4);
                auto start = rtdName.find_first_not_of(" \t");
                auto end = rtdName.find_last_not_of(" \t");
                if (start != std::string::npos) {
                    rtdName = rtdName.substr(start, end - start + 1);
                }
                KlippySettings::RtdConfig rtdCfg;
                if (section.has("nominal_resistance")) {
                    try { rtdCfg.nominalResistance = section.getDouble("nominal_resistance"); } catch (...) {}
                }
                if (section.has("alpha")) {
                    try { rtdCfg.alpha = section.getDouble("alpha"); } catch (...) {}
                }
                if (section.has("reference_resistor")) {
                    try { rtdCfg.referenceResistor = section.getDouble("reference_resistor"); } catch (...) {}
                }
                if (section.has("adc_max")) {
                    try { rtdCfg.adcMax = section.getDouble("adc_max"); } catch (...) {}
                }
                if (section.has("reference_voltage")) {
                    try { rtdCfg.referenceVoltage = section.getDouble("reference_voltage"); } catch (...) {}
                }
                settings_.rtds[rtdName] = rtdCfg;
            }

            // [rotary_delta] — rotary delta printer geometry
            else if (name == "rotary_delta") {
                if (section.has("upper_arm_length")) {
                    try { settings_.rotaryDeltaGeometry.upperArmLength = section.getDouble("upper_arm_length"); } catch (...) {}
                }
                if (section.has("forearm_length")) {
                    try { settings_.rotaryDeltaGeometry.forearmLength = section.getDouble("forearm_length"); } catch (...) {}
                }
                if (section.has("base_radius")) {
                    try { settings_.rotaryDeltaGeometry.baseRadius = section.getDouble("base_radius"); } catch (...) {}
                }
                if (section.has("effector_radius")) {
                    try { settings_.rotaryDeltaGeometry.effectorRadius = section.getDouble("effector_radius"); } catch (...) {}
                }
                if (section.has("base_height")) {
                    try { settings_.rotaryDeltaGeometry.baseHeight = section.getDouble("base_height"); } catch (...) {}
                }
                if (section.has("tower_a_angle")) {
                    try { settings_.rotaryDeltaGeometry.towerAngleA = section.getDouble("tower_a_angle"); } catch (...) {}
                }
                if (section.has("tower_b_angle")) {
                    try { settings_.rotaryDeltaGeometry.towerAngleB = section.getDouble("tower_b_angle"); } catch (...) {}
                }
                if (section.has("tower_c_angle")) {
                    try { settings_.rotaryDeltaGeometry.towerAngleC = section.getDouble("tower_c_angle"); } catch (...) {}
                }
                // Endstop angle adjustments
                if (section.has("angle_a")) {
                    try { settings_.rotaryDeltaEndstopAdjust.adjA = section.getDouble("angle_a") * M_PI / 180.0; } catch (...) {}
                }
                if (section.has("angle_b")) {
                    try { settings_.rotaryDeltaEndstopAdjust.adjB = section.getDouble("angle_b") * M_PI / 180.0; } catch (...) {}
                }
                if (section.has("angle_c")) {
                    try { settings_.rotaryDeltaEndstopAdjust.adjC = section.getDouble("angle_c") * M_PI / 180.0; } catch (...) {}
                }
            }

            // [polar] — polar printer configuration
            else if (name == "polar") {
                if (section.has("max_radius")) {
                    try { settings_.polarConfig.maxRadius = section.getDouble("max_radius"); } catch (...) {}
                }
                if (section.has("max_angle")) {
                    try { settings_.polarConfig.maxAngle = section.getDouble("max_angle"); } catch (...) {}
                }
                if (section.has("continuous_rotation")) {
                    std::string val = section.get("continuous_rotation", "0");
                    settings_.polarConfig.continuousRotation = (val == "1" || val == "true" || val == "True");
                }
            }

            // [winch] — winch/cable printer configuration
            else if (name == "winch") {
                if (section.has("anchor_radius")) {
                    try { settings_.winchConfig.anchorRadius = section.getDouble("anchor_radius"); } catch (...) {}
                }
                if (section.has("anchor_height")) {
                    try { settings_.winchConfig.anchorHeight = section.getDouble("anchor_height"); } catch (...) {}
                }
                if (section.has("anchor_count")) {
                    try { settings_.winchConfig.anchorCount = section.getInt("anchor_count"); } catch (...) {}
                }
            }

            // [temperature_fan <name>]
            else if (name.substr(0, 16) == "temperature_fan ") {
                std::string fanName = name.substr(16);
                auto start = fanName.find_first_not_of(" \t");
                auto end = fanName.find_last_not_of(" \t");
                if (start != std::string::npos) {
                    fanName = fanName.substr(start, end - start + 1);
                }
                KlippySettings::TemperatureFanConfig tfanCfg;
                tfanCfg.pin = section.get("pin", "");
                tfanCfg.sensorType = section.get("sensor_type", "NTC 100K");
                tfanCfg.sensorPin = section.get("sensor_pin", "");
                if (section.has("max_power")) {
                    try { tfanCfg.maxPower = section.getDouble("max_power"); } catch (...) {}
                }
                if (section.has("target_temp")) {
                    try { tfanCfg.targetTemp = section.getDouble("target_temp"); } catch (...) {}
                }
                if (section.has("min_temp")) {
                    try { tfanCfg.minTemp = section.getDouble("min_temp"); } catch (...) {}
                }
                if (section.has("max_temp")) {
                    try { tfanCfg.maxTemp = section.getDouble("max_temp"); } catch (...) {}
                }
                settings_.temperatureFans[fanName] = tfanCfg;
            }

            // [heater_fan] or [heater_fan <name>]
            else if (name == "heater_fan" || name.substr(0, 11) == "heater_fan ") {
                std::string fanName = (name.size() > 11) ? name.substr(11) : "heater_fan";
                auto start = fanName.find_first_not_of(" \t");
                auto end = fanName.find_last_not_of(" \t");
                if (start != std::string::npos) {
                    fanName = fanName.substr(start, end - start + 1);
                }
                KlippySettings::HeaterFanConfig hfanCfg;
                hfanCfg.pin = section.get("pin", "");
                if (section.has("max_power")) {
                    try { hfanCfg.maxPower = section.getDouble("max_power"); } catch (...) {}
                }
                if (section.has("heater")) {
                    hfanCfg.heater = section.get("heater", "extruder");
                }
                if (section.has("heater_temp")) {
                    try { hfanCfg.heaterTemp = section.getDouble("heater_temp"); } catch (...) {}
                }
                settings_.heaterFans[fanName] = hfanCfg;
            }

            // [controller_fan] or [controller_fan <name>]
            else if (name == "controller_fan" || name.substr(0, 15) == "controller_fan ") {
                std::string fanName = (name.size() > 15) ? name.substr(15) : "controller_fan";
                auto start = fanName.find_first_not_of(" \t");
                auto end = fanName.find_last_not_of(" \t");
                if (start != std::string::npos) {
                    fanName = fanName.substr(start, end - start + 1);
                }
                KlippySettings::ControllerFanConfig cfanCfg;
                cfanCfg.pin = section.get("pin", "");
                if (section.has("max_power")) {
                    try { cfanCfg.maxPower = section.getDouble("max_power"); } catch (...) {}
                }
                if (section.has("idle_speed")) {
                    try { cfanCfg.idleSpeed = section.getDouble("idle_speed"); } catch (...) {}
                }
                if (section.has("idle_timeout")) {
                    try { cfanCfg.idleTimeout = section.getDouble("idle_timeout"); } catch (...) {}
                }
                settings_.controllerFans[fanName] = cfanCfg;
            }

            // [tmc2209 <stepper>], [tmc2208 <stepper>], [tmc5160 <stepper>],
            // [tmc2130 <stepper>], [tmc2660 <stepper>], [tmc2240 <stepper>]
            else if (name.substr(0, 8) == "tmc2209 " || name.substr(0, 8) == "tmc2208 " ||
                     name.substr(0, 8) == "tmc5160 " || name.substr(0, 8) == "tmc2130 " ||
                     name.substr(0, 8) == "tmc2660 " || name.substr(0, 8) == "tmc2240 ") {
                std::string driverType = name.substr(0, 7);
                std::string stepper = name.substr(8);
                auto start = stepper.find_first_not_of(" \t");
                auto end = stepper.find_last_not_of(" \t");
                if (start != std::string::npos) {
                    stepper = stepper.substr(start, end - start + 1);
                }
                KlippySettings::TmcSectionConfig tmcCfg;
                tmcCfg.driverType = driverType;
                tmcCfg.stepper = stepper;
                tmcCfg.uartPin = section.get("uart_pin", "");
                tmcCfg.spiBus = section.get("spi_bus", "");
                tmcCfg.csPin = section.get("cs_pin", "");
                if (section.has("run_current")) {
                    try { tmcCfg.runCurrent = section.getDouble("run_current"); } catch (...) {}
                }
                if (section.has("hold_current")) {
                    try { tmcCfg.holdCurrent = section.getDouble("hold_current"); } catch (...) {}
                }
                if (section.has("stealthchop_threshold")) {
                    try { tmcCfg.stealthchopThreshold = section.getDouble("stealthchop_threshold"); } catch (...) {}
                }
                if (section.has("interpolate")) {
                    tmcCfg.interpolate = section.get("interpolate", "False") == "True";
                }
                if (section.has("uart_address")) {
                    try { tmcCfg.uartAddress = static_cast<int>(section.getInt("uart_address")); } catch (...) {}
                }
                // Advanced TMC features
                if (section.has("stealthchop")) {
                    tmcCfg.stealthchop = section.get("stealthchop", "0") == "1" ||
                                         section.get("stealthchop", "False") == "True";
                }
                if (section.has("spreadcycle_threshold")) {
                    try { tmcCfg.spreadCycleThreshold = section.getDouble("spreadcycle_threshold"); } catch (...) {}
                }
                if (section.has("chopper_timing")) {
                    try { tmcCfg.chopperTiming = static_cast<int>(section.getInt("chopper_timing")); } catch (...) {}
                }
                if (section.has("coolstep_threshold")) {
                    try { tmcCfg.coolstepThreshold = section.getDouble("coolstep_threshold"); } catch (...) {}
                }
                if (section.has("stallguard")) {
                    tmcCfg.stallguard = section.get("stallguard", "0") == "1" ||
                                        section.get("stallguard", "False") == "True";
                }
                if (section.has("stallguard_threshold")) {
                    try { tmcCfg.stallguardThreshold = section.getDouble("stallguard_threshold"); } catch (...) {}
                }
                if (section.has("microsteps")) {
                    try { tmcCfg.microsteps = static_cast<int>(section.getInt("microsteps")); } catch (...) {}
                }
                if (section.has("multi_homing")) {
                    tmcCfg.multiHoming = section.get("multi_homing", "0") == "1" ||
                                         section.get("multi_homing", "False") == "True";
                }
                if (section.has("home_current")) {
                    try { tmcCfg.homeCurrent = section.getDouble("home_current"); } catch (...) {}
                }
                settings_.tmcDrivers[stepper] = tmcCfg;
            }

            // [adxl345]
            else if (name == "adxl345") {
                settings_.adxl345Configured = true;
                settings_.adxl345SpiBus = section.get("spi_bus", "");
                settings_.adxl345CsPin = section.get("cs_pin", "");
                if (section.has("rate")) {
                    try { settings_.adxl345Rate = static_cast<int>(section.getInt("rate")); } catch (...) {}
                }
                if (section.has("axes_map")) {
                    settings_.adxl345AxesMap = section.get("axes_map", "xyz");
                }
            }

            // [tsl1401cl_filament_width_sensor]
            else if (name == "tsl1401cl_filament_width_sensor") {
                settings_.tsl1401clConfig.configured = true;
                settings_.tsl1401clConfig.sensorPin = section.get("sensor_pin", "");
                if (section.has("nominal_width")) {
                    try { settings_.tsl1401clConfig.nominalWidth = section.getDouble("nominal_width"); } catch (...) {}
                }
                if (section.has("tolerance")) {
                    try { settings_.tsl1401clConfig.tolerance = section.getDouble("tolerance"); } catch (...) {}
                }
                if (section.has("min_width")) {
                    try { settings_.tsl1401clConfig.minWidth = section.getDouble("min_width"); } catch (...) {}
                }
                if (section.has("max_width")) {
                    try { settings_.tsl1401clConfig.maxWidth = section.getDouble("max_width"); } catch (...) {}
                }
                if (section.has("pixel_count")) {
                    try { settings_.tsl1401clConfig.pixelCount = static_cast<int>(section.getInt("pixel_count")); } catch (...) {}
                }
                if (section.has("pixel_spacing")) {
                    try { settings_.tsl1401clConfig.pixelSpacing = section.getDouble("pixel_spacing"); } catch (...) {}
                }
            }

            // [input_shaper]
            else if (name == "input_shaper") {
                if (section.has("shaper_freq_x")) {
                    try { settings_.inputShaperFreqX = section.getDouble("shaper_freq_x"); } catch (...) {}
                }
                if (section.has("shaper_freq_y")) {
                    try { settings_.inputShaperFreqY = section.getDouble("shaper_freq_y"); } catch (...) {}
                }
                if (section.has("shaper_type_x")) {
                    settings_.inputShaperTypeX = section.get("shaper_type_x", "ei");
                }
                if (section.has("shaper_type_y")) {
                    settings_.inputShaperTypeY = section.get("shaper_type_y", "ei");
                }
                if (section.has("damping_ratio_x")) {
                    try { settings_.inputShaperDampingX = section.getDouble("damping_ratio_x"); } catch (...) {}
                }
                if (section.has("damping_ratio_y")) {
                    try { settings_.inputShaperDampingY = section.getDouble("damping_ratio_y"); } catch (...) {}
                }
            }

            // [autotune] — configures the PID autotuning method
            // All methods are delegated to the Tether autotuning framework.
            else if (name == "autotune") {
                if (section.has("method")) {
                    settings_.pidAutotuneMethod = section.get("method", "relay_feedback");
                }
                if (section.has("form")) {
                    settings_.pidAutotuneForm = section.get("form", "pid");
                }
                if (section.has("lambda")) {
                    try { settings_.pidAutotuneLambda = section.getDouble("lambda"); } catch (...) {}
                }
            }

            // [skew_correction]
            else if (name == "skew_correction") {
                if (section.has("xy_skew")) {
                    try { settings_.skewParams.xy = section.getDouble("xy_skew"); } catch (...) {}
                }
                if (section.has("xz_skew")) {
                    try { settings_.skewParams.xz = section.getDouble("xz_skew"); } catch (...) {}
                }
                if (section.has("yz_skew")) {
                    try { settings_.skewParams.yz = section.getDouble("yz_skew"); } catch (...) {}
                }
            }

            // [z_tilt]
            else if (name == "z_tilt") {
                settings_.zTiltEnabled = true;
                if (section.has("z_positions")) {
                    settings_.zTiltPositions = section.get("z_positions", "");
                }
                if (section.has("retries")) {
                    try { settings_.zTiltRetries = static_cast<int>(section.getInt("retries")); } catch (...) {}
                }
                if (section.has("retry_tolerance")) {
                    try { settings_.zTiltRetryTolerance = section.getDouble("retry_tolerance"); } catch (...) {}
                }
            }

            // [quad_gantry_level]
            else if (name == "quad_gantry_level") {
                settings_.qglEnabled = true;
                if (section.has("z_positions")) {
                    settings_.qglPositions = section.get("z_positions", "");
                }
                if (section.has("retries")) {
                    try { settings_.qglRetries = static_cast<int>(section.getInt("retries")); } catch (...) {}
                }
            }

            // [bed_screws]
            else if (name == "bed_screws") {
                settings_.bedScrewsEnabled = true;
                if (section.has("screws")) {
                    settings_.bedScrewsList = section.get("screws", "");
                }
                if (section.has("probe_speed")) {
                    try { settings_.bedScrewsProbeSpeed = section.getDouble("probe_speed"); } catch (...) {}
                }
            }

            // [screws_tilt_adjust]
            else if (name == "screws_tilt_adjust") {
                settings_.screwsTiltEnabled = true;
                if (section.has("screws")) {
                    settings_.screwsTiltList = section.get("screws", "");
                }
                if (section.has("screw_thread")) {
                    settings_.screwsTiltThread = section.get("screw_thread", "CW-M3");
                }
                if (section.has("horizontal_move_z")) {
                    try { settings_.screwsTiltHorizontalZ = section.getDouble("horizontal_move_z"); } catch (...) {}
                }
            }

            // [gcode_macro <name>]
            if (name.substr(0, 12) == "gcode_macro ") {
                std::string macroName = name.substr(12);
                // Trim whitespace
                auto start = macroName.find_first_not_of(" \t");
                auto end = macroName.find_last_not_of(" \t");
                if (start != std::string::npos) {
                    macroName = macroName.substr(start, end - start + 1);
                }
                GcodeMacro macro;
                macro.name = macroName;
                macro.gcode = section.get("gcode", "");
                macro.description = section.get("description", "G-Code macro");
                // Register with the macro registry
                macros_->registerMacro(macro);
                // Also register as a printer object
                auto obj = std::make_shared<GcodeMacroObject>(macro.name, macros_);
                server_.registerObject(obj);
            }

            // [delayed_gcode <name>]
            else if (name.substr(0, 13) == "delayed_gcode ") {
                std::string dgName = name.substr(13);
                auto start = dgName.find_first_not_of(" \t");
                auto end = dgName.find_last_not_of(" \t");
                if (start != std::string::npos) {
                    dgName = dgName.substr(start, end - start + 1);
                }
                DelayedGcode dg;
                dg.gcode = section.get("gcode", "");
                try { dg.delay = std::stod(section.get("initial_duration", "0")); } catch (...) {}
                dg.enabled = false;
                delayedGcodes_[dgName] = dg;
            }

            // [firmware_retraction]
            else if (name == "firmware_retraction") {
                FirmwareRetractionParams params;
                try {
                    params.retractLength = std::stod(section.get("retract_length", "0"));
                    params.retractSpeed = std::stod(section.get("retract_speed", "20"));
                    params.unretractLength = std::stod(section.get("unretract_extra_length", "0"));
                    params.unretractSpeed = std::stod(section.get("unretract_speed", "10"));
                    params.zHop = std::stod(section.get("z_hop", "0"));
                } catch (...) {}
                if (firmwareRetraction_) {
                    firmwareRetraction_->setParams(params);
                }
            }

            // [exclude_object]
            else if (name == "exclude_object") {
                excludeObjectEnabled_ = true;
            }

            // [save_variables]
            else if (name == "save_variables") {
                saveVariablesEnabled_ = true;
            }

            // [force_move]
            else if (name == "force_move") {
                forceMoveEnabled_ = section.get("enable_force_move", "false") == "true";
                if (forceMoveObj_) forceMoveObj_->setEnableForceMove(forceMoveEnabled_);
            }

            // [homing_override]
            else if (name.substr(0, 16) == "homing_override ") {
                std::string hoName = name.substr(16);
                homingOverrides_[hoName] = section.get("gcode", "");
            }

            // [endstop_phase <stepper>]
            else if (name.substr(0, 14) == "endstop_phase ") {
                std::string stepper = name.substr(14);
                auto start = stepper.find_first_not_of(" \t");
                auto end = stepper.find_last_not_of(" \t");
                if (start != std::string::npos) {
                    stepper = stepper.substr(start, end - start + 1);
                }
                EndstopPhaseState eps;
                try { eps.endstopAlignTolerance = std::stoi(section.get("endstop_align_tolerance", "0")); } catch (...) {}
                endstopPhases_[stepper] = eps;
            }

            // [menu <name>]
            else if (name.substr(0, 5) == "menu ") {
                std::string menuName = name.substr(5);
                menuDefinitions_[menuName] = section.get("name", menuName);
            }

            // [palette2]
            else if (name == "palette2") {
                palette2Configured_ = true;
                if (palette2Obj_) palette2Obj_->setConnected(true);
            }
        }
    }

    // ------------------------------------------------------------------
    // Settings persistence
    // ------------------------------------------------------------------

    void saveSettingsToFile() {
        std::ofstream f(config_.settingsPath);
        if (!f) return;
        f << "# Tether Klippy settings (M500)\n";
        f << "[stepper]\n";
        for (const auto& [axis, steps] : settings_.stepsPerMm)
            f << "steps_per_mm_" << axis << " = " << steps << "\n";
        for (const auto& [axis, ms] : settings_.microstepping)
            f << "microsteps_" << axis << " = " << ms << "\n";
        for (const auto& [axis, cur] : settings_.stepperCurrent)
            f << "current_" << axis << " = " << cur << "\n";
        f << "[motion]\n";
        f << "acceleration = " << settings_.acceleration << "\n";
        f << "travel_acceleration = " << settings_.travelAcceleration << "\n";
        f << "jerk = " << settings_.jerk << "\n";
        f << "[offsets]\n";
        for (const auto& [axis, off] : settings_.homeOffset)
            f << "home_offset_" << axis << " = " << off << "\n";
        f << "probe_offset = " << settings_.probeOffset << "\n";
        f << "[filament]\n";
        f << "diameter = " << settings_.filamentDiameter << "\n";
        f << "[pid]\n";
        f << "hotend_kp = " << settings_.hotendKp << "\n";
        f << "hotend_ki = " << settings_.hotendKi << "\n";
        f << "hotend_kd = " << settings_.hotendKd << "\n";
        f << "bed_kp = " << settings_.bedKp << "\n";
        f << "bed_ki = " << settings_.bedKi << "\n";
        f << "bed_kd = " << settings_.bedKd << "\n";
    }

    void loadSettingsFromFile() {
        std::ifstream f(config_.settingsPath);
        if (!f) return;
        std::string line;
        while (std::getline(f, line)) {
            // Simple key = value parsing
            auto eqPos = line.find('=');
            if (eqPos == std::string::npos) continue;
            std::string key = line.substr(0, eqPos);
            std::string val = line.substr(eqPos + 1);
            // Trim whitespace
            while (!key.empty() && std::isspace(key.back())) key.pop_back();
            while (!key.empty() && std::isspace(key.front())) key.erase(0, 1);
            while (!val.empty() && std::isspace(val.back())) val.pop_back();
            while (!val.empty() && std::isspace(val.front())) val.erase(0, 1);
            if (key.empty() || key[0] == '#') continue;

            // Parse known keys
            if (key == "acceleration") settings_.acceleration = std::stod(val);
            else if (key == "travel_acceleration") settings_.travelAcceleration = std::stod(val);
            else if (key == "jerk") settings_.jerk = std::stod(val);
            else if (key == "probe_offset") {
                settings_.probeOffset = std::stod(val);
                if (probeObj_) probeObj_->setZOffset(settings_.probeOffset);
            }
            else if (key == "diameter") settings_.filamentDiameter = std::stod(val);
            else if (key == "hotend_kp") settings_.hotendKp = std::stod(val);
            else if (key == "hotend_ki") settings_.hotendKi = std::stod(val);
            else if (key == "hotend_kd") settings_.hotendKd = std::stod(val);
            else if (key == "bed_kp") settings_.bedKp = std::stod(val);
            else if (key == "bed_ki") settings_.bedKi = std::stod(val);
            else if (key == "bed_kd") settings_.bedKd = std::stod(val);
            // Axis-specific keys
            else if (key.find("steps_per_mm_") == 0) {
                std::string axis = key.substr(13);
                settings_.stepsPerMm[axis] = std::stod(val);
            }
            else if (key.find("microsteps_") == 0) {
                std::string axis = key.substr(11);
                settings_.microstepping[axis] = std::stoi(val);
            }
            else if (key.find("current_") == 0) {
                std::string axis = key.substr(8);
                settings_.stepperCurrent[axis] = std::stod(val);
            }
            else if (key.find("home_offset_") == 0) {
                std::string axis = key.substr(12);
                settings_.homeOffset[axis] = std::stod(val);
            }
        }
    }

    std::string reportSettings() const {
        std::ostringstream ss;
        ss << "; Tether Klippy settings (M503)\n";
        ss << "M92 X" << settings_.stepsPerMm.at("x")
           << " Y" << settings_.stepsPerMm.at("y")
           << " Z" << settings_.stepsPerMm.at("z")
           << " E" << settings_.stepsPerMm.at("e") << "\n";
        ss << "M203 X" << settings_.maxFeedrate.at("x")
           << " Y" << settings_.maxFeedrate.at("y")
           << " Z" << settings_.maxFeedrate.at("z")
           << " E" << settings_.maxFeedrate.at("e") << "\n";
        ss << "M204 P" << settings_.acceleration
           << " T" << settings_.travelAcceleration << "\n";
        ss << "M206 X" << settings_.homeOffset.at("x")
           << " Y" << settings_.homeOffset.at("y")
           << " Z" << settings_.homeOffset.at("z") << "\n";
        ss << "M851 Z" << settings_.probeOffset << "\n";
        ss << "M200 D" << settings_.filamentDiameter << "\n";
        ss << "M301 P" << settings_.hotendKp
           << " I" << settings_.hotendKi
           << " D" << settings_.hotendKd << "\n";
        ss << "M304 P" << settings_.bedKp
           << " I" << settings_.bedKi
           << " D" << settings_.bedKd << "\n";
        return ss.str();
    }
