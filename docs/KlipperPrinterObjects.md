Klipper Printer Object Reference
=================================

This document provides a complete reference for all printer objects
exposed by Tether's Klipper layer via the ``objects/query`` and
``objects/subscribe`` endpoints.

.. contents::
   :depth: 2
   :local:

Overview
--------

Tether's Klipper layer defines **79 printer object classes** that expose
printer state to Moonraker frontends. Objects are queried via the
``printer/objects/query`` endpoint with a list of requested objects and
optional field filters.

Core Objects
------------

===========================  ===========================================  ==================================
Object                       Description                                 Fields
===========================  ===========================================  ==================================
``webhooks``                 Webhook state (printer state)               state, state_message
``gcode_move``               G-code move state                           position, speed, extrude_factor, etc.
``toolhead``                 Toolhead state                              position, status, homed_axes, etc.
``configfile``               Configuration status                        load_state, config_crc
``print_stats``              Print statistics                            filename, total_duration, print_duration, filament_used, state, message, info
``virtual_sdcard``           Virtual SD card                             progress, is_active, file_position, file_size
``display_status``           Display status                              progress, message
``pause_resume``             Pause/resume state                          is_paused
``gcode``                    G-code executor state                       commands, info, config_commands, move_gcode_position
===========================  ===========================================  ==================================

Heater Objects
--------------

===========================  ===========================================  ==================================
Object                       Description                                 Fields
===========================  ===========================================  ==================================
``extruder``                 Primary extruder                            temperature, target, power, pressure_advance, smooth_time
``heater_bed``               Heated bed                                  temperature, target, power
``heaters``                  All heaters summary                         available_heaters, available_sensors
``heater_generic``           Generic heater                              temperature, target, power
``temperature_sensor``       Temperature sensor                          temperature, measured_min_temp, measured_max_temp
``temperature_fan``          Temperature-controlled fan                  temperature, target, speed
``temperature_probe``        Probe temperature sensor                    temperature
===========================  ===========================================  ==================================

Fan Objects
-----------

===========================  ===========================================  ==================================
Object                       Description                                 Fields
===========================  ===========================================  ==================================
``fan``                      Primary part cooling fan                    speed
``controller_fan``           Controller fan                              speed, rpm
``heater_fan``               Heater fan                                  speed
``fan_generic``              Generic fan                                 speed
===========================  ===========================================  ==================================

Motion Objects
--------------

===========================  ===========================================  ==================================
Object                       Description                                 Fields
===========================  ===========================================  ==================================
``mcu``                      MCU state                                   mcu_version, mcu_build_versions, last_stats
``motion_report``            Motion report                               live_position, live_velocity, live_extruder_velocity
``idle_timeout``             Idle timeout state                          state, printing_time, timeout
``stepper_enable``           Stepper enable state                        steppers
``force_move``               Force move state                            enable_force_move
``dual_carriage``            Dual carriage state                         carriage, mode
``extruder_stepper``         Extruder stepper                            motion_queue, pressure_advance
``manual_stepper``           Manual stepper                              position, speed
===========================  ===========================================  ==================================

Probe and Bed Leveling Objects
------------------------------

===========================  ===========================================  ==================================
Object                       Description                                 Fields
===========================  ===========================================  ==================================
``probe``                    Probe state                                 last_query, last_z_result
``bed_mesh``                 Bed mesh state                              profile_name, mesh_min, mesh_max, probed_matrix, matrix
``bed_tilt``                 Bed tilt state                              x_adjust, y_adjust, z_adjust, applied
``z_tilt``                   Z tilt state                               applied, z_positions
``quad_gantry_level``        QGL state                                  applied, z_positions
``screws_tilt_adjust``       Screws tilt state                           error, max_base, base, adjusted
``bed_screws``               Bed screws state                            state, fast_state, adjusted_screws
``delta_calibrate``          Delta calibration state                    radius, positions, applied
``safe_z_home``              Safe Z home config                         home_xy_position, z_hop, z_hop_speed, xy_home_speed, move_to_previous
``manual_probe``             Manual probe state                         is_active, z_position, z_position_lower, z_position_upper
``smart_effector``           Smart Effector state                       sensitivity
``endstop_phase``            Endstop phase state                         mcu_phase_offset, phase_found_position, endstop_align_tolerance
``bltouch``                  BLTouch state                              last_query, last_z_result
===========================  ===========================================  ==================================

Sensor Objects
--------------

===========================  ===========================================  ==================================
Object                       Description                                 Fields
===========================  ===========================================  ==================================
``filament_switch_sensor``   Filament switch sensor                      filament_detected, enabled
``filament_motion_sensor``   Filament motion sensor                      filament_detected, distance, enabled
``load_cell``                Load cell sensor                            load, tare_value, threshold, enabled
``canbus_stats``             CAN bus statistics                          rx_error, tx_error, bus_state, msg_count
``adc``                      ADC values                                  (per-channel values)
``query_adc``                ADC query                                   (per-channel values)
``angle``                    Angle sensor                                angle, velocity, temperature
===========================  ===========================================  ==================================

LED Objects
-----------

===========================  ===========================================  ==================================
Object                       Description                                 Fields
===========================  ===========================================  ==================================
``led``                      LED strip                                   color_data
``neopixel``                 NeoPixel strip                              color_data, pixel_count
``dotstar``                  DotStar strip                               color_data, pixel_count
``pwm_cycle_time``           PWM cycle time output                       value, cycle_time
``pwm_tool``                 PWM tool                                    value, cycle_time
===========================  ===========================================  ==================================

TMC Driver Objects
------------------

===========================  ===========================================  ==================================
Object                       Description                                 Fields
===========================  ===========================================  ==================================
``tmc_driver``               TMC driver state                            mc_phase_offset, drv_status, run_current, hold_current
``tmc_uart``                 TMC UART state                              mcu_phase_offset, drv_status, run_current, hold_current
===========================  ===========================================  ==================================

Advanced Objects
----------------

===========================  ===========================================  ==================================
Object                       Description                                 Fields
===========================  ===========================================  ==================================
``input_shaper``             Input shaper config                         shaper_type_x, shaper_freq_x, shaper_type_y, shaper_freq_y
``pressure_advance``         Pressure advance state                      pressure_advance, smooth_time
``skew_correction``          Skew correction state                       x_factor, y_factor, xy_factor, max_skew, current_skew
``exclude_object``           Exclude object state                        objects, excluded_objects, current_object
``firmware_retraction``      Firmware retraction                         retract_length, retract_speed, unretract_extra_length, unretract_speed, z_hop, is_retracted
``z_thermal_adjust``         Z thermal adjust                            temperature, scale, enabled
``resonance_tester``         Resonance tester                            min_freq, max_freq, accel_per_hz, is_measuring
``adxl345``                  ADXL345 accelerometer                       axes, data
===========================  ===========================================  ==================================

Output and Pin Objects
----------------------

===========================  ===========================================  ==================================
Object                       Description                                 Fields
===========================  ===========================================  ==================================
``output_pin``               Output pin                                  value
``servo``                    Servo                                       angle, width
``multi_pin``                Multi-pin                                   pins, value
``button``                   Button                                      state
===========================  ===========================================  ==================================

Misc Objects
------------

===========================  ===========================================  ==================================
Object                       Description                                 Fields
===========================  ===========================================  ==================================
``gcode_macro``              G-code macro (one per macro)                (macro-defined variables)
``menu``                     Menu state                                  enabled, timeout, buttons
``palette2``                 Palette2 state                              connected, loading, remaining, error, filename
``case_light``               Case light                                  brightness
===========================  ===========================================  ==================================

Querying Objects
----------------

Objects can be queried via the UDS endpoint:

.. code-block:: json

    {
      "method": "printer/objects/query",
      "params": {
        "objects": {
          "toolhead": [],
          "extruder": ["temperature", "target"],
          "print_stats": []
        }
      }
    }

Or via the C++ API:

.. code-block:: cpp

    auto status = server.queryObjects({
        {"toolhead", {}},
        {"extruder", {"temperature", "target"}},
        {"print_stats", {}}
    });

Examples
--------

See the **klipper_printer_objects** example for a full demonstration of all
printer objects and their status fields.

Klipper-Specific Scope
----------------------

All printer objects in this module are **Klipper-specific**. They live in
the ``tether::klipper::klippy`` namespace and are designed for 3D printer
control via Moonraker frontends. They have no equivalent in the main Tether
RS274/NGC CNC interpreter, which uses a different state model based on
machine coordinates, tool tables, and work coordinate systems (G54-G59.3).

The bed leveling objects (``bed_mesh``, ``bed_tilt``, ``z_tilt``,
``quad_gantry_level``, ``screws_tilt_adjust``, ``bed_screws``) are
particularly 3D-printer-specific. The underlying ``BedMesh`` class in
``tether/klipper/objects/BedLevel.hpp`` provides 2D mesh interpolation
for Z compensation — a concept that does not apply to CNC machining.
