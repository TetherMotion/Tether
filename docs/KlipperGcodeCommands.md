Klipper G-code Command Reference
================================

This document provides a complete reference for all G-code commands
supported by Tether's Klipper compatibility layer.

.. contents::
   :depth: 2
   :local:

Overview
--------

Tether's Klipper layer supports **84+ extended commands** and **95+ standard
G/M commands**, covering the full range of operations needed for 3D printer
control via Moonraker frontends (Mainsail, Fluidd, etc.).

Standard G-code Commands
------------------------

Motion Commands
~~~~~~~~~~~~~~~

============  =================================================  ==================================
Command       Description                                        Parameters
============  =================================================  ==================================
``G0``        Rapid move                                         X, Y, Z, E, F
``G1``        Linear move                                        X, Y, Z, E, F
``G2``        Clockwise arc move                                 X, Y, Z, E, F, I, J, R
``G3``        Counter-clockwise arc move                         X, Y, Z, E, F, I, J, R
``G4``        Dwell (pause)                                      P (ms), S (seconds)
``G5``        Bezier spline move                                 X, Y, E, F, I, J, P, Q
``G12``       Clean nozzle pattern                               P, S, R, T
``G28``       Home axes                                          X, Y, Z (optional, all if none)
``G29``       Bed leveling (probe)                               --
``G30``       Single probe                                       X, Y, Z, P
``G38``       Probe toward target                                X, Y, Z, E, F
============  =================================================  ==================================

Coordinate System Commands
~~~~~~~~~~~~~~~~~~~~~~~~~~

============  =================================================  ==================================
Command       Description                                        Parameters
============  =================================================  ==================================
``G17``       Select XY arc plane                                --
``G18``       Select XZ arc plane                                --
``G19``       Select YZ arc plane                                --
``G20``       Set units to inches                                --
``G21``       Set units to millimeters                           --
``G53``       Move in machine coordinates                        X, Y, Z, F
``G54-G59.3`` Select coordinate system 0-8                       --
``G60``       Save position to slot                              S (slot 0-9)
``G61``       Restore position from slot                         S (slot 0-9)
``G90``       Set absolute coordinates                           --
``G91``       Set relative coordinates                           --
``G92``       Set position                                       X, Y, Z, E
``G92.1``     Reset position to zero                             --
``G92.2``     Reset position without saving                      --
``G92.3``     Restore last G92 position                          --
============  =================================================  ==================================

Temperature Commands
~~~~~~~~~~~~~~~~~~~~

============  =================================================  ==================================
Command       Description                                        Parameters
============  =================================================  ==================================
``M104``      Set extruder temperature (no wait)                 S (target temp)
``M109``      Set extruder temperature (wait)                    S (target temp), R (wait for cool)
``M140``      Set bed temperature (no wait)                      S (target temp)
``M190``      Set bed temperature (wait)                         S (target temp)
``M116``      Wait for all temperatures                          P, H, C
``M155``      Auto temperature report                            S (interval in seconds)
============  =================================================  ==================================

Fan and Motor Commands
~~~~~~~~~~~~~~~~~~~~~~

============  =================================================  ==================================
Command       Description                                        Parameters
============  =================================================  ==================================
``M106``      Set fan speed                                      S (0-255)
``M107``      Fan off                                            --
``M17``       Enable all steppers                                --
``M18``       Disable all steppers                               --
``M84``       Disable steppers (idle)                            S (timeout)
============  =================================================  ==================================

SD Card Commands
~~~~~~~~~~~~~~~~

============  =================================================  ==================================
Command       Description                                        Parameters
============  =================================================  ==================================
``M20``       List SD files                                      --
``M23``       Select SD file                                     filename
``M24``       Start/resume SD print                              --
``M25``       Pause SD print                                     --
``M26``       Set SD position                                    S (position)
``M27``       Report SD print status                             --
``M524``      Abort SD print                                     --
============  =================================================  ==================================

Information Commands
~~~~~~~~~~~~~~~~~~~~

============  =================================================  ==================================
Command       Description                                        Parameters
============  =================================================  ==================================
``M112``      Emergency stop                                     --
``M114``      Get current position                               --
``M115``      Get firmware info                                  --
``M117``      Set LCD message                                    text
``M118``      Send message to host                               S, P, E
``M119``      Get endstop states                                 --
============  =================================================  ==================================

Extended G-code Commands
------------------------

Bed Leveling Commands
~~~~~~~~~~~~~~~~~~~~~

================================  =================================================  ==================================
Command                           Description                                        Parameters
================================  =================================================  ==================================
``BED_MESH_CALIBRATE``            Probe bed and build mesh                           --
``BED_MESH_CLEAR``                Clear current mesh                                 --
``BED_MESH_MAP``                  Output mesh as map                                 --
``BED_MESH_OFFSET``               Apply XY offset to mesh                            X, Y
``BED_MESH_OUTPUT``               Output mesh data                                   PGP (parameter)
``BED_MESH_PROFILE``              Save/load/remove mesh profile                      SAVE, LOAD, REMOVE, NAME
``BED_SCREWS_ADJUST``             Manual bed screw adjustment                        --
``DELTA_ANALYZE``                 Analyze delta geometry                             CALIBRATE_RADIUS, other delta params
``DELTA_CALIBRATE``               Calibrate delta printer                            --
``PROBE``                         Single probe                                       --
``PROBE_ACCURACY``                Test probe repeatability                           SAMPLES, PROBE_SPEED, RETRACT, etc.
``PROBE_CALIBRATE``               Calibrate probe Z offset                           --
``QUAD_GANTRY_LEVEL``             Level gantry at 4 points                           --
``SCREWS_TILT_ADJUST``            Auto-adjust bed screws                             --
``Z_OFFSET_APPLY_ENDSTOP``        Apply Z offset to endstop                          --
``Z_OFFSET_APPLY_PROBE``          Apply Z offset to probe                            --
``Z_TILT_ADJUST``                 Adjust Z tilt                                      --
================================  =================================================  ==================================

Motion and State Commands
~~~~~~~~~~~~~~~~~~~~~~~~~

================================  =================================================  ==================================
Command                           Description                                        Parameters
================================  =================================================  ==================================
``SAVE_GCODE_STATE``              Save current G-code state                          NAME
``RESTORE_GCODE_STATE``           Restore saved G-code state                         NAME
``SET_GCODE_OFFSET``              Set G-code offset                                  X, Y, Z, E, RESET, ADJUST
``SET_GCODE_POSITION``            Set current position without moving                X, Y, Z, E
``SET_VELOCITY_LIMIT``            Set velocity/accel limits                          ACCEL, VELOCITY, etc.
``SET_IDLE_TIMEOUT``              Set idle timeout                                   TIMEOUT
``SET_POSITION``                  Set stepper position                               X, Y, Z, E
================================  =================================================  ==================================

Extruder Commands
~~~~~~~~~~~~~~~~~

================================  =================================================  ==================================
Command                           Description                                        Parameters
================================  =================================================  ==================================
``ACTIVATE_EXTRUDER``             Switch active extruder                             EXTRUDER
``SET_EXTRUDER_ROTATION_DISTANCE`` Set rotation distance                            EXTRUDER, DISTANCE
``SET_EXTRUDER_STEP_DISTANCE``    Set step distance                                  EXTRUDER, DISTANCE
``SET_PRESSURE_ADVANCE``          Set pressure advance                               ADVANCE, SMOOTH_TIME
``SYNC_EXTRUDER_STEPPER``         Sync extruder to stepper                           EXTRUDER, STEPPER
================================  =================================================  ==================================

Input Shaper Commands
~~~~~~~~~~~~~~~~~~~~~

================================  =================================================  ==================================
Command                           Description                                        Parameters
================================  =================================================  ==================================
``SET_INPUT_SHAPER``              Set input shaper parameters                        SHAPER_TYPE_X/Y, SHAPER_FREQ_X/Y
``TEST_RESONANCES``               Test resonances                                    AXIS, MIN_FREQ, MAX_FREQ, etc.
``SHAPER_CALIBRATE``              Calibrate input shaper                             AXIS, FREQ_START, FREQ_END
================================  =================================================  ==================================

Force Move Commands
~~~~~~~~~~~~~~~~~~~

================================  =================================================  ==================================
Command                           Description                                        Parameters
================================  =================================================  ==================================
``FORCE_MOVE``                    Force stepper move                                 STEPPER, DISTANCE, VELOCITY, ACCEL
``STEPPER_BUZZ``                  Buzz stepper motor                                 STEPPER
``MANUAL_STEPPER``                Manual stepper move                                STEPPER, DISTANCE, SPEED, ACCEL
``SET_KINEMATICS``                Set kinematics type                                KINEMATICS
================================  =================================================  ==================================

Probe and Endstop Commands
~~~~~~~~~~~~~~~~~~~~~~~~~~

================================  =================================================  ==================================
Command                           Description                                        Parameters
================================  =================================================  ==================================
``QUERY_ENDSTOPS``                Query endstop states                               --
``QUERY_PROBE``                   Query probe state                                  --
``ENDSTOP_HOME``                  Configure endstop homing                           STEPPER, POSITION
``ENDSTOP_PHASE``                 Calibrate endstop phase                            STEPPER
``SET_HOME_POSITION``             Set home position                                  AXIS, POSITION
``SET_STEPPER_ENABLE``            Enable/disable stepper                             STEPPER, ENABLE
================================  =================================================  ==================================

Pin and LED Commands
~~~~~~~~~~~~~~~~~~~~

================================  =================================================  ==================================
Command                           Description                                        Parameters
================================  =================================================  ==================================
``SET_PIN``                       Set output pin value                               PIN, VALUE
``SET_PWM_PIN``                   Set PWM pin value                                  PIN, VALUE
``SET_DIGITAL_PIN``               Set digital pin                                    PIN, VALUE
``SET_SERVO``                     Set servo angle                                    PIN, ANGLE, WIDTH
``SET_MULTI_PIN``                 Set multi-pin value                                PIN, VALUE
``SET_LED``                       Set LED color                                      LED, RED, GREEN, BLUE, WHITE, INDEX
``SET_NEOPIXEL``                  Set NeoPixel color                                 LED, RED, GREEN, BLUE, WHITE, INDEX
``SET_DOTSTAR``                   Set DotStar color                                  LED, RED, GREEN, BLUE, WHITE, INDEX
================================  =================================================  ==================================

Heater and Fan Commands
~~~~~~~~~~~~~~~~~~~~~~~

================================  =================================================  ==================================
Command                           Description                                        Parameters
================================  =================================================  ==================================
``SET_HEATER_TEMPERATURE``        Set heater target                                  HEATER, TARGET
``SET_FAN_SPEED``                 Set fan speed                                      FAN, SPEED
``SET_TEMPERATURE_FAN``           Set temperature fan target                         TEMPERATURE_FAN, TARGET
``PID_CALIBRATE``                 Run PID autotune                                   HEATER, TARGET
================================  =================================================  ==================================

TMC Driver Commands
~~~~~~~~~~~~~~~~~~~

================================  =================================================  ==================================
Command                           Description                                        Parameters
================================  =================================================  ==================================
``SET_CURRENT``                   Set stepper current                                STEPPER, CURRENT, HOLD_CURRENT
``SET_TMC_FIELD``                 Set TMC register field                             STEPPER, FIELD, VALUE
``DUMP_TMC``                      Dump TMC registers                                 STEPPER
``INIT_TMC``                      Initialize TMC driver                               STEPPER
================================  =================================================  ==================================

Firmware Retraction Commands
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

================================  =================================================  ==================================
Command                           Description                                        Parameters
================================  =================================================  ==================================
``SET_RETRACTION``                Set retraction parameters                          RETRACT_LENGTH, RETRACT_SPEED,
                                                                                     UNRETRACT_EXTRA_LENGTH,
                                                                                     UNRETRACT_SPEED, Z_HOP
================================  =================================================  ==================================

(Note: ``G10`` and ``G11`` are standard commands for retract/unretract)

Exclude Object Commands
~~~~~~~~~~~~~~~~~~~~~~~

================================  =================================================  ==================================
Command                           Description                                        Parameters
================================  =================================================  ==================================
``EXCLUDE_OBJECT_DEFINE``         Define an excludable object                        NAME, POLYGON, CENTER, RADIUS
``EXCLUDE_OBJECT_START``          Mark start of object                               NAME
``EXCLUDE_OBJECT_END``            Mark end of object                                 NAME
``EXCLUDE_OBJECT``                Exclude a specific object                          NAME
``EXCLUDE_OBJECT_RESET``          Reset all exclude state                            --
================================  =================================================  ==================================

Filament Commands
~~~~~~~~~~~~~~~~~

================================  =================================================  ==================================
Command                           Description                                        Parameters
================================  =================================================  ==================================
``FILAMENT_LOAD``                 Load filament                                      LENGTH, SPEED
``FILAMENT_UNLOAD``               Unload filament                                    LENGTH, SPEED
``FILAMENT_PURGE``                Purge filament                                     LENGTH, SPEED
================================  =================================================  ==================================

Communication Commands
~~~~~~~~~~~~~~~~~~~~~~

================================  =================================================  ==================================
Command                           Description                                        Parameters
================================  =================================================  ==================================
``RESPOND``                       Send response to host                              TYPE (echo/error/command), MSG
``ECHO``                          Echo a message                                     MSG
================================  =================================================  ==================================

Macro and Variable Commands
~~~~~~~~~~~~~~~~~~~~~~~~~~~

================================  =================================================  ==================================
Command                           Description                                        Parameters
================================  =================================================  ==================================
``SET_GCODE_VARIABLE``            Set macro variable                                 MACRO, VARIABLE, VALUE
``SAVE_VARIABLE``                 Save a persistent variable                         VARIABLE, VALUE
``SET_DELAYED_GCODE``             Schedule a delayed G-code                          ID, GCODE
``UPDATE_DELAYED_GCODE``          Update a delayed G-code                            ID, GCODE
================================  =================================================  ==================================

Miscellaneous Commands
~~~~~~~~~~~~~~~~~~~~~~

================================  =================================================  ==================================
Command                           Description                                        Parameters
================================  =================================================  ==================================
``SAVE_CONFIG``                   Save configuration to file                         --
``SET_DISPLAY_GROUP``             Set active display group                           DISPLAY
``SET_PRINT_STATS_INFO``          Set print stats info                               TOTAL_LAYER, CURRENT_LAYER
``SET_DUAL_CARRIAGE``             Set dual carriage mode                             CARRIAGE, MODE
``SET_SKEW``                      Set skew correction                                XY, XZ, YZ
``SET_BUTTON_TEMPLATE``           Configure button template                          BUTTON, PARAM, VALUE
``SET_SMART_EFFECTOR``            Set SmartEffector sensitivity                      SENSITIVITY
``ACCELEROMETER_MEASURE``         Start accelerometer measurement                    CHIP
``ACCELEROMETER_QUERY``           Query accelerometer data                           CHIP
``CALIBRATE_PICOMM``              Calibrate PICAN communication                      --
``QUERY_ADC``                     Query ADC value                                    NAME
================================  =================================================  ==================================

Examples
--------

See the following example programs for comprehensive demonstrations:

1. **klipper_gcode_commands** — All G-code command categories
2. **klipper_bed_leveling** — Complete bed leveling workflow
3. **klipper_moonraker_api** — Full Moonraker API surface
4. **klipper_printer_objects** — Printer object model and status queries
5. **klipper_config_macros** — Config file parsing and G-code macros

Build and run examples:

.. code-block:: bash

   cd /path/to/Tether
   cmake -B build -S .
   cmake --build build --target klipper_gcode_commands
   ./build/bin/klipper_gcode_commands

Feature Limitations Relative to Main Tether Parser
---------------------------------------------------

Tether ships two G-code interpreters:

1. **Klipper G-code executor** (``tether::klipper::klippy::GCodeExecutor``) —
   a lightweight parser designed for 3D printer control via Moonraker
   frontends. It supports the standard RepRap/Klipper G/M command set plus
   50+ Klipper extended commands (``SET_*``, ``BED_MESH_*``, etc.).
2. **Tether RS274/NGC interpreter** (``tether::gcode::GCodeInterpreter``) — a
   full LinuxCNC-compatible interpreter with control flow, variables,
   expressions, tool compensation, and canned cycles.

The Klipper executor is intentionally minimal and does **not** support the
following features that the main Tether interpreter provides:

O-Code Control Flow
~~~~~~~~~~~~~~~~~~~

The Klipper executor has no control flow constructs. The main Tether
interpreter supports:

- ``o<name> sub`` / ``o<name> endsub`` / ``o<name> call`` — subroutines
- ``o<n> if`` / ``o<n> elseif`` / ``o<n> else`` / ``o<n> endif`` — conditionals
- ``o<n> while`` / ``o<n> endwhile`` / ``o<n> do`` / ``o<n> repeat`` — loops
- ``o<n> break`` / ``o<n> continue`` — flow control
- ``M98`` / ``M99`` — Fanuc-style subprogram call/return
- ``M70``–``M73`` — modal state save/restore

Parameters and Expressions
~~~~~~~~~~~~~~~~~~~~~~~~~~

The Klipper executor does not support ``#`` parameters or expression
evaluation. The main Tether interpreter provides:

- Numbered parameters (``#1``–``#30`` local, ``#31``–``#5000`` global,
  ``#5001``–``#5999`` system)
- Named parameters (``#<name>``, ``#<_name>``)
- Full expression evaluation with arithmetic, comparison, and logical
  operators (``**``, ``MOD``, ``EQ``, ``AND``, ``OR``, etc.)
- Mathematical functions (``ABS``, ``SIN``, ``COS``, ``SQRT``, ``ATAN``, etc.)
- Predefined system parameters (probe results, G28 positions, G92 offsets,
  tool table, current position)

Tool Compensation
~~~~~~~~~~~~~~~~~

The Klipper executor has no tool compensation. The main Tether interpreter
supports:

- ``G43`` / ``G43.1`` / ``G43.2`` — tool length offset
- ``G44`` — tool length offset (subtract)
- ``G49`` — cancel tool length offset
- ``G41`` / ``G41.1`` — cutter compensation left
- ``G42`` / ``G42.1`` — cutter compensation right
- ``G40`` — cancel cutter compensation
- Tool table with 256 tools, wear offsets, and lathe orientation
- Gouge detection and corner arc generation

Canned Cycles
~~~~~~~~~~~~~

The Klipper executor accepts ``G81``–``G89`` but does not implement full
canned cycle semantics. The main Tether interpreter provides:

- ``G73`` — high-speed peck drill (chip-break)
- ``G74`` — left-hand tapping (CCW)
- ``G76`` — fine boring (spindle orient, shift)
- ``G81``–``G89`` — full canned cycle set with rigid tapping
- ``G98`` / ``G99`` — retract to initial/R plane
- Spindle orient for boring cycles

Advanced Splines
~~~~~~~~~~~~~~~~

The Klipper executor supports ``G5`` (Bezier spline). The main Tether
interpreter additionally supports:

- ``G5.1`` — quadratic B-spline
- ``G5.2`` / ``G5.3`` — NURBS blocks

Advanced Path Blending
~~~~~~~~~~~~~~~~~~~~~~

The Klipper executor supports ``G64`` with basic tolerance. The main Tether
interpreter provides:

- ``G64`` with ``P`` (tolerance) and ``Q`` (naive CAM tolerance)
- Lookahead velocity planning
- Trochoidal milling (``G12.1``)
- Volumetric compensation (3D error grid with trilinear interpolation)
- Velocity-dependent backlash compensation

Coordinate Rotation & Scaling
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Both the Klipper executor and the main Tether interpreter support
coordinate rotation and scaling via a composed ``CoordinateTransform``
(Eigen-based affine transform).

**G68 — Coordinate System Rotation**

Three syntax variants are supported:

- ``G68 X__ Y__ R__`` — 2D rotation in the active plane (G17=XY about Z,
  G18=ZX about Y, G19=YZ about X). ``X``/``Y`` are the in-plane pivot.
- ``G68 X__ Y__ Z__ A__ B__ C__`` — 3D intrinsic XYZ Euler rotation about
  pivot ``(X, Y, Z)``. ``A``/``B``/``C`` are angles in degrees.
- ``G68 X__ Y__ Z__ I__ J__ K__ R__`` — 3D axis-angle rotation about
  pivot ``(X, Y, Z)``. ``I``/``J``/``K`` is the rotation axis, ``R`` is
  the angle in degrees.

**G69 — Cancel Coordinate Rotation**

Cancels the active G68 rotation and resets all rotation parameters.

**G51 — Scaling**

- ``G51 P<scale>`` — uniform scale factor for all axes
- ``G51 X__ Y__ Z__`` — per-axis scale factors

**G50 — Cancel Scaling**

Resets all scale factors to 1.0.

**G52 — Local Coordinate Offset**

- ``G52 X__ Y__ Z__`` — set local offset (unspecified axes unchanged)
- ``G52`` (no axis words) — reset local offset to zero

**Transform Pipeline**

The full coordinate transform pipeline (applied in order, innermost first):

::

    Program coords
      → G52 local offset
      → G51 scaling
      → G68 rotation (about pivot)
      → G92 offset
      → WCS offset (G54-G59.3)
      → Machine coords

The transform is composed as a single ``Eigen::Affine3d`` matrix and
applied to all motion commands. Position reporting (M114,
``gcode_move.gcode_position``) uses the inverse transform to show
program coordinates.

Execution Modes
~~~~~~~~~~~~~~~

The Klipper executor runs in a simple line-by-line mode. The main Tether
interpreter supports:

- **AUTO** — run program from file
- **MDI** — manual data input (single lines)
- **STEP** — single-step through program
- **VERIFY** — dry run (no motion output)
- Block delete (``/`` prefix)
- Optional stop (``M1`` behavior)

Conversely, the Klipper executor provides many 3D-printer-specific features
that the main Tether interpreter does not:

- Temperature control (``M104``, ``M109``, ``M140``, ``M190``)
- Part cooling fan (``M106``, ``M107``)
- Firmware retract (``G10``, ``G11``, ``M207``, ``M208``)
- Pressure advance (``M900``, ``SET_PRESSURE_ADVANCE``)
- Input shaping (``M593``, ``TEST_RESONANCES``, ``SHAPER_CALIBRATE``)
- Bed mesh leveling (``BED_MESH_*``, ``M420``, ``M421``)
- Delta calibration (``G33``, ``DELTA_CALIBRATE``, ``M665``, ``M666``)
- TMC driver configuration (``M907``–``M914``)
- LED control (``M150``, ``SET_LED``, ``SET_NEOPIXEL``, ``SET_DOTSTAR``)
- Filament handling (``M600``, ``M701``–``M708``)
- 50+ Klipper extended commands (``SET_*``, ``QUERY_*``, etc.)

The two interpreters serve complementary purposes: the Klipper executor is
optimized for 3D printer control, while the Tether RS274/NGC interpreter is
designed for CNC machining with full RS274/NGC compliance.
