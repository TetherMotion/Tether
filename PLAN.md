# Tether GCode Parser — Missing G-Code Extensions

Scope: well-known G-code extensions that the Tether parser does **not**
currently support, based on an audit of `src/gcode/GCodeParser.cpp`,
`src/gcode/GCodeLexer.cpp`, `src/gcode/GCodeVariables.cpp`, the headers under
`include/tether/gcode/`, and the dialect handlers under
`include/tether/motion_planner/` and `include/tether/gcode/`.

> **Note on scope:** This document covers the **Tether G-code parser**
> (`src/gcode/`, `include/tether/gcode/`) — the dialect-agnostic parser that
> handles RS274/NGC, Fanuc, Haas, GRBL, and Marlin dialects. It does **not**
> cover the **Klipper layer** (`src/klipper/`, `include/tether/klipper/`),
> which has its own independent G-code executor
> (`tether/klipper/klippy/GCodeExecutor.hpp`) with 84+ extended and 95+
> standard G/M commands. The Klipper layer's command coverage is tracked
> separately in its test suite (`tests/klipper/`).

Each item is tagged:
- **MISSING** — not recognized by the lexer/parser at all (silently dropped
  or rejected).
- **STUB** — recognized syntactically (token/enum/struct exists) but never
  translated into motion or state; the corresponding handler is a header-only
  declaration with no `.cpp` implementation, or `parseOCode`/`parseGCode`
  returns "not implemented".

Priority tags (for future implementation work):
- **P0** — blocks a primary use case (LinuxCNC / Fanuc / 3D-printer workflows).
- **P1** — commonly expected on at least one major dialect.
- **P2** — niche or vendor-specific.

---

## 1. Fanuc-style macro and cycle extensions

| Feature | Status | Priority | Notes |
|---|---|---|---|
| **G65** (non-modal macro call) | DONE | P0 | Dispatched via `callSubprogram` + `collectMacroArgs`; jumps into `O<num>` subprograms. |
| **G66** (modal macro call) | DONE | P0 | Modal macro re-invokes on axis-word blocks; G67 cancels. G66.1 not implemented. |
| **G67** (modal macro cancel) | DONE | P0 | Clears modal macro state. |
| **G51** (scaling) | DONE | P1 | Implemented via `CoordinateTransform` (Eigen). Per-axis and uniform scaling. G50 cancels. |
| **G50** (max RPM clamp, Fanuc lathe) | DONE | P1 | `G50 S<rpm>` sets `MachineState::maxSpindleSpeed` (clamped in M3/M4); bare G50 still cancels scaling. |
| **G68 / G69** (coordinate system rotation) | DONE | P1 | Full 2D plane rotation + 3D Euler XYZ + axis-angle via `CoordinateTransform`. G69 cancels. |
| **G51.1 / G50.1** (mirror image) | DONE | P2 | Per-axis reflection `p' = 2c - p` composed into `CoordinateTransform` (innermost, program space); flips arc CW/CCW and center offsets; G50.1 cancels per-axis or all. |
| **G70 / G71 / G72 / G73** (Fanuc lathe roughing/finishing) | DONE | P1 | Single-line form; contour from block range `N<P>..N<Q>` (G2/G3 arcs tessellated). G71/G72 raster-clear the contour+stock-boundary polygon (levels spaced by `D`, U/W finish allowances); G73 pattern-repeats the contour with `R` relief divisions; G70 traces the finish pass. G73 stays RS274 peck-drill unless `P`+`Q` present. |
| **G70 / G71** (Imperial/Metric in some Fanuc lathe dialects) | MISSING | P2 | Only G20/G21 are supported for units. |
| **M98 / M99** (Fanuc sub call/return) | DONE | P0 | `executeM98`/`executeM99` implemented; M98 call-stack frames with repeat counts; bare `O<num>` labels serve as subprogram targets. |
| **G65 with `#<name> = expr` argument binding** | DONE | P0 | `PARAM_ASSIGN` lexer token + deferred assignment in `executeBlock`; G65 args mapped to `#1`-`#30` local frame. |

---

## 2. Haas-specific extensions

No Haas handler class exists (only `MarlinMCodeHandler`).

| Feature | Status | Priority | Notes |
|---|---|---|---|
| **G150** (generic pocket milling) | DONE | P1 | Raster-clears the closed boundary from subprogram `P` (arcs tessellated, even-odd scanline clipping, retract between spans), Z-stepped by `Q`, optional boundary finish pass, `I/J` stepover, `F`/`S` applied. |
| **G187** (smoothing / high-speed machining) | DONE | P1 | Sets `PathMode::BLEND`; `E` word sets `blendTolerance`. |
| **G12 / G13** (circular pocket milling) | DONE | P1 | Emits plunge + lead-in + full-circle arc per radius pass (I/K/Q/Z/L). |
| **G70 / G71** (Haas lathe rough/finish) | DONE | P1 | See §1 — shared Fanuc lathe implementation. |
| **M19** (spindle orient) | DONE | P1 | Handled in `dispatchMCode`; forwards angle to spindle callback. |
| **M41–M44** (spindle gear range) | DONE | P2 | Forwarded to user M-code hook. |
| **M60** (pallet change) | DONE | P2 | Forwarded to user M-code hook. |
| **`G54.1 Pxx`** (additional work offsets, Haas ENS) | DONE | P1 | `Pn` selects extended WCS `9+n`; `G10 L2 P10+` writes them; stored in `m_extWcs`. |

---

## 3. GRBL real-time protocol

Only the G-code text subset is parsed; GRBL's real-time protocol is absent.

| Feature | Status | Priority | Notes |
|---|---|---|---|
| **`$` settings commands** | DONE | P1 | `systemCommand` handles `$G`, `$#`, `$I`, `$X`, `$H`, `$J=`, `$C`, `$N0=`/`$N1=`; `$n=` writes stored in `grblSettings`, `$$` reports all. |
| **`$I`** (info), **`$G`** (parser state), **`$#`** (gcode parameters) | DONE | P1 | `systemCommand` emits reports via the message callback. |
| **`$H`** (home), **`$X`** (unlock), **`$C`** (check mode) | DONE | P1 | `$X` unlocks; `$C` toggles check mode; `$H`/`$J=` deferred to the realtime callback (host performs motion). |
| **`~`** (cycle resume), **`!`** (feed hold), **`?`** (status report) | DONE | P0 | `feedHold()`/`cycleResume()`/`statusReport()` + `processRealtimeChar`. |
| **`Ctrl-X`** (soft reset) | DONE | P1 | `softReset()` via `processRealtimeChar(0x18)`. |
| **`$N=`** startup lines | DONE | P2 | `$N0=`/`$N1=` stored; `$N` reports. |

---

## 4. RepRap / Marlin / 3D-printer G-codes

`MarlinMCodeHandler.hpp` registers ~35 M-codes and is wired into
`Interpreter::dispatchMarlinMCode` — Marlin-domain M-codes update
`Interpreter::marlinState()` (kinematic limits, steps/mm, offsets, PID,
servo, probe/ABL state, feed/flow overrides), emit report messages via the
message callback, and still reach the user M-code hook. A full Marlin
firmware exposes 80+. Below are the notable gaps.

### G-codes

| Feature | Status | Priority | Notes |
|---|---|---|---|
| **G10 / G11** (retract / unretract) | DONE | P0 | `G10` without L word and `G11` forward to `UserGCodeCallback`; `G10 L2/L20` stays RS274 WCS-set. |
| **G20 / G21** (units) | RECOGNIZED | — | OK. |
| **G28** (homing) | DONE | P0 | Rapids to stored reference point (`getG28Reference`). |
| **G29** (auto bed leveling) | DONE | P0 | Forwarded to `UserGCodeCallback` (host performs probing). |
| **G30** (Z probe point) | DONE | P1 | RS274 reference-point move by default; `Feature::G30_PROBE` switches to Marlin single-point probing (optional XY, Z probe, `Bed X:..` report). |
| **G80** (cancel bed leveling) | DONE | P1 | RS274 canned-cycle cancel by default; `Feature::G80_CANCEL_LEVELING` also clears `MarlinMachineState::bedLevelingEnabled`. |
| **G90 / G91** (absolute/relative) | RECOGNIZED | — | OK for motion; extrusion absolute/relative is via M82/M83 only. |
| **G92** (set position) | DONE | P0 | `processG92` sets offsets; G92.1/.2/.3 reset/restore. |

### M-codes missing from the Marlin handler

| M-code | Meaning | Priority |
|---|---|---|
| ~~**M84**~~ | Disable motors — **DONE** | P1 |
| ~~**M85**~~ | Inactivity timeout — forwarded to user hook | P2 |
| ~~**M92**~~ | Steps/mm — **DONE** (`stepsPerMm`) | P1 |
| ~~**M114**~~ | Report position — **DONE** (message callback) | P1 |
| ~~**M204 / M205**~~ | Accel/jerk settings — **DONE** (`kinematicLimits` + state fields) | P1 |
| ~~**M206**~~ | Home offset — **DONE** (`homeOffset`) | P1 |
| ~~**M208**~~ | Software endstops — forwarded to user hook | P2 |
| ~~**M210 / M211**~~ | Software endstop enable — forwarded to user hook | P2 |
| ~~**M218**~~ | Tool offset — **DONE** (`toolOffsets[T]`) | P1 |
| ~~**M220 / M221**~~ | Feed/flow override — **DONE** (M220 drives `MachineState::feedOverride`) | P1 |
| ~~**M226**~~ | Wait for pin — forwarded to user hook | P2 |
| ~~**M240**~~ | Trigger camera — forwarded to user hook | P2 |
| ~~**M250**~~ | LCD contrast — forwarded to user hook | P2 |
| ~~**M280**~~ | Servo — **DONE** (`servoAngles[P]`) | P1 |
| ~~**M300**~~ | Beep — forwarded to user hook | P2 |
| ~~**M301**~~ | Hotend PID — **DONE** (`hotendPid`) | P1 |
| ~~**M304**~~ | Bed PID — **DONE** (`bedPid`) | P1 |
| ~~**M305**~~ | Thermistor — forwarded to user hook | P2 |
| ~~**M350 / M351**~~ | Microstepping — forwarded to user hook | P2 |
| ~~**M355**~~ | Case light — forwarded to user hook | P2 |
| **M360–M378** | Various config | P2 |
| ~~**M400**~~ | Wait for queue — **DONE** | P0 |
| ~~**M401 / M402**~~ | Deploy/stow probe — **DONE** (`probeDeployed`) | P1 |
| ~~**M420**~~ | ABL state — **DONE** (`bedLevelingEnabled`) | P1 |
| ~~**M421**~~ | Set mesh point — **DONE** (`bedMesh`) | P1 |
| ~~**M500 / M501 / M502 / M503**~~ | EEPROM save/load/reset/report — **DONE** | P1 |
| ~~**M540**~~ | SD card — forwarded to user hook | P2 |
| ~~**M600**~~ | Filament change — **DONE** | P1 |
| ~~**M605**~~ | Multi-nozzle — forwarded to user hook | P2 |
| ~~**M665**~~ | Delta config — forwarded to user hook | P2 |
| ~~**M666**~~ | Delta endstop — forwarded to user hook | P2 |
| ~~**M851**~~ | Probe offset — **DONE** (`probeOffset`, echoes report) | P1 |
| ~~**M900**~~ | Linear advance — **DONE** | P1 |
| ~~**M911 / M912**~~ | Power loss — forwarded to user hook | P2 |
| ~~**M913 / M914**~~ | Stepper bump — forwarded to user hook | P2 |

---

## 5. LinuxCNC O-code execution gaps

`OCodeExecutor` (`src/gcode/GCodeOCodes.cpp`) is fully implemented and
wired into `Interpreter::executeBlock`.

| Feature | Status | Priority | Notes |
|---|---|---|---|
| **`O<name> SUB … ENDSUB`** | DONE | P0 | Executed by `OCodeExecutor` (wired into `executeBlock`). |
| **`O<name> CALL [args]`** | DONE | P0 | Args bound to `#1`-`#30` local frame via `pushFrame`. |
| **`O<name> IF / ELSEIF / ELSE / ENDIF`** | DONE | P0 | Executor resolves branch targets internally (`JUMP` actions). |
| **`ELSEIF`** | DONE | P0 | Parsed and executed. |
| **`O<name> WHILE … ENDWHILE`** | DONE | P0 | Executed; iteration limit enforced. |
| **`DO … WHILE`** | DONE | P0 | Executed. |
| **`O<name> REPEAT … ENDREPEAT`** | DONE | P1 | Executed. |
| **`BREAK` / `CONTINUE`** | DONE | P1 | Executed via resolved jumps. |
| **`RETURN`** | DONE | P1 | Executed; pops call stack. |
| **`debug` / `print` / `log`** | DONE | P2 | `OCodeType::DEBUG/LOG/PRINT`; `o<n> debug, [expr]` emits via message callback. |

---

## 6. Expression evaluator gaps

`ExpressionEvaluator` (`src/gcode/GCodeVariables.cpp`) is otherwise complete.

| Feature | Status | Priority | Notes |
|---|---|---|---|
| **Parameter assignment** (`#<foo> = expr`) | DONE | P0 | `PARAM_ASSIGN` token carries RHS text; `executeBlock` evaluates and assigns at runtime (named + numbered). |
| **Ternary `? :`** | DONE | P1 | `parseTernary` evaluates `cond ? true : false` (right-associative). |
| **`ATAN[x]`** (single-arg, returns degrees) | DONE | P2 | Single-arg `atan` and two-arg `atan2` both handled. |
| **Bitwise operators** (`AND`/`OR`/`XOR` on integers) | DONE | P2 | Bitwise on integral operands (Fanuc), logical on fractional — identical for 0/1. |

---

## 7. Header-only stubs (recognized but not interpreted)

These are accepted by the lexer/parser and have full header APIs but **no
`.cpp` implementation**, so they produce no motion or state change. All are
**P0** for any real-machine workflow.

| Feature | Header | Notes |
|---|---|---|
| G0–G3 motion execution | `GCodeG0G1.hpp`, `GCodeG2G3.hpp` | No `.cpp`. Parser emits `Block`; motion planner consumes it, but no canonical motion command is generated by the parser itself. |
| G33 / G33.1 threading / rigid tap | `GCodeMotion.hpp` | **DONE** — G33 emits `THREADING` segments (K pitch along Z, I for tapered; feed = pitch×RPM, spindle required); G33.1 emits the synchronized in/out pair (negative pitch on retract signals spindle reversal). |
| G38.2–G38.5 probing | `GCodeProbing.hpp` | **DONE** — `executeProbe` emits PROBE segments, calls `setProbeCallback` handler (simulated trip otherwise), fills `#5061`-`#5070`. |
| G73–G89 canned cycles | `GCodeCannedCycles.hpp` | **DONE** — `executeCannedCycle` expands G73/G74/G76/G81–G89 (peck, dwell, tap, bore) incl. G98/G99, L repeats, modal re-invocation; G80 cancels. |
| G5 / G5.1 / G5.2 / G5.3 splines + NURBS | `GCodeSplines.hpp` | **DONE** — `executeSpline` emits SPLINE segments (cubic I,J/P,Q; quadratic I,J); `executeNurbs` collects G5.2 control points (P weights, L order) and tessellates via de Boor on G5.3. |
| G41/G42/G41.1/G42.1 cutter comp | `GCodeToolComp.hpp` | **DONE** — geometric XY offset in `emitCompMove`: lead-in/out moves, convex corners roll an r-arc around the vertex, concave corners cut to the offset-line intersection, arcs tessellate then offset per chord. |
| G43/G43.1/G43.2/G49 tool length comp | `GCodeToolComp.hpp` | **DONE** — G43 applies H-word tool-table offsets (X/Y/Z + wear), G43.1 dynamic, G43.2 additive, G49 cancels. `ToolTable` implemented. |
| G54–G59.3 / G52 / G92 / G28 / G30 / G10 L2/L20 | `GCodeCoordinates.hpp` | `CoordinateSystemManager` class + `GCodeCoordinates.cpp`. Full implementation with `CoordinateTransform` (Eigen). |
| O-code control flow | `GCodeOCodes.hpp` | **DONE** — `GCodeOCodes.cpp` implements and is wired into `executeBlock`. |
| M98/M99 Fanuc subroutines | `GCodeOCodes.hpp` | **DONE** — see §1. |
| Feed/spindle/rapid override M48–M53 | `GCodeTypes.hpp` | **DONE** — M48/M49 toggle override enables; M50/M51 set P-word scales; M52/M53 hold/reset. |

---

## 8. Cutter compensation and arc/plane edge cases

| Feature | Status | Priority | Notes |
|---|---|---|---|
| **G41.1 / G42.1** (cutter comp with dynamic D) | DONE | P1 | In modal group 7; D word sets radius directly. |
| **G43.1 / G43.2** (tool length, dynamic / additional) | DONE | P1 | Handled in `TOOL_LENGTH` dispatch. |
| **G17.1 / G18.1 / G19.1** (polar planes) | DONE | P2 | Dispatched in `PLANE` group → `Plane::UV/WU/VW`. |
| **R-word arc mode** | DONE | P1 | `handleArc` computes center from R (negative R = major arc). |
| **Helical arcs** | DONE | P1 | Third axis interpolates linearly across arc tessellation; `helixDelta` populated in arc-segment mode. |

---

## 9. Standard M-codes with no interpretation

`isValidMCode` accepts any 0–999. `getMCodeDescription` only knows M0, M2, M3,
M5. These well-known M-codes are **accepted but have no handler**:

| M-code | Meaning | Priority |
|---|---|---|
| M1 | Optional pause | P1 |
| M6 | Tool change | P0 |
| M7 / M8 / M9 | Coolant mist/flood/off | P1 |
| ~~M19~~ | Spindle orient — **DONE** | — |
| M30 | Program end + rewind | P1 |
| ~~M48 / M49~~ | Override enable/disable — **DONE** | — |
| M99 | Subroutine return (Fanuc) | P0 |
| M100–M199 | User-defined | P2 |

---

## 10. Other RS274/NGC and dialect gaps

| Feature | Status | Priority | Notes |
|---|---|---|---|
| **`G4` dwell execution** | DONE | P1 | Emits DWELL segment (P ms or S s). |
| **`G61 / G61.1 / G64` path control** | DONE | P2 | Sets `pathMode`/`blendTolerance`/`naiveCamTolerance`. |
| **`G53` (machine coordinates, non-modal)** | DONE | P1 | Non-modal machine-coord rapid. |
| **`G92.1 / G92.2 / G92.3`** (reset G92) | DONE | P1 | `processG92_1/2/3` wired in NON_MODAL dispatch. |
| **Block skip `/` beyond first column** | DONE | P2 | `/` emits BLOCK_DELETE anywhere; with `skipBlockDelete` the rest of the line is skipped. |
| **Multiple `M` words on one line** | RECOGNIZED | — | OK (up to `mCodes.size()`). |
| **`P` / `Q` / `L` words for canned cycles** | DONE | — | Consumed by `executeCannedCycle` (dwell / peck / repeat). |

---

## Implementation priority (suggested order)

1. **O-code executor** (§5) — sub/call/if/while/do/repeat/return. Unblocks
   LinuxCNC subroutine workflows. (P0)
2. **Expression assignment** (§6) — `#<name> = expr`. Required for G65 args
   and LinuxCNC named-param writes. (P0)
3. **G65 / G66 / G67 Fanuc macros** (§1) — depends on (2). (P0)
4. **M98/M99 execution** (§1, §7) — Fanuc sub call/return. (P0)
5. **G0–G3 motion generation** (§7) — wire `Block` → canonical motion. (P0)
6. **Canned cycles G73–G89** (§7) — depends on (5). (P0)
7. **Probing G38.2–G38.5** (§7) — depends on (5). (P0)
8. **Tool compensation G41/G42/G43/G49** (§7). (P1)
9. **Coordinate systems G54–G59.3 / G52 / G92 / G28 / G30 / G10 L2/L20** (§7).
   (P1)
10. **Splines / NURBS G5.x** (§7). (P1)
11. **GRBL real-time protocol** (§3) — `$`, `~`, `!`, `?`, `Ctrl-X`. (P1)
12. **RepRap/Marlin M-code coverage** (§4) — **DONE**: all P0/P1 codes now
    update `MarlinMachineState` via the wired `MarlinMCodeHandler`; only P2
    config codes remain forwarded to the user hook.
    (P1)
13. **Ternary `? :`** (§6). (P1)
14. **Coordinate rotation G68/G69, scaling G51** (§1). (P1) — **DONE**
15. **Haas extensions G150/G187/G12/G13** (§2). (P1)
