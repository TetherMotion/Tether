# G-Code Interpreter Guide

How to load, verify and run G-code programs with
`tether/gcode/GCodeInterpreter.hpp`, and how the interpreter interacts
with the motion planner, coordinate systems and tool compensation.

---

## 1. The two ways to drive the interpreter

| Style | Use when | Entry points |
|---|---|---|
| **Whole program** | Production runs, file-based jobs | `loadFile()` / `loadString()` → `verify()` → `run()` |
| **Line at a time** | MDI, jogging, interactive UI, tests | `executeLine()` in `InterpreterMode::MDI` |

### Whole-program run

```cpp
#include "tether/gcode/GCodeInterpreter.hpp"

GCode::Interpreter interpreter;

interpreter.setMotionCallback([](const GCode::MotionSegment& seg) {
    planner.enqueue(seg);            // hand segments to the planner
});

if (interpreter.loadFile("part.ngc") != GCode::Error::OK) { /* report */ }
if (interpreter.verify() != GCode::Error::OK)             { /* report */ }

while (!interpreter.isFinished()) {
    const auto err = interpreter.step();
    if (err != GCode::Error::OK) break;   // inspect interpreter.getState()
}
```

`run()` is the same loop without the opportunity to interleave work;
`step()` is the cooperative variant — use it when the caller owns the
tick (realtime loop, test harness, UI that must stay responsive).

### MDI (manual data input)

```cpp
interpreter.setMode(GCode::InterpreterMode::MDI);
interpreter.executeLine("G0 X10 Y20 Z5");
interpreter.executeLine("G1 X50 F500");
```

MDI mode executes each line immediately and keeps modal state (feed rate,
plane, units, coordinate system) between calls — exactly what an operator
panel expects.

---

## 2. Verification before motion

`verify()` walks the program without producing motion and reports:

- parse errors (syntax, unknown words)
- undefined subroutines / missing files for `M98`
- unbalanced control flow (`if`/`while`/`o`-codes)
- parameters outside configured ranges

Run it on every program that comes from outside the machine (file upload,
API call). It is cheap compared to a failed run, and it is the only
reliable way to catch structural errors before axes move.

```cpp
if (interpreter.verify() != GCode::Error::OK) {
    for (const auto& e : interpreter.getErrors())   // line + message
        report(e);
}
```

`setDryRun(true)` is the complementary tool: it executes the full
interpreter (including variables and control flow) but discards motion.
Use it to validate a program's *logic* (loops, conditionals, subroutines)
without touching the hardware.

---

## 3. Modes, state and pausing

`InterpreterMode`:

| Mode | Behaviour |
|---|---|
| `Auto` | Runs a loaded program; `step()` advances one block |
| `MDI` | `executeLine()` runs immediately, no program needed |

`InterpreterState` reports where the interpreter is (`Idle`, `Running`,
`Paused`, `Finished`, `Error`). `pause()`/`resume()` are safe to call from
the UI thread as long as you do not `reset()` concurrently; `stop()` ends
the program and leaves the machine at its current position.

> **Modal state survives `pause()`/`resume()` and MDI calls** — feed rate,
> plane (G17/18/19), distance mode (G90/91), units and WCS are all part of
> the interpreter's persistent state. `reset()` clears everything.

---

## 4. What the interpreter does *not* do

The interpreter turns text into `MotionSegment`s and state changes. It
does not:

- **plan velocities** — that is the velocity profiler's job
  (see `motion/VelocityProfilerSelection.md`)
- **execute motion** — segments go to your planner via the callback
- **talk to drives** — that is `CiA402::MotionController` /
  `EtherCAT::Master`

Keeping these separate is deliberate: the same interpreter drives
simulation, tests and real hardware.

### Suggested pipeline

```
G-code text
   │  Interpreter (parse, modal state, tool/coord comp)
   ▼
MotionSegment list
   │  Velocity profiler (limits, lookahead, blending)
   ▼
Timed trajectory
   │  Motion controller / master (PDOs)
   ▼
Drives
```

---

## 5. Coordinate systems and compensation

The interpreter delegates the G-code state machines to dedicated classes;
they are constructed with references to the machine state and variable
system:

| G-codes | Handler | Guide |
|---|---|---|
| G54–G59.3, G92, G52, G68, G51 | `CoordinateSystemManager` | Doxygen |
| G43/G44/G49 | `ToolLengthComp` | Doxygen |
| G40–G42.1 | `CutterRadiusComp` | Doxygen |
| G61/G61.1/G64 | `PathBlender` | Doxygen |
| G12.1 | `TrochoidalHandler` | Doxygen |
| G93/G94, feed overrides | `OverrideController` | Doxygen |

Practical ordering rules:

1. **Set the WCS before the first move** — a program that assumes G54 but
   runs with G55 active will crash.
2. **Tool length first, then cutter radius** — cutter compensation works in
   the compensated (program) plane.
3. **Cancel cutter comp (G40) before any rapid that leaves the part** —
   compensated paths are offset; a rapid to a "safe" point is only safe in
   the uncompensated frame.
4. `checkGouge()` before enabling G41/G42 on tight internal corners — the
   interpreter can reject a compensation move instead of gouging.

---

## 6. Feed and rapid overrides

`OverrideController` implements the operator controls:

```cpp
override.setFeedOverride(0.5);        // 50 % feed
override.setRapidOverride(0.0);       // no rapids (dry run)
override.setSpindleOverride(1.1);     // 110 % spindle
```

The applied values are exposed through `#<_feed_override>`,
`#<_speed_override>` (spindle) and `#<_rapid_override>` so the UI can
display what the machine is actually doing. `M48`/`M49` enable/disable overrides;
`M50`/`M51` toggle feed/spindle override from within a program.

> **Overrides do not change the program's commanded feed** — they scale it
> at execution time. Two runs of the same file at different overrides
> produce different timings but identical geometry.

---

## 7. Error handling

Every entry point returns `GCode::Error`; `Error::OK` is success. On
failure:

1. `getErrors()` — human-readable messages with line context
2. `getState()` — `InterpreterState::Error` distinguishes "stopped because
   of an error" from "finished normally"
3. The variable system retains the modal state at the point of failure, so
   the UI can show where the program stopped

Treat an interpreter error as terminal for that program: `reset()` before
loading the next one.

---

## 8. Testing G-code logic without a machine

Because the interpreter is decoupled from motion, you can exercise control
flow, subroutines and compensation in unit tests:

```cpp
GCode::Interpreter interp;
interp.setDryRun(true);
interp.loadString(R"(
    #<n> = 0
    o100 while [#<n> LT 3]
      G1 X[#<n> * 10] F600
      #<n> = [#<n> + 1]
    o100 endwhile
    M30
)");
EXPECT_EQ(interp.verify(), GCode::Error::OK);
while (!interp.isFinished()) ASSERT_EQ(interp.step(), GCode::Error::OK);
```

Use a motion callback that records segments instead of planning them — that
is how the repository's interpreter tests assert geometry.

---

## 9. Checklist for integrating a new G-code feature

1. Parse it in the interpreter (`GCodeParser`), not in the handlers.
2. Put the state machine in a dedicated handler class (see the table in
   §5) — handlers own their state, the interpreter owns the dispatch.
3. Update the variable system (`updateToolVariables`,
   `syncToVariables`, …) so the feature is observable from G-code.
4. Add a `verify()` check if the feature has preconditions (tool defined,
   file exists, comp active).
5. Test with `setDryRun(true)` first, then a motion callback test.
6. Document the G-code in `KlipperGcodeCommands.md` or the API docs.
