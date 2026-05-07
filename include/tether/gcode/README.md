# G-Code Interpreter

## Overview

This is a complete RS274/NGC (LinuxCNC) compatible G-code interpreter designed for embedded systems, particularly ESP32 with EtherCAT motion control.

## Features

### Motion Control
- **G0** - Rapid positioning
- **G1** - Linear feed
- **G2/G3** - Clockwise/counterclockwise arc
- **G5** - Cubic B-spline
- **G5.1** - Quadratic B-spline  
- **G5.2/G5.3** - NURBS curves
- **G33** - Spindle-synchronized motion (threading)
- **G33.1** - Rigid tapping

### Canned Cycles
- **G73** - High-speed peck drilling
- **G74** - Left-hand tapping
- **G76** - Fine boring (spindle orient)
- **G80** - Cancel canned cycle
- **G81** - Simple drilling
- **G82** - Drilling with dwell
- **G83** - Deep hole peck drilling
- **G84** - Right-hand tapping
- **G85** - Boring (feed out)
- **G86** - Boring (spindle stop)
- **G87** - Back boring
- **G88** - Boring (manual retract)
- **G89** - Boring (dwell, feed out)

### Probing (G38.x)
- **G38.2** - Probe toward, error on no contact
- **G38.3** - Probe toward, no error
- **G38.4** - Probe away, error on no break
- **G38.5** - Probe away, no error

### Coordinate Systems
- **G53** - Machine coordinates (non-modal)
- **G54-G59.3** - Work coordinate systems (9 total)
- **G10 L2** - Set WCS offset (absolute)
- **G10 L20** - Set WCS based on current position
- **G92** - Coordinate system offset
- **G92.1/G92.2/G92.3** - G92 control

### Tool Compensation
- **G40** - Cancel cutter compensation
- **G41/G41.1** - Cutter compensation left
- **G42/G42.1** - Cutter compensation right
- **G43/G43.1/G43.2** - Tool length offset
- **G44** - Tool length offset (negative)
- **G49** - Cancel tool length offset

### Other G-Codes
- **G4** - Dwell
- **G17/G18/G19** - Plane selection (XY/XZ/YZ)
- **G20/G21** - Units (inch/mm)
- **G28/G28.1** - Return to/set reference point
- **G30/G30.1** - Secondary reference point
- **G61/G61.1/G64** - Path control modes
- **G90/G91** - Absolute/incremental distance
- **G90.1/G91.1** - Absolute/incremental arc mode
- **G93/G94/G95** - Feed rate modes
- **G96/G97** - Constant surface speed
- **G98/G99** - Canned cycle retract modes

### M-Codes
- **M0** - Program stop
- **M1** - Optional stop
- **M2** - Program end
- **M3/M4/M5** - Spindle control
- **M6** - Tool change
- **M7/M8/M9** - Coolant control
- **M30** - Program end and rewind
- **M48/M49** - Enable/disable feed override
- **M50-M53** - Feed and spindle override control
- **M70-M73** - Modal state save/restore
- **M98/M99** - Fanuc-style subroutine call/return
- **M100-M199** - User-defined M-codes

### O-Codes (Control Flow)
- **O sub/endsub** - Subroutine definition
- **O call** - Subroutine call
- **O return** - Return from subroutine
- **O if/elseif/else/endif** - Conditional execution
- **O while/endwhile** - While loop
- **O do/while** - Do-while loop
- **O repeat/endrepeat** - Repeat loop
- **O break** - Break from loop
- **O continue** - Continue to next iteration

### Variables & Expressions
- **#1-#30** - Local parameters (subroutine scope)
- **#31-#5000** - Global parameters
- **#<name>** - Named parameters
- **Expression evaluation** - Mathematical expressions in []
- **Math functions** - SIN, COS, TAN, ATAN, SQRT, ABS, etc.
- **Logical operators** - AND, OR, XOR, EQ, NE, GT, LT, GE, LE

### Advanced Features
- **Multiblock lookahead** (50 blocks)
- **Multiblock lookbehind** (20 blocks)
- **Negative feed rates** (reverse motion)
- **Path blending** (G64)
- **Trochoidal milling**
- **Volumetric compensation**
- **Backlash compensation**
- **Adaptive feed rate** (M52)
- **Polar coordinates**

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     GCodeInterpreter                        │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌──────────┐  ┌──────────┐  ┌──────────────────────────┐  │
│  │  Lexer   │→ │  Parser  │→ │  Block Queue (Lookahead) │  │
│  └──────────┘  └──────────┘  └──────────────────────────┘  │
│                                          │                  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │                Variable System                        │  │
│  │  #1-#30 Local | #31-#5000 Global | #<name> Named     │  │
│  └──────────────────────────────────────────────────────┘  │
│                                          │                  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │               O-Code Executor                         │  │
│  │  Sub/Call | If/Else | While | Repeat | Break/Continue│  │
│  └──────────────────────────────────────────────────────┘  │
│                                          │                  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │                Motion Handlers                        │  │
│  │  G0/G1 | G2/G3 | Splines | Cycles | Probe | Tool    │  │
│  └──────────────────────────────────────────────────────┘  │
│                                          │                  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │              Coordinate Systems                       │  │
│  │  G53 | G54-G59.3 | G92 | Tool Length | Cutter Comp  │  │
│  └──────────────────────────────────────────────────────┘  │
│                                          │                  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │              Advanced Motion                          │  │
│  │  Path Blend | Volumetric | Backlash | Adaptive Feed │  │
│  └──────────────────────────────────────────────────────┘  │
│                                          ↓                  │
│                              ┌────────────────────┐         │
│                              │   Motion Segments  │────→ OUT│
│                              └────────────────────┘         │
└─────────────────────────────────────────────────────────────┘
```

## Quick Start

### Basic Usage

```cpp
#include "gcode/GCodeInterpreter.hpp"

using namespace GCode;

// Create interpreter with default config
Interpreter interpreter;

// Set motion callback (required)
interpreter.setMotionCallback([](const MotionSegment& seg) {
    // Send segment to motion controller
    printf("Move to (%.3f, %.3f, %.3f)\n", 
           seg.end.x, seg.end.y, seg.end.z);
    return Error{};  // Success
});

// Load and run program
Error err = interpreter.loadFile("program.ngc");
if (!err) {
    err = interpreter.run();
}

if (err) {
    printf("Error: %s at line %d\n", err.message, err.line);
}
```

### MDI Mode (Single Line Execution)

```cpp
interpreter.setMode(InterpreterMode::MDI);

interpreter.executeLine("G21");          // Set mm mode
interpreter.executeLine("G0 X0 Y0 Z10"); // Rapid home
interpreter.executeLine("G1 X50 F500");  // Linear move
```

### Accessing Variables

```cpp
VariableSystem& vars = interpreter.getVariables();

// Set parameters
vars.set(100, 25.4);           // #100 = 25.4
vars.setNamed("depth", -5.0);  // #<depth> = -5.0

// Get parameters
double val = vars.get(100);
double depth = vars.getNamed("depth");

// Evaluate expression
ExpressionEvaluator eval(vars);
double result = eval.evaluate("[#100 * 2 + SIN[45]]");
```

### Tool Table Management

```cpp
ToolTable& tools = interpreter.getToolTable();

// Define a tool
ToolEntry tool;
tool.toolNumber = 1;
tool.diameter = 10.0;
tool.zOffset = 50.5;
tools.setTool(1, tool);

// Load from file
tools.loadFromFile("tools.tbl");
```

### Work Coordinate Systems

```cpp
CoordinateSystemManager& coords = interpreter.getCoordinates();

// Select WCS
coords.selectWCS(1);  // G54

// Set offset (G10 L2 P1 X10 Y20)
WorkCoordinateSystem& g54 = coords.getWCS(1);
g54.offset.x = 10.0;
g54.offset.y = 20.0;

// Save for persistence
coords.saveToFile("wcs.txt");
```

## Configuration

### Master Configuration

```cpp
InterpreterConfig config;

// Motion limits
config.linearMotion.maxFeedRate = 10000;  // mm/min
config.linearMotion.maxPlungeFeed = 500;

// Arc settings
config.arcMotion.maxArcRadius = 100000;
config.arcMotion.arcRadiusTolerance = 0.005;

// Features
config.features.enableNURBS = true;
config.features.enableTrochoidalMilling = true;
config.features.enableVolumetricComp = false;

// Lookahead
config.parser.maxLookahead = 50;
config.parser.maxLookbehind = 20;

Interpreter interpreter(config);
```

### Per-Component Configuration

Each subsystem can be configured independently:

```cpp
// Path blending
PathBlendConfig blend;
blend.mode = PathBlendMode::CONTINUOUS;
blend.toleranceP = 0.05;
interpreter.getPathBlender().setConfig(blend);

// Cutter compensation
CutterCompConfig cutterComp;
cutterComp.checkGouging = true;
cutterComp.errorOnGouge = true;
```

## File Structure

```
main/include/gcode/
├── GCodeTypes.hpp          # Core types, enums, Position, Block
├── GCodeConfig.hpp         # Configuration structures
├── GCodeVariables.hpp      # Variable system & expressions
├── GCodeLexer.hpp          # Tokenizer
├── GCodeParser.hpp         # Block parser
├── GCodeOCodes.hpp         # Control flow (sub/if/while)
├── GCodeInterpreter.hpp    # Main interpreter class
└── motion/
    ├── GCodeG0G1.hpp       # Linear motion
    ├── GCodeG2G3.hpp       # Arc motion
    ├── GCodeSplines.hpp    # B-spline & NURBS
    ├── GCodeCannedCycles.hpp # Drilling, tapping, boring
    ├── GCodeProbing.hpp    # Probe cycles
    ├── GCodeToolComp.hpp   # Tool compensation
    ├── GCodeCoordinates.hpp # Coordinate systems
    └── GCodeAdvancedMotion.hpp # Trochoidal, volumetric, etc.
```

## Error Handling

```cpp
Error err = interpreter.run();

if (err) {
    switch (err.code) {
        case ErrorCode::SYNTAX_ERROR:
            // Parse error
            break;
        case ErrorCode::UNDEFINED_VARIABLE:
            // Variable not set
            break;
        case ErrorCode::ARC_RADIUS_MISMATCH:
            // Arc endpoint doesn't match radius
            break;
        // ... etc
    }
    
    printf("Error %d: %s\n", static_cast<int>(err.code), err.message);
    printf("  Line %d: %s\n", err.line, err.lineContent);
}

// Get all errors
for (const Error& e : interpreter.getErrors()) {
    // Process each error
}
```

## Theory of Operation

### Motion Generation Pipeline

1. **Lexer** - Splits line into tokens (G, X, numbers, etc.)
2. **Parser** - Combines tokens into Block structure
3. **Expression Evaluation** - Resolves [] expressions and variables
4. **Modal State Update** - Updates active G/M codes
5. **Motion Handler** - Generates MotionSegments
6. **Compensation** - Applies tool length, cutter comp, etc.
7. **Path Planning** - Velocity/acceleration limits, blending
8. **Output** - Calls motion callback with segments

### Lookahead/Lookbehind

The block queue enables:

- **Velocity Planning** - Decelerate before corners
- **Path Blending** - Calculate blend arcs at corners
- **Cutter Compensation** - Look ahead for corner handling
- **Reverse Motion** - Look behind for negative feed rates

### Modal Groups

G-codes are organized into modal groups. Only one code per group can be active:

| Group | Function | Codes |
|-------|----------|-------|
| 1 | Motion | G0, G1, G2, G3, G33, G38.x, G73-G89 |
| 2 | Plane | G17, G18, G19 |
| 3 | Distance | G90, G91 |
| 5 | Feed Mode | G93, G94, G95 |
| 6 | Units | G20, G21 |
| 7 | Cutter Comp | G40, G41, G42 |
| 8 | Tool Length | G43, G44, G49 |
| 12 | WCS | G54-G59.3 |
| 13 | Path Mode | G61, G61.1, G64 |

## Examples

### Simple Pocket

```gcode
G21                         ; mm mode
G54                         ; WCS 1
T1 M6                       ; Tool 1
S5000 M3                    ; Spindle on
G43 H1                      ; Tool length offset

G0 X0 Y0 Z10                ; Position
G0 Z1                       ; Approach

G1 Z-5 F100                 ; Plunge
G1 X50 F500                 ; Cut
G1 Y50
G1 X0
G1 Y0

G0 Z10                      ; Retract
M5                          ; Spindle off
M30                         ; End
```

### Subroutine Example

```gcode
; Main program
#<depth> = -10
#<size> = 25

O100 call [10] [10] [#<size>] [#<depth>]
O100 call [50] [10] [#<size>] [#<depth>]
O100 call [10] [50] [#<size>] [#<depth>]
O100 call [50] [50] [#<size>] [#<depth>]
M30

; Square pocket subroutine
O100 sub
    G0 X#1 Y#2
    G0 Z1
    G1 Z#4 F100
    G1 X[#1+#3] F500
    G1 Y[#2+#3]
    G1 X#1
    G1 Y#2
    G0 Z5
O100 endsub
```

### Loop with Variables

```gcode
#<holes> = 8
#<radius> = 25
#<i> = 0

O200 while [#<i> LT #<holes>]
    #<angle> = [#<i> * 360 / #<holes>]
    #<x> = [#<radius> * COS[#<angle>]]
    #<y> = [#<radius> * SIN[#<angle>]]
    
    G0 X#<x> Y#<y>
    G81 Z-10 R2 F200
    
    #<i> = [#<i> + 1]
O200 endwhile

G80
```

## License

See main project license.

## References

- [LinuxCNC G-code Reference](http://linuxcnc.org/docs/html/gcode/g-code.html)
- [RS274/NGC Interpreter Standard](https://www.nist.gov/publications/nist-rs274ngc-interpreter-version-3)
- [Fanuc Programming Manual](https://www.fanuc.co.jp/en/product/cnc/index.html)
