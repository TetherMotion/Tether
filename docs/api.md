# Tether API Documentation

This document provides an overview of the Tether library's public API using MyST-Parser with Breathe directives.

## Overview

The Tether library is a modular C++ library for EtherCAT motion control. It provides components for:

- Hardware abstraction layer (HAL)
- Control algorithms (PID, motion profiles)
- G-code parsing and trajectory generation
- EtherCAT master and slave implementations
- Motion planning and replanning
- Simulation systems

## Core Classes

### Example Class Documentation

Below is an example of how to document C++ classes using Breathe directives in Markdown.

```{doxygenclass} MyClass
:members:
:protected-members:
:private-members:
:undoc-members:
```

### EtherCAT Master

```{doxygenclass} EtherCATMaster
:members:
```

### Motion Controller

```{doxygenclass} MotionController
:members:
```

## API Reference by Component

### Common Utilities

```{doxygennamespace} tether::common
:members:
```

### Hardware Abstraction Layer

```{doxygenclass} HAL
:members:
```

### Control Algorithms

```{doxygenclass} PIDController
:members:
```

### G-code Interpreter

```{doxygenclass} GCodeParser
:members:
```

### Motion Planning

```{doxygenclass} MotionPlanner
:members:
```

### EtherCAT Components

#### Common Types

```{doxygennamespace} tether::ethercat
:members:
```

#### Master Implementation

```{doxygenclass} EtherCATMaster
:members:
```

#### Slave Emulation

```{doxygenclass} EtherCATSlave
:members:
```

## Using the API

### Basic Example

```cpp
#include <tether/ethercat/EtherCATMaster.hpp>

using namespace tether;

int main() {
    // Initialize the EtherCAT master
    ethercat::EtherCATMaster master("eth0");
    
    // Configure the master
    master.configure();
    
    // Start the master
    master.start();
    
    // Run the control loop
    while (running) {
        master.update();
    }
    
    // Stop the master
    master.stop();
    
    return 0;
}
```

### Motion Control Example

```cpp
#include <tether/control/PIDController.hpp>
#include <tether/motion/MotionPlanner.hpp>

using namespace tether;

void setupMotionControl() {
    // Create a PID controller
    control::PIDController pid;
    pid.setGains(1.0, 0.1, 0.01);
    
    // Create a motion planner
    motion::MotionPlanner planner;
    planner.setMaxVelocity(100.0);
    planner.setMaxAcceleration(500.0);
    
    // Plan a trajectory
    auto trajectory = planner.planToPosition(target_position);
}
```

## Additional Documentation

For more detailed information on specific components, see:

- [EtherCAT Inventory](ETHERCAT_INVENTORY.md)
- [IO Protocol](IOProtocol.md)
- [Motion Replanner](MotionReplanner.md)
- [HAL Porting Guide](HAL_PORTING_GUIDE.md)
