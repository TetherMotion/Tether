# EtherCAT Slave Implementation

A comprehensive, software-based EtherCAT slave implementation supporting all major CiA profiles and mailbox protocols.

## Features

### Core Infrastructure
- **SlaveCore**: Full ESC (EtherCAT Slave Controller) emulation
  - All 14 datagram commands (APRD, APWR, APRW, FPRD, FPWR, FPRW, BRD, BWR, BRW, LRD, LWR, LRW, ARMW, FRMW)
  - FMMU (Fieldbus Memory Management Unit) support
  - SyncManager configuration
  - Distributed Clocks (DC) synchronization
  - AL (Application Layer) state machine (Init → PreOp → SafeOp → Op)
  - EEPROM emulation

### CiA Device Profiles
| Profile | Description | Features |
|---------|-------------|----------|
| CiA 401 | Digital/Analog I/O | 16 digital inputs/outputs, 4 analog channels |
| CiA 402 | Drives and Motion | Full state machine, PP/PV/HM/CSP/CSV/CST modes |
| CiA 404 | Measuring Devices | PID control, alarm limits, multiple channels |
| CiA 405 | IEC 61131-3 PLC | Task scheduling, I/O variables, run states |
| CiA 406 | Encoder | Single/multi-turn, velocity calculation, preset |
| CiA 408 | Hydraulic Drive | Pressure/flow/position modes, emergency stop |
| CiA 410 | Inclinometer | 2-axis measurement, calibration, alarms |
| CiA 417 | Lift Controller | Floor management, door control, safety |
| CiA 430 | Power Supply | CV/CC modes, protection, monitoring |

### Mailbox Protocols
| Protocol | Description |
|----------|-------------|
| CoE | CANopen over EtherCAT - SDO, PDO mapping |
| FoE | File over EtherCAT - File transfer |
| EoE | Ethernet over EtherCAT - IP tunneling |
| SoE | SERCOS over EtherCAT - IDN access |
| VoE | Vendor-specific over EtherCAT |
| AoE | ADS over EtherCAT - TwinCAT compatible |

### HAL (Hardware Abstraction Layer) Modes
| Mode | Description | Use Case |
|------|-------------|----------|
| DirectLoopback | Synchronous callback | Unit testing |
| ThreadedLoopback | Async with worker thread | Performance testing |
| FIFOLoopback | POSIX FIFO communication | Inter-process testing |
| UDPLoopback | UDP socket communication | Network testing |
| MasterFIFO/SlaveFIFO | Dedicated FIFO endpoints | Process separation |

### Logging
- **PcapNG**: Wireshark-compatible packet capture
  - Section Header Block (SHB)
  - Interface Description Block (IDB)
  - Enhanced Packet Block (EPB)
  - File rotation support
  - Buffered and direct write modes
- **SlaveLogger**: Async logging with minimal latency impact
  - Configurable log levels
  - File rotation
  - Real-time friendly (lock-free queue)

## Directory Structure

```
main/
├── include/
│   ├── slave/
│   │   ├── core/
│   │   │   ├── SlaveCore.hpp      # Core ESC emulation
│   │   │   └── SlaveTypes.hpp     # Type definitions
│   │   ├── hal/
│   │   │   └── ISlaveHAL.hpp      # HAL interface
│   │   ├── logging/
│   │   │   └── SlaveLogger.hpp    # Async logging
│   │   ├── mailbox/
│   │   │   └── IMailboxHandler.hpp # Mailbox interface
│   │   └── profiles/
│   │       ├── ProfileSlave.hpp   # Base profile class
│   │       ├── CiA401Slave.hpp    # I/O profile
│   │       ├── CiA402Slave.hpp    # Drive profile
│   │       └── ...                # Other profiles
│   └── shared/
│       └── PcapLogger.hpp         # Shared PCAP logging
└── src/
    ├── slave/
    │   ├── SlaveCore.cpp
    │   ├── hal/
    │   │   └── LoopbackHAL.cpp
    │   ├── logging/
    │   │   └── SlaveLogger.cpp
    │   ├── mailbox/
    │   │   ├── CoEHandler.cpp
    │   │   ├── FoEHandler.cpp
    │   │   └── ...
    │   └── profiles/
    │       ├── ProfileSlave.cpp
    │       ├── CiA401Slave.cpp
    │       └── ...
    └── shared/
        └── PcapLogger.cpp

host_tests/full_suite/
├── SlaveIntegrationTests.cpp   # Core slave tests
├── HALTests.cpp                # HAL unit tests
├── ProfileTests.cpp            # Profile-specific tests
├── MasterSlaveTests.cpp        # Integration tests
├── slave_tests/
│   └── CMakeLists.txt          # Build configuration
└── examples/
    ├── simple_slave.cpp        # Basic I/O slave
    ├── fifo_communication.cpp  # Inter-process comm
    └── cia402_drive.cpp        # Servo drive example
```

## Building

### Prerequisites
- CMake 3.16+
- C++17 compiler (GCC 8+, Clang 7+)
- Google Test (fetched automatically if not found)
- pthread

### Build Commands

```bash
cd host_tests/full_suite/slave_tests
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### Build Options

```bash
# Enable AddressSanitizer
cmake -DENABLE_ASAN=ON ..

# Enable ThreadSanitizer
cmake -DENABLE_TSAN=ON ..

# Disable tests
cmake -DBUILD_TESTS=OFF ..

# Disable examples
cmake -DBUILD_EXAMPLES=OFF ..
```

### Running Tests

```bash
# Run all tests
make run_all_tests

# Run individual test suites
./slave_integration_tests
./hal_tests
./profile_tests
./master_slave_tests

# Run with verbose output
./slave_integration_tests --gtest_filter="*StateMachine*"
```

## Usage Examples

### Creating a Simple I/O Slave

```cpp
#include "slave/profiles/CiA401Slave.hpp"
#include "slave/hal/LoopbackHAL.hpp"

auto slave = std::make_unique<CiA401Slave>(1);
slave->setConfiguredAddress(0x1001);

// Configure SyncManagers
slave->configureSyncManager(2, 0x1100, 2, SyncManagerType::Output, true);
slave->configureSyncManager(3, 0x1000, 2, SyncManagerType::Input, true);

// Set digital inputs (e.g., from sensors)
slave->setDigitalInput(0, true);
slave->setAnalogInput(0, 2048);

// Update slave (call in main loop)
slave->update();
```

### Creating a CiA 402 Drive

```cpp
#include "slave/profiles/CiA402Slave.hpp"

auto drive = std::make_unique<CiA402Slave>(1);
drive->setConfiguredAddress(0x1001);

// Configure for CSP mode
drive->setModesOfOperation(8);  // Cyclic Sync Position

// In your control loop:
drive->setTargetPosition(targetPos);
drive->update();

// Get status
uint16_t status = drive->getStatusWord();
int32_t actualPos = drive->getActualPosition();
```

### Using FIFO Communication

```cpp
#include "slave/hal/LoopbackHAL.hpp"

FIFOConfig config;
config.masterToSlavePath = "/tmp/ecat_m2s";
config.slaveToMasterPath = "/tmp/ecat_s2m";
config.createFifos = true;

auto hal = std::make_unique<SlaveFIFOHAL>(config, processCallback);
```

### Enabling PcapNG Logging

```cpp
#include "shared/PcapLogger.hpp"

PcapNgConfig config;
config.filePath = "ethercat_trace.pcapng";
config.interfaceName = "EtherCAT";
config.maxFileSize = 100 * 1024 * 1024;  // 100MB

auto logger = std::make_unique<PcapNgLogger>(config);

// Use with HAL
auto hal = std::make_unique<DirectLoopbackHAL>(callback, logger.get());
```

## Architecture

### Frame Processing Flow

```
[Master] -> EtherCAT Frame
                |
                v
        +---------------+
        |  HAL Layer    |  (DirectLoopback, FIFO, UDP)
        +---------------+
                |
                v
        +---------------+
        |  SlaveCore    |  (Frame parsing, ESC registers)
        +---------------+
                |
        +-------+-------+
        |               |
        v               v
  +----------+    +-----------+
  |   FMMU   |    | SyncMgr   |
  +----------+    +-----------+
        |               |
        +-------+-------+
                |
                v
        +---------------+
        | Profile Slave |  (CiA 401, 402, etc.)
        +---------------+
                |
                v
        +---------------+
        |   Mailbox     |  (CoE, FoE, etc.)
        +---------------+
```

### State Machine

```
         +------+
         | Init |<-----------------------+
         +------+                        |
            |                            |
            v                            |
       +--------+                        |
       | PreOp  |<------------------+    |
       +--------+                   |    |
            |                       |    |
            v                       |    |
       +--------+                   |    |
       | SafeOp |<--------------+   |    |
       +--------+               |   |    |
            |                   |   |    |
            v                   |   |    |
         +----+                 |   |    |
         | Op |---(Error)-------+---+----+
         +----+
```

## Testing Strategy

### Unit Tests
- State machine transitions
- Register read/write
- FMMU mapping
- SyncManager operations
- DC time synchronization

### Integration Tests
- Complete frame exchange
- PDO communication
- CoE/SDO transfers
- Multi-slave scenarios

### Profile Tests
- CiA 402 drive state machine
- All operating modes
- Encoder position tracking
- I/O channel operations

### HAL Tests
- All HAL mode functionality
- Thread safety
- Error handling
- PCAP logging integration

## Performance Considerations

- Lock-free logging queue for real-time safety
- Minimal allocation in hot paths
- Configurable buffer sizes
- Optional PCAP logging (NullPcapLogger for production)

## License

See LICENSE file in repository root.
