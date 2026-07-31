# Klipper Protocol Implementation

This document describes the clean-slate Klipper protocol implementation in the
Tether motion kernel (`tether_klipper` component).

## Overview

The `tether_klipper` component provides a complete implementation of the
Klipper protocol with two roles:

- **Klippy (host)**: Connects to a device, downloads the data dictionary,
  synchronises the clock, and dispatches commands. Translates Tether
  `MotionPlan`s into `queue_step` sequences.
- **Device**: Serves the data dictionary, processes commands, and executes
  motion via Tether's motion subsystem (passthrough or reconstruct+replan).

All motion planning leverages Tether's existing `tether_motion_planner`
subsystem; the Klipper protocol is purely a wire format and execution layer.

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                      Klippy Host                         │
│  ┌──────────┐  ┌──────────┐  ┌──────────────┐            │
│  │KlippyHost│→ │SerialQueue│→ │  Transport   │            │
│  │          │  │ (reliability)│  (TCP/Pipe/  │            │
│  │ ClockSync│  │           │  │   Loopback)  │            │
│  └──────────┘  └──────────┘  └──────┬───────┘            │
│  ┌──────────────────────────────────┘                     │
│  │ MotionTranslator: MotionPlan → queue_step             │
│  └──────────────────────────────────────────────────────  │
└─────────────────────────────────────────────────────────┘
                           │ Wire protocol
┌──────────────────────────┴────────────────────────────────┐
│                     Klipper Device                         │
│  ┌──────────┐  ┌──────────┐  ┌──────────────┐              │
│  │KlipperDevice│→ │IdentifyServer│→ │  Transport   │       │
│  │           │  │           │  │              │              │
│  │ McuClock  │  │ CommandTable│  └──────────────┘          │
│  └──────────┘  └──────────┘                               │
│  ┌──────────────────────────────────────────────────────  │
│  │ PeripheralHandler: Stepper, DigitalOut, PwmOut, ...    │
│  │ MotionReconstructor → MotionBlockSink (analysis)      │
│  └──────────────────────────────────────────────────────  │
└─────────────────────────────────────────────────────────┘
```

## Wire Protocol

The Klipper wire protocol uses a binary command/response system:

- **Message block**: 1-byte sync (0x7E), 1-byte header (4-bit seq + 4-bit
  length), content bytes, 2-byte CRC-16 (big-endian), 1-byte sync.
- **Content**: One or more messages, each consisting of a VLQ-encoded msgid
  followed by VLQ-encoded parameters.
- **Data dictionary**: Maps human-readable format strings (e.g.
  `queue_step oid=%c interval=%u count=%hu add=%hi`) to integer msgids.
  Downloaded via the identify handshake at connection time.
- **CRC-16**: Computed over the header and content bytes.

### Key files

| Component | Header | Source |
|-----------|--------|--------|
| CRC-16 | `protocol/Crc16.hpp` | — |
| VLQ encoding | `protocol/Vlq.hpp` | — |
| Message block | `protocol/MessageBlock.hpp` | — |
| Constants | `protocol/Constants.hpp` | — |
| Parameter format | `protocol/ParameterFormat.hpp` | `protocol/ParameterFormat.cpp` |
| Data dictionary | `protocol/DataDictionary.hpp` | `protocol/DataDictionary.cpp` |
| Identify protocol | `protocol/IdentifyProtocol.hpp` | `protocol/IdentifyProtocol.cpp` |
| Command table | `protocol/CommandTable.hpp` | `protocol/CommandTable.cpp` |

## Transports

All transports implement the `IByteStreamTransport` interface:

| Transport | Header | Use case |
|-----------|--------|----------|
| Loopback | `transport/LoopbackTransport.hpp` | In-process tests/examples |
| Pipe | `transport/PipeTransport.hpp` | Unix pipe I/O |
| TCP | `transport/TcpStreamTransport.hpp` | Network communication |
| CAN | `transport/CanTransport.hpp` | CAN bus (gated by `TETHER_ENABLE_KLIPPER_CAN`) |

The CAN transport requires the HAL `ICan` interface (`tether/hal/ICan.hpp`).
The Linux SocketCAN implementation (`src/hal/LinuxCan.cpp`) is compiled when
`TETHER_ENABLE_KLIPPER_CAN` is ON.

### Enabling the CAN Transport

To build with CAN support, enable the CMake option:

```cmake
# In your CMakeLists.txt or cmake invocation:
set(TETHER_ENABLE_KLIPPER_CAN ON)
```

Or from the command line:

```bash
cmake -DTETHER_ENABLE_KLIPPER_CAN=ON ..
```

This pulls in the `LinuxCan` HAL implementation (which uses SocketCAN) and
compiles the `CanTransport` class.  On targets without SocketCAN (e.g.
bare-metal MCUs), provide your own `ICan` implementation and register it
with the transport before use.

### CAN Transport Configuration

The CAN transport uses a configurable CAN ID for transmit and receive.
Frames are segmented to fit the classic CAN MTU (8 bytes) or CAN-FD MTU
(64 bytes) depending on the underlying interface.

## Reliability

The host-side `SerialQueue` provides:

- Sliding-window flow control (configurable max pending blocks).
- Ack/nak processing with in-order sequence tracking.
- RTO estimation (RFC 6298-style SRTT/RTTVAR).
- Automatic retransmission on timeout.

## Clock Synchronisation

The `ClockSync` class uses decaying linear regression to fit MCU clock
readings against host time:

- Periodic `get_clock` exchanges yield (sendTime, recvTime, mcuClock) samples.
- Exponential decay weighting ensures recent samples dominate.
- The fitted slope maps host-time delays to MCU clock ticks.

The `McuClock` class models the 32-bit MCU clock with wraparound tracking,
converting to a 64-bit monotonic tick count.

## Object Model

Peripherals are identified by 8-bit OIDs allocated during the config phase:

| Object | Header | Description |
|--------|--------|-------------|
| OidAllocator | `objects/OidAllocator.hpp` | OID allocation |
| Stepper | `objects/Stepper.hpp` | Step execution with queue_step |
| DigitalOut | `objects/Peripherals.hpp` | Digital output with scheduling |
| PwmOut | `objects/Peripherals.hpp` | PWM output with scheduling |
| AnalogIn | `objects/Peripherals.hpp` | Analog input sampling |
| Endstop | `objects/Peripherals.hpp` | Endstop input |
| Trsync | `objects/Peripherals.hpp` | Trigger synchronisation |
| Spi | `objects/Peripherals.hpp` | SPI bus |
| I2c | `objects/Peripherals.hpp` | I2C bus |

## Motion Subsystem

The motion subsystem bridges Tether's `MotionPlan` and Klipper's `queue_step`:

| Component | Header | Description |
|-----------|--------|-------------|
| MotionBlock | `motion/MotionBlock.hpp` | Decoded step sequence for analysis |
| MotionTranslator | `motion/MotionTranslator.hpp` | MotionPlan → queue_step |
| MotionReconstructor | `motion/MotionReconstructor.hpp` | queue_step → trajectory |
| MotionBlockSink | `motion/MotionBlockSink.hpp` | Block sink (recording, printing) |

## Code-as-Config

Following Tether's "code-as-config" approach, the device's command set is
declared in code via `KlipperConfig`:

```cpp
config::KlipperConfig cfg;
config::withStandardCommands(cfg, 180000000);  // standard command set
cfg.addCommand("my_custom_command oid=%c value=%u");
cfg.addResponse("my_custom_response result=%u");
auto dict = cfg.build();
```

## Build Configuration

The `tether_klipper` component is enabled with:

```cmake
# Option 1: Feature flag (builds all enabled components)
set(TETHER_BUILD_KLIPPER ON)

# Option 2: Component selection (builds only klipper + deps)
set(TETHER_COMPONENTS klipper)
```

Optional CAN transport:
```cmake
set(TETHER_ENABLE_KLIPPER_CAN ON)  # requires SocketCAN
```

## Examples

| Example | Description |
|---------|-------------|
| `klipper_loopback_demo` | Full handshake: dict download + clock sync |
| `klipper_motion_blocks` | Stepper execution + trajectory reconstruction |
| `klipper_clock_sync` | Clock sync convergence demonstration |
| `klipper_gpio_pwm` | GPIO/PWM/endstop/trsync peripherals |

## Tests

The `tether_klipper_tests` executable covers:

- Wire protocol: CRC, VLQ, message blocks
- Transports: loopback, pipe
- Reliability: serial queue, RTO estimator, sequence counter
- Clock sync: MCU clock, clock sync
- Object model: OID allocator, stepper, peripherals
- End-to-end: dict download, clock sync, command dispatch
