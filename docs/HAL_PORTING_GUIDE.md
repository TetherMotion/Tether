# Tether HAL (Hardware Abstraction Layer) Porting Guide

## Overview

The Tether library uses a comprehensive Hardware Abstraction Layer (HAL) to achieve platform independence. This document describes the HAL architecture, existing implementations, and how to port to new platforms.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     Application Code                         │
├─────────────────────────────────────────────────────────────┤
│                     Tether Core Library                      │
│    (EtherCAT Master, CiA Profiles, Motion Control, etc.)    │
├─────────────────────────────────────────────────────────────┤
│                     HAL Interfaces                           │
│  IThreading  │  IEthernet  │  IClock  │  IPeriodicTimer     │
├─────────────────────────────────────────────────────────────┤
│                  Platform Implementations                    │
│    LinuxHAL    │    ESP32HAL    │    STM32HAL               │
└─────────────────────────────────────────────────────────────┘
```

## HAL Interfaces

### Threading (`IThreading.hpp`)

The threading HAL provides platform-independent primitives for concurrent programming:

| Interface | Description |
|-----------|-------------|
| `IThread` | Thread creation and management |
| `IMutex` | Mutual exclusion (normal and recursive) |
| `IConditionVariable` | Thread synchronization |
| `ISemaphore` | Counting semaphore |
| `IEvent` | Binary event/notification |
| `IQueue` | Thread-safe FIFO queue |
| `IThreadingFactory` | Factory for creating primitives |

**Key Features:**
- RAII lock guards (`LockGuard`, `UniqueLock`)
- Timeout support on all blocking operations
- Priority levels (`Idle`, `Low`, `Normal`, `High`, `Realtime`)
- CPU affinity support
- Realtime scheduling option (SCHED_FIFO on Linux)

### Ethernet (`IEthernet.hpp`)

Raw Ethernet access for EtherCAT frame transmission:

| Interface | Description |
|-----------|-------------|
| `IEthernet` | Raw frame send/receive |
| `IEthernetFactory` | Factory for creating interfaces |

**Features:**
- Zero-copy receive path (callback-based)
- Scatter-gather transmit support
- VLAN 802.1q support
- Hardware timestamping (when available)
- Link status monitoring
- Statistics collection

### Clock (`IClock.hpp`)

High-resolution timing and periodic timers:

| Interface | Description |
|-----------|-------------|
| `IClock` | Time measurement (µs, ms, ns resolution) |
| `IPeriodicTimer` | Hardware or software periodic callbacks |
| `IClockFactory` | Factory for creating clocks/timers |

**Features:**
- Monotonic timestamps (not affected by system time changes)
- Nanosecond precision (platform dependent)
- Busy-wait delays for tight timing
- Sleep delays for power efficiency
- Periodic interrupt/callback timers

## Existing Implementations

### Linux HAL

Location: `src/hal/Linux*.cpp`

**Threading (`LinuxThreading.cpp`):**
- Uses POSIX pthreads
- SCHED_FIFO for realtime threads
- pthread_mutex for mutexes
- pthread_cond for condition variables
- sem_t for semaphores
- Custom queue using pthread_mutex + pthread_cond + circular buffer

**Ethernet (`LinuxEthernet.cpp`):**
- Uses raw sockets (AF_PACKET, SOCK_RAW)
- Direct access to Ethernet frames
- Requires CAP_NET_RAW capability or root
- Uses PACKET_MMAP for zero-copy receive (optional)

**Clock (`LinuxClock.cpp`):**
- Uses clock_gettime(CLOCK_MONOTONIC) for timestamps
- clock_nanosleep for delays
- timerfd for periodic timers

### ESP32 HAL (ESP-IDF)

Location: `src/hal/ESP32*.cpp` (in ESP-IDF builds)

**Threading:**
- Uses FreeRTOS directly (xTaskCreate, xSemaphore*, xQueue*)
- Native priority levels mapped to FreeRTOS priorities

**Ethernet:**
- Uses ESP-IDF esp_eth driver
- Hardware MAC/PHY support (LAN87xx, RTL8201, etc.)
- DMA-based frame handling

**Clock:**
- Uses esp_timer for microsecond timing
- Hardware timer for periodic callbacks
- RTC for wall-clock time

### STM32 HAL (Best-Effort)

Location: `src/hal/STM32*.cpp`

The STM32 implementation provides best-effort support without FreeRTOS:

**Threading:**
- Cooperative multitasking using polling
- Critical section disabling for mutexes
- Software timers for delays

**Ethernet:**
- STM32 Ethernet peripheral (RMII/MII)
- DMA descriptors for frame handling
- Requires HAL drivers from ST

**Clock:**
- SysTick for millisecond timing
- DWT cycle counter for microseconds
- TIM peripheral for periodic timers

## Porting to a New Platform

### Step 1: Create Platform Implementation Files

```
src/hal/
├── YourPlatformThreading.cpp
├── YourPlatformEthernet.cpp
└── YourPlatformClock.cpp
```

### Step 2: Implement IThreadingFactory

```cpp
namespace EtherCAT::hal {

class YourPlatformThread : public IThread {
    // Implement all virtual methods
};

class YourPlatformMutex : public IMutex {
    // ...
};

// ... other primitives ...

class YourPlatformThreadingFactory : public IThreadingFactory {
public:
    std::unique_ptr<IThread> createThread(const ThreadConfig& config) override {
        return std::make_unique<YourPlatformThread>(config);
    }
    
    std::unique_ptr<IMutex> createMutex(MutexType type) override {
        return std::make_unique<YourPlatformMutex>(type);
    }
    
    std::unique_ptr<IQueue> createQueue(size_t itemSize, size_t capacity) override {
        return std::make_unique<YourPlatformQueue>(itemSize, capacity);
    }
    
    // ... other factory methods ...
    
    void sleep(Milliseconds ms) override {
        // Platform-specific sleep
    }
    
    void yield() override {
        // Platform-specific yield
    }
    
    uint32_t currentThreadId() override {
        // Return unique thread identifier
    }
};

} // namespace
```

### Step 3: Implement IEthernetFactory

```cpp
class YourPlatformEthernet : public IEthernet {
    Error init(const EthernetConfig& config) override;
    Error transmit(const uint8_t* frame, size_t length) override;
    Error transmitScatter(const struct iovec* iov, size_t iovCount) override;
    void setRxCallback(RxCallback callback, void* userData) override;
    LinkStatus getLinkStatus() override;
    // ...
};
```

### Step 4: Implement IClockFactory

```cpp
class YourPlatformClock : public IClock {
    Timestamp nowMicros() override;
    Timestamp systemTimeMillis() override;
    Nanoseconds resolution() override;
    void delayMicros(Microseconds us) override;
    void delayMillis(Milliseconds ms) override;
};
```

### Step 5: Register Factory in HAL Initialization

In your platform's initialization code:

```cpp
// In YourPlatformHAL.cpp
namespace EtherCAT::hal {

static std::unique_ptr<IThreadingFactory> g_threadingFactory;
static std::unique_ptr<IEthernetFactory> g_ethernetFactory;
static std::unique_ptr<IClockFactory> g_clockFactory;

IThreadingFactory& getThreadingFactory() {
    if (!g_threadingFactory) {
        g_threadingFactory = std::make_unique<YourPlatformThreadingFactory>();
    }
    return *g_threadingFactory;
}

// Similar for other factories...

} // namespace
```

### Step 6: Update CMakeLists.txt

```cmake
option(TETHER_ENABLE_YOURPLATFORM_HAL "Enable YourPlatform HAL" OFF)

if(TETHER_ENABLE_YOURPLATFORM_HAL)
    file(GLOB HAL_YOURPLATFORM_SOURCES 
        "src/hal/YourPlatform*.cpp"
    )
    list(APPEND TETHER_SOURCES ${HAL_YOURPLATFORM_SOURCES})
endif()
```

## FreeRTOS Compatibility Layer

The `EspCompat.hpp` header provides FreeRTOS-compatible macros that work on any platform through the HAL:

```cpp
// These work on Linux, ESP32, STM32, etc.
QueueHandle_t q = xQueueCreate(10, sizeof(MyData));
xQueueSend(q, &data, portMAX_DELAY);
xQueueReceive(q, &data, pdMS_TO_TICKS(100));

SemaphoreHandle_t sem = xSemaphoreCreateBinary();
xSemaphoreGive(sem);
xSemaphoreTake(sem, portMAX_DELAY);
```

The compatibility layer uses `HalObjectRegistry` to track HAL objects and provide the FreeRTOS API.

## Error Handling

All HAL operations return `Error` enum values:

```cpp
enum class Error {
    OK = 0,
    InvalidArgument,
    Timeout,
    NotInitialized,
    AlreadyInitialized,
    NotSupported,
    ResourceBusy,
    OutOfMemory,
    IOError,
    InvalidState,
    Empty,        // Queue/buffer empty
    BufferFull,   // Queue/buffer full
    Unknown = 0xFF
};
```

## Thread Safety

- All HAL interfaces are designed to be thread-safe
- Factory methods may be called from any thread
- Created objects (IThread, IMutex, etc.) have documented thread safety
- Lock guards provide RAII-style mutex management

## Performance Considerations

### Realtime Requirements

For EtherCAT applications with strict timing:

1. Use `ThreadPriority::Realtime` for cyclic tasks
2. Enable `useRealtimeScheduling` in ThreadConfig
3. Set CPU affinity to isolate realtime threads
4. Use `IQueue` for communication instead of shared memory + locks
5. Minimize allocations in realtime paths

### Memory Usage

The HAL implementations are designed to minimize dynamic allocation:

- Thread stacks are allocated once at creation
- Queues pre-allocate their circular buffer
- Semaphores/mutexes have fixed-size internal state

## Testing

Each HAL implementation should pass the common test suite:

```bash
cd Tether/build
cmake .. -DTETHER_ENABLE_TESTS=ON -DTETHER_ENABLE_YOUR_HAL=ON
make
./tests/tether_tests --gtest_filter=HAL*
```

## Debugging

### Linux

```bash
# Run with thread sanitizer
cmake .. -DCMAKE_CXX_FLAGS="-fsanitize=thread"
make
./tether_tests

# Profile with perf
perf record -g ./your_app
perf report
```

### ESP32

```bash
# Enable HAL logging
idf.py menuconfig  # Enable Component config > Log output
idf.py flash monitor
```

## Version History

| Version | Changes |
|---------|---------|
| 1.0.0 | Initial HAL with IThread, IMutex, ISemaphore, IEthernet, IClock |
| 1.1.0 | Added IQueue interface for FreeRTOS compatibility |
| 1.2.0 | Added IConditionVariable and IEvent |

## See Also

- [EtherCAT Protocol Specification](https://www.ethercat.org/)
- [FreeRTOS API Reference](https://www.freertos.org/a00106.html)
- [POSIX Threads](https://pubs.opengroup.org/onlinepubs/9699919799/basedefs/pthread.h.html)
