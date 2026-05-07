# libSLIPStream

SLIP encoder & decoder focused on embedded systems, in C++.

This small, unopinionated library provides buffer-based helpers to encode and decode data using the SLIP (Serial Line Internet Protocol) framing. It is designed to be lightweight and safe for constrained environments: callers provide input/output buffers and the library returns explicit error codes when buffers are too small or input is malformed.

## Key symbols (namespaced)

- `#include "SLIPStream/Buffer.hpp"` — high-level buffer helpers
- `SLIPStream::encoded_length(const uint8_t* in, size_t inlen)` — compute encoded size (including final END)
- `SLIPStream::encode_packet(const uint8_t* in, size_t inlen, uint8_t* out, size_t outlen)` — encode into `out`
- `SLIPStream::decoded_length(const uint8_t* in, size_t inlen)` — compute decoded size up to first END
- `SLIPStream::decode_packet(const uint8_t* in, size_t inlen, uint8_t* out, size_t outlen)` — decode into `out`
- `SLIPStream::ENCODE_ERROR` / `SLIPStream::DECODE_ERROR` — functions return this (`SIZE_MAX`) on errors

See the header `include/SLIPStream/Buffer.hpp` for documentation and function contracts.

## Basic usage (encoding)

Typical pattern for encoding safely:

1. Call `SLIPStream::encoded_length()` to determine how many bytes the encoded packet will occupy.
2. Allocate an output buffer of at least that size (stack, static, or heap as appropriate).
3. Call `SLIPStream::encode_packet()` and check the return value.

Example:

```cpp
#include "SLIPStream/Buffer.hpp"
#include <vector>
#include <cstdio>

void encode_example() {
	const uint8_t payload[] = { 0x01, 0xC0, 0x02 }; // contains a SLIP END (0xC0)
	size_t needed = SLIPStream::encoded_length(payload, sizeof(payload));
	std::vector<uint8_t> out(needed);

	size_t written = SLIPStream::encode_packet(payload, sizeof(payload), out.data(), out.size());
	if (written == SLIPStream::ENCODE_ERROR) {
		// handle insufficient buffer or other internal error
		std::puts("encode failed: output buffer too small");
		return;
	}

	// 'written' bytes in out.data() now contain a single SLIP-framed packet (terminated by END)
}
```

Notes:
- Always use the length returned by `SLIPStream::encoded_length()` when allocating the destination buffer.
- The encoder appends a single SLIP END byte at the end of the encoded packet.

## Basic usage (decoding)

Typical pattern for decoding safely:

1. Receive a raw buffer from the transport (serial, socket, etc.). The buffer must contain at least one SLIP END byte for a full frame.
2. Call `SLIPStream::decoded_length()` to determine how many decoded bytes will result (up to the first END).
3. Allocate a buffer of that size and call `SLIPStream::decode_packet()`.

Example:

```cpp
#include "SLIPStream/Buffer.hpp"
#include <vector>
#include <cstdio>

void decode_example(const uint8_t* rx, size_t rxlen) {
	size_t decoded_needed = SLIPStream::decoded_length(rx, rxlen);
	if (decoded_needed == SLIPStream::DECODE_ERROR) {
		std::puts("decode length failed: malformed input or missing END");
		return;
	}

	std::vector<uint8_t> out(decoded_needed);
	size_t got = SLIPStream::decode_packet(rx, rxlen, out.data(), out.size());
	if (got == SLIPStream::DECODE_ERROR) {
		std::puts("decode failed: output buffer too small or malformed input");
		return;
	}

	// 'got' bytes in out.data() are the decoded payload
}
```

Notes:
- `SLIPStream::decoded_length()` stops at the first SLIP END; if no END is present it returns `SLIPStream::DECODE_ERROR`.
- `SLIPStream::decode_packet()` returns `SLIPStream::DECODE_ERROR` for malformed escape sequences or if the provided output buffer is too small.

## Error handling and defensive tips

- Treat `SLIPStream::ENCODE_ERROR` and `SLIPStream::DECODE_ERROR` as fatal for the current operation and recover by discarding the partial buffer or requesting a retry.
- When reading from a stream, accumulate bytes until you observe an END (0xC0) before attempting `SLIPStream::decoded_length()` and decode. That avoids repeatedly scanning incomplete frames.
- Be careful about concurrent access: functions are buffer-based and reentrant as long as you don't share the same input/output buffers concurrently.

## Building & testing

This repository includes a standalone CMake test project under `test/` that builds a single test executable named `test_all` containing all unit tests.

The top-level `CMakeLists.txt` is an ESP-IDF component file and is not intended for standalone test builds. Use the `test/` directory for normal C++ test compilation.

From the `test/` directory (out-of-source build recommended):

```sh
cd test
mkdir -p build && cd build
cmake -DENABLE_COVERAGE=ON ..
cmake --build .
ctest -V
```

You can also run the test binary directly to list or execute GoogleTest cases and get more verbose output:

```sh
# list registered test suites/cases
./test/test_all --gtest_list_tests
# run the tests directly
./test/test_all
```

If you prefer CTest to discover and report individual GoogleTest cases, enable GoogleTest discovery in `test/CMakeLists.txt` by adding the following (optional):

```cmake
include(GoogleTest)
gtest_discover_tests(test_all)
```

If you'd like, I can enable `gtest_discover_tests(test_all)` in `test/CMakeLists.txt` so `ctest -V` lists each test case separately.

Adjust the commands for your environment (toolchain, cross-compile, or embedded workflow).

## License

See `LICENSE` for license information.

## Python Library

This repository also includes a comprehensive Python library and monitoring tool for SLIP frame handling.

### Installation

```bash
# Install from the python directory
cd python
pip install -e .

# Or install dependencies manually
pip install pyserial  # For serial port support
```

### Python Quick Start

#### Basic Encoding and Decoding

```python
from slipstream import encode_packet, decode_packet

# Encode data
data = b"Hello, World!"
encoded = encode_packet(data)
print(f"Encoded: {encoded.hex()}")

# Decode data
decoded, consumed = decode_packet(encoded)
print(f"Decoded: {decoded}")
```

#### Streaming Frame Monitoring (Serial Port)

The most common use case - continuously monitor SLIP frames from a serial port:

```python
from slipstream import create_connection, FrameMonitor

# Create connection (auto-detects serial or TCP)
connection = create_connection('/dev/ttyUSB0:115200')

# Create monitor with CRC32 validation
monitor = FrameMonitor(connection, check_crc=True, hex_output=False)

# Define callback for each frame
def process_frame(frame_data):
    print(f"Received {len(frame_data)} bytes: {frame_data.hex()}")
    # Extract and verify CRC if enabled
    last = monitor.get_last_frame()
    if last['crc_valid'] is False:
        print("Warning: CRC validation failed!")

monitor.frame_callback = process_frame

# Monitor for 30 seconds
monitor.monitor(duration=30)

# Print statistics
monitor.print_stats()
monitor.close()
```

#### Interactive Monitoring with ncurses UI

For real-time monitoring with a full-screen statistics dashboard:

```bash
# Monitor serial port with ncurses UI (interactive mode)
python3 -m slipstream.scripts.monitor_slip -i /dev/ttyUSB0

# Monitor TCP connection
python3 -m slipstream.scripts.monitor_slip -i tcp:192.168.1.100:5000

# Monitor with hex display and 60-second timeout
python3 -m slipstream.scripts.monitor_slip -x -t 60 /dev/ttyUSB0:115200
```

Or run the monitoring script directly:

```bash
./python/scripts/monitor_slip.py -i /dev/ttyUSB0
```

**Interactive Mode Controls:**
- **q** - Quit the application
- **h** - Toggle hex display
- **c** - Clear frame history
- **Any key** - Trigger UI refresh

### Command-Line Tool Features

```bash
# Basic monitoring
./python/scripts/monitor_slip.py /dev/ttyUSB0

# With custom baudrate
./python/scripts/monitor_slip.py /dev/ttyUSB0:115200

# TCP connection
./python/scripts/monitor_slip.py tcp:192.168.1.1:5000

# Interactive ncurses UI
./python/scripts/monitor_slip.py -i /dev/ttyUSB0

# Hex dump of frames
./python/scripts/monitor_slip.py -x /dev/ttyUSB0

# 30-second timeout
./python/scripts/monitor_slip.py -t 30 /dev/ttyUSB0

# ASCII display
./python/scripts/monitor_slip.py -a /dev/ttyUSB0

# Skip CRC validation
./python/scripts/monitor_slip.py --no-crc /dev/ttyUSB0
```

### CRC32 Validation

The library supports CRC32 with the Ethernet polynomial (0x04C11DB7). CRC is typically stored in the last 4 bytes of the frame payload:

```python
from slipstream import calculate_crc32, append_crc32, extract_crc32, verify_crc32

# Calculate CRC32
payload = b"application_data"
crc = calculate_crc32(payload)
print(f"CRC32: 0x{crc:08X}")

# Append CRC32 to payload (little-endian)
frame_with_crc = append_crc32(payload)

# Extract and verify CRC from received frame
received_frame = b"application_data\x12\x34\x56\x78"
payload, crc_bytes = extract_crc32(received_frame)
is_valid = verify_crc32(payload, crc_bytes)
print(f"CRC Valid: {is_valid}")
```

### Statistics and Diagnostics

The monitoring tool automatically tracks:
- **Frames received** - Total count of successfully decoded frames
- **Frames with bad CRC** - Count of CRC validation failures
- **Frames with errors** - Count of frames with decoding errors
- **Total bytes received** - Raw bytes from the connection
- **Payload bytes** - Decoded payload bytes
- **Frames per second** - Throughput metric
- **Bytes per second** - Raw throughput metric
- **Frame size statistics** - Min/max/average frame sizes

Access statistics programmatically:

```python
stats = monitor.get_stats()
print(f"Frames received: {stats['frames_received']}")
print(f"Frames with errors: {stats['frames_with_errors']}")
print(f"Throughput: {stats['bytes_per_second']:.2f} bps")
print(f"Average frame size: {stats['avg_payload_size']:.1f} bytes")
```

### SLIP Frame Format

For detailed information about the SLIP frame structure and CRC32 encoding, see [FramingConvention.md](FramingConvention.md).

Key points:
- **Frame structure:** `[END byte] [payload with SLIP escaping] [END byte]`
- **Escape sequences:**
  - 0xC0 (END) inside payload → 0xDB, 0xDC
  - 0xDB (ESC) inside payload → 0xDB, 0xDD
- **CRC32:** Ethernet polynomial, stored in last 4 bytes of payload (little-endian)

### API Reference

#### Core Functions

- `encode_packet(data: bytes) -> bytes` - SLIP encode data
- `decode_packet(data: bytes) -> (bytes, int)` - SLIP decode, returns (payload, consumed_bytes)
- `StreamingDecoder` - Stateful decoder for continuous streams
- `calculate_crc32(data: bytes) -> int` - Calculate CRC32
- `verify_crc32(data: bytes, stored_crc: bytes) -> bool` - Verify CRC

#### Connection Classes

- `SerialConnection(port, baudrate, timeout)` - Serial port handler
- `TCPConnection(host, port, timeout)` - TCP socket handler
- `create_connection(string) -> Connection` - Factory function

#### High-Level Classes

- `FrameMonitor` - High-level frame monitor with CRC and stats
- `FrameStatistics` - Statistics tracker

### Requirements

- Python 3.6+
- `pyserial` (optional, for serial port support)
- `curses` (built-in on Linux/macOS; optional for interactive mode)

### License

The Python library is part of libSLIPStream and is subject to the same license terms. See `LICENSE`.

