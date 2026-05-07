# SLIP Frame Framing Convention

## Overview

SLIP (Serial Line Internet Protocol) is a simple protocol for framing messages over serial connections. This document describes the SLIP frame format as implemented in libSLIPStream, with particular emphasis on CRC32 validation.

**Reference:** RFC 1055 - Nonstandard Transmission of IP Datagrams over Serial Lines: SLIP

## Basic SLIP Framing

### Special Bytes

SLIP uses four special bytes to implement the framing:

| Byte    | Name      | Hex    | Purpose |
|---------|-----------|--------|---------|
| END     | End Mark  | 0xC0   | Marks the beginning and end of a SLIP frame |
| ESC     | Escape    | 0xDB   | Escapes special bytes within the payload |
| ESCEND  | Escaped END   | 0xDC   | Represents literal 0xC0 in the payload |
| ESCESC  | Escaped ESC   | 0xDD   | Represents literal 0xDB in the payload |

### Escape Sequences

When encoding, if the payload data contains any of the special bytes, they must be escaped:

- A literal **END byte (0xC0)** in the payload is encoded as: **ESC, ESCEND** (0xDB, 0xDC)
- A literal **ESC byte (0xDB)** in the payload is encoded as: **ESC, ESCESC** (0xDB, 0xDD)
- Any other byte is transmitted as-is

When decoding, the reverse process occurs:

- When an ESC byte is encountered, the next byte is inspected:
  - If followed by ESCEND (0xDC), it represents a literal END byte (0xC0)
  - If followed by ESCESC (0xDD), it represents a literal ESC byte (0xDB)
  - Any other byte following ESC is an error and the frame should be discarded

## Frame Structure with CRC32

The libSLIPStream library supports optional CRC32 validation. When CRC checking is enabled, the frame structure is:

```
[END] [PAYLOAD_DATA] [CRC32] [END]
 0xC0  (escaped)   (4 bytes) 0xC0
```

### Detailed Frame Layout

```
┌──────┬─────────────────────────────┬──────────────┬──────┐
│ END  │   PAYLOAD DATA (escaped)    │  CRC32       │ END  │
│ 0xC0 │   (variable length)         │  (4 bytes)   │ 0xC0 │
└──────┴─────────────────────────────┴──────────────┴──────┘
  
  START             APPLICATION DATA        INTEGRITY    FRAME
  MARKER           + EMBEDDED CRC32        CHECK        MARKER
```

### CRC32 Specification

- **Polynomial:** Ethernet polynomial 0x04C11DB7
- **Bit order:** MSB-first (big-endian per polynomial definition)
- **Initial value:** 0xFFFFFFFF
- **Final XOR:** Applied during computation
- **Byte order in frame:** Little-endian (LSB first in transmitted bytes)
- **Position:** Last 4 bytes of the decoded frame payload

#### CRC32 Calculation

The CRC32 should be calculated over the application data (excluding the CRC32 bytes themselves):

```python
import struct
from slipstream import calculate_crc32, extract_crc32

# When sending a frame with CRC:
payload_data = b"Hello, World!"
crc = calculate_crc32(payload_data)
frame = payload_data + struct.pack('<I', crc)  # Little-endian format

# When receiving a frame with CRC:
received_frame = b"Hello, World!" + b'\x12\x34\x56\x78'
payload, crc_bytes = extract_crc32(received_frame)
calculated_crc = calculate_crc32(payload)
is_valid = calculated_crc == struct.unpack('<I', crc_bytes)[0]
```

## Transmission Example

### Example 1: Simple Frame Without Special Bytes

**Sending:** `"HELLO"` (5 bytes)

```
Raw frame payload:     48 45 4C 4C 4F
SLIP encoded frame:    C0 48 45 4C 4C 4F C0
                       ││            └─ END marker
                       └─ START marker
```

### Example 2: Frame Containing Special Bytes

**Sending:** `[0xC0, 0x42, 0xDB]` (3 bytes)

The payload contains END (0xC0) and ESC (0xDB) which must be escaped:

```
Raw frame payload:     C0 42 DB
Escaped:               DB DC 42 DB DD
SLIP encoded frame:    C0 DB DC 42 DB DD C0
                       ││ ├──┘  └─────┴─ Escaped bytes
                       │└─────────────── ESC, ESCEND for 0xC0
                       └───────────────── ESC for following byte
```

### Example 3: Frame With CRC32

**Sending:** `"DATA"` with CRC32

Assume CRC32("DATA") = 0x12345678

```
Raw frame payload:     44 41 54 41 78 56 34 12  (little-endian CRC)
                       └─ "DATA" ─┤  └─ CRC32 ┘
SLIP encoded frame:    C0 44 41 54 41 78 56 34 12 C0
```

(In this example, no bytes need escaping since none of the CRC32 bytes are 0xC0 or 0xDB, but typically at least some bytes would require escaping in real scenarios.)

## Implementation Notes

### Handling Errors

1. **Invalid Escape Sequences:** If an ESC byte is not followed by ESCEND or ESCESC, discard the frame and reset the decoder.

2. **Missing END Marker:** If no END marker is received within a reasonable time, the partial frame should be discarded.

3. **Buffer Overflow:** If the frame payload exceeds the maximum buffer size, discard the frame and reset the decoder.

4. **CRC Mismatch:** If CRC32 validation is enabled and the calculated CRC does not match the CRC in the frame, the frame should be flagged as corrupted but may still be delivered depending on application requirements.

### Maximum Frame Sizes

- **Minimum frame size:** 1 byte (just END marker) - represents empty frame
- **Typical payload:** 64-256 bytes without special characters
- **Maximum payload:** Limited only by available memory and application buffers

For embedded systems, typical frame sizes are:
- Header + data + CRC32: 4-256 bytes unencoded
- Worst case with frequent special bytes: up to ~2x the unencoded size

## Python Implementation

The Python `slipstream` library provides convenient functions for working with SLIP frames including CRC32:

```python
from slipstream import encode_packet, decode_packet, calculate_crc32, append_crc32, verify_crc32

# Encode a simple frame
data = b"Hello"
encoded = encode_packet(data)

# Encode with CRC32
data = b"Hello"
data_with_crc = append_crc32(data)
encoded = encode_packet(data_with_crc)

# Decode a frame
decoded, consumed = decode_packet(encoded)

# Verify CRC
payload, crc_bytes = extract_crc32(decoded)
crc_valid = verify_crc32(payload, crc_bytes)
```

## C++ Implementation

The C++ `libSLIPStream` library provides buffer-based utilities for SLIP encoding/decoding. CRC32 validation is left to the application layer.

```cpp
#include "SLIPStream/Buffer.hpp"

// Encode
uint8_t payload[] = {0x01, 0xC0, 0x02};
size_t needed = SLIPStream::encoded_length(payload, 3);
uint8_t buffer[needed];
size_t written = SLIPStream::encode_packet(payload, 3, buffer, needed);

// Decode
size_t decoded_len = SLIPStream::decoded_length(buffer, written);
uint8_t decoded[decoded_len];
size_t got = SLIPStream::decode_packet(buffer, written, decoded, decoded_len);
```

## Common Issues and Solutions

### Issue: Frames Corrupt After Certain Bytes

**Cause:** Special bytes (0xC0, 0xDB) in payload are not being properly escaped.

**Solution:** Ensure escape sequences are applied correctly during encoding:
- 0xC0 → 0xDB 0xDC
- 0xDB → 0xDB 0xDD

### Issue: CRC32 Validation Always Fails

**Cause:** Byte order mismatch or CRC initialization.

**Solution:** Verify:
1. CRC is calculated with initial value 0xFFFFFFFF
2. CRC is stored in little-endian byte order (LSB first)
3. CRC is calculated over application data only, not including the CRC bytes themselves

### Issue: CRC Works on Sender but Not Receiver

**Cause:** Potential payload corruption during transmission or endianness mismatch.

**Solution:**
1. Verify SLIP escaping is correct on both ends
2. Use Python's `struct.unpack('<I', crc_bytes)` for little-endian CRC parsing
3. Add logging to compare calculated vs. received CRC values
4. Check if transmission channel is corrupting data (use raw TCP instead of Telnet, etc.)

### Issue: Partial Frames Being Processed

**Cause:** Frame processing starting before complete END marker received.

**Solution:**
1. Use streaming decoder that accumulates until END marker
2. Ensure no frame processing starts until 0xC0 END byte is received
3. Reset decoder state on invalid escape sequences

## References

- **RFC 1055:** Non-Standard Transmission of IP Datagrams over Serial Lines: SLIP
- **CRC32 Polynomial:** Ethernet (0x04C11DB7)
- **IEEE 802.3:** Standard for Ethernet, including CRC definitions
- **ZLIB:** Reference implementation of CRC32 using Ethernet polynomial

## See Also

- [libSLIPStream C++ Library](../README.md)
- [Python slipstream Package](../python/README.md)
- [RFC 1055](https://tools.ietf.org/html/rfc1055)
