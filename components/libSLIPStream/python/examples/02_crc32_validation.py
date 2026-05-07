#!/usr/bin/env python3
"""
Example 2: CRC32 Validation

This example demonstrates CRC32 calculation and validation using the
Ethernet polynomial (0x04C11DB7).
"""

import sys
from pathlib import Path
import struct

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from slipstream import (
    calculate_crc32, 
    verify_crc32, 
    append_crc32, 
    extract_crc32,
    encode_packet,
    decode_packet
)


def example_crc32():
    """Demonstrate CRC32 calculation and validation."""
    
    print("=" * 60)
    print("CRC32 Validation Examples")
    print("=" * 60)
    
    # Example 1: Calculate CRC32
    print("\n1. Calculate CRC32")
    print("-" * 60)
    
    payload = b"sensor_data_123"
    crc = calculate_crc32(payload)
    print(f"Payload: {payload}")
    print(f"CRC32:   0x{crc:08X}")
    
    # Example 2: Append CRC32 to payload
    print("\n2. Append CRC32 to Payload")
    print("-" * 60)
    
    frame_with_crc = append_crc32(payload)
    print(f"Original payload: {payload.hex()}")
    print(f"Frame with CRC:   {frame_with_crc.hex()}")
    print(f"  Payload length: {len(payload)} bytes")
    print(f"  CRC32 bytes:    {frame_with_crc[-4:].hex()}")
    print(f"  Total length:   {len(frame_with_crc)} bytes")
    
    # Example 3: Extract and verify CRC
    print("\n3. Verify CRC32 from Received Frame")
    print("-" * 60)
    
    received_frame = frame_with_crc
    extracted_payload, stored_crc = extract_crc32(received_frame)
    is_valid = verify_crc32(extracted_payload, stored_crc)
    
    print(f"Received frame: {received_frame.hex()}")
    print(f"Extracted payload: {extracted_payload}")
    print(f"Stored CRC:     0x{struct.unpack('<I', stored_crc)[0]:08X}")
    print(f"Is valid:       {is_valid}")
    
    # Example 4: Detect corrupted CRC
    print("\n4. Detect Corrupted CRC")
    print("-" * 60)
    
    # Corrupted frame (change last 2 bits of CRC)
    corrupted = frame_with_crc[:-4] + b'\xFF\xFF\xFF\xFF'
    try:
        payload, crc_bytes = extract_crc32(corrupted)
        is_valid = verify_crc32(payload, crc_bytes)
        print(f"Corrupted frame: {corrupted.hex()}")
        print(f"CRC Valid:       {is_valid}")
    except ValueError as e:
        print(f"Error: {e}")
    
    # Example 5: SLIP frame with embedded CRC32
    print("\n5. SLIP Frame with Embedded CRC32")
    print("-" * 60)
    
    app_data = b"SENSOR_ID=42"
    
    # Create frame with CRC
    frame_with_crc = append_crc32(app_data)
    
    # Encode using SLIP
    slip_encoded = encode_packet(frame_with_crc)
    
    print(f"Application data: {app_data}")
    print(f"With CRC:         {frame_with_crc.hex()}")
    print(f"SLIP encoded:     {slip_encoded.hex()}")
    
    # Now decode the frame
    decoded, _ = decode_packet(slip_encoded)
    payload, crc_bytes = extract_crc32(decoded)
    is_valid = verify_crc32(payload, crc_bytes)
    
    print(f"\nAfter SLIP decoding:")
    print(f"Decoded payload:  {payload}")
    print(f"CRC Valid:        {is_valid}")
    
    print("\n" + "=" * 60)


if __name__ == "__main__":
    example_crc32()
