#!/usr/bin/env python3
"""
Example 1: Basic SLIP Encoding and Decoding

This example shows how to encode and decode SLIP frames programmatically.
"""

import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from slipstream import encode_packet, decode_packet


def example_basic():
    """Demonstrate basic encoding and decoding."""
    
    print("=" * 60)
    print("SLIP Encoding and Decoding Examples")
    print("=" * 60)
    
    # Example 1: Simple data without special bytes
    print("\n1. Simple Data (no special bytes)")
    print("-" * 60)
    
    data = b"Hello, World!"
    print(f"Original: {data}")
    print(f"  Length: {len(data)} bytes")
    
    encoded = encode_packet(data)
    print(f"Encoded:  {encoded.hex()}")
    print(f"  Length: {len(encoded)} bytes")
    print(f"  Overhead: {len(encoded) - len(data)} bytes")
    
    # Decode
    decoded, consumed = decode_packet(encoded)
    print(f"Decoded:  {decoded}")
    print(f"  Consumed: {consumed} bytes")
    print(f"  Match: {decoded == data}")
    
    # Example 2: Data containing special bytes
    print("\n2. Data with Special Bytes")
    print("-" * 60)
    
    # Contains END (0xC0) and ESC (0xDB)
    data = bytes([0x01, 0xC0, 0x02, 0xDB, 0x03])
    print(f"Original: {data.hex()}")
    print(f"  Contains: 0xC0 (END), 0xDB (ESC)")
    
    encoded = encode_packet(data)
    print(f"Encoded:  {encoded.hex()}")
    print(f"  Length: {len(encoded)} bytes")
    
    decoded, consumed = decode_packet(encoded)
    print(f"Decoded:  {decoded.hex()}")
    print(f"  Match: {decoded == data}")
    
    # Example 3: Multiple frames in sequence
    print("\n3. Multiple Frames")
    print("-" * 60)
    
    frame1 = b"FRAME1"
    frame2 = b"FRAME2"
    frame3 = b"FRAME3"
    
    encoded_all = encode_packet(frame1) + encode_packet(frame2) + encode_packet(frame3)
    print(f"Combined encoded: {encoded_all.hex()}")
    
    # Decode frames one by one
    pos = 0
    frame_num = 1
    while pos < len(encoded_all):
        try:
            decoded, consumed = decode_packet(encoded_all[pos:])
            print(f"  Frame {frame_num}: {decoded}")
            pos += consumed
            frame_num += 1
        except ValueError:
            break
    
    print("\n" + "=" * 60)


if __name__ == "__main__":
    example_basic()
