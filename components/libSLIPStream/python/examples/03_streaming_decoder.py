#!/usr/bin/env python3
"""
Example 3: Streaming Decoder for Continuous Data

This example shows how to use the StreamingDecoder to process
continuous byte streams, which is the typical use case for serial/TCP monitoring.
"""

import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from slipstream import StreamingDecoder, encode_packet


def example_streaming():
    """Demonstrate streaming decoder for continuous byte processing."""
    
    print("=" * 60)
    print("Streaming Decoder Examples")
    print("=" * 60)
    
    # Example 1: Process complete frames
    print("\n1. Process Complete Frames")
    print("-" * 60)
    
    frames_received = []
    
    def on_frame(data):
        frames_received.append(data)
        print(f"Frame received: {data}")
    
    decoder = StreamingDecoder(callback=on_frame)
    
    # Simulate data arriving from a stream
    frame1 = b"Message1"
    frame2 = b"Message2"
    
    encoded_stream = encode_packet(frame1) + encode_packet(frame2)
    print(f"Feeding encoded stream: {encoded_stream.hex()}")
    
    decoder.feed(encoded_stream)
    print(f"Frames received: {len(frames_received)}")
    
    # Example 2: Process fragmented data
    print("\n2. Process Fragmented Data (arrives in chunks)")
    print("-" * 60)
    
    frames_received = []
    decoder = StreamingDecoder(callback=on_frame)
    
    # Encode a frame
    message = b"Fragmented Data"
    encoded = encode_packet(message)
    print(f"Total frame: {encoded.hex()}")
    print(f"Frame size: {len(encoded)} bytes")
    
    # Feed in small chunks to simulate serial port arrival
    chunk_size = 3
    for i in range(0, len(encoded), chunk_size):
        chunk = encoded[i:i+chunk_size]
        print(f"  Chunk {i//chunk_size}: {chunk.hex()}")
        decoder.feed(chunk)
    
    print(f"Frames received: {len(frames_received)}")
    if frames_received:
        print(f"Decoded: {frames_received[0]}")
    
    # Example 3: Multiple frames in a stream
    print("\n3. Multiple Frames in Single Stream")
    print("-" * 60)
    
    frames_received = []
    decoder = StreamingDecoder(callback=on_frame)
    
    # Create multiple frames
    messages = [b"START", b"DATA1", b"DATA2", b"STOP"]
    stream = b"".join(encode_packet(msg) for msg in messages)
    
    print(f"Stream with {len(messages)} frames: {stream.hex()[:80]}...")
    decoder.feed(stream)
    print(f"Total frames decoded: {len(frames_received)}")
    for i, frame in enumerate(frames_received):
        print(f"  Frame {i}: {frame}")
    
    # Example 4: Error handling - invalid escape sequence
    print("\n4. Error Handling - Invalid Escape Sequence")
    print("-" * 60)
    
    frames_received = []
    decoder = StreamingDecoder(callback=on_frame)
    
    # Create stream with an invalid escape sequence
    # (ESC followed by invalid byte)
    invalid_stream = bytes([0xC0, 0xDB, 0xFF, 0xC0])  # ESC followed by invalid byte
    print(f"Stream with invalid escape: {invalid_stream.hex()}")
    
    decoder.feed(invalid_stream)
    stats = decoder.get_stats()
    print(f"Frames received: {stats['frames_received']}")
    print(f"Frames with errors: {stats['frames_with_errors']}")
    
    # Example 5: State tracking
    print("\n5. Decoder Statistics")
    print("-" * 60)
    
    frames_received = []
    decoder = StreamingDecoder(callback=on_frame)
    
    # Feed some valid frames
    for i in range(5):
        decoder.feed(encode_packet(f"Frame{i}".encode()))
    
    stats = decoder.get_stats()
    print(f"Total frames received: {stats['frames_received']}")
    print(f"Total errors: {stats['frames_with_errors']}")
    
    print("\n" + "=" * 60)


if __name__ == "__main__":
    example_streaming()
