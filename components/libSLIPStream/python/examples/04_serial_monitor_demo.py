#!/usr/bin/env python3
"""
Example 4: Serial Port Monitoring (Most Common Use Case)

This example demonstrates how to monitor a serial port for SLIP frames
with CRC32 validation and real-time statistics.

This is the typical use case when you have a device sending SLIP frames
over a serial connection and you want to capture and analyze them.

To test this without actual hardware, you can:
1. Create a virtual serial port pair with socat
2. Run a sender in one terminal
3. Run this monitor in another terminal

Example setup:
    # Terminal 1: Create virtual serial port pair
    socat -d -d pty,raw,echo=0 pty,raw,echo=0
    
    # Terminal 2: Send test frames to one virtual port
    python3 04_serial_monitor_demo.py --sender /dev/pts/X
    
    # Terminal 3: Monitor the other virtual port
    python3 04_serial_monitor_demo.py --monitor /dev/pts/Y
"""

import sys
import time
import argparse
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from slipstream import (
    create_connection,
    FrameMonitor,
    encode_packet,
    append_crc32,
    hexlify_frame
)


def sender_mode(port, duration=30):
    """
    Sender mode: Send test SLIP frames to a serial port.
    
    This simulates a device sending SLIP-encoded frames with CRC32.
    """
    print(f"Sender Mode: Sending test frames to {port}")
    print(f"Duration: {duration} seconds")
    print("=" * 60)
    
    try:
        conn = create_connection(port)
    except Exception as e:
        print(f"Error: Cannot open {port}: {e}")
        return 1
    
    start_time = time.time()
    frame_count = 0
    
    try:
        while time.time() - start_time < duration:
            # Create a test frame
            app_data = f"FRAME_{frame_count:04d}".encode()
            
            # Add CRC32
            frame_with_crc = append_crc32(app_data)
            
            # Encode with SLIP
            slip_frame = encode_packet(frame_with_crc)
            
            # Send the frame
            conn.write(slip_frame)
            frame_count += 1
            
            print(f"[{frame_count:3d}] Sent {len(app_data):3d} byte payload, "
                  f"{len(slip_frame):3d} bytes SLIP encoded")
            
            time.sleep(0.5)  # Send one frame per 0.5 seconds
    
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        conn.close()
    
    print(f"\nSent {frame_count} frames in {time.time() - start_time:.1f} seconds")
    return 0


def monitor_mode(port, duration=None, interactive=False):
    """
    Monitor mode: Receive and analyze SLIP frames from a serial port.
    
    This demonstrates the main use case of slipstream library.
    """
    print(f"Monitor Mode: Receiving SLIP frames from {port}")
    if duration:
        print(f"Duration: {duration} seconds")
    print("=" * 60)
    
    try:
        conn = create_connection(port)
    except Exception as e:
        print(f"Error: Cannot open {port}: {e}")
        return 1
    
    # Create monitor with CRC validation
    monitor = FrameMonitor(conn, check_crc=True, hex_output=False)
    
    # Custom callback to print frame details
    def on_frame(data):
        last = monitor.get_last_frame()
        if last:
            crc_status = "✓ OK" if last['crc_valid'] else "✗ BAD"
            timestamp = time.strftime('%H:%M:%S', time.localtime(last['timestamp']))
            
            print(f"\n[{timestamp}] Frame received:")
            print(f"  Total size: {len(data)} bytes (incl. CRC)")
            print(f"  Payload:    {len(last['payload'])} bytes")
            print(f"  CRC Status: {crc_status}")
            
            # Try to decode as ASCII
            try:
                ascii_repr = last['payload'].decode('ascii', errors='replace')
                print(f"  Content:    {ascii_repr}")
            except Exception:
                pass
            
            # Hex dump
            print(f"  Hex dump:")
            hex_lines = hexlify_frame(data, width=16).split('\n')
            for line in hex_lines[:3]:  # Show first 3 lines
                print(f"    {line}")
            if len(hex_lines) > 3:
                print(f"    ...")
    
    monitor.frame_callback = on_frame
    
    try:
        print("Waiting for frames...\n")
        monitor.monitor(duration=duration)
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    finally:
        monitor.close()
    
    # Print final statistics
    print("\n\n")
    monitor.print_stats()
    
    return 0


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='SLIP Serial Monitor - Demo sender and monitor',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  # Monitor a serial port
  python3 04_serial_monitor_demo.py --monitor /dev/ttyUSB0
  
  # Monitor with custom baudrate
  python3 04_serial_monitor_demo.py --monitor /dev/ttyUSB0:9600
  
  # Send test frames for 60 seconds
  python3 04_serial_monitor_demo.py --sender /dev/ttyUSB0 --duration 60
  
  # Monitor for 30 seconds only
  python3 04_serial_monitor_demo.py --monitor /dev/ttyUSB0 --duration 30
        '''
    )
    
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--monitor', help='Monitor mode: read SLIP frames from serial port')
    group.add_argument('--sender', help='Sender mode: send test SLIP frames to serial port')
    
    parser.add_argument(
        '--duration',
        type=float,
        help='Duration in seconds (default: infinite for monitor, 30 for sender)'
    )
    
    args = parser.parse_args()
    
    if args.monitor:
        duration = args.duration if args.duration else None
        return monitor_mode(args.monitor, duration=duration)
    else:
        duration = args.duration if args.duration else 30
        return sender_mode(args.sender, duration=duration)


if __name__ == '__main__':
    sys.exit(main())
