#!/usr/bin/env python3
"""
SLIP Frame Monitor - Monitor and analyze SLIP frames from serial ports or TCP connections.

Features:
- Real-time SLIP frame monitoring
- CRC32 validation (Ethernet polynomial)
- Frame statistics (count, bytes, errors)
- Hex display of frame contents
- Interactive ncurses UI mode (-i/--interactive)
- Timeout support
- Statistical reporting

Usage:
    # Monitor serial port
    ./monitor_slip.py /dev/ttyUSB0
    
    # Monitor with custom baudrate
    ./monitor_slip.py /dev/ttyUSB0:115200
    
    # Monitor TCP connection
    ./monitor_slip.py tcp:192.168.1.1:5000
    
    # Interactive ncurses mode
    ./monitor_slip.py -i /dev/ttyUSB0
    
    # Monitor for 30 seconds
    ./monitor_slip.py -t 30 /dev/ttyUSB0

Author: Uli Köhler
License: See LICENSE file in libSLIPStream repository
"""

import argparse
import sys
import time
import signal
import os
from pathlib import Path

# Add parent directory to path for slipstream import
sys.path.insert(0, str(Path(__file__).parent.parent))

import slipstream
from slipstream import create_connection, FrameMonitor, hexlify_frame


class NonInteractiveMonitor:
    """Simple line-by-line frame monitor for terminal output."""
    
    def __init__(self, connection, check_crc=True, show_hex=False, show_ascii=False):
        """
        Initialize non-interactive monitor.
        
        Args:
            connection: Connection object
            check_crc: Whether to validate CRC
            show_hex: Whether to display frame in hex
            show_ascii: Whether to display ASCII representation
        """
        self.connection = connection
        self.show_hex = show_hex
        self.show_ascii = show_ascii
        self.monitor = FrameMonitor(connection, check_crc=check_crc)
        self.monitor.frame_callback = self._print_frame
        self.frame_count = 0
    
    def _print_frame(self, frame_data: bytes) -> None:
        """Print a received frame."""
        self.frame_count += 1
        
        last = self.monitor.get_last_frame()
        if not last:
            return
        
        crc_status = ""
        if last['crc_valid'] is not None:
            crc_status = " [CRC: OK]" if last['crc_valid'] else " [CRC: BAD]"
        
        print(f"\n[Frame {self.frame_count}] {len(frame_data)} bytes{crc_status}")
        
        if last['crc_valid'] is False:
            received = last.get('crc_received')
            expected = last.get('crc_expected')
            diagnostic = last.get('crc_diagnostic')
            if received is not None and expected is not None:
                print(f"  CRC received = 0x{received:08X}")
                print(f"  CRC expected = 0x{expected:08X}")
            if diagnostic:
                print(f"  CRC diagnosis = {diagnostic}")
        
        if self.show_hex:
            hex_output = hexlify_frame(frame_data)
            print(hex_output)
        elif self.show_ascii:
            try:
                ascii_str = frame_data.decode('ascii', errors='replace')
                print(f"ASCII: {ascii_str}")
            except Exception:
                pass
        else:
            # Show brief hex
            hex_str = frame_data.hex()
            if len(hex_str) > 80:
                print(f"HEX: {hex_str[:80]}...")
            else:
                print(f"HEX: {hex_str}")
    
    def run(self, duration=None):
        """
        Run the monitor.
        
        Args:
            duration: Duration in seconds (None for infinite)
        """
        try:
            print(f"Monitoring {self.connection.host}:{self.connection.port}" 
                  if hasattr(self.connection, 'host')
                  else f"Monitoring {self.connection.port} @ {self.connection.baudrate} baud")
            print("Press Ctrl+C to stop...\n")
            
            self.monitor.monitor(duration=duration)
        except KeyboardInterrupt:
            print("\n\nStopped by user.")
        finally:
            self.monitor.close()


class InteractiveMonitor:
    """Interactive ncurses-based monitor for real-time display."""
    
    def __init__(self, connection, check_crc=True):
        """
        Initialize interactive monitor.
        
        Args:
            connection: Connection object
            check_crc: Whether to validate CRC
        """
        try:
            import curses
            self.curses = curses
        except ImportError:
            raise ImportError("curses library required for interactive mode")
        
        self.connection = connection
        self.check_crc = check_crc
        self.monitor = FrameMonitor(connection, check_crc=check_crc)
        self.monitor.frame_callback = self._frame_received
        
        self.running = True
        self.display_hex = False
        self.last_frames = []
        self.max_history = 10
        
        # Signal handler for graceful shutdown
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)
    
    def _signal_handler(self, signum, frame):
        """Handle interrupt signals."""
        self.running = False
    
    def _frame_received(self, frame_data: bytes) -> None:
        """Record a received frame."""
        last = self.monitor.get_last_frame()
        if last:
            self.last_frames.append(last)
            if len(self.last_frames) > self.max_history:
                self.last_frames = self.last_frames[-self.max_history:]
    
    def _draw_header(self, stdscr, width):
        """Draw the header section."""
        stdscr.attron(self.curses.color_pair(1) | self.curses.A_BOLD)
        stdscr.addstr(0, 0, "SLIP Frame Monitor".center(width)[:width])
        stdscr.attroff(self.curses.color_pair(1) | self.curses.A_BOLD)
    
    def _draw_stats(self, stdscr, stats, row, width):
        """Draw statistics panel."""
        y = row
        
        # Title
        stdscr.attron(self.curses.color_pair(2) | self.curses.A_BOLD)
        stdscr.addstr(y, 0, "Statistics".ljust(width)[:width])
        stdscr.attroff(self.curses.color_pair(2) | self.curses.A_BOLD)
        y += 1
        
        # Stats lines
        stats_lines = [
            f"  Frames:      {stats['frames_received']:6d}  |  Errors: {stats['frames_with_errors']:3d}  |  Bad CRC: {stats['frames_with_bad_crc']:3d}",
            f"  Bytes RX:    {stats['total_bytes_received']:6d}  |  Payload: {stats['total_payload_bytes']:6d}",
            f"  Rate:        {stats['frames_per_second']:6.2f} fps  |  {stats['bytes_per_second']:8.2f} bps",
            f"  Elapsed:     {stats['elapsed_seconds']:6.1f} seconds",
        ]
        
        if stats['frames_received'] > 0 and stats['min_frame_size'] is not None:
            stats_lines.append(
                f"  Frame Size:  min={stats['min_frame_size']:3d}  max={stats['max_frame_size']:3d}  "
                f"avg={stats['avg_payload_size']:6.1f}"
            )
        
        for line in stats_lines:
            if y < stdscr.getmaxyx()[0] - 1:
                stdscr.addstr(y, 0, line.ljust(width)[:width])
                y += 1
        
        return y + 1
    
    def _draw_frames(self, stdscr, row, width, height):
        """Draw recent frames panel."""
        y = row
        max_rows = height - row - 1
        
        if max_rows < 2:
            return y
        
        # Title
        stdscr.attron(self.curses.color_pair(2) | self.curses.A_BOLD)
        stdscr.addstr(y, 0, "Recent Frames".ljust(width)[:width])
        stdscr.attroff(self.curses.color_pair(2) | self.curses.A_BOLD)
        y += 1
        max_rows -= 1
        
        # Display frames in reverse order (newest first)
        for frame in reversed(self.last_frames[-max_rows:]):
            if y >= height - 1:
                break
            
            crc_indicator = ""
            extra_crc = ""
            if frame['crc_valid'] is True:
                crc_indicator = " [✓CRC]"
            elif frame['crc_valid'] is False:
                crc_indicator = " [✗CRC]"
                if frame.get('crc_received') is not None and frame.get('crc_expected') is not None:
                    extra_crc = f" recv={frame['crc_received']:08X} exp={frame['crc_expected']:08X}"
            
            timestamp = time.strftime('%H:%M:%S', time.localtime(frame['timestamp']))
            frame_str = (
                f"  {timestamp} | {len(frame['payload']):3d} payload bytes | "
                f"HEX: {frame['raw'][:20].hex().upper()}...{crc_indicator}{extra_crc}"
            )
            
            if frame['crc_valid'] is False:
                stdscr.attron(self.curses.color_pair(3))
            
            stdscr.addstr(y, 0, frame_str.ljust(width)[:width])
            
            if frame['crc_valid'] is False:
                stdscr.attroff(self.curses.color_pair(3))
            
            y += 1
        
        return y + 1
    
    def _draw_footer(self, stdscr, height, width):
        """Draw footer with help text."""
        y = height - 1
        footer = " q: quit  | h: toggle hex  | c: clear  | Press any key for refresh"
        stdscr.attron(self.curses.color_pair(1))
        stdscr.addstr(y, 0, footer.ljust(width)[:width])
        stdscr.attroff(self.curses.color_pair(1))
    
    def _main_ui(self, stdscr):
        """Main UI loop."""
        # Setup colors
        self.curses.init_pair(1, self.curses.COLOR_WHITE, self.curses.COLOR_BLUE)
        self.curses.init_pair(2, self.curses.COLOR_CYAN, self.curses.COLOR_BLACK)
        self.curses.init_pair(3, self.curses.COLOR_RED, self.curses.COLOR_BLACK)
        
        # Non-blocking input
        stdscr.nodelay(True)
        stdscr.timeout(100)
        
        last_render = 0
        
        while self.running:
            height, width = stdscr.getmaxyx()
            
            try:
                # Read from connection
                chunk = self.connection.read(timeout=0.05)
                if chunk:
                    self.monitor.process_chunk(chunk)
            except Exception:
                pass
            
            # Render UI every 100ms
            now = time.time()
            if now - last_render > 0.1:
                try:
                    stdscr.clear()
                    
                    self._draw_header(stdscr, width)
                    row = 1
                    
                    stats = self.monitor.get_stats()
                    row = self._draw_stats(stdscr, stats, row, width)
                    
                    if row < height - 2:
                        row = self._draw_frames(stdscr, row, width, height)
                    
                    self._draw_footer(stdscr, height, width)
                    
                    stdscr.refresh()
                    last_render = now
                except self.curses.error:
                    pass
            
            # Handle input
            try:
                key = stdscr.getch()
                if key != -1:
                    if key == ord('q'):
                        self.running = False
                    elif key == ord('h'):
                        self.display_hex = not self.display_hex
                    elif key == ord('c'):
                        self.last_frames = []
            except self.curses.error:
                pass
    
    def run(self, duration=None):
        """
        Run the interactive monitor.
        
        Args:
            duration: Duration in seconds (None for infinite)
        """
        try:
            self.curses.wrapper(self._main_ui)
        except KeyboardInterrupt:
            self.running = False
        finally:
            print("\nShutting down...")
            self.monitor.close()


def create_parser() -> argparse.ArgumentParser:
    """Create and return the argument parser."""
    parser = argparse.ArgumentParser(
        description='Monitor SLIP frames from a serial port or TCP connection',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  # Monitor serial port with default settings
  monitor_slip.py /dev/ttyUSB0
  
  # Monitor with custom baudrate
  monitor_slip.py /dev/ttyUSB0:115200
  
  # Monitor TCP connection
  monitor_slip.py tcp:192.168.1.1:5000
  
  # Interactive ncurses UI mode
  monitor_slip.py -i /dev/ttyUSB0
  
  # Monitor for 30 seconds with statistics
  monitor_slip.py -t 30 /dev/ttyUSB0
  
  # Show hex dump of each frame
  monitor_slip.py -x /dev/ttyUSB0
        '''
    )
    
    parser.add_argument(
        'connection',
        help='Connection string: /dev/ttyUSB0, /dev/ttyUSB0:115200, or tcp:host:port'
    )
    
    parser.add_argument(
        '-i', '--interactive',
        action='store_true',
        help='Use interactive ncurses UI mode'
    )
    
    parser.add_argument(
        '-t', '--timeout',
        type=float,
        help='Monitor for N seconds then exit'
    )
    
    parser.add_argument(
        '-x', '--hex',
        action='store_true',
        help='Show hex dump of each frame'
    )
    
    parser.add_argument(
        '-a', '--ascii',
        action='store_true',
        help='Show ASCII representation of frames'
    )
    
    parser.add_argument(
        '--no-crc',
        action='store_true',
        help='Disable CRC32 validation'
    )
    
    return parser


def main():
    """Main entry point."""
    parser = create_parser()
    args = parser.parse_args()
    
    # Create connection
    try:
        connection = create_connection(args.connection, timeout=0.1)
        print(f"Connected to {args.connection}", file=sys.stderr)
    except Exception as e:
        print(f"Error: Failed to connect: {e}", file=sys.stderr)
        return 1
    
    try:
        # Select monitor type
        if args.interactive:
            monitor = InteractiveMonitor(connection, check_crc=not args.no_crc)
            monitor.run(duration=args.timeout)
        else:
            monitor = NonInteractiveMonitor(
                connection,
                check_crc=not args.no_crc,
                show_hex=args.hex,
                show_ascii=args.ascii
            )
            monitor.run(duration=args.timeout)
        
        # Print final statistics
        connection.close()
        time.sleep(0.1)
        
        print("\n")
        if args.interactive:
            monitor.monitor.print_stats()
        else:
            monitor.monitor.print_stats()
        
        return 0
    
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        connection.close()
        return 1


if __name__ == '__main__':
    sys.exit(main())
