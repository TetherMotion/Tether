"""
High-level SLIP frame monitoring interface.

Combines connection handling, SLIP decoding, CRC validation, and statistics.
"""

import struct
import time
from typing import Optional, Callable, Dict, Any
from .slip import StreamingDecoder
from .crc import extract_crc32, verify_crc32, calculate_crc32, diagnose_crc_error
from .stats import FrameStatistics
from .connections import Connection


class FrameMonitor:
    """
    High-level SLIP frame monitor with CRC validation and statistics.
    """
    
    def __init__(
        self,
        connection: Connection,
        frame_callback: Optional[Callable[[bytes], None]] = None,
        check_crc: bool = True,
        hex_output: bool = False
    ):
        """
        Initialize frame monitor.
        
        Args:
            connection: Connection object (SerialConnection or TCPConnection)
            frame_callback: Optional callback for each complete frame
                           Signature: callback(frame_data: bytes)
            check_crc: Whether to validate CRC32 on frames (expects last 4 bytes to be CRC)
            hex_output: Whether to display frames in hex format
        """
        self.connection = connection
        self.frame_callback = frame_callback
        self.check_crc = check_crc
        self.hex_output = hex_output
        
        self.stats = FrameStatistics()
        self.last_frame = None
        self.last_frame_crc_valid = None
        
        # Create streaming decoder
        self.decoder = StreamingDecoder(callback=self._handle_frame)
    
    def _handle_frame(self, frame_data: bytes) -> None:
        """Internal callback for decoded frames."""
        crc_valid = None
        crc_received = None
        crc_expected = None
        crc_diagnostic = None
        payload = frame_data
        
        # Check CRC if enabled
        if self.check_crc and len(frame_data) >= 4:
            try:
                payload, crc_bytes = extract_crc32(frame_data)
                crc_valid = verify_crc32(payload, crc_bytes)
                crc_received = int.from_bytes(crc_bytes, 'little')
                crc_expected = calculate_crc32(payload)
                if not crc_valid:
                    diag = diagnose_crc_error(payload, crc_bytes)
                    crc_diagnostic = diag.get('diagnosis')
            except (ValueError, struct.error):
                crc_valid = False
                crc_diagnostic = 'Frame too short or invalid CRC field'
        
        # Record statistics
        self.stats.add_frame(
            raw_frame_len=len(frame_data),
            payload_len=len(payload),
            crc_valid=crc_valid
        )
        
        self.last_frame = {
            'raw': frame_data,
            'payload': payload,
            'crc_valid': crc_valid,
            'crc_received': crc_received,
            'crc_expected': crc_expected,
            'crc_diagnostic': crc_diagnostic,
            'timestamp': time.time()
        }
        
        # Call user callback if provided
        if self.frame_callback:
            self.frame_callback(frame_data)
    
    def process_chunk(self, chunk: bytes) -> None:
        """
        Process a chunk of raw data from the connection.
        
        Args:
            chunk: Raw bytes from the connection
        """
        if chunk:
            self.decoder.feed(chunk)
    
    def monitor(self, duration: Optional[float] = None) -> None:
        """
        Monitor the connection for SLIP frames.
        
        Args:
            duration: Monitor for this many seconds (None for infinite)
        """
        start_time = time.time()
        
        while True:
            # Check timeout
            if duration is not None:
                elapsed = time.time() - start_time
                if elapsed >= duration:
                    break
            
            # Read data from connection
            try:
                chunk = self.connection.read(timeout=0.1)
                if chunk:
                    self.process_chunk(chunk)
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"Error reading from connection: {e}")
                break
    
    def get_last_frame(self) -> Optional[Dict[str, Any]]:
        """Get the last received frame."""
        return self.last_frame
    
    def get_stats(self) -> Dict[str, Any]:
        """Get current statistics."""
        return self.stats.get_stats()
    
    def print_stats(self) -> None:
        """Print statistics report."""
        self.stats.print_report()
    
    def close(self) -> None:
        """Close the connection."""
        self.connection.close()


def hexlify_frame(frame: bytes, width: int = 16) -> str:
    """
    Convert a frame to hex representation.
    
    Args:
        frame: Bytes to convert
        width: Bytes per row in output
        
    Returns:
        Formatted hex string
    """
    lines = []
    for i in range(0, len(frame), width):
        chunk = frame[i:i+width]
        hex_part = ' '.join(f'{b:02X}' for b in chunk)
        ascii_part = ''.join(chr(b) if 32 <= b < 127 else '.' for b in chunk)
        lines.append(f"{i:04X}: {hex_part:<{width*3-1}}  {ascii_part}")
    return '\n'.join(lines)
