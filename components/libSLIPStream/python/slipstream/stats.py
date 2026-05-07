"""
Statistics tracking for SLIP frame monitoring.

Tracks frame counts, byte counts, error counts, and timing information.
"""

import time
from typing import Dict, Any


class FrameStatistics:
    """
    Collects statistics about received SLIP frames.
    """
    
    def __init__(self):
        """Initialize statistics tracker."""
        self.start_time = time.time()
        self.frames_received = 0
        self.frames_with_bad_crc = 0
        self.frames_with_errors = 0
        self.total_bytes_received = 0
        self.total_payload_bytes = 0
        self.min_frame_size = None
        self.max_frame_size = None
        self.min_payload_size = None
        self.max_payload_size = None
        self.last_frame_time = None
    
    def add_frame(self, raw_frame_len: int, payload_len: int, crc_valid: bool = None) -> None:
        """
        Record receipt of a frame.
        
        Args:
            raw_frame_len: Length of raw encoded frame (including SLIP markers)
            payload_len: Length of decoded payload
            crc_valid: Whether CRC was valid (None if not checked)
        """
        self.frames_received += 1
        self.total_bytes_received += raw_frame_len
        self.total_payload_bytes += payload_len
        self.last_frame_time = time.time()
        
        # Track frame size statistics
        if self.min_frame_size is None or raw_frame_len < self.min_frame_size:
            self.min_frame_size = raw_frame_len
        if self.max_frame_size is None or raw_frame_len > self.max_frame_size:
            self.max_frame_size = raw_frame_len
        
        if self.min_payload_size is None or payload_len < self.min_payload_size:
            self.min_payload_size = payload_len
        if self.max_payload_size is None or payload_len > self.max_payload_size:
            self.max_payload_size = payload_len
        
        if crc_valid is False:
            self.frames_with_bad_crc += 1
    
    def add_error(self) -> None:
        """Record a frame with decoding/parsing errors."""
        self.frames_with_errors += 1
    
    def get_stats(self) -> Dict[str, Any]:
        """
        Get all statistics as a dictionary.
        
        Returns:
            Dictionary with all tracked statistics
        """
        elapsed = time.time() - self.start_time
        
        stats = {
            'elapsed_seconds': elapsed,
            'frames_received': self.frames_received,
            'frames_with_bad_crc': self.frames_with_bad_crc,
            'frames_with_errors': self.frames_with_errors,
            'total_bytes_received': self.total_bytes_received,
            'total_payload_bytes': self.total_payload_bytes,
            'frames_per_second': self.frames_received / elapsed if elapsed > 0 else 0,
            'bytes_per_second': self.total_bytes_received / elapsed if elapsed > 0 else 0,
            'avg_payload_size': (
                self.total_payload_bytes / self.frames_received
                if self.frames_received > 0 else 0
            ),
            'min_frame_size': self.min_frame_size,
            'max_frame_size': self.max_frame_size,
            'min_payload_size': self.min_payload_size,
            'max_payload_size': self.max_payload_size,
        }
        
        return stats
    
    def print_report(self) -> None:
        """Print a human-readable statistics report."""
        stats = self.get_stats()
        
        print("\n" + "=" * 60)
        print("SLIP FRAME STATISTICS")
        print("=" * 60)
        print(f"Elapsed Time:          {stats['elapsed_seconds']:.2f} seconds")
        print(f"Total Frames:          {stats['frames_received']}")
        print(f"Frames with Bad CRC:   {stats['frames_with_bad_crc']}")
        print(f"Frames with Errors:    {stats['frames_with_errors']}")
        print(f"Total Bytes Received:  {stats['total_bytes_received']}")
        print(f"Total Payload Bytes:   {stats['total_payload_bytes']}")
        print(f"Frames per Second:     {stats['frames_per_second']:.2f}")
        print(f"Bytes per Second:      {stats['bytes_per_second']:.2f}")
        
        if stats['frames_received'] > 0:
            print(f"\nFrame Statistics:")
            print(f"  Min Frame Size:      {stats['min_frame_size']} bytes")
            print(f"  Max Frame Size:      {stats['max_frame_size']} bytes")
            print(f"  Avg Payload Size:    {stats['avg_payload_size']:.2f} bytes")
            if stats['min_payload_size'] is not None:
                print(f"  Min Payload Size:    {stats['min_payload_size']} bytes")
                print(f"  Max Payload Size:    {stats['max_payload_size']} bytes")
        
        print("=" * 60)
