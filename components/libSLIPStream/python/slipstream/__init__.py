"""
slipstream: Python library for SLIP frame monitoring and processing.

This library provides comprehensive SLIP (Serial Line Internet Protocol) support including:
- Encoding and decoding of SLIP packets
- Streaming/continuous frame decoding
- CRC32 validation (Ethernet polynomial)
- Serial port and TCP connection handling
- Frame statistics tracking
- Hex formatting utilities

Basic Usage:
    from slipstream import FrameMonitor, create_connection
    
    connection = create_connection('/dev/ttyUSB0:115200')
    monitor = FrameMonitor(connection, check_crc=True)
    monitor.monitor(duration=10)
    monitor.print_stats()

References:
    - RFC 1055: Nonstandard Transmission of IP Datagrams over Serial Lines: SLIP
    - Ethernet polynomial CRC32: 0x04C11DB7
"""

from .slip import encode_packet, decode_packet, StreamingDecoder, END, ESC, ESCEND, ESCESC
from .crc import calculate_crc32, verify_crc32, append_crc32, extract_crc32, crc32_to_hex, hex_to_crc32
from .stats import FrameStatistics
from .connections import Connection, SerialConnection, TCPConnection, create_connection
from .streaming import FrameMonitor, hexlify_frame

__version__ = "1.0.0"
__author__ = "Uli Köhler"

__all__ = [
    # SLIP encoding/decoding
    'encode_packet',
    'decode_packet',
    'StreamingDecoder',
    'END',
    'ESC',
    'ESCEND',
    'ESCESC',
    
    # CRC32
    'calculate_crc32',
    'verify_crc32',
    'append_crc32',
    'extract_crc32',
    'crc32_to_hex',
    'hex_to_crc32',
    
    # Statistics
    'FrameStatistics',
    
    # Connections
    'Connection',
    'SerialConnection',
    'TCPConnection',
    'create_connection',
    
    # High-level monitoring
    'FrameMonitor',
    'hexlify_frame',
]
