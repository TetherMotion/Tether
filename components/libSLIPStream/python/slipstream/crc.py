"""
CRC32 calculation and validation for SLIP frames.

This module provides CRC32 computation using the Ethernet polynomial (0x04C11DB7).
The CRC32 is typically placed in the last 4 bytes of the SLIP frame payload.

CRC32 Polynomial (Ethernet): 0x04C11DB7
This is the polynomial used in standard Ethernet and ZIP archives.
"""

import struct
from typing import Tuple


# CRC32 polynomial (Ethernet polynomial)
CRC32_POLYNOMIAL = 0x04C11DB7


def _make_crc32_table() -> list:
    """Generate the CRC32 lookup table for Ethernet polynomial."""
    table = []
    for i in range(256):
        crc = i << 24
        for _ in range(8):
            crc <<= 1
            if crc & 0x100000000:
                crc ^= (CRC32_POLYNOMIAL << 1)
        table.append(crc & 0xFFFFFFFF)
    return table


# Pre-computed CRC32 lookup table
_CRC32_TABLE = _make_crc32_table()


def calculate_crc32(data: bytes, initial: int = 0xFFFFFFFF) -> int:
    """
    Calculate CRC32 checksum using Ethernet polynomial.
    
    Args:
        data: Bytes to calculate CRC32 for
        initial: Initial CRC value (default: 0xFFFFFFFF for standard CRC32)
        
    Returns:
        CRC32 value as a 32-bit integer
    """
    crc = initial
    
    for byte in data:
        crc = ((crc << 8) ^ _CRC32_TABLE[((crc >> 24) ^ byte) & 0xFF]) & 0xFFFFFFFF
    
    return crc


def verify_crc32(data: bytes, stored_crc: bytes) -> bool:
    """
    Verify CRC32 checksum in data.
    
    The CRC should be stored as the last 4 bytes of the data in little-endian format.
    
    Args:
        data: Complete frame data (excluding SLIP framing)
        stored_crc: The 4-byte CRC value from the frame
        
    Returns:
        True if CRC is valid, False otherwise
    """
    if len(stored_crc) != 4:
        return False
    
    # Calculate CRC32 of all data except the CRC field itself
    calculated = calculate_crc32(data)
    
    # Convert stored CRC from bytes (little-endian)
    stored = struct.unpack('<I', stored_crc)[0]
    
    return calculated == stored


def append_crc32(data: bytes) -> bytes:
    """
    Append CRC32 checksum to data.
    
    Args:
        data: Bytes to append CRC to
        
    Returns:
        Original data with 4-byte CRC32 appended (little-endian)
    """
    crc = calculate_crc32(data)
    return data + struct.pack('<I', crc)


def extract_crc32(data: bytes) -> Tuple[bytes, bytes]:
    """
    Extract CRC32 from the last 4 bytes of data.
    
    Args:
        data: Complete frame data (with CRC in last 4 bytes)
        
    Returns:
        Tuple of (payload, crc_bytes)
        - payload: All data except last 4 bytes
        - crc_bytes: The last 4 bytes (the CRC)
    """
    if len(data) < 4:
        raise ValueError("Data too short to contain CRC32")
    
    return data[:-4], data[-4:]


def crc32_to_hex(crc_value: int) -> str:
    """
    Format a CRC32 value as a hex string (little-endian byte order).
    
    Args:
        crc_value: CRC32 value as integer
        
    Returns:
        Hex string representation (e.g., "12345678")
    """
    return struct.pack('<I', crc_value).hex().upper()


def hex_to_crc32(hex_str: str) -> int:
    """
    Parse a hex string into a CRC32 value (little-endian byte order).
    
    Args:
        hex_str: Hex string (e.g., "12345678")
        
    Returns:
        CRC32 value as integer
    """
    return struct.unpack('<I', bytes.fromhex(hex_str))[0]


def diagnose_crc_error(data: bytes, stored_crc: bytes) -> dict:
    """Diagnose why a CRC32 validation failed."""
    if len(stored_crc) != 4:
        return {
            'error': 'invalid_crc_length',
            'received': None,
            'expected': None,
            'diagnosis': 'CRC field must be 4 bytes long'
        }

    expected = calculate_crc32(data)
    received = struct.unpack('<I', stored_crc)[0]
    expected_bytes = expected.to_bytes(4, 'little')
    received_big_endian = int.from_bytes(expected_bytes[::-1], 'little')
    expected_xor = (~expected) & 0xFFFFFFFF
    expected_xor_be = int.from_bytes(expected_xor.to_bytes(4, 'little')[::-1], 'little')

    diagnostics = []
    if received == expected_xor:
        diagnostics.append('missing final XOR (0xFFFFFFFF)')
    if received == received_big_endian:
        diagnostics.append('CRC bytes appear swapped (endianness mismatch)')
    if received == expected_xor_be:
        diagnostics.append('CRC bytes swapped and final XOR missing')
    if not diagnostics:
        diagnostics.append('CRC mismatch')

    return {
        'received': received,
        'expected': expected,
        'expected_xor': expected_xor,
        'received_big_endian': received_big_endian,
        'expected_xor_be': expected_xor_be,
        'diagnosis': '; '.join(diagnostics)
    }
