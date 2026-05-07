"""
SLIP (Serial Line Internet Protocol) encoder and decoder.

This module provides functions to encode and decode data using the SLIP protocol.
The SLIP protocol uses special escape sequences to frame messages:
- 0xC0 (END): End of packet marker
- 0xDB (ESC): Escape character
- 0xDC (ESCEND): Escaped END byte (represents 0xC0 in data)
- 0xDD (ESCESC): Escaped ESC byte (represents 0xDB in data)

Reference: RFC 1055 - Nonstandard Transmission of IP Datagrams over Serial Lines: SLIP
"""

from typing import Tuple

# SLIP special byte definitions
END = 0xC0      # End of packet
ESC = 0xDB      # Escape character
ESCEND = 0xDC   # Escaped END (represents literal 0xC0)
ESCESC = 0xDD   # Escaped ESC (represents literal 0xDB)


def encode_packet(data: bytes) -> bytes:
    """
    Encode a packet using SLIP encoding.
    
    Args:
        data: Raw bytes to encode
        
    Returns:
        SLIP-encoded bytes with END marker appended
    """
    encoded = bytearray()
    
    for byte in data:
        if byte == END:
            # Escape END byte
            encoded.append(ESC)
            encoded.append(ESCEND)
        elif byte == ESC:
            # Escape ESC byte
            encoded.append(ESC)
            encoded.append(ESCESC)
        else:
            # Regular byte, no escaping needed
            encoded.append(byte)
    
    # Append END marker to indicate end of packet
    encoded.append(END)
    
    return bytes(encoded)


def decode_packet(data: bytes) -> Tuple[bytes, int]:
    """
    Decode a SLIP-encoded packet from a buffer.
    
    Decoding stops at the first END byte encountered.
    
    Args:
        data: Buffer containing SLIP-encoded data
        
    Returns:
        Tuple of (decoded_bytes, bytes_consumed)
        - decoded_bytes: The decoded payload
        - bytes_consumed: Number of input bytes consumed (including END marker)
        
    Raises:
        ValueError: If the data contains invalid escape sequences or no END marker
    """
    decoded = bytearray()
    i = 0
    
    while i < len(data):
        byte = data[i]
        
        if byte == END:
            # End of packet reached
            return bytes(decoded), i + 1
        
        elif byte == ESC:
            # Escape sequence - need next byte
            if i + 1 >= len(data):
                raise ValueError("Incomplete escape sequence at end of buffer")
            
            next_byte = data[i + 1]
            if next_byte == ESCEND:
                decoded.append(END)
                i += 2
            elif next_byte == ESCESC:
                decoded.append(ESC)
                i += 2
            else:
                raise ValueError(f"Invalid escape sequence: ESC followed by 0x{next_byte:02X}")
        
        else:
            # Regular byte
            decoded.append(byte)
            i += 1
    
    # No END marker found
    raise ValueError("No END marker found in data")


class StreamingDecoder:
    """
    Stateful SLIP decoder for processing continuous streams of data.
    
    Usage:
        decoder = StreamingDecoder(callback=my_frame_handler)
        decoder.feed(serial_data)
        decoder.feed(more_data)
    """
    
    def __init__(self, callback=None):
        """
        Initialize the streaming decoder.
        
        Args:
            callback: Optional function to call when a frame is decoded.
                     Signature: callback(decoded_bytes: bytes)
        """
        self.buffer = bytearray()
        self.escape_next = False
        self.callback = callback
        self.frames_received = 0
        self.frames_with_errors = 0
    
    def feed(self, data: bytes) -> None:
        """
        Feed data into the decoder and process new frames.
        
        Args:
            data: Bytes to be decoded
        """
        for byte in data:
            self._process_byte(byte)
    
    def _process_byte(self, byte: int) -> None:
        """Process a single byte from the stream."""
        if self.escape_next:
            # Previous byte was ESC, handle escape sequence
            if byte == ESCEND:
                self.buffer.append(END)
            elif byte == ESCESC:
                self.buffer.append(ESC)
            else:
                # Invalid escape sequence - discard frame
                self.buffer.clear()
                self.frames_with_errors += 1
            self.escape_next = False
        
        elif byte == END:
            # End of frame marker
            if len(self.buffer) > 0:
                # We have a complete frame
                self.frames_received += 1
                if self.callback:
                    self.callback(bytes(self.buffer))
                self.buffer.clear()
            # Even if buffer is empty, ignore empty frames
        
        elif byte == ESC:
            # Start of escape sequence
            self.escape_next = True
        
        else:
            # Regular byte
            self.buffer.append(byte)
    
    def reset(self) -> None:
        """Reset the decoder state (discard any partial frames)."""
        self.buffer.clear()
        self.escape_next = False
    
    def get_stats(self) -> dict:
        """Get decoding statistics."""
        return {
            'frames_received': self.frames_received,
            'frames_with_errors': self.frames_with_errors,
        }
