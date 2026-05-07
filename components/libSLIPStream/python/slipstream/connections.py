"""
Connection handlers for serial ports and TCP sockets.

Provides unified interface for reading from serial ports or TCP connections.
"""

import socket
import time
from typing import Optional, Union
from abc import ABC, abstractmethod


class Connection(ABC):
    """Abstract base class for connections."""
    
    @abstractmethod
    def read(self, timeout: Optional[float] = None) -> bytes:
        """
        Read available data from the connection.
        
        Args:
            timeout: Timeout in seconds (None for blocking)
            
        Returns:
            Bytes read, or empty bytes if timeout
        """
        pass
    
    @abstractmethod
    def write(self, data: bytes) -> int:
        """
        Write data to the connection.
        
        Args:
            data: Bytes to write
            
        Returns:
            Number of bytes written
        """
        pass
    
    @abstractmethod
    def close(self) -> None:
        """Close the connection."""
        pass
    
    @abstractmethod
    def is_open(self) -> bool:
        """Check if connection is open."""
        pass


class SerialConnection(Connection):
    """Handler for serial port connections."""
    
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.1):
        """
        Initialize serial connection.
        
        Args:
            port: Serial port name (e.g., '/dev/ttyUSB0', 'COM3')
            baudrate: Baud rate (default: 115200)
            timeout: Read timeout in seconds
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self._open()
    
    def _open(self) -> None:
        """Open the serial connection."""
        try:
            import serial
        except ImportError:
            raise ImportError("pyserial is required for serial connections. Install with: pip install pyserial")
        
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
        except Exception as e:
            raise RuntimeError(f"Failed to open serial port {self.port}: {e}")
    
    def read(self, timeout: Optional[float] = None) -> bytes:
        """Read data from serial port."""
        if not self.is_open():
            return b''
        
        if timeout is not None:
            old_timeout = self.serial.timeout
            self.serial.timeout = timeout
            data = self.serial.read(4096)
            self.serial.timeout = old_timeout
            return data
        else:
            return self.serial.read(4096)
    
    def write(self, data: bytes) -> int:
        """Write data to serial port."""
        if not self.is_open():
            return 0
        return self.serial.write(data)
    
    def close(self) -> None:
        """Close the serial connection."""
        if self.serial:
            self.serial.close()
            self.serial = None
    
    def is_open(self) -> bool:
        """Check if serial connection is open."""
        return self.serial is not None and self.serial.is_open


class TCPConnection(Connection):
    """Handler for TCP socket connections."""
    
    def __init__(self, host: str, port: int, timeout: float = 0.1):
        """
        Initialize TCP connection.
        
        Args:
            host: Hostname or IP address
            port: Port number
            timeout: Socket timeout in seconds
        """
        self.host = host
        self.port = port
        self.timeout = timeout
        self.socket = None
        self._open()
    
    def _open(self) -> None:
        """Open the TCP connection."""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(self.timeout)
            self.socket.connect((self.host, self.port))
        except Exception as e:
            self.socket = None
            raise RuntimeError(f"Failed to connect to {self.host}:{self.port}: {e}")
    
    def read(self, timeout: Optional[float] = None) -> bytes:
        """Read data from TCP socket."""
        if not self.is_open():
            return b''
        
        try:
            if timeout is not None:
                self.socket.settimeout(timeout)
                data = self.socket.recv(4096)
                self.socket.settimeout(self.timeout)
                return data
            else:
                return self.socket.recv(4096)
        except socket.timeout:
            return b''
        except Exception:
            return b''
    
    def write(self, data: bytes) -> int:
        """Write data to TCP socket."""
        if not self.is_open():
            return 0
        
        try:
            return self.socket.send(data)
        except Exception:
            return 0
    
    def close(self) -> None:
        """Close the TCP connection."""
        if self.socket:
            try:
                self.socket.close()
            except Exception:
                pass
            self.socket = None
    
    def is_open(self) -> bool:
        """Check if TCP connection is open."""
        return self.socket is not None


def create_connection(connection_string: str, timeout: float = 0.1) -> Connection:
    """
    Create a connection from a connection string.
    
    Args:
        connection_string: 
            - For serial: '/dev/ttyUSB0' or 'COM3' or '/dev/ttyUSB0:115200'
            - For TCP: 'tcp:192.168.1.1:5000' or 'localhost:5000'
        timeout: Read timeout in seconds
        
    Returns:
        Connection object (SerialConnection or TCPConnection)
    """
    if connection_string.startswith('tcp:'):
        # TCP connection
        tcp_part = connection_string[4:]
        parts = tcp_part.rsplit(':', 1)
        if len(parts) != 2:
            raise ValueError(f"Invalid TCP connection string: {connection_string}")
        host, port_str = parts
        try:
            port = int(port_str)
        except ValueError:
            raise ValueError(f"Invalid port number in connection string: {port_str}")
        return TCPConnection(host, port, timeout=timeout)
    else:
        # Serial connection
        if ':' in connection_string:
            port, baud_str = connection_string.rsplit(':', 1)
            try:
                baudrate = int(baud_str)
            except ValueError:
                raise ValueError(f"Invalid baudrate in connection string: {baud_str}")
        else:
            port = connection_string
            baudrate = 115200
        
        return SerialConnection(port, baudrate=baudrate, timeout=timeout)
