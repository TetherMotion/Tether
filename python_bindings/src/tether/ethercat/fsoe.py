"""
Tether FSoE (Fail-Safe over EtherCAT) Module

Safety communication protocol implementation.

Classes:
    FSoEConnection: FSoE connection management
    FSoESlave: FSoE slave implementation
    SafetyData: Safety I/O data handling

Example:
    >>> from tether.ethercat import fsoe
    >>>
    >>> # Create FSoE connection
    >>> conn = fsoe.FSoEConnection()
    >>> conn.configure(connection_id=1, watchdog_time=100)
    >>>
    >>> # Exchange safety data
    >>> safe_inputs = conn.read_safe_inputs()
    >>> conn.write_safe_outputs(outputs)
"""

# Import the native extension - handle both development and installed paths
try:
    try:
        from tether._fsoe import *
    except ImportError:
        from _fsoe import *
except ImportError as e:
    raise ImportError(
        "Failed to import _fsoe native module. "
        "Ensure the package is properly installed or the native module is in the path."
    ) from e

__all__ = [
    "placeholder",  # From stub
]
