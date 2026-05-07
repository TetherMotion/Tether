"""
Tether EtherCAT Slave Module

EtherCAT slave emulation for testing and simulation.

Classes:
    SlaveCore: Core slave state machine
    SlaveProfile: Base class for profile implementation
    SlaveHAL: Hardware abstraction for slave

Example:
    >>> from tether.ethercat import slave
    >>>
    >>> # Create slave emulator
    >>> s = slave.SlaveCore()
    >>> s.configure(vendor_id=0x1234, product_code=0x5678)
    >>>
    >>> # Add CiA 402 profile
    >>> s.add_profile(slave.CiA402Profile())
    >>>
    >>> # Start slave
    >>> s.start()
"""

# Import the native extension - handle both development and installed paths
try:
    try:
        from tether._ethercat_slave import *
    except ImportError:
        from _ethercat_slave import *
except ImportError as e:
    raise ImportError(
        "Failed to import _ethercat_slave native module. "
        "Ensure the package is properly installed or the native module is in the path."
    ) from e

__all__ = [
    "placeholder",  # From stub
]
