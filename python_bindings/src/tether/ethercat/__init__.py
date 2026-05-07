"""
Tether EtherCAT Module

EtherCAT master and slave functionality with CiA device profiles.

Submodules:
    ethercat.master: EtherCAT master implementation
    ethercat.slave: EtherCAT slave emulation
    ethercat.cia402: CiA 402 drive profile (servo/stepper)
    ethercat.cia401: CiA 401 I/O module profile
    ethercat.fsoe: Fail-safe over EtherCAT

Example:
    >>> import tether.ethercat as ecat
    >>>
    >>> # Create EtherCAT master
    >>> master = ecat.Master()
    >>> master.init("eth0")
    >>> master.scan_network()
    >>>
    >>> # Configure CiA 402 drive
    >>> drive = ecat.CiA402Drive(master, slave_index=1)
    >>> drive.enable()
    >>> drive.set_target_position(10000)
"""

# Lazy load submodules
def __getattr__(name: str):
    if name == "master":
        from tether.ethercat import master
        return master
    elif name == "slave":
        from tether.ethercat import slave
        return slave
    elif name == "cia402":
        from tether.ethercat import cia402
        return cia402
    elif name == "cia401":
        from tether.ethercat import cia401
        return cia401
    elif name == "fsoe":
        from tether.ethercat import fsoe
        return fsoe
    raise AttributeError(f"module 'tether.ethercat' has no attribute '{name}'")


# Import common types that are always needed
try:
    try:
        from tether._ethercat_common import *
    except ImportError:
        from _ethercat_common import *
except ImportError as e:
    raise ImportError(
        "Failed to import _ethercat_common native module. "
        "Ensure the package is properly installed or the native module is in the path."
    ) from e

__all__ = [
    "master",
    "slave",
    "cia402",
    "cia401",
    "fsoe",
    # Common types
    "SlaveState",
]