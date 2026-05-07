"""
Tether CiA 402 Drive Profile Module

CiA 402 servo and stepper motor drive profile implementation.

Classes:
    CiA402Drive: Full CiA 402 drive implementation
    DriveStateMachine: State machine management
    HomingHandler: Homing operations
    MotionProfile: Motion profile generation

Example:
    >>> from tether.ethercat import master
    >>> from tether.ethercat import cia402
    >>>
    >>> # Get drive from master
    >>> m = master.Master()
    >>> m.init("eth0")
    >>> drive = cia402.CiA402Drive(m, slave_index=1)
    >>>
    >>> # Enable drive
    >>> drive.set_mode(cia402.OperationMode.PROFILE_POSITION)
    >>> drive.enable()
    >>>
    >>> # Move to position
    >>> drive.set_target_position(10000)
    >>> drive.start_motion()
"""

# Import the native extension - handle both development and installed paths
try:
    try:
        from tether._cia402 import *
    except ImportError:
        from _cia402 import *
except ImportError as e:
    raise ImportError(
        "Failed to import _cia402 native module. "
        "Ensure the package is properly installed or the native module is in the path."
    ) from e

__all__ = [
    "placeholder",  # From stub
]
