"""
Tether CiA 401 I/O Profile Module

CiA 401 digital and analog I/O module profile implementation.

Classes:
    CiA401IO: Full CiA 401 I/O module implementation
    DigitalInputs: Digital input handling
    DigitalOutputs: Digital output handling
    AnalogInputs: Analog input handling
    AnalogOutputs: Analog output handling

Example:
    >>> from tether.ethercat import master
    >>> from tether.ethercat import cia401
    >>>
    >>> # Get I/O module from master
    >>> m = master.Master()
    >>> m.init("eth0")
    >>> io = cia401.CiA401IO(m, slave_index=2)
    >>>
    >>> # Read digital inputs
    >>> inputs = io.read_digital_inputs()
    >>> print(f"Input 0: {inputs[0]}")
    >>>
    >>> # Set digital outputs
    >>> io.set_digital_output(0, True)
"""

# Import the native extension - handle both development and installed paths
try:
    try:
        from tether._cia401 import *
    except ImportError:
        from _cia401 import *
except ImportError as e:
    raise ImportError(
        "Failed to import _cia401 native module. "
        "Ensure the package is properly installed or the native module is in the path."
    ) from e

__all__ = [
    "placeholder",  # From stub
]
