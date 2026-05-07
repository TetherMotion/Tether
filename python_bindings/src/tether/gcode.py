"""
Tether G-code Module

G-code parsing, interpretation, and trajectory generation.

Classes:
    Parser: G-code block parser
    Interpreter: G-code execution with trajectory generation
    Trajectory: Collection of trajectory samples
    TrajectorySample: Single trajectory point with kinematics
    PlanningSegment: Motion planning segment

Example:
    >>> import tether.gcode as gcode
    >>> 
    >>> # Parse G-code
    >>> parser = gcode.Parser()
    >>> parser.parse_string("G0 X10 Y20\\nG1 X30 Y40 F1000")
    >>> print(f"Parsed {parser.block_count()} blocks")
    >>>
    >>> # Generate trajectory  
    >>> interp = gcode.Interpreter()
    >>> interp.configure(max_vel=100.0, max_accel=500.0)
    >>> interp.process_blocks(parser)
    >>> trajectory = interp.get_trajectory()
"""

# Import the native extension - handle both development and installed paths
try:
    # First try the standard installed location
    try:
        from tether._gcode import *
        from tether._gcode import (
            Parser,
            Interpreter,
            TrajectoryGenerator,
        )
    except ImportError:
        # Fall back to standalone module (for development)
        from _gcode import *
        from _gcode import (
            Parser,
            Interpreter,
            TrajectoryGenerator,
        )
except ImportError as e:
    raise ImportError(
        "Failed to import _gcode native module. "
        "Ensure the package is properly installed or the native module is in the path."
    ) from e

# Re-export commonly used classes
__all__ = [
    "Parser",
    "Interpreter",
    "TrajectoryGenerator",
    "clear_error",
    "last_error",
    "last_error_code",
]