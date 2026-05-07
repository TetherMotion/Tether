"""
Tether Python Bindings - Modular EtherCAT and G-code Library

This package provides Python bindings for the Tether library, organized
into submodules for different functionality:

- tether.gcode: G-code parsing and trajectory generation
- tether.export: SVG/CSV export utilities
- tether.motion_planner: Motion planning and replanning
- tether.motion_control: Real-time motion control
- tether.controls: PID and control algorithms
- tether.ethercat: EtherCAT master/slave functionality

Submodules are lazily loaded - the shared libraries are only loaded
when the specific submodule is first imported.

Example:
    >>> import tether.gcode
    >>> parser = tether.gcode.Parser()
    >>> parser.parse_string("G0 X10 Y20")
"""

__version__ = "2.0.0"
__all__ = [
    "gcode",
    "export", 
    "motion_planner",
    "motion_control",
    "controls",
    "ethercat",
]


def __getattr__(name: str):
    """Lazy module loading - only load submodules when accessed."""
    if name == "gcode":
        from . import gcode
        return gcode
    elif name == "export":
        from . import export
        return export
    elif name == "motion_planner":
        from . import motion_planner
        return motion_planner
    elif name == "motion_control":
        from . import motion_control
        return motion_control
    elif name == "controls":
        from . import controls
        return controls
    elif name == "ethercat":
        from . import ethercat
        return ethercat
    raise AttributeError(f"module 'tether' has no attribute '{name}'")


def __dir__():
    return __all__ + ["__version__"]
