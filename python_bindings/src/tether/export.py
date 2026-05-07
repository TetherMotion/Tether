"""
Tether Export Module

SVG and CSV export utilities for trajectory visualization and analysis.

Classes:
    SVGExporter: Export trajectories to SVG files
    SVGConfig: Configuration for SVG export
    CSVExporter: Export trajectories to CSV files
    TrajectoryAnalyzer: Analyze trajectory kinematics

Example:
    >>> import tether.gcode as gcode
    >>> import tether.export as export
    >>>
    >>> # Generate trajectory
    >>> parser = gcode.Parser()
    >>> parser.parse_string("G0 X0 Y0\\nG1 X100 Y100 F1000")
    >>> interp = gcode.Interpreter()
    >>> interp.process_blocks(parser)
    >>> trajectory = interp.get_trajectory()
    >>>
    >>> # Export to SVG
    >>> svg = export.SVGExporter()
    >>> svg.export_to_file("output.svg", trajectory)
    >>>
    >>> # Analyze trajectory
    >>> analyzer = export.TrajectoryAnalyzer()
    >>> stats = analyzer.analyze(trajectory)
    >>> print(f"Max velocity: {stats.max_velocity}")
"""

# Import the native extension - handle both development and installed paths
try:
    try:
        from tether._export import *
        from tether._export import (
            SVGExporter,
            SVGConfig,
            CSVExporter,
            TrajectoryAnalyzer,
        )
    except ImportError:
        from _export import *
        from _export import (
            SVGExporter,
            SVGConfig,
            CSVExporter,
            TrajectoryAnalyzer,
        )
except ImportError as e:
    raise ImportError(
        "Failed to import _export native module. "
        "Ensure the package is properly installed or the native module is in the path."
    ) from e

__all__ = [
    "SVGExporter",
    "SVGConfig",
    "CSVExporter",
    "TrajectoryAnalyzer",
]