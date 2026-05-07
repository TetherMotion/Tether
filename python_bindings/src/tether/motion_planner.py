"""
Tether Motion Planner Module

Motion planning, replanning, and machine testing utilities.

Classes:
    MotionReplanner: Dynamic motion replanning
    MachineTester: Machine performance testing
    SystemIdentifier: System identification utilities
    PerformanceHeatmap: Generate performance heatmaps
    GCodeGenerator: Generate G-code from test patterns

Example:
    >>> import tether.motion_planner as planner
    >>>
    >>> # Test machine performance
    >>> tester = planner.MachineTester()
    >>> tester.configure(max_velocity=100.0, max_accel=500.0)
    >>> results = tester.run_test_pattern("zigzag")
    >>>
    >>> # Generate performance heatmap
    >>> heatmap = planner.PerformanceHeatmap()
    >>> heatmap.add_results(results)
    >>> heatmap.export_svg("performance.svg")
"""

# Import the native extension - handle both development and installed paths
try:
    try:
        from tether._motion_planner import *
    except ImportError:
        from _motion_planner import *
except ImportError as e:
    raise ImportError(
        "Failed to import _motion_planner native module. "
        "Ensure the package is properly installed or the native module is in the path."
    ) from e

__all__ = [
    "placeholder",  # From stub
]