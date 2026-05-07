"""
Tether Motion Control Module

Real-time motion control, motor models, and profile generation.

Classes:
    MotionGenerator: Generate motion profiles
    MotionController: Real-time motion control
    MotorModel: Motor dynamics simulation
    ProfileGenerator: Velocity/acceleration profiles

Example:
    >>> import tether.motion_control as mc
    >>>
    >>> # Create motion profile
    >>> profile = mc.ProfileGenerator()
    >>> profile.configure(max_vel=100.0, max_accel=500.0, max_jerk=2000.0)
    >>> segments = profile.generate_trapezoidal(distance=100.0)
    >>>
    >>> # Simulate motor response
    >>> motor = mc.MotorModel()
    >>> motor.configure(inertia=0.001, friction=0.01)
    >>> response = motor.simulate(segments)
"""

# Import the native extension - handle both development and installed paths
try:
    try:
        from tether._motion_control import *
    except ImportError:
        from _motion_control import *
except ImportError as e:
    raise ImportError(
        "Failed to import _motion_control native module. "
        "Ensure the package is properly installed or the native module is in the path."
    ) from e

__all__ = [
    "placeholder",  # From stub
]