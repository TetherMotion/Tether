"""
Tether Controls Module

PID, LQR, LQG, and other control algorithms.

Classes:
    PIDController: Classic PID control
    LQRController: Linear Quadratic Regulator
    LQGController: Linear Quadratic Gaussian control
    KalmanFilter: State estimation
    FOPIDController: Fractional-order PID
    RobustController: H-infinity and mu-synthesis controllers
    AdaptiveController: Model reference adaptive control
    LearningController: Iterative learning control

Example:
    >>> import tether.controls as ctrl
    >>>
    >>> # PID control
    >>> pid = ctrl.PIDController()
    >>> pid.configure(kp=1.0, ki=0.1, kd=0.01)
    >>> output = pid.update(setpoint=100.0, measurement=95.0, dt=0.001)
    >>>
    >>> # LQR control
    >>> lqr = ctrl.LQRController()
    >>> lqr.configure(A=A_matrix, B=B_matrix, Q=Q_matrix, R=R_matrix)
    >>> u = lqr.compute(state)
"""

# Import the native extension - handle both development and installed paths
try:
    try:
        from tether._controls import *
    except ImportError:
        from _controls import *
except ImportError as e:
    raise ImportError(
        "Failed to import _controls native module. "
        "Ensure the package is properly installed or the native module is in the path."
    ) from e

__all__ = [
    "placeholder",  # From stub
]
