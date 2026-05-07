#!/usr/bin/env python3
"""
Motion Replanner Visualization Suite

Comprehensive visualization tools for motion replanning analysis:
- Trajectory comparison plots
- Error analysis and statistics
- Performance heatmaps (1D, 2D, 3D)
- System identification results
- Test result reports

Requirements:
    pip install numpy matplotlib pandas scipy seaborn
"""

import argparse
import json
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any
import warnings

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.colors import Normalize, TwoSlopeNorm
from matplotlib.patches import Ellipse
import matplotlib.cm as cm

try:
    import seaborn as sns
    HAS_SEABORN = True
except ImportError:
    HAS_SEABORN = False
    warnings.warn("seaborn not available, using matplotlib defaults")


# =============================================================================
# Configuration
# =============================================================================

class PlotConfig:
    """Global plot configuration"""
    
    def __init__(self):
        self.figure_dpi = 150
        self.figure_size = (12, 8)
        self.font_size = 10
        self.title_size = 12
        self.colormap = 'viridis'
        self.error_colormap = 'RdYlGn_r'  # Red = bad, Green = good
        self.line_width = 1.5
        self.marker_size = 3
        self.alpha = 0.8
        self.grid_alpha = 0.3
        
        # Trajectory analysis sampling
        self.sampling_rate_hz = 100000      # 100kHz = 0.00001s timestep
        self.time_step = 1.0 / self.sampling_rate_hz  # 10µs
        
        # Style
        self.style = 'seaborn-v0_8-whitegrid' if HAS_SEABORN else 'ggplot'
        
    def set_sampling_rate(self, rate_hz: float):
        """Set the trajectory sampling rate"""
        self.sampling_rate_hz = rate_hz
        self.time_step = 1.0 / rate_hz
        
    def apply(self):
        """Apply configuration to matplotlib"""
        try:
            plt.style.use(self.style)
        except:
            plt.style.use('ggplot')
        
        plt.rcParams['figure.dpi'] = self.figure_dpi
        plt.rcParams['font.size'] = self.font_size
        plt.rcParams['axes.titlesize'] = self.title_size
        plt.rcParams['axes.labelsize'] = self.font_size
        plt.rcParams['xtick.labelsize'] = self.font_size - 1
        plt.rcParams['ytick.labelsize'] = self.font_size - 1
        plt.rcParams['legend.fontsize'] = self.font_size - 1


CONFIG = PlotConfig()


# =============================================================================
# Data Loading and Processing
# =============================================================================

def load_csv(filename: str, **kwargs) -> pd.DataFrame:
    """Load CSV file with automatic comment handling"""
    return pd.read_csv(filename, comment='#', **kwargs)


def load_json(filename: str) -> Dict:
    """Load JSON file"""
    with open(filename, 'r') as f:
        return json.load(f)


def resample_trajectory(df: pd.DataFrame, target_rate_hz: float = None) -> pd.DataFrame:
    """
    Resample trajectory data to specified sampling rate using linear interpolation.
    
    Args:
        df: DataFrame with 'time' column and position columns
        target_rate_hz: Target sampling rate in Hz (default: CONFIG.sampling_rate_hz)
    
    Returns:
        DataFrame resampled at the target rate
    """
    if target_rate_hz is None:
        target_rate_hz = CONFIG.sampling_rate_hz
    
    target_dt = 1.0 / target_rate_hz
    
    # Create new time array
    t_start = df['time'].iloc[0]
    t_end = df['time'].iloc[-1]
    new_time = np.arange(t_start, t_end, target_dt)
    
    # Interpolate all columns except time
    result = pd.DataFrame({'time': new_time})
    
    for col in df.columns:
        if col != 'time':
            result[col] = np.interp(new_time, df['time'], df[col])
    
    return result


def compute_derivatives(df: pd.DataFrame, dt: float = None) -> pd.DataFrame:
    """
    Compute velocity, acceleration, and jerk from position data using central differences.
    
    Args:
        df: DataFrame with 'time' and position columns (x, y, z)
        dt: Time step (default: CONFIG.time_step)
    
    Returns:
        DataFrame with added velocity, acceleration, and jerk columns
    """
    if dt is None:
        dt = CONFIG.time_step
    
    result = df.copy()
    
    # Compute derivatives using numpy gradient (central difference)
    for axis in ['x', 'y', 'z']:
        if axis in df.columns:
            # Velocity (mm/s)
            result[f'v{axis}'] = np.gradient(df[axis], dt)
            # Acceleration (mm/s²)
            result[f'a{axis}'] = np.gradient(result[f'v{axis}'], dt)
            # Jerk (mm/s³)
            result[f'j{axis}'] = np.gradient(result[f'a{axis}'], dt)
    
    # Compute linear (combined) magnitudes
    if all(c in result.columns for c in ['vx', 'vy', 'vz']):
        result['velocity'] = np.sqrt(result['vx']**2 + result['vy']**2 + result['vz']**2)
    
    if all(c in result.columns for c in ['ax', 'ay', 'az']):
        result['acceleration'] = np.sqrt(result['ax']**2 + result['ay']**2 + result['az']**2)
    
    if all(c in result.columns for c in ['jx', 'jy', 'jz']):
        result['jerk'] = np.sqrt(result['jx']**2 + result['jy']**2 + result['jz']**2)
    
    return result


def analyze_kinematic_limits(df: pd.DataFrame, 
                             max_velocity: float = 100.0,
                             max_acceleration: float = 1000.0, 
                             max_jerk: float = 50000.0) -> Dict[str, Any]:
    """
    Analyze trajectory for kinematic limit violations.
    
    Args:
        df: DataFrame with computed derivatives (velocity, acceleration, jerk)
        max_velocity: Maximum allowed velocity (mm/s)
        max_acceleration: Maximum allowed acceleration (mm/s²)
        max_jerk: Maximum allowed jerk (mm/s³)
    
    Returns:
        Dictionary with analysis results
    """
    results = {
        'max_velocity': df.get('velocity', df.get('vx', pd.Series([0]))).abs().max() if 'velocity' in df or 'vx' in df else 0,
        'max_acceleration': df.get('acceleration', pd.Series([0])).abs().max() if 'acceleration' in df else 0,
        'max_jerk': df.get('jerk', pd.Series([0])).abs().max() if 'jerk' in df else 0,
        'violations': []
    }
    
    # Check for violations
    if 'velocity' in df:
        vel_violations = df[df['velocity'] > max_velocity]
        if len(vel_violations) > 0:
            results['violations'].append({
                'type': 'velocity',
                'count': len(vel_violations),
                'max_value': vel_violations['velocity'].max(),
                'limit': max_velocity,
                'overshoot_pct': (vel_violations['velocity'].max() / max_velocity - 1) * 100
            })
    
    if 'acceleration' in df:
        acc_violations = df[df['acceleration'] > max_acceleration]
        if len(acc_violations) > 0:
            results['violations'].append({
                'type': 'acceleration',
                'count': len(acc_violations),
                'max_value': acc_violations['acceleration'].max(),
                'limit': max_acceleration,
                'overshoot_pct': (acc_violations['acceleration'].max() / max_acceleration - 1) * 100
            })
    
    if 'jerk' in df:
        jerk_violations = df[df['jerk'] > max_jerk]
        if len(jerk_violations) > 0:
            results['violations'].append({
                'type': 'jerk',
                'count': len(jerk_violations),
                'max_value': jerk_violations['jerk'].max(),
                'limit': max_jerk,
                'overshoot_pct': (jerk_violations['jerk'].max() / max_jerk - 1) * 100
            })
    
    results['meets_limits'] = len(results['violations']) == 0
    
    return results


def load_trajectory(filename: str) -> Tuple[pd.DataFrame, pd.DataFrame]:
    """Load trajectory data (CSV or JSON)"""
    path = Path(filename)
    
    if path.suffix == '.json':
        data = load_json(filename)
        desired = pd.DataFrame({
            'time': data['desired']['time'],
            'x': data['desired']['x'],
            'y': data['desired']['y'],
            'z': data['desired']['z'],
            'vx': data['desired'].get('vx', [0] * len(data['desired']['time'])),
            'vy': data['desired'].get('vy', [0] * len(data['desired']['time'])),
            'vz': data['desired'].get('vz', [0] * len(data['desired']['time'])),
        })
        actual = pd.DataFrame({
            'time': data['actual']['time'],
            'x': data['actual']['x'],
            'y': data['actual']['y'],
            'z': data['actual']['z'],
            'vx': data['actual'].get('vx', [0] * len(data['actual']['time'])),
            'vy': data['actual'].get('vy', [0] * len(data['actual']['time'])),
            'vz': data['actual'].get('vz', [0] * len(data['actual']['time'])),
        })
        return desired, actual
    else:
        df = load_csv(filename)
        desired = pd.DataFrame({
            'time': df['time'],
            'x': df['desired_x'],
            'y': df['desired_y'],
            'z': df['desired_z'],
            'vx': df.get('desired_vx', 0),
            'vy': df.get('desired_vy', 0),
            'vz': df.get('desired_vz', 0),
        })
        actual = pd.DataFrame({
            'time': df['time'],
            'x': df['actual_x'],
            'y': df['actual_y'],
            'z': df['actual_z'],
            'vx': df.get('actual_vx', 0),
            'vy': df.get('actual_vy', 0),
            'vz': df.get('actual_vz', 0),
        })
        return desired, actual


# =============================================================================
# Trajectory Visualization
# =============================================================================

def plot_trajectory_comparison(desired: pd.DataFrame, actual: pd.DataFrame,
                                output: Optional[str] = None,
                                title: str = "Trajectory Comparison"):
    """Plot desired vs actual trajectory in 2D and 3D views"""
    CONFIG.apply()
    
    fig = plt.figure(figsize=(16, 10))
    gs = gridspec.GridSpec(2, 3, figure=fig, hspace=0.3, wspace=0.3)
    
    # XY view
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(desired['x'], desired['y'], 'b-', label='Desired', lw=CONFIG.line_width)
    ax1.plot(actual['x'], actual['y'], 'r--', label='Actual', lw=CONFIG.line_width, alpha=0.7)
    ax1.set_xlabel('X (mm)')
    ax1.set_ylabel('Y (mm)')
    ax1.set_title('XY Plane')
    ax1.legend()
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=CONFIG.grid_alpha)
    
    # XZ view
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(desired['x'], desired['z'], 'b-', label='Desired', lw=CONFIG.line_width)
    ax2.plot(actual['x'], actual['z'], 'r--', label='Actual', lw=CONFIG.line_width, alpha=0.7)
    ax2.set_xlabel('X (mm)')
    ax2.set_ylabel('Z (mm)')
    ax2.set_title('XZ Plane')
    ax2.legend()
    ax2.grid(True, alpha=CONFIG.grid_alpha)
    
    # YZ view
    ax3 = fig.add_subplot(gs[0, 2])
    ax3.plot(desired['y'], desired['z'], 'b-', label='Desired', lw=CONFIG.line_width)
    ax3.plot(actual['y'], actual['z'], 'r--', label='Actual', lw=CONFIG.line_width, alpha=0.7)
    ax3.set_xlabel('Y (mm)')
    ax3.set_ylabel('Z (mm)')
    ax3.set_title('YZ Plane')
    ax3.legend()
    ax3.grid(True, alpha=CONFIG.grid_alpha)
    
    # 3D view
    ax4 = fig.add_subplot(gs[1, 0], projection='3d')
    ax4.plot(desired['x'], desired['y'], desired['z'], 'b-', label='Desired', lw=CONFIG.line_width)
    ax4.plot(actual['x'], actual['y'], actual['z'], 'r--', label='Actual', lw=CONFIG.line_width, alpha=0.7)
    ax4.set_xlabel('X (mm)')
    ax4.set_ylabel('Y (mm)')
    ax4.set_zlabel('Z (mm)')
    ax4.set_title('3D View')
    ax4.legend()
    
    # Time series - position
    ax5 = fig.add_subplot(gs[1, 1])
    for i, (axis, color) in enumerate(zip(['x', 'y', 'z'], ['r', 'g', 'b'])):
        ax5.plot(desired['time'], desired[axis], f'{color}-', label=f'Desired {axis.upper()}', lw=1)
        ax5.plot(actual['time'], actual[axis], f'{color}--', label=f'Actual {axis.upper()}', lw=1, alpha=0.7)
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Position (mm)')
    ax5.set_title('Position vs Time')
    ax5.legend(loc='upper right', ncol=2, fontsize=8)
    ax5.grid(True, alpha=CONFIG.grid_alpha)
    
    # Error magnitude
    error = np.sqrt(
        (desired['x'] - actual['x'])**2 +
        (desired['y'] - actual['y'])**2 +
        (desired['z'] - actual['z'])**2
    )
    
    ax6 = fig.add_subplot(gs[1, 2])
    ax6.plot(desired['time'], error, 'k-', lw=CONFIG.line_width)
    ax6.fill_between(desired['time'], 0, error, alpha=0.3)
    ax6.set_xlabel('Time (s)')
    ax6.set_ylabel('Position Error (mm)')
    ax6.set_title(f'Error Magnitude (max: {error.max():.4f} mm)')
    ax6.grid(True, alpha=CONFIG.grid_alpha)
    
    fig.suptitle(title, fontsize=14, fontweight='bold')
    
    if output:
        plt.savefig(output, dpi=CONFIG.figure_dpi, bbox_inches='tight')
        print(f"Saved: {output}")
    else:
        plt.show()
    
    plt.close()


def plot_error_over_path(desired: pd.DataFrame, actual: pd.DataFrame,
                         output: Optional[str] = None):
    """Plot error magnitude as color along the path"""
    CONFIG.apply()
    
    error = np.sqrt(
        (desired['x'] - actual['x'])**2 +
        (desired['y'] - actual['y'])**2 +
        (desired['z'] - actual['z'])**2
    )
    
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    
    # XY plane with error color
    ax = axes[0]
    points = np.array([desired['x'], desired['y']]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    
    from matplotlib.collections import LineCollection
    norm = Normalize(vmin=0, vmax=error.max())
    lc = LineCollection(segments, cmap=CONFIG.error_colormap, norm=norm)
    lc.set_array(error[:-1])
    lc.set_linewidth(3)
    ax.add_collection(lc)
    ax.autoscale()
    
    cbar = plt.colorbar(lc, ax=ax)
    cbar.set_label('Error (mm)')
    
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_title('Path with Error Magnitude')
    ax.set_aspect('equal')
    ax.grid(True, alpha=CONFIG.grid_alpha)
    
    # Error histogram
    ax = axes[1]
    ax.hist(error, bins=50, color='steelblue', edgecolor='black', alpha=0.7)
    ax.axvline(error.mean(), color='red', linestyle='--', label=f'Mean: {error.mean():.4f}')
    ax.axvline(np.percentile(error, 95), color='orange', linestyle='--', label=f'95%: {np.percentile(error, 95):.4f}')
    ax.set_xlabel('Error (mm)')
    ax.set_ylabel('Count')
    ax.set_title('Error Distribution')
    ax.legend()
    ax.grid(True, alpha=CONFIG.grid_alpha)
    
    plt.tight_layout()
    
    if output:
        plt.savefig(output, dpi=CONFIG.figure_dpi, bbox_inches='tight')
        print(f"Saved: {output}")
    else:
        plt.show()
    
    plt.close()


# =============================================================================
# Error Statistics Visualization
# =============================================================================

def plot_error_statistics(stats_file: str, output: Optional[str] = None):
    """Plot error statistics from JSON file"""
    CONFIG.apply()
    
    stats = load_json(stats_file)
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    
    # Box plot style summary
    ax = axes[0, 0]
    metrics = ['minError', 'meanError', 'geometricMean', 'maxError']
    labels = ['Min', 'Mean', 'Geo. Mean', 'Max']
    values = [stats[m] for m in metrics]
    
    colors = ['green', 'blue', 'purple', 'red']
    bars = ax.bar(labels, values, color=colors, alpha=0.7, edgecolor='black')
    ax.set_ylabel('Error (mm)')
    ax.set_title('Error Summary')
    ax.grid(True, alpha=CONFIG.grid_alpha, axis='y')
    
    # Add values on bars
    for bar, val in zip(bars, values):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.001,
                f'{val:.4f}', ha='center', va='bottom', fontsize=9)
    
    # Percentiles
    ax = axes[0, 1]
    percentiles = ['p50', 'p90', 'p95', 'p99']
    pct_labels = ['50%', '90%', '95%', '99%']
    pct_values = [stats['percentiles'][p] for p in percentiles]
    
    ax.bar(pct_labels, pct_values, color='steelblue', alpha=0.7, edgecolor='black')
    ax.set_ylabel('Error (mm)')
    ax.set_title('Error Percentiles')
    ax.grid(True, alpha=CONFIG.grid_alpha, axis='y')
    
    # Per-axis errors
    ax = axes[1, 0]
    if 'axisErrors' in stats:
        axis_names = ['X', 'Y', 'Z']
        x = np.arange(len(axis_names))
        width = 0.25
        
        means = [stats['axisErrors'][a]['mean'] for a in axis_names]
        maxs = [stats['axisErrors'][a]['max'] for a in axis_names]
        stds = [stats['axisErrors'][a]['stdDev'] for a in axis_names]
        
        ax.bar(x - width, means, width, label='Mean', color='blue', alpha=0.7)
        ax.bar(x, maxs, width, label='Max', color='red', alpha=0.7)
        ax.bar(x + width, stds, width, label='Std Dev', color='green', alpha=0.7)
        
        ax.set_xticks(x)
        ax.set_xticklabels(axis_names)
        ax.set_ylabel('Error (mm)')
        ax.set_title('Per-Axis Error')
        ax.legend()
        ax.grid(True, alpha=CONFIG.grid_alpha, axis='y')
    else:
        ax.text(0.5, 0.5, 'No axis data', ha='center', va='center', transform=ax.transAxes)
    
    # Corner analysis
    ax = axes[1, 1]
    if 'cornerAnalysis' in stats:
        corner = stats['cornerAnalysis']
        metrics = ['Corner Count', 'Max Corner Error', 'Mean Corner Error']
        values = [corner['cornerCount'], corner['maxCornerError'], corner['meanCornerError']]
        
        ax.bar(metrics, values, color=['gray', 'red', 'orange'], alpha=0.7, edgecolor='black')
        ax.set_ylabel('Value')
        ax.set_title('Corner Analysis')
        ax.grid(True, alpha=CONFIG.grid_alpha, axis='y')
    else:
        ax.text(0.5, 0.5, 'No corner data', ha='center', va='center', transform=ax.transAxes)
    
    plt.tight_layout()
    
    if output:
        plt.savefig(output, dpi=CONFIG.figure_dpi, bbox_inches='tight')
        print(f"Saved: {output}")
    else:
        plt.show()
    
    plt.close()


# =============================================================================
# Heatmap Visualization
# =============================================================================

def plot_heatmap_2d(filename: str, output: Optional[str] = None,
                    metric: str = 'velocity_limit', title: str = None):
    """Plot 2D heatmap from CSV or JSON"""
    CONFIG.apply()
    
    path = Path(filename)
    
    if path.suffix == '.json':
        data = load_json(filename)
        config = data['config']
        resolution = config['resolution']
        values = np.array(data.get(metric, data.get('velocityLimit', []))).reshape(resolution, resolution)
    else:
        # Load as matrix
        df = load_csv(filename)
        values = df.values
    
    fig, ax = plt.subplots(figsize=(10, 8))
    
    im = ax.imshow(values, cmap=CONFIG.colormap, aspect='auto', origin='lower')
    cbar = plt.colorbar(im, ax=ax)
    cbar.set_label(metric.replace('_', ' ').title())
    
    ax.set_xlabel('X Index')
    ax.set_ylabel('Y Index')
    ax.set_title(title or f'2D Heatmap: {metric}')
    
    if output:
        plt.savefig(output, dpi=CONFIG.figure_dpi, bbox_inches='tight')
        print(f"Saved: {output}")
    else:
        plt.show()
    
    plt.close()


def plot_heatmap_1d(filename: str, output: Optional[str] = None):
    """Plot 1D heatmap (per-axis limits)"""
    CONFIG.apply()
    
    df = load_csv(filename)
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    
    # Velocity limit
    ax = axes[0, 0]
    ax.plot(df['position'], df['velocity_limit'], 'b-', lw=CONFIG.line_width)
    ax.fill_between(df['position'], 0, df['velocity_limit'], alpha=0.3)
    ax.set_xlabel('Position (mm)')
    ax.set_ylabel('Velocity Limit (mm/s)')
    ax.set_title('Velocity Limit vs Position')
    ax.grid(True, alpha=CONFIG.grid_alpha)
    
    # Acceleration limit
    ax = axes[0, 1]
    ax.plot(df['position'], df['acceleration_limit'], 'r-', lw=CONFIG.line_width)
    ax.fill_between(df['position'], 0, df['acceleration_limit'], alpha=0.3, color='red')
    ax.set_xlabel('Position (mm)')
    ax.set_ylabel('Acceleration Limit (mm/s²)')
    ax.set_title('Acceleration Limit vs Position')
    ax.grid(True, alpha=CONFIG.grid_alpha)
    
    # Max error
    ax = axes[1, 0]
    ax.plot(df['position'], df['max_error'], 'k-', lw=CONFIG.line_width)
    ax.fill_between(df['position'], 0, df['max_error'], alpha=0.3, color='gray')
    ax.set_xlabel('Position (mm)')
    ax.set_ylabel('Max Error (mm)')
    ax.set_title('Max Error vs Position')
    ax.grid(True, alpha=CONFIG.grid_alpha)
    
    # Sample count
    ax = axes[1, 1]
    ax.bar(df['position'], df['sample_count'], width=df['position'].diff().mean(), alpha=0.7)
    ax.set_xlabel('Position (mm)')
    ax.set_ylabel('Sample Count')
    ax.set_title('Sample Distribution')
    ax.grid(True, alpha=CONFIG.grid_alpha)
    
    plt.tight_layout()
    
    if output:
        plt.savefig(output, dpi=CONFIG.figure_dpi, bbox_inches='tight')
        print(f"Saved: {output}")
    else:
        plt.show()
    
    plt.close()


def plot_heatmap_3d(filename: str, output: Optional[str] = None):
    """Plot 3D heatmap as scatter plot with color"""
    CONFIG.apply()
    
    df = load_csv(filename)
    
    fig = plt.figure(figsize=(14, 6))
    
    # Velocity limit
    ax1 = fig.add_subplot(121, projection='3d')
    sc1 = ax1.scatter(df['x'], df['y'], df['z'], c=df['velocity_limit'],
                      cmap=CONFIG.colormap, s=20, alpha=0.6)
    plt.colorbar(sc1, ax=ax1, label='Velocity Limit (mm/s)', shrink=0.6)
    ax1.set_xlabel('X (mm)')
    ax1.set_ylabel('Y (mm)')
    ax1.set_zlabel('Z (mm)')
    ax1.set_title('Velocity Limits in Workspace')
    
    # Acceleration limit
    ax2 = fig.add_subplot(122, projection='3d')
    sc2 = ax2.scatter(df['x'], df['y'], df['z'], c=df['acceleration_limit'],
                      cmap=CONFIG.colormap, s=20, alpha=0.6)
    plt.colorbar(sc2, ax=ax2, label='Accel Limit (mm/s²)', shrink=0.6)
    ax2.set_xlabel('X (mm)')
    ax2.set_ylabel('Y (mm)')
    ax2.set_zlabel('Z (mm)')
    ax2.set_title('Acceleration Limits in Workspace')
    
    plt.tight_layout()
    
    if output:
        plt.savefig(output, dpi=CONFIG.figure_dpi, bbox_inches='tight')
        print(f"Saved: {output}")
    else:
        plt.show()
    
    plt.close()


def plot_differential_heatmap(filename: str, output: Optional[str] = None):
    """Plot differential heatmap (actual - expected)"""
    CONFIG.apply()
    
    df = load_csv(filename)
    values = df.values
    
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # Use diverging colormap centered at 0
    vmax = np.abs(values).max()
    norm = TwoSlopeNorm(vmin=-vmax, vcenter=0, vmax=vmax)
    
    im = ax.imshow(values, cmap='RdBu_r', norm=norm, aspect='auto', origin='lower')
    cbar = plt.colorbar(im, ax=ax)
    cbar.set_label('Performance Difference (Actual - Expected)')
    
    ax.set_xlabel('X Index')
    ax.set_ylabel('Y Index')
    ax.set_title('Differential Heatmap')
    
    if output:
        plt.savefig(output, dpi=CONFIG.figure_dpi, bbox_inches='tight')
        print(f"Saved: {output}")
    else:
        plt.show()
    
    plt.close()


# =============================================================================
# System Identification Visualization
# =============================================================================

def plot_friction_model(filename: str, output: Optional[str] = None):
    """Plot friction identification results"""
    CONFIG.apply()
    
    data = load_json(filename)
    
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    
    # Velocity vs Force with model fit
    ax = axes[0]
    velocities = np.array(data['velocities'])
    forces = np.array(data['forces'])
    fitted = np.array(data['fittedForces'])
    
    ax.scatter(velocities, forces, s=10, alpha=0.5, label='Measured', c='blue')
    
    # Sort for line plot
    sort_idx = np.argsort(velocities)
    ax.plot(velocities[sort_idx], fitted[sort_idx], 'r-', lw=2, label='Model Fit')
    
    ax.set_xlabel('Velocity (mm/s)')
    ax.set_ylabel('Force (N)')
    ax.set_title(f"Friction Model: {data['bestModel']['type']}")
    ax.legend()
    ax.grid(True, alpha=CONFIG.grid_alpha)
    
    # Model parameters
    ax = axes[1]
    model = data['bestModel']
    params = [
        ('Coulomb', model.get('coulombForce', 0)),
        ('Static', model.get('staticFriction', 0)),
        ('Viscous', model.get('viscousCoeff', 0)),
        ('R²', model.get('rSquared', 0)),
    ]
    
    labels = [p[0] for p in params]
    values = [p[1] for p in params]
    
    bars = ax.barh(labels, values, color='steelblue', alpha=0.7, edgecolor='black')
    ax.set_xlabel('Value')
    ax.set_title('Model Parameters')
    
    # Add values
    for bar, val in zip(bars, values):
        ax.text(bar.get_width() + 0.01, bar.get_y() + bar.get_height()/2,
                f'{val:.4f}', ha='left', va='center', fontsize=9)
    
    ax.grid(True, alpha=CONFIG.grid_alpha, axis='x')
    
    plt.tight_layout()
    
    if output:
        plt.savefig(output, dpi=CONFIG.figure_dpi, bbox_inches='tight')
        print(f"Saved: {output}")
    else:
        plt.show()
    
    plt.close()


def plot_pid_assessment(filename: str, output: Optional[str] = None):
    """Plot PID tuning assessment"""
    CONFIG.apply()
    
    data = load_json(filename)
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    
    # Scores
    ax = axes[0, 0]
    scores = data['scores']
    score_names = ['Stability', 'Response', 'Accuracy', 'Overall']
    score_values = [scores['stability'], scores['response'], scores['accuracy'], scores['overall']]
    colors = ['green' if v >= 70 else 'orange' if v >= 40 else 'red' for v in score_values]
    
    bars = ax.barh(score_names, score_values, color=colors, alpha=0.7, edgecolor='black')
    ax.set_xlim(0, 100)
    ax.set_xlabel('Score')
    ax.set_title('Tuning Quality Scores')
    ax.axvline(70, color='green', linestyle='--', alpha=0.5, label='Good')
    ax.axvline(40, color='orange', linestyle='--', alpha=0.5, label='Fair')
    ax.grid(True, alpha=CONFIG.grid_alpha, axis='x')
    
    # Performance metrics
    ax = axes[0, 1]
    perf = data['performance']
    metrics = ['Rise Time\n(s)', 'Settling Time\n(s)', 'Overshoot\n(%)', 'SS Error\n(%)']
    values = [perf['riseTime'], perf['settlingTime'], perf['overshoot'], perf['steadyStateError']]
    
    bars = ax.bar(metrics, values, color='steelblue', alpha=0.7, edgecolor='black')
    ax.set_ylabel('Value')
    ax.set_title('Performance Metrics')
    ax.grid(True, alpha=CONFIG.grid_alpha, axis='y')
    
    # Current vs Suggested gains
    ax = axes[1, 0]
    observed = data['observedGains']
    suggested = data['suggestedGains']
    
    x = np.arange(3)
    width = 0.35
    
    current = [observed['Kp'], observed['Ki'], observed['Kd']]
    suggest = [suggested['Kp'], suggested['Ki'], suggested['Kd']]
    
    ax.bar(x - width/2, current, width, label='Current', color='blue', alpha=0.7)
    ax.bar(x + width/2, suggest, width, label='Suggested', color='green', alpha=0.7)
    
    ax.set_xticks(x)
    ax.set_xticklabels(['Kp', 'Ki', 'Kd'])
    ax.set_ylabel('Gain Value')
    ax.set_title('PID Gains: Current vs Suggested')
    ax.legend()
    ax.grid(True, alpha=CONFIG.grid_alpha, axis='y')
    
    # Issues and recommendations
    ax = axes[1, 1]
    ax.axis('off')
    
    text = "Issues:\n"
    for issue in data.get('issues', []):
        text += f"  • {issue}\n"
    
    text += "\nRecommendations:\n"
    for rec in data.get('recommendations', []):
        text += f"  • {rec}\n"
    
    ax.text(0.1, 0.9, text, transform=ax.transAxes, fontsize=10,
            verticalalignment='top', fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    ax.set_title('Issues and Recommendations')
    
    plt.tight_layout()
    
    if output:
        plt.savefig(output, dpi=CONFIG.figure_dpi, bbox_inches='tight')
        print(f"Saved: {output}")
    else:
        plt.show()
    
    plt.close()


def plot_delay_identification(filename: str, output: Optional[str] = None):
    """Plot delay identification results"""
    CONFIG.apply()
    
    data = load_json(filename)
    
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    
    # Delay visualization
    ax = axes[0]
    metrics = ['Transport\nDelay (ms)', 'Rise Time\n(ms)', 'Settling Time\n(ms)', 'Overshoot\n(%)']
    values = [
        data['transportDelay'] * 1000,
        data['riseTime'] * 1000,
        data['settlingTime'] * 1000,
        data['overshoot']
    ]
    
    bars = ax.bar(metrics, values, color='steelblue', alpha=0.7, edgecolor='black')
    ax.set_ylabel('Value')
    ax.set_title(f"Delay Identification (Confidence: {data['confidence']*100:.1f}%)")
    ax.grid(True, alpha=CONFIG.grid_alpha, axis='y')
    
    # Add values on bars
    for bar, val in zip(bars, values):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.5,
                f'{val:.2f}', ha='center', va='bottom', fontsize=9)
    
    # Summary text
    ax = axes[1]
    ax.axis('off')
    
    text = f"""
Delay Identification Results
{'='*40}

Transport Delay: {data['transportDelay']*1000:.3f} ms
Confidence:      {data['confidence']*100:.1f}%

Step Response Characteristics:
  Rise Time:     {data['riseTime']*1000:.3f} ms
  Settling Time: {data['settlingTime']*1000:.3f} ms
  Overshoot:     {data['overshoot']:.2f}%

Correlation Analysis:
  Peak Correlation: {data['crossCorrelation']:.4f}
  Lag (samples):    {data['crossCorrelationLag']}
  Sample Period:    {data['samplingPeriod']*1000:.3f} ms
"""
    
    ax.text(0.1, 0.9, text, transform=ax.transAxes, fontsize=11,
            verticalalignment='top', fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.3))
    
    plt.tight_layout()
    
    if output:
        plt.savefig(output, dpi=CONFIG.figure_dpi, bbox_inches='tight')
        print(f"Saved: {output}")
    else:
        plt.show()
    
    plt.close()


# =============================================================================
# Test Result Visualization
# =============================================================================

def plot_test_result(filename: str, output: Optional[str] = None):
    """Plot complete test result"""
    CONFIG.apply()
    
    path = Path(filename)
    
    if path.suffix == '.json':
        data = load_json(filename)
        
        desired = pd.DataFrame({
            'time': data['desired']['time'],
            'x': data['desired']['x'],
            'y': data['desired']['y'],
            'z': data['desired']['z'],
        })
        actual = pd.DataFrame({
            'time': data['actual']['time'],
            'x': data['actual']['x'],
            'y': data['actual']['y'],
            'z': data['actual']['z'],
        })
        
        test_name = data.get('testName', 'Unknown')
        passed = data.get('passed', False)
        stats = data.get('statistics', {})
    else:
        df = load_csv(filename)
        # Parse header comments for metadata
        test_name = "Test"
        passed = True
        stats = {}
        
        desired = pd.DataFrame({
            'time': df['time'],
            'x': df['desired_x'],
            'y': df['desired_y'],
            'z': df['desired_z'],
        })
        actual = pd.DataFrame({
            'time': df['time'],
            'x': df['actual_x'],
            'y': df['actual_y'],
            'z': df['actual_z'],
        })
    
    title = f"Test: {test_name} - {'PASSED ✓' if passed else 'FAILED ✗'}"
    plot_trajectory_comparison(desired, actual, output, title)


# =============================================================================
# Batch Visualization
# =============================================================================

def process_directory(input_dir: str, output_dir: str):
    """Process all data files in a directory"""
    input_path = Path(input_dir)
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    CONFIG.apply()
    
    # Find and process files
    for file in input_path.glob('*.csv'):
        name = file.stem
        output_file = output_path / f"{name}.png"
        
        if 'trajectory' in name.lower():
            desired, actual = load_trajectory(str(file))
            plot_trajectory_comparison(desired, actual, str(output_file))
        elif 'heatmap_xy' in name.lower() or 'heatmap_xz' in name.lower() or 'heatmap_yz' in name.lower():
            plot_heatmap_2d(str(file), str(output_file))
        elif 'heatmap_3d' in name.lower():
            plot_heatmap_3d(str(file), str(output_file))
        elif 'heatmap_axis' in name.lower():
            plot_heatmap_1d(str(file), str(output_file))
        elif 'error' in name.lower():
            # Tracking errors
            pass
    
    for file in input_path.glob('*.json'):
        name = file.stem
        output_file = output_path / f"{name}.png"
        
        if 'statistics' in name.lower():
            plot_error_statistics(str(file), str(output_file))
        elif 'friction' in name.lower():
            plot_friction_model(str(file), str(output_file))
        elif 'pid' in name.lower():
            plot_pid_assessment(str(file), str(output_file))
        elif 'delay' in name.lower():
            plot_delay_identification(str(file), str(output_file))
        elif 'test_' in name.lower():
            plot_test_result(str(file), str(output_file))
    
    print(f"Processed files from {input_dir} to {output_dir}")


# =============================================================================
# Main CLI
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='Motion Replanner Visualization Suite',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s trajectory data.csv -o trajectory.png
  %(prog)s trajectory data.csv -o trajectory.png --sampling-rate 100000
  %(prog)s analyze data.csv --max-vel 100 --max-acc 1000 --max-jerk 50000
  %(prog)s heatmap2d heatmap_xy.csv -o heatmap.png
  %(prog)s statistics error_stats.json -o stats.png
  %(prog)s friction friction_model.json -o friction.png
  %(prog)s batch ./data_dir ./output_dir
        """
    )
    
    # Global options
    parser.add_argument('--sampling-rate', '-s', type=float, default=100000,
                        help='Trajectory sampling rate in Hz (default: 100000 = 100kHz)')
    
    subparsers = parser.add_subparsers(dest='command', help='Visualization type')
    
    # Trajectory
    p = subparsers.add_parser('trajectory', help='Plot trajectory comparison')
    p.add_argument('input', help='Input CSV or JSON file')
    p.add_argument('-o', '--output', help='Output image file')
    p.add_argument('--resample', action='store_true', help='Resample trajectory to target rate')
    
    # Error over path
    p = subparsers.add_parser('error_path', help='Plot error along path')
    p.add_argument('input', help='Input CSV or JSON file')
    p.add_argument('-o', '--output', help='Output image file')
    
    # Kinematic Analysis
    p = subparsers.add_parser('analyze', help='Analyze trajectory kinematic limits')
    p.add_argument('input', help='Input CSV or JSON file')
    p.add_argument('-o', '--output', help='Output image file')
    p.add_argument('--max-vel', type=float, default=100.0, help='Max velocity limit (mm/s)')
    p.add_argument('--max-acc', type=float, default=1000.0, help='Max acceleration limit (mm/s²)')
    p.add_argument('--max-jerk', type=float, default=50000.0, help='Max jerk limit (mm/s³)')
    p.add_argument('--resample', action='store_true', help='Resample to target rate before analysis')
    
    # Statistics
    p = subparsers.add_parser('statistics', help='Plot error statistics')
    p.add_argument('input', help='Input JSON file')
    p.add_argument('-o', '--output', help='Output image file')
    
    # 2D Heatmap
    p = subparsers.add_parser('heatmap2d', help='Plot 2D heatmap')
    p.add_argument('input', help='Input CSV or JSON file')
    p.add_argument('-o', '--output', help='Output image file')
    p.add_argument('-m', '--metric', default='velocity_limit', help='Metric to plot')
    
    # 1D Heatmap
    p = subparsers.add_parser('heatmap1d', help='Plot 1D (per-axis) heatmap')
    p.add_argument('input', help='Input CSV file')
    p.add_argument('-o', '--output', help='Output image file')
    
    # 3D Heatmap
    p = subparsers.add_parser('heatmap3d', help='Plot 3D heatmap')
    p.add_argument('input', help='Input CSV file')
    p.add_argument('-o', '--output', help='Output image file')
    
    # Differential heatmap
    p = subparsers.add_parser('diff_heatmap', help='Plot differential heatmap')
    p.add_argument('input', help='Input CSV file')
    p.add_argument('-o', '--output', help='Output image file')
    
    # Friction
    p = subparsers.add_parser('friction', help='Plot friction model')
    p.add_argument('input', help='Input JSON file')
    p.add_argument('-o', '--output', help='Output image file')
    
    # PID
    p = subparsers.add_parser('pid', help='Plot PID assessment')
    p.add_argument('input', help='Input JSON file')
    p.add_argument('-o', '--output', help='Output image file')
    
    # Delay
    p = subparsers.add_parser('delay', help='Plot delay identification')
    p.add_argument('input', help='Input JSON file')
    p.add_argument('-o', '--output', help='Output image file')
    
    # Test result
    p = subparsers.add_parser('test', help='Plot test result')
    p.add_argument('input', help='Input CSV or JSON file')
    p.add_argument('-o', '--output', help='Output image file')
    
    # Batch
    p = subparsers.add_parser('batch', help='Process directory of files')
    p.add_argument('input_dir', help='Input directory')
    p.add_argument('output_dir', help='Output directory')
    
    args = parser.parse_args()
    
    if args.command is None:
        parser.print_help()
        return
    
    # Configure sampling rate
    CONFIG.set_sampling_rate(args.sampling_rate)
    CONFIG.apply()
    
    if args.command == 'trajectory':
        desired, actual = load_trajectory(args.input)
        if hasattr(args, 'resample') and args.resample:
            desired = resample_trajectory(desired)
            actual = resample_trajectory(actual)
        plot_trajectory_comparison(desired, actual, args.output)
    elif args.command == 'error_path':
        desired, actual = load_trajectory(args.input)
        plot_error_over_path(desired, actual, args.output)
    elif args.command == 'analyze':
        desired, actual = load_trajectory(args.input)
        if args.resample:
            desired = resample_trajectory(desired)
        desired_with_derivs = compute_derivatives(desired)
        results = analyze_kinematic_limits(
            desired_with_derivs, 
            args.max_vel, args.max_acc, args.max_jerk
        )
        
        print(f"\n=== Kinematic Analysis Results (@ {args.sampling_rate/1000:.0f} kHz) ===")
        print(f"Max Velocity:     {results['max_velocity']:.4f} mm/s (limit: {args.max_vel})")
        print(f"Max Acceleration: {results['max_acceleration']:.4f} mm/s² (limit: {args.max_acc})")
        print(f"Max Jerk:         {results['max_jerk']:.4f} mm/s³ (limit: {args.max_jerk})")
        print(f"Meets Limits:     {'YES ✓' if results['meets_limits'] else 'NO ✗'}")
        
        if results['violations']:
            print("\nViolations:")
            for v in results['violations']:
                print(f"  {v['type'].upper()}: {v['count']} samples exceed limit "
                      f"(max: {v['max_value']:.4f}, {v['overshoot_pct']:.2f}% over)")
        
        if args.output:
            # Plot kinematic analysis
            plot_kinematic_analysis(desired_with_derivs, args.output, 
                                    args.max_vel, args.max_acc, args.max_jerk)
    elif args.command == 'statistics':
        plot_error_statistics(args.input, args.output)
    elif args.command == 'heatmap2d':
        plot_heatmap_2d(args.input, args.output, args.metric)
    elif args.command == 'heatmap1d':
        plot_heatmap_1d(args.input, args.output)
    elif args.command == 'heatmap3d':
        plot_heatmap_3d(args.input, args.output)
    elif args.command == 'diff_heatmap':
        plot_differential_heatmap(args.input, args.output)
    elif args.command == 'friction':
        plot_friction_model(args.input, args.output)
    elif args.command == 'pid':
        plot_pid_assessment(args.input, args.output)
    elif args.command == 'delay':
        plot_delay_identification(args.input, args.output)
    elif args.command == 'test':
        plot_test_result(args.input, args.output)
    elif args.command == 'batch':
        process_directory(args.input_dir, args.output_dir)


def plot_kinematic_analysis(df: pd.DataFrame, output: str,
                            max_vel: float, max_acc: float, max_jerk: float):
    """Plot kinematic analysis results"""
    CONFIG.apply()
    
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    
    time = df['time']
    
    # Velocity plot
    ax = axes[0]
    if 'velocity' in df:
        ax.plot(time, df['velocity'], 'b-', lw=1, label='Linear Velocity')
    ax.axhline(max_vel, color='r', linestyle='--', lw=2, label=f'Limit ({max_vel} mm/s)')
    ax.fill_between(time, 0, df.get('velocity', 0), alpha=0.3)
    ax.set_ylabel('Velocity (mm/s)')
    ax.set_title('Velocity Profile')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    
    # Acceleration plot
    ax = axes[1]
    if 'acceleration' in df:
        ax.plot(time, df['acceleration'], 'g-', lw=1, label='Linear Acceleration')
    ax.axhline(max_acc, color='r', linestyle='--', lw=2, label=f'Limit ({max_acc} mm/s²)')
    ax.fill_between(time, 0, df.get('acceleration', 0), alpha=0.3, color='green')
    ax.set_ylabel('Acceleration (mm/s²)')
    ax.set_title('Acceleration Profile')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    
    # Jerk plot
    ax = axes[2]
    if 'jerk' in df:
        ax.plot(time, df['jerk'], 'm-', lw=1, label='Linear Jerk')
    ax.axhline(max_jerk, color='r', linestyle='--', lw=2, label=f'Limit ({max_jerk} mm/s³)')
    ax.fill_between(time, 0, df.get('jerk', 0), alpha=0.3, color='purple')
    ax.set_ylabel('Jerk (mm/s³)')
    ax.set_xlabel('Time (s)')
    ax.set_title('Jerk Profile')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output, dpi=CONFIG.figure_dpi, bbox_inches='tight')
    print(f"Saved: {output}")
    plt.close()


if __name__ == '__main__':
    main()
