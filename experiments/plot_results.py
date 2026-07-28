"""
Ball-on-Beam Performance Analysis and Plotting
Generates publication-quality figures for journal papers comparing RL vs PID control

Usage:
    python plot_paper_figures.py

Configure data files and settings in the Config class below.
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
from scipy import stats
from typing import List, Dict, Tuple, Optional
import warnings
warnings.filterwarnings('ignore')


# =============================================================================
# CONFIGURATION
# =============================================================================

class Config:
    """Configuration for plotting"""
    
    # Data files - Update these paths to your actual data
    DATA_DIR = Path(r"C:\Users\Yasiru Fernando\Desktop\Ball Balancing\Code\plots_new\experiment_c")
    
    # RL trial files (list all your RL trials)
    RL_FILES = [
        "RL_trial_21.csv",
        "RL_trial_22.csv",
        "RL_trial_23.csv",
        "RL_trial_24.csv",
        "RL_trial_25.csv",
        # "RL_trial_16.csv",
        # "RL_trial_17.csv",
        # "RL_trial_18.csv",
        # "RL_trial_19.csv",
        # "RL_trial_20.csv"

        # Add more trials here
    ]
    
    # PID trial files (list all your PID trials)
    PID_FILES = [
        "PID_trial_21.csv",
        "PID_trial_22.csv",
        "PID_trial_23.csv",
        "PID_trial_24.csv",
        "PID_trial_25.csv",
        # "PID_trial_16.csv",
        # "PID_trial_17.csv",
        # "PID_trial_18.csv",
        # "PID_trial_19.csv",
        # "PID_trial_20.csv",


        # Add more trials here
    ]
    
    # Simulation trial files - NEW!
    SIM_FILES = [
        "sim_exp_c_trial_1.csv",
        "sim_exp_c_trial_2.csv",
        "sim_exp_c_trial_3.csv",
        "sim_exp_c_trial_4.csv",
        "sim_exp_c_trial_5.csv",
        # "sim_exp_b_trial_6.csv",
        # "sim_exp_b_trial_7.csv",
        # "sim_exp_b_trial_8.csv",
        # "sim_exp_b_trial_9.csv",
        # "sim_exp_b_trial_10.csv",
        # "sim_trial_2.csv",
        # Add simulation trials here
    ]
    
    # Plotting settings
    OUTPUT_DIR = Path(r"C:\Users\Yasiru Fernando\Desktop\Ball Balancing\Code\plots_new\results_experiment_c")
    DPI = 300
    FIGURE_FORMAT = 'png'  # or 'pdf' for LaTeX
    
    # Style
    RL_COLOR = '#2E86AB'  # Blue
    PID_COLOR = '#A23B72'  # Red/Purple
    SIM_COLOR = '#E8871E'  # Orange - NEW!
    FONT_SIZE = 10
    LINE_WIDTH = 1.5
    ALPHA = 0.7
    
    # Analysis settings
    TIME_WINDOW = None  # Trim data to this duration (seconds), None = use all
    SETTLING_THRESHOLD = 0.01  # 1cm for settling time calculation
    SETTLING_DURATION = 1.0  # Must stay within threshold for this long
    
    # Experiment B settings - NEW!
    FLIP_NEGATIVE_INITIAL_POSITION = True  # Flip sign of trials that start at -10cm


# =============================================================================
# DATA LOADING AND PREPROCESSING
# =============================================================================

def load_csv_data(filepath: Path) -> pd.DataFrame:
    """Load CSV file and return dataframe"""
    df = pd.read_csv(filepath, comment='#')
    return df


def preprocess_trial(df: pd.DataFrame, time_window: float = None, flip_if_negative: bool = False) -> pd.DataFrame:
    """
    Preprocess trial data.
    
    Args:
        df: Raw dataframe
        time_window: Trim to this duration (seconds)
        flip_if_negative: If True, flip sign of position if trial starts negative (for Experiment B)
    
    Returns:
        Preprocessed dataframe
    """
    df = df.copy()
    
    # Check if trial starts at negative position (Experiment B with -10cm start)
    if flip_if_negative and len(df) > 0:
        initial_x = df['x'].iloc[0]
        if initial_x < -0.05:  # Started at -10cm (negative side)
            print(f"  Flipping trial that started at {initial_x*100:.1f}cm")
            df['x'] = -df['x']
            df['xdot'] = -df['xdot']
            df['theta'] = -df['theta']
            df['thdot'] = -df['thdot']
            # Also flip theta_cmd if it exists
            if 'theta_cmd' in df.columns:
                df['theta_cmd'] = -df['theta_cmd']
                df['theta_cmd_dot'] = -df['theta_cmd_dot']
    
    # FIX: Always recalculate error consistently for ALL trials (not just flipped)
    # This ensures RL, PID, and simulation trials all use the same error definition
    df['error'] = df['x'].abs()
    df['error_sq'] = df['x'] ** 2
    
    # Trim to time window if specified
    if time_window is not None:
        df = df[df['time_rel'] <= time_window]
    
    # Remove any NaN values
    df = df.dropna(subset=['x', 'theta'])
    
    return df


def load_all_trials(file_list: List[str], data_dir: Path, time_window: float = None, 
                   flip_if_negative: bool = False) -> List[pd.DataFrame]:
    """Load and preprocess all trials"""
    trials = []
    for filename in file_list:
        filepath = data_dir / filename
        if filepath.exists():
            df = load_csv_data(filepath)
            df = preprocess_trial(df, time_window, flip_if_negative)
            trials.append(df)
        else:
            print(f"Warning: File not found: {filepath}")
    return trials


# =============================================================================
# PERFORMANCE METRICS
# =============================================================================

def calculate_rmse(df: pd.DataFrame) -> float:
    """Root mean square error"""
    return np.sqrt(df['error_sq'].mean())


def calculate_mae(df: pd.DataFrame) -> float:
    """Mean absolute error"""
    return df['error'].mean()


def calculate_max_error(df: pd.DataFrame) -> float:
    """Maximum absolute error"""
    return df['error'].max()


def calculate_settling_time(df: pd.DataFrame, 
                           threshold: float = Config.SETTLING_THRESHOLD,
                           duration: float = Config.SETTLING_DURATION) -> float:
    """
    Calculate settling time (time to reach and stay within threshold).
    
    Returns:
        Settling time in seconds, or np.inf if never settles
    """
    # Check if error stays below threshold for required duration
    sample_rate = 1.0 / df['dt'].mean()
    window_samples = int(duration * sample_rate)
    
    within_threshold = (df['error'] < threshold).rolling(window=window_samples).sum()
    settled_mask = within_threshold == window_samples
    
    if settled_mask.any():
        return df.loc[settled_mask.idxmax(), 'time_rel']
    else:
        return np.inf


def calculate_control_effort(df: pd.DataFrame) -> float:
    """
    Total variation of control signal (smoothness metric).
    
    FIX: Use theta_deg (actual beam angle) for BOTH RL and PID to ensure
    a fair apples-to-apples comparison. Previously RL used setpoint_sent
    (raw command signal) while PID used theta_deg (physically smoothed
    measured angle), which made RL look artificially worse.
    """
    signal = df['theta_deg'].values
    
    return np.sum(np.abs(np.diff(signal)))


def calculate_all_metrics(df: pd.DataFrame) -> Dict[str, float]:
    """Calculate all performance metrics for a trial"""
    return {
        'RMSE': calculate_rmse(df),
        'MAE': calculate_mae(df),
        'Max Error': calculate_max_error(df),
        'Settling Time': calculate_settling_time(df),
        'Control Effort': calculate_control_effort(df),
        'Mean |Angle|': df['theta_deg'].abs().mean(),
        'Std Error': df['error'].std(),
    }


def aggregate_metrics(trials: List[pd.DataFrame]) -> Dict[str, Tuple[float, float]]:
    """
    Calculate mean and std of metrics across multiple trials.
    
    Returns:
        Dict of {metric_name: (mean, std)}
    """
    all_metrics = [calculate_all_metrics(trial) for trial in trials]
    
    aggregated = {}
    for key in all_metrics[0].keys():
        values = [m[key] for m in all_metrics if m[key] != np.inf]
        if values:
            aggregated[key] = (np.mean(values), np.std(values))
        else:
            aggregated[key] = (np.inf, 0)
    
    return aggregated


# =============================================================================
# STATISTICAL TESTS
# =============================================================================

def perform_statistical_tests(rl_trials: List[pd.DataFrame], 
                              pid_trials: List[pd.DataFrame]) -> Dict[str, Dict]:
    """
    Perform statistical comparisons between RL and PID.
    
    Returns:
        Dict of {metric_name: {'t_stat': ..., 'p_value': ..., 'significant': ...}}
    """
    rl_metrics = [calculate_all_metrics(trial) for trial in rl_trials]
    pid_metrics = [calculate_all_metrics(trial) for trial in pid_trials]
    
    results = {}
    for key in rl_metrics[0].keys():
        rl_values = np.array([m[key] for m in rl_metrics if m[key] != np.inf])
        pid_values = np.array([m[key] for m in pid_metrics if m[key] != np.inf])
        
        if len(rl_values) > 0 and len(pid_values) > 0:
            # Paired t-test if same number of samples
            if len(rl_values) == len(pid_values):
                t_stat, p_value = stats.ttest_rel(rl_values, pid_values)
            else:
                # Independent t-test otherwise
                t_stat, p_value = stats.ttest_ind(rl_values, pid_values)
            
            results[key] = {
                't_stat': t_stat,
                'p_value': p_value,
                'significant': p_value < 0.05,
                'rl_mean': np.mean(rl_values),
                'pid_mean': np.mean(pid_values),
                'improvement': (np.mean(pid_values) - np.mean(rl_values)) / np.mean(pid_values) * 100
            }
    
    return results


# =============================================================================
# PLOTTING FUNCTIONS
# =============================================================================

def setup_plot_style():
    """Configure matplotlib style for publication quality"""
    plt.rcParams.update({
        'font.size': Config.FONT_SIZE,
        'font.family': 'serif',
        'font.serif': ['Times New Roman', 'DejaVu Serif'],
        'axes.labelsize': Config.FONT_SIZE,
        'axes.titlesize': Config.FONT_SIZE + 1,
        'xtick.labelsize': Config.FONT_SIZE - 1,
        'ytick.labelsize': Config.FONT_SIZE - 1,
        'legend.fontsize': Config.FONT_SIZE - 1,
        'figure.dpi': Config.DPI,
        'savefig.dpi': Config.DPI,
        'savefig.bbox': 'tight',
        'savefig.pad_inches': 0.05,
        'lines.linewidth': Config.LINE_WIDTH,
        'axes.grid': True,
        'grid.alpha': 0.3,
        'axes.axisbelow': True,
    })


def plot_single_trial_comparison(rl_df: pd.DataFrame, pid_df: pd.DataFrame, 
                                 output_path: Path, sim_df: Optional[pd.DataFrame] = None):
    """
    Figure 1: Representative trial showing time-domain response.
    
    Three subplots:
    - Position tracking
    - Beam angle
    - Position error
    
    This is the most important figure - shows actual performance.
    """
    fig, axes = plt.subplots(3, 1, figsize=(7, 8), sharex=True)
    
    # Position tracking
    axes[0].plot(rl_df['time_rel'], rl_df['x'] * 100, 
                label='RL (Hardware)', color=Config.RL_COLOR, alpha=Config.ALPHA, linewidth=Config.LINE_WIDTH)
    axes[0].plot(pid_df['time_rel'], pid_df['x'] * 100, 
                label='PID (Hardware)', color=Config.PID_COLOR, alpha=Config.ALPHA, linewidth=Config.LINE_WIDTH)
    if sim_df is not None:
        axes[0].plot(sim_df['time_rel'], sim_df['x'] * 100, 
                    label='RL (Simulation)', color=Config.SIM_COLOR, alpha=Config.ALPHA, linewidth=Config.LINE_WIDTH, linestyle='--')
    axes[0].axhline(0, color='black', linestyle='--', linewidth=0.8, alpha=0.5)
    axes[0].set_ylabel('Position (cm)')
    axes[0].legend(loc='upper right', framealpha=0.9)
    axes[0].set_title('Representative Trial Comparison')
    
    # Beam angle
    axes[1].plot(rl_df['time_rel'], rl_df['theta_deg'], 
                label='RL (Hardware)', color=Config.RL_COLOR, alpha=Config.ALPHA, linewidth=Config.LINE_WIDTH)
    axes[1].plot(pid_df['time_rel'], pid_df['theta_deg'], 
                label='PID (Hardware)', color=Config.PID_COLOR, alpha=Config.ALPHA, linewidth=Config.LINE_WIDTH)
    if sim_df is not None:
        axes[1].plot(sim_df['time_rel'], sim_df['theta_deg'], 
                    label='RL (Simulation)', color=Config.SIM_COLOR, alpha=Config.ALPHA, linewidth=Config.LINE_WIDTH, linestyle='--')
    axes[1].axhline(0, color='black', linestyle='--', linewidth=0.8, alpha=0.5)
    axes[1].set_ylabel('Beam Angle (deg)')
    axes[1].legend(loc='upper right', framealpha=0.9)
    
    # Position error
    axes[2].plot(rl_df['time_rel'], rl_df['error'] * 100, 
                label='RL (Hardware)', color=Config.RL_COLOR, alpha=Config.ALPHA, linewidth=Config.LINE_WIDTH)
    axes[2].plot(pid_df['time_rel'], pid_df['error'] * 100, 
                label='PID (Hardware)', color=Config.PID_COLOR, alpha=Config.ALPHA, linewidth=Config.LINE_WIDTH)
    if sim_df is not None:
        axes[2].plot(sim_df['time_rel'], sim_df['error'] * 100, 
                    label='RL (Simulation)', color=Config.SIM_COLOR, alpha=Config.ALPHA, linewidth=Config.LINE_WIDTH, linestyle='--')
    axes[2].set_ylabel('|Error| (cm)')
    axes[2].set_xlabel('Time (s)')
    axes[2].legend(loc='upper right', framealpha=0.9)
    
    plt.tight_layout()
    plt.savefig(output_path, format=Config.FIGURE_FORMAT, dpi=Config.DPI)
    plt.close()
    print(f"Saved: {output_path}")


def plot_error_distribution(rl_trials: List[pd.DataFrame], 
                           pid_trials: List[pd.DataFrame], 
                           output_path: Path,
                           sim_trials: Optional[List[pd.DataFrame]] = None):
    """
    Figure 2: Error distribution comparison.
    
    Shows violin plots or box plots of position error distribution.
    Demonstrates consistency and outlier behavior.
    """
    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    
    # Combine all errors
    rl_errors = np.concatenate([trial['error'].values * 100 for trial in rl_trials])
    pid_errors = np.concatenate([trial['error'].values * 100 for trial in pid_trials])
    
    # Violin plot
    if sim_trials:
        sim_errors = np.concatenate([trial['error'].values * 100 for trial in sim_trials])
        parts = axes[0].violinplot([rl_errors, pid_errors, sim_errors], 
                                   positions=[1, 2, 3],
                                   showmeans=True, showmedians=True)
        axes[0].set_xticks([1, 2, 3])
        axes[0].set_xticklabels(['RL\n(Hardware)', 'PID\n(Hardware)', 'RL\n(Simulation)'])
        
        # Color the violin plots
        for pc, color in zip(parts['bodies'], [Config.RL_COLOR, Config.PID_COLOR, Config.SIM_COLOR]):
            pc.set_facecolor(color)
            pc.set_alpha(0.6)
    else:
        parts = axes[0].violinplot([rl_errors, pid_errors], 
                                   positions=[1, 2],
                                   showmeans=True, showmedians=True)
        axes[0].set_xticks([1, 2])
        axes[0].set_xticklabels(['RL', 'PID'])
        
        # Color the violin plots
        for pc, color in zip(parts['bodies'], [Config.RL_COLOR, Config.PID_COLOR]):
            pc.set_facecolor(color)
            pc.set_alpha(0.6)
    
    axes[0].set_ylabel('|Error| (cm)')
    axes[0].set_title('Error Distribution (Violin Plot)')
    
    # Histogram
    axes[1].hist(rl_errors, bins=50, alpha=0.6, color=Config.RL_COLOR, 
                label='RL (Hardware)', density=True, edgecolor='black', linewidth=0.5)
    axes[1].hist(pid_errors, bins=50, alpha=0.6, color=Config.PID_COLOR, 
                label='PID (Hardware)', density=True, edgecolor='black', linewidth=0.5)
    if sim_trials:
        axes[1].hist(sim_errors, bins=50, alpha=0.6, color=Config.SIM_COLOR, 
                    label='RL (Simulation)', density=True, edgecolor='black', linewidth=0.5)
    axes[1].set_xlabel('|Error| (cm)')
    axes[1].set_ylabel('Probability Density')
    axes[1].set_title('Error Histogram')
    axes[1].legend()
    
    plt.tight_layout()
    plt.savefig(output_path, format=Config.FIGURE_FORMAT, dpi=Config.DPI)
    plt.close()
    print(f"Saved: {output_path}")


def plot_metrics_comparison(rl_metrics: Dict, pid_metrics: Dict, 
                           stats_results: Dict, output_path: Path):
    """
    Figure 3: Bar chart comparing key performance metrics.
    
    Shows mean ± std for RMSE, MAE, settling time, etc.
    Includes significance markers.
    """
    # Select metrics to plot
    metrics_to_plot = ['RMSE', 'MAE', 'Max Error', 'Settling Time']
    
    fig, axes = plt.subplots(2, 2, figsize=(10, 7))
    axes = axes.flatten()
    
    for idx, metric in enumerate(metrics_to_plot):
        ax = axes[idx]
        
        # Get values
        rl_mean, rl_std = rl_metrics[metric]
        pid_mean, pid_std = pid_metrics[metric]
        
        # Convert to appropriate units
        if metric in ['RMSE', 'MAE', 'Max Error']:
            rl_mean *= 1000  # to mm
            rl_std *= 1000
            pid_mean *= 1000
            pid_std *= 1000
            ylabel = 'mm'
        else:
            ylabel = 's'
        
        # Bar plot
        x = np.arange(2)
        bars = ax.bar(x, [rl_mean, pid_mean], 
                      yerr=[rl_std, pid_std],
                      color=[Config.RL_COLOR, Config.PID_COLOR],
                      alpha=0.7, capsize=5, error_kw={'linewidth': 2})
        
        ax.set_xticks(x)
        ax.set_xticklabels(['RL', 'PID'])
        ax.set_ylabel(f'{metric} ({ylabel})')
        ax.set_title(metric)
        
        # Add significance marker
        if metric in stats_results and stats_results[metric]['significant']:
            p_val = stats_results[metric]['p_value']
            if p_val < 0.001:
                sig_text = '***'
            elif p_val < 0.01:
                sig_text = '**'
            else:
                sig_text = '*'
            
            y_max = max(rl_mean + rl_std, pid_mean + pid_std)
            ax.text(0.5, y_max * 1.1, sig_text, ha='center', fontsize=14)
        
        # Add improvement percentage
        if metric in stats_results:
            improvement = stats_results[metric]['improvement']
            ax.text(0.5, -0.15, f'{improvement:+.1f}%', 
                   ha='center', va='top', transform=ax.transAxes,
                   fontsize=9, style='italic')
    
    plt.tight_layout()
    plt.savefig(output_path, format=Config.FIGURE_FORMAT, dpi=Config.DPI)
    plt.close()
    print(f"Saved: {output_path}")


def plot_control_effort(rl_trials: List[pd.DataFrame], 
                       pid_trials: List[pd.DataFrame], 
                       output_path: Path):
    """
    Figure 4: Control effort comparison.
    
    Shows control commands over time and smoothness.
    Demonstrates energy efficiency and smoothness.
    """
    fig, axes = plt.subplots(2, 2, figsize=(10, 7))
    
    # Pick first trial for time series
    rl_df = rl_trials[0]
    pid_df = pid_trials[0]
    
    # Time series of control signal
    if 'setpoint_sent' in rl_df.columns:
        axes[0, 0].plot(rl_df['time_rel'], rl_df['setpoint_sent'], 
                       color=Config.RL_COLOR, alpha=Config.ALPHA, linewidth=Config.LINE_WIDTH)
        axes[0, 0].set_ylabel('Setpoint (deg)')
        axes[0, 0].set_title('RL Control Commands')
        axes[0, 0].set_xlabel('Time (s)')
    
    axes[0, 1].plot(pid_df['time_rel'], pid_df['theta_deg'], 
                   color=Config.PID_COLOR, alpha=Config.ALPHA, linewidth=Config.LINE_WIDTH)
    axes[0, 1].set_ylabel('Beam Angle (deg)')
    axes[0, 1].set_title('PID Control Response')
    axes[0, 1].set_xlabel('Time (s)')
    
    # Control effort distribution
    rl_efforts = [calculate_control_effort(trial) for trial in rl_trials]
    pid_efforts = [calculate_control_effort(trial) for trial in pid_trials]
    
    axes[1, 0].bar([0, 1], [np.mean(rl_efforts), np.mean(pid_efforts)],
                  yerr=[np.std(rl_efforts), np.std(pid_efforts)],
                  color=[Config.RL_COLOR, Config.PID_COLOR],
                  alpha=0.7, capsize=5)
    axes[1, 0].set_xticks([0, 1])
    axes[1, 0].set_xticklabels(['RL', 'PID'])
    axes[1, 0].set_ylabel('Total Variation (deg)')
    axes[1, 0].set_title('Control Effort')
    
    # Mean absolute angle
    rl_angles = [trial['theta_deg'].abs().mean() for trial in rl_trials]
    pid_angles = [trial['theta_deg'].abs().mean() for trial in pid_trials]
    
    axes[1, 1].bar([0, 1], [np.mean(rl_angles), np.mean(pid_angles)],
                  yerr=[np.std(rl_angles), np.std(pid_angles)],
                  color=[Config.RL_COLOR, Config.PID_COLOR],
                  alpha=0.7, capsize=5)
    axes[1, 1].set_xticks([0, 1])
    axes[1, 1].set_xticklabels(['RL', 'PID'])
    axes[1, 1].set_ylabel('Mean |Angle| (deg)')
    axes[1, 1].set_title('Average Control Magnitude')
    
    plt.tight_layout()
    plt.savefig(output_path, format=Config.FIGURE_FORMAT, dpi=Config.DPI)
    plt.close()
    print(f"Saved: {output_path}")


def plot_phase_portrait(rl_trials: List[pd.DataFrame], 
                       pid_trials: List[pd.DataFrame], 
                       output_path: Path):
    """
    Figure 5: Phase portrait (position vs velocity).
    
    Shows system dynamics and stability.
    Demonstrates different control strategies.
    """
    fig, axes = plt.subplots(1, 2, figsize=(10, 4.5))
    
    # RL phase portrait
    for trial in rl_trials:
        axes[0].plot(trial['x'] * 100, trial['xdot'] * 100, 
                    color=Config.RL_COLOR, alpha=0.3, linewidth=0.8)
    axes[0].scatter([0], [0], color='red', marker='x', s=100, zorder=5, label='Origin')
    axes[0].set_xlabel('Position (cm)')
    axes[0].set_ylabel('Velocity (cm/s)')
    axes[0].set_title('RL Phase Portrait')
    axes[0].legend()
    axes[0].axhline(0, color='k', linestyle='--', linewidth=0.5, alpha=0.3)
    axes[0].axvline(0, color='k', linestyle='--', linewidth=0.5, alpha=0.3)
    
    # PID phase portrait
    for trial in pid_trials:
        axes[1].plot(trial['x'] * 100, trial['xdot'] * 100, 
                    color=Config.PID_COLOR, alpha=0.3, linewidth=0.8)
    axes[1].scatter([0], [0], color='red', marker='x', s=100, zorder=5, label='Origin')
    axes[1].set_xlabel('Position (cm)')
    axes[1].set_ylabel('Velocity (cm/s)')
    axes[1].set_title('PID Phase Portrait')
    axes[1].legend()
    axes[1].axhline(0, color='k', linestyle='--', linewidth=0.5, alpha=0.3)
    axes[1].axvline(0, color='k', linestyle='--', linewidth=0.5, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_path, format=Config.FIGURE_FORMAT, dpi=Config.DPI)
    plt.close()
    print(f"Saved: {output_path}")


def plot_cumulative_error(rl_trials: List[pd.DataFrame], 
                         pid_trials: List[pd.DataFrame], 
                         output_path: Path):
    """
    Figure 6: Cumulative squared error over time.
    
    Shows how error accumulates and which controller stabilizes faster.
    """
    fig, ax = plt.subplots(1, 1, figsize=(7, 4.5))
    
    # Calculate mean cumulative error
    def calc_cumulative_error(trials):
        # Find common time base
        min_length = min(len(trial) for trial in trials)
        cum_errors = []
        
        for trial in trials:
            # FIX: Weight by dt to get a proper time-integral of squared error.
            # Without this, whichever controller has a higher sample rate 
            # accumulates more error regardless of actual performance.
            error_sq = trial['error_sq'].values[:min_length]
            dt = trial['dt'].values[:min_length]
            cum_error = np.cumsum(error_sq * dt)
            cum_errors.append(cum_error)
        
        cum_errors = np.array(cum_errors)
        mean_cum = np.mean(cum_errors, axis=0)
        std_cum = np.std(cum_errors, axis=0)
        time_base = trials[0]['time_rel'].values[:min_length]
        
        return time_base, mean_cum, std_cum
    
    rl_time, rl_mean, rl_std = calc_cumulative_error(rl_trials)
    pid_time, pid_mean, pid_std = calc_cumulative_error(pid_trials)
    
    # Plot with confidence bands
    ax.plot(rl_time, rl_mean * 1e4, color=Config.RL_COLOR, linewidth=2, label='RL')
    ax.fill_between(rl_time, (rl_mean - rl_std) * 1e4, (rl_mean + rl_std) * 1e4,
                    color=Config.RL_COLOR, alpha=0.2)
    
    ax.plot(pid_time, pid_mean * 1e4, color=Config.PID_COLOR, linewidth=2, label='PID')
    ax.fill_between(pid_time, (pid_mean - pid_std) * 1e4, (pid_mean + pid_std) * 1e4,
                    color=Config.PID_COLOR, alpha=0.2)
    
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Cumulative Squared Error (cm²)')
    ax.set_title('Cumulative Error Accumulation')
    ax.legend()
    
    plt.tight_layout()
    plt.savefig(output_path, format=Config.FIGURE_FORMAT, dpi=Config.DPI)
    plt.close()
    print(f"Saved: {output_path}")


# =============================================================================
# SUMMARY TABLE GENERATION
# =============================================================================

def generate_latex_table(rl_metrics: Dict, pid_metrics: Dict, 
                        stats_results: Dict, output_path: Path):
    """
    Generate LaTeX table of results for paper.
    """
    lines = []
    lines.append(r"\begin{table}[h]")
    lines.append(r"\centering")
    lines.append(r"\caption{Performance Comparison (mean $\pm$ std)}")
    lines.append(r"\label{tab:performance}")
    lines.append(r"\begin{tabular}{lccc}")
    lines.append(r"\hline")
    lines.append(r"Metric & RL & PID & Improvement \\")
    lines.append(r"\hline")
    
    metrics_display = {
        'RMSE': ('RMSE (mm)', 1000),
        'MAE': ('MAE (mm)', 1000),
        'Max Error': ('Max Error (mm)', 1000),
        'Settling Time': ('Settling Time (s)', 1),
        'Control Effort': ('Control Effort (deg)', 1),
    }
    
    for key, (display_name, scale) in metrics_display.items():
        if key in rl_metrics and key in pid_metrics:
            rl_mean, rl_std = rl_metrics[key]
            pid_mean, pid_std = pid_metrics[key]
            
            rl_mean *= scale
            rl_std *= scale
            pid_mean *= scale
            pid_std *= scale
            
            improvement = ""
            if key in stats_results:
                imp_pct = stats_results[key]['improvement']
                p_val = stats_results[key]['p_value']
                
                sig = ""
                if p_val < 0.001:
                    sig = "^{***}"
                elif p_val < 0.01:
                    sig = "^{**}"
                elif p_val < 0.05:
                    sig = "^{*}"
                
                improvement = f"${imp_pct:+.1f}\\%{sig}$"
            
            line = f"{display_name} & ${rl_mean:.2f} \\pm {rl_std:.2f}$ & ${pid_mean:.2f} \\pm {pid_std:.2f}$ & {improvement} \\\\"
            lines.append(line)
    
    lines.append(r"\hline")
    lines.append(r"\multicolumn{4}{l}{\footnotesize $^{*}p<0.05$, $^{**}p<0.01$, $^{***}p<0.001$} \\")
    lines.append(r"\end{tabular}")
    lines.append(r"\end{table}")
    
    # Save to file
    with open(output_path, 'w') as f:
        f.write('\n'.join(lines))
    
    print(f"Saved LaTeX table: {output_path}")


def generate_markdown_table(rl_metrics: Dict, pid_metrics: Dict, 
                           stats_results: Dict, output_path: Path):
    """
    Generate markdown table for easy viewing.
    """
    lines = []
    lines.append("# Performance Comparison Results\n")
    lines.append("| Metric | RL | PID | Improvement | p-value |")
    lines.append("|--------|----|----|-------------|---------|")
    
    metrics_display = {
        'RMSE': ('RMSE (mm)', 1000),
        'MAE': ('MAE (mm)', 1000),
        'Max Error': ('Max Error (mm)', 1000),
        'Settling Time': ('Settling Time (s)', 1),
        'Control Effort': ('Control Effort (deg)', 1),
    }
    
    for key, (display_name, scale) in metrics_display.items():
        if key in rl_metrics and key in pid_metrics:
            rl_mean, rl_std = rl_metrics[key]
            pid_mean, pid_std = pid_metrics[key]
            
            rl_mean *= scale
            rl_std *= scale
            pid_mean *= scale
            pid_std *= scale
            
            improvement = ""
            p_val_str = ""
            if key in stats_results:
                imp_pct = stats_results[key]['improvement']
                p_val = stats_results[key]['p_value']
                
                improvement = f"{imp_pct:+.1f}%"
                p_val_str = f"{p_val:.4f}" if p_val >= 0.0001 else "< 0.0001"
            
            line = f"| {display_name} | {rl_mean:.2f} ± {rl_std:.2f} | {pid_mean:.2f} ± {pid_std:.2f} | {improvement} | {p_val_str} |"
            lines.append(line)
    
    # Save to file
    with open(output_path, 'w') as f:
        f.write('\n'.join(lines))
    
    print(f"Saved Markdown table: {output_path}")


# =============================================================================
# MAIN ANALYSIS PIPELINE
# =============================================================================

def main():
    """Main analysis and plotting pipeline"""
    
    # Setup
    setup_plot_style()
    Config.OUTPUT_DIR.mkdir(exist_ok=True)
    
    print("="*60)
    print("Ball-on-Beam Performance Analysis")
    print("="*60)
    
    # Load data
    print("\nLoading data...")
    rl_trials = load_all_trials(Config.RL_FILES, Config.DATA_DIR, Config.TIME_WINDOW, 
                                Config.FLIP_NEGATIVE_INITIAL_POSITION)
    pid_trials = load_all_trials(Config.PID_FILES, Config.DATA_DIR, Config.TIME_WINDOW,
                                 Config.FLIP_NEGATIVE_INITIAL_POSITION)
    
    # Load simulation data if available
    sim_trials = []
    if Config.SIM_FILES:
        sim_trials = load_all_trials(Config.SIM_FILES, Config.DATA_DIR, Config.TIME_WINDOW,
                                    Config.FLIP_NEGATIVE_INITIAL_POSITION)
        print(f"Loaded {len(sim_trials)} simulation trials")
    
    print(f"Loaded {len(rl_trials)} RL trials")
    print(f"Loaded {len(pid_trials)} PID trials")
    
    if len(rl_trials) == 0 or len(pid_trials) == 0:
        print("\nError: No trials found! Check Config.RL_FILES and Config.PID_FILES")
        return
    
    # Calculate metrics
    print("\nCalculating metrics...")
    rl_metrics = aggregate_metrics(rl_trials)
    pid_metrics = aggregate_metrics(pid_trials)
    if sim_trials:
        sim_metrics = aggregate_metrics(sim_trials)
    
    # Statistical tests
    print("\nPerforming statistical tests...")
    stats_results = perform_statistical_tests(rl_trials, pid_trials)
    
    # Print results
    print("\n" + "="*60)
    print("RESULTS SUMMARY")
    print("="*60)
    
    for metric in ['RMSE', 'MAE', 'Max Error', 'Settling Time']:
        if metric in rl_metrics and metric in pid_metrics:
            rl_mean, rl_std = rl_metrics[metric]
            pid_mean, pid_std = pid_metrics[metric]
            
            unit = 'mm' if metric in ['RMSE', 'MAE', 'Max Error'] else 's'
            scale = 1000 if metric in ['RMSE', 'MAE', 'Max Error'] else 1
            
            print(f"\n{metric}:")
            print(f"  RL (Hardware):  {rl_mean*scale:.2f} ± {rl_std*scale:.2f} {unit}")
            print(f"  PID (Hardware): {pid_mean*scale:.2f} ± {pid_std*scale:.2f} {unit}")
            
            if sim_trials and metric in sim_metrics:
                sim_mean, sim_std = sim_metrics[metric]
                print(f"  RL (Simulation): {sim_mean*scale:.2f} ± {sim_std*scale:.2f} {unit}")
            
            if metric in stats_results:
                imp = stats_results[metric]['improvement']
                p_val = stats_results[metric]['p_value']
                sig = "***" if p_val < 0.001 else "**" if p_val < 0.01 else "*" if p_val < 0.05 else "ns"
                print(f"  Improvement: {imp:+.1f}% (p={p_val:.4f} {sig})")
    
    # Generate plots
    print("\n" + "="*60)
    print("GENERATING FIGURES")
    print("="*60)
    
    print("\nGenerating Figure 1: Single trial comparison...")
    plot_single_trial_comparison(
        rl_trials[0], pid_trials[0],
        Config.OUTPUT_DIR / f"fig1_trial_comparison.{Config.FIGURE_FORMAT}",
        sim_df=sim_trials[0] if sim_trials else None
    )
    
    print("Generating Figure 2: Error distribution...")
    plot_error_distribution(
        rl_trials, pid_trials,
        Config.OUTPUT_DIR / f"fig2_error_distribution.{Config.FIGURE_FORMAT}",
        sim_trials=sim_trials if sim_trials else None
    )
    
    print("Generating Figure 3: Metrics comparison...")
    plot_metrics_comparison(
        rl_metrics, pid_metrics, stats_results,
        Config.OUTPUT_DIR / f"fig3_metrics_comparison.{Config.FIGURE_FORMAT}"
    )
    
    print("Generating Figure 4: Control effort...")
    plot_control_effort(
        rl_trials, pid_trials,
        Config.OUTPUT_DIR / f"fig4_control_effort.{Config.FIGURE_FORMAT}"
    )
    
    print("Generating Figure 5: Phase portrait...")
    plot_phase_portrait(
        rl_trials, pid_trials,
        Config.OUTPUT_DIR / f"fig5_phase_portrait.{Config.FIGURE_FORMAT}"
    )
    
    print("Generating Figure 6: Cumulative error...")
    plot_cumulative_error(
        rl_trials, pid_trials,
        Config.OUTPUT_DIR / f"fig6_cumulative_error.{Config.FIGURE_FORMAT}"
    )
    
    # Generate tables
    print("\n" + "="*60)
    print("GENERATING TABLES")
    print("="*60)
    
    print("\nGenerating LaTeX table...")
    generate_latex_table(
        rl_metrics, pid_metrics, stats_results,
        Config.OUTPUT_DIR / "table_results.tex"
    )
    
    print("Generating Markdown table...")
    generate_markdown_table(
        rl_metrics, pid_metrics, stats_results,
        Config.OUTPUT_DIR / "table_results.md"
    )
    
    print("\n" + "="*60)
    print("ANALYSIS COMPLETE!")
    print("="*60)
    print(f"\nAll figures saved to: {Config.OUTPUT_DIR}/")
    print("\nRecommended figures for journal paper:")
    print("  - Figure 1: Representative trial (MUST HAVE)")
    print("  - Figure 2: Error distribution")
    print("  - Figure 3: Metrics comparison (MUST HAVE)")
    print("  - Figure 4 or 5: Control effort or Phase portrait")
    print("  - Table: Performance comparison (MUST HAVE)")


if __name__ == "__main__":
    main()