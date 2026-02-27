#!/usr/bin/env python3
"""
Visualise Grace balancing controller CSV log.

Reads the CSV produced by lqr_balancing_controller and plots:
  - Measured vs estimated pitch (θ)
  - Measured vs estimated pitch rate (θ̇)
  - Measured vs estimated position (p)
  - Measured vs estimated velocity (ṗ)
  - Control torque (τ)

Usage:
    python3 visualize_balancing.py                         # default /tmp/grace_balancing_log.csv
    python3 visualize_balancing.py /path/to/log.csv
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), 'balancing_log.csv')

    data = np.genfromtxt(path, delimiter=',', names=True)
    t = data['time_s']

    has_observer = not np.allclose(data['theta_meas'], data['theta_est'], atol=1e-12)

    obs_label = 'observer' if has_observer else None
    title_suffix = '  (observer ON)' if has_observer else '  (observer OFF)'

    fig, axes = plt.subplots(5, 1, figsize=(14, 12), sharex=True)
    fig.suptitle('Grace Balancing Controller Log' + title_suffix, fontsize=14)

    # Color scheme
    c_meas = 'tab:blue'
    c_est  = 'tab:orange'
    c_err  = 'tab:green'

    # ---- Pitch ----
    axes[0].plot(t, np.degrees(data['theta_meas']), color=c_meas,
                 label='measured', linewidth=0.8, alpha=0.8)
    if has_observer:
        axes[0].plot(t, np.degrees(data['theta_est']), color=c_est,
                     label='observer', linewidth=0.8, linestyle='--')
        err = np.degrees(data['theta_meas'] - data['theta_est'])
        ax0r = axes[0].twinx()
        ax0r.fill_between(t, err, alpha=0.15, color=c_err, label='error')
        ax0r.set_ylabel('error [°]', fontsize=8, color=c_err)
        ax0r.tick_params(axis='y', labelcolor=c_err, labelsize=7)
    axes[0].set_ylabel('θ [°]')
    axes[0].legend(loc='upper right', fontsize=8)
    axes[0].grid(True, alpha=0.3)

    # ---- Pitch rate ----
    axes[1].plot(t, np.degrees(data['theta_dot_meas']), color=c_meas,
                 label='measured', linewidth=0.8, alpha=0.8)
    if has_observer:
        axes[1].plot(t, np.degrees(data['theta_dot_est']), color=c_est,
                     label='observer', linewidth=0.8, linestyle='--')
        err = np.degrees(data['theta_dot_meas'] - data['theta_dot_est'])
        ax1r = axes[1].twinx()
        ax1r.fill_between(t, err, alpha=0.15, color=c_err, label='error')
        ax1r.set_ylabel('error [°/s]', fontsize=8, color=c_err)
        ax1r.tick_params(axis='y', labelcolor=c_err, labelsize=7)
    axes[1].set_ylabel('θ̇ [°/s]')
    axes[1].legend(loc='upper right', fontsize=8)
    axes[1].grid(True, alpha=0.3)

    # ---- Position ----
    axes[2].plot(t, data['p_meas'], color=c_meas,
                 label='measured', linewidth=0.8, alpha=0.8)
    if has_observer:
        axes[2].plot(t, data['p_est'], color=c_est,
                     label='observer', linewidth=0.8, linestyle='--')
        err = data['p_meas'] - data['p_est']
        ax2r = axes[2].twinx()
        ax2r.fill_between(t, err, alpha=0.15, color=c_err, label='error')
        ax2r.set_ylabel('error [m]', fontsize=8, color=c_err)
        ax2r.tick_params(axis='y', labelcolor=c_err, labelsize=7)
    axes[2].set_ylabel('p [m]')
    axes[2].legend(loc='upper right', fontsize=8)
    axes[2].grid(True, alpha=0.3)

    # ---- Velocity ----
    axes[3].plot(t, data['p_dot_meas'], color=c_meas,
                 label='measured', linewidth=0.8, alpha=0.8)
    if has_observer:
        axes[3].plot(t, data['p_dot_est'], color=c_est,
                     label='observer', linewidth=0.8, linestyle='--')
        err = data['p_dot_meas'] - data['p_dot_est']
        ax3r = axes[3].twinx()
        ax3r.fill_between(t, err, alpha=0.15, color=c_err, label='error')
        ax3r.set_ylabel('error [m/s]', fontsize=8, color=c_err)
        ax3r.tick_params(axis='y', labelcolor=c_err, labelsize=7)
    axes[3].set_ylabel('ṗ [m/s]')
    axes[3].legend(loc='upper right', fontsize=8)
    axes[3].grid(True, alpha=0.3)

    # ---- Torque ----
    axes[4].plot(t, data['tau_cmd'], color='tab:red', linewidth=0.8)
    axes[4].set_ylabel('τ [N·m]')
    axes[4].set_xlabel('Time [s]')
    axes[4].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
