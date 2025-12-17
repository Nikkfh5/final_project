#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Offline trajectory visualizer.

Reads trajectory data from CSV/JSON file (format: t, x, y, theta) and
visualizes it with matplotlib. Interactive mode adds a slider and click
selection to inspect points.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import csv
import json
import argparse
import os


def read_csv_trajectory(filename):
    """
    Read trajectory from CSV file.
    
    Expected format: t, x, y, theta [, frame] (one row per sample)
    
    Args:
        filename: Path to CSV file
        
    Returns:
        (times, x, y, theta, frame_numbers) as numpy arrays
        frame_numbers may be None if not present in file
    """
    times = []
    x = []
    y = []
    theta = []
    frame_numbers = []
    has_frames = False
    
    with open(filename, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            if len(row) < 4:
                continue
            try:
                t_val = float(row[0])
                x_val = float(row[1])
                y_val = float(row[2])
                theta_val = float(row[3])
                if len(row) >= 5:
                    frame_numbers.append(int(row[4]))
                    has_frames = True
                else:
                    frame_numbers.append(None)
            except ValueError:
                # Likely a header row
                continue
            times.append(t_val)
            x.append(x_val)
            y.append(y_val)
            theta.append(theta_val)
    
    frames = np.array(frame_numbers) if has_frames else None
    return np.array(times), np.array(x), np.array(y), np.array(theta), frames


def read_json_trajectory(filename):
    """
    Read trajectory from JSON file.
    
    Expected format: list of dicts with keys 't', 'x', 'y', 'theta' [, 'frame']
    or dict with keys 'times', 'x', 'y', 'theta' [, 'frames'] as arrays
    
    Args:
        filename: Path to JSON file
        
    Returns:
        (times, x, y, theta, frame_numbers) as numpy arrays
        frame_numbers may be None if not present
    """
    with open(filename, 'r') as f:
        data = json.load(f)
    
    if isinstance(data, list):
        times = [item.get('t') for item in data]
        x = [item.get('x') for item in data]
        y = [item.get('y') for item in data]
        theta = [item.get('theta') for item in data]
        frames = [item.get('frame') for item in data] if any('frame' in item for item in data) else None
    elif isinstance(data, dict):
        if all(k in data for k in ['times', 'x', 'y', 'theta']):
            times = data['times']
            x = data['x']
            y = data['y']
            theta = data['theta']
            frames = data.get('frames', None)
        else:
            raise ValueError("Unsupported JSON format. Expected list of {t,x,y,theta} or dict with arrays.")
    else:
        raise ValueError("Unsupported JSON structure.")
    
    frame_array = np.array(frames) if frames is not None else None
    return np.array(times), np.array(x), np.array(y), np.array(theta), frame_array


def visualize_trajectory(times, x, y, theta, interactive=False, frame_numbers=None):
    """
    Visualize trajectory as 2D plot.
    
    Args:
        times: Time array
        x: X coordinates
        y: Y coordinates
        theta: Orientations (radians)
        interactive: If True, add time slider and interactive click-to-select
        frame_numbers: Optional array of frame numbers for each point
    """
    if len(times) == 0:
        print("No points to visualize.")
        return

    fig = plt.figure(figsize=(14, 10))
    if interactive:
        plt.subplots_adjust(bottom=0.2, right=0.75)
    
    ax = fig.add_subplot(111)
    
    # Plot trajectory path
    (traj_line,) = ax.plot(x, y, 'b-', linewidth=2, label='Trajectory', alpha=0.7)
    current_point, = ax.plot([], [], 'ro', markersize=10, label='Selected', zorder=5)
    time_text = ax.text(0.02, 0.98, '', transform=ax.transAxes, 
                       bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
                       fontsize=10, verticalalignment='top')
    
    # Mark start and end
    if len(x) > 0:
        ax.plot(x[0], y[0], 'go', markersize=10, label='Start')
        ax.plot(x[-1], y[-1], 'ro', markersize=10, label='End')
        
        # Draw orientation arrows at start and end
        dx_start = 0.1 * np.cos(theta[0])
        dy_start = 0.1 * np.sin(theta[0])
        ax.arrow(x[0], y[0], dx_start, dy_start, head_width=0.05, 
                head_length=0.05, fc='green', ec='green')
        
        dx_end = 0.1 * np.cos(theta[-1])
        dy_end = 0.1 * np.sin(theta[-1])
        ax.arrow(x[-1], y[-1], dx_end, dy_end, head_width=0.05, 
                head_length=0.05, fc='red', ec='red')
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Robot Trajectory')
    ax.legend()
    ax.grid(True)
    ax.axis('equal')
    
    def update_marker(idx):
        current_point.set_data([x[idx]], [y[idx]])
        frame_info = ""
        if frame_numbers is not None and idx < len(frame_numbers):
            frame_info = f", frame={frame_numbers[idx]}"
        time_text.set_text("t=%.2f s\nx=%.3f m\ny=%.3f m\ntheta=%.2f rad (%.1f°)%s" %
                           (times[idx], x[idx], y[idx], theta[idx], 
                            np.degrees(theta[idx]), frame_info))
        fig.canvas.draw_idle()

    if interactive:
        slider_ax = plt.axes([0.15, 0.05, 0.7, 0.03])
        time_slider = Slider(slider_ax, 't', times[0], times[-1], valinit=times[0])

        def slider_update(val):
            idx = np.searchsorted(times, val, side='left')
            idx = min(max(idx, 0), len(times) - 1)
            update_marker(idx)

        time_slider.on_changed(slider_update)

        def onclick(event):
            if event.inaxes != ax:
                return
            dists = (x - event.xdata) ** 2 + (y - event.ydata) ** 2
            idx = int(np.argmin(dists))
            time_slider.set_val(times[idx])
            update_marker(idx)

        fig.canvas.mpl_connect('button_press_event', onclick)
        update_marker(0)
    
    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(description='Visualize robot trajectory from CSV/JSON')
    parser.add_argument('filename', type=str, help='Path to trajectory file (CSV or JSON)')
    parser.add_argument('--format', type=str, choices=['csv', 'json', 'auto'],
                       default='auto', help='File format (auto-detect if not specified)')
    parser.add_argument('--interactive', action='store_true',
                       help='Enable interactive slider and click-to-select point')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.filename):
        print("Error: File not found: %s" % args.filename)
        return
    
    # Detect format
    if args.format == 'auto':
        if args.filename.endswith('.csv'):
            format_type = 'csv'
        elif args.filename.endswith('.json'):
            format_type = 'json'
        else:
            print("Error: Cannot auto-detect format. Please specify --format")
            return
    else:
        format_type = args.format
    
    # Read trajectory
    try:
        if format_type == 'csv':
            times, x, y, theta, frames = read_csv_trajectory(args.filename)
        else:
            times, x, y, theta, frames = read_json_trajectory(args.filename)
    except Exception as e:
        print("Error reading trajectory: %s" % str(e))
        return
    
    print("Loaded trajectory with %d points" % len(times))
    if frames is not None:
        # Count non-None frames
        valid_frames = frames[frames != None] if len(frames) > 0 else []
        print("Frame numbers available: %d frames" % len(valid_frames))
    
    # Visualize
    visualize_trajectory(times, x, y, theta, interactive=args.interactive, frame_numbers=frames)


if __name__ == '__main__':
    main()
