#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Offline trajectory visualizer.

Reads trajectory data from CSV/JSON file (format: t, x, y, theta) and
visualizes it with matplotlib.
"""

import rospy
import numpy as np
import matplotlib.pyplot as plt
import csv
import json
import argparse
import os


def read_csv_trajectory(filename):
    """
    Read trajectory from CSV file.
    
    Expected format: t, x, y, theta (one row per sample)
    
    Args:
        filename: Path to CSV file
        
    Returns:
        (times, x, y, theta) as numpy arrays
    """
    times = []
    x = []
    y = []
    theta = []
    
    with open(filename, 'r') as f:
        reader = csv.reader(f)
        # TODO: Handle header row if present
        for row in reader:
            if len(row) >= 4:
                times.append(float(row[0]))
                x.append(float(row[1]))
                y.append(float(row[2]))
                theta.append(float(row[3]))
    
    return np.array(times), np.array(x), np.array(y), np.array(theta)


def read_json_trajectory(filename):
    """
    Read trajectory from JSON file.
    
    Expected format: list of dicts with keys 't', 'x', 'y', 'theta'
    or dict with keys 'times', 'x', 'y', 'theta' as arrays
    
    Args:
        filename: Path to JSON file
        
    Returns:
        (times, x, y, theta) as numpy arrays
    """
    with open(filename, 'r') as f:
        data = json.load(f)
    
    # TODO: Handle different JSON formats
    if isinstance(data, list):
        times = [item['t'] for item in data]
        x = [item['x'] for item in data]
        y = [item['y'] for item in data]
        theta = [item['theta'] for item in data]
    else:
        times = data['times']
        x = data['x']
        y = data['y']
        theta = data['theta']
    
    return np.array(times), np.array(x), np.array(y), np.array(theta)


def visualize_trajectory(times, x, y, theta, interactive=False):
    """
    Visualize trajectory as 2D plot.
    
    Args:
        times: Time array
        x: X coordinates
        y: Y coordinates
        theta: Orientations (radians)
        interactive: If True, add time slider and interactive features (TODO)
    """
    fig, ax = plt.subplots(figsize=(10, 10))
    
    # Plot trajectory path
    ax.plot(x, y, 'b-', linewidth=2, label='Trajectory')
    
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
    
    # TODO: Add time slider for interactive visualization
    # TODO: Add click handler to show coordinates and time at point
    
    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(description='Visualize robot trajectory from CSV/JSON')
    parser.add_argument('filename', type=str, help='Path to trajectory file (CSV or JSON)')
    parser.add_argument('--format', type=str, choices=['csv', 'json', 'auto'],
                       default='auto', help='File format (auto-detect if not specified)')
    parser.add_argument('--interactive', action='store_true',
                       help='Enable interactive features (TODO)')
    
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
            times, x, y, theta = read_csv_trajectory(args.filename)
        else:
            times, x, y, theta = read_json_trajectory(args.filename)
    except Exception as e:
        print("Error reading trajectory: %s" % str(e))
        return
    
    print("Loaded trajectory with %d points" % len(times))
    
    # Visualize
    visualize_trajectory(times, x, y, theta, interactive=args.interactive)


if __name__ == '__main__':
    main()

