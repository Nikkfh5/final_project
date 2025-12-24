import plotly.graph_objects as go
import webbrowser
import sys
import json
import os
import argparse
import matplotlib.pyplot as plt
import mplcursors
import math
import numpy as np
from matplotlib.collections import LineCollection
from plotly.subplots import make_subplots


# --- Configurable thresholds ---
TELEPORT_SPEED_THRESHOLD = 3.0  # m/s
MIN_DT = 1e-6
STOP_TELEPORT_WINDOW = 1.0  # s
EXCESSIVE_STOP_SHORT_DURATION = 1.0  # s
EXCESSIVE_STOP_COUNT_THRESHOLD = 3
EXCESSIVE_STOP_WINDOW = 10.0  # s
ORIENTATION_FLIP_THRESHOLD = math.pi / 2.0

EVENT_STYLE_MPL = {
    "TELEPORT": {"color": "black", "marker": "x", "size": 150, "label": "TELEPORT", "linewidth": 2.2},
    "ANOMALY_STOP_TELEPORT": {
        "color": "#6a1b9a",
        "marker": "*",
        "size": 140,
        "label": "ANOMALY_STOP_TELEPORT",
        "linewidth": 1.2,
    },
    "ANOMALY_EXCESSIVE_STOPS": {
        "color": "#ef6c00",
        "marker": "v",
        "size": 120,
        "label": "ANOMALY_EXCESSIVE_STOPS",
        "linewidth": 1.2,
    },
    "ANOMALY_ORIENTATION_FLIP": {
        "color": "#c2185b",
        "marker": "D",
        "size": 110,
        "label": "ANOMALY_ORIENTATION_FLIP",
        "linewidth": 1.2,
    },
}

EVENT_STYLE_PLOTLY = {
    "TELEPORT": {"color": "black", "symbol": "x", "size": 14, "name": "TELEPORT"},
    "ANOMALY_STOP_TELEPORT": {"color": "#6a1b9a", "symbol": "star-diamond", "size": 12, "name": "ANOMALY_STOP_TELEPORT"},
    "ANOMALY_EXCESSIVE_STOPS": {"color": "#ef6c00", "symbol": "triangle-down", "size": 12, "name": "ANOMALY_EXCESSIVE_STOPS"},
    "ANOMALY_ORIENTATION_FLIP": {"color": "#c2185b", "symbol": "diamond", "size": 11, "name": "ANOMALY_ORIENTATION_FLIP"},
}


def arrow_head_triangle(x0, y0, th, head_len, head_w):
    """
    Возвращает координаты треугольника-наконечника стрелки.
    Вершина в направлении th, основание перпендикулярно.
    """
    # Вектор направления
    ux = math.cos(th)
    uy = math.sin(th)

    # Перпендикуляр
    px = -uy
    py = ux

    # Вершина (tip) будет в точке (x0, y0)
    tip_x, tip_y = x0, y0

    # Центр основания (back)
    back_x = tip_x - head_len * ux
    back_y = tip_y - head_len * uy

    # Два угла основания
    left_x = back_x + (head_w / 2.0) * px
    left_y = back_y + (head_w / 2.0) * py
    right_x = back_x - (head_w / 2.0) * px
    right_y = back_y - (head_w / 2.0) * py

    # Замыкаем треугольник (последняя точка = первая)
    tri_x = [tip_x, left_x, right_x, tip_x]
    tri_y = [tip_y, left_y, right_y, tip_y]
    return tri_x, tri_y


def split_points_and_events(data):
    """
    Split raw trajectory JSON into trajectory points and non-point events.
    Events with type/event STOP_START/STOP_END/TELEPORT/ANOMALY_* are kept in the event list.
    """
    points = []
    events = []

    def append_event(item):
        if not isinstance(item, dict):
            return False
        evt_type = item.get("event") or item.get("type")
        if not evt_type:
            return False
        events.append(
            {
                "event": evt_type,
                "t": float(item.get("t", 0.0)),
                "x": float(item.get("x", 0.0)),
                "y": float(item.get("y", 0.0)),
                "theta": item.get("theta"),
                "frame": item.get("frame"),
                "speed": item.get("speed"),
                "details": item.get("details"),
            }
        )
        return True

    events_raw = []
    iterable = []
    if isinstance(data, list):
        iterable = data
    elif isinstance(data, dict):
        events_raw = data.get("events", [])
        if "points" in data:
            iterable = data.get("points", [])
        elif all(k in data for k in ["times", "x", "y", "theta"]):
            frames = data.get("frames")
            for idx, (t_val, x_val, y_val, th_val) in enumerate(
                zip(data["times"], data["x"], data["y"], data["theta"])
            ):
                frame_id = frames[idx] if frames and idx < len(frames) else idx
                points.append(
                    {"t": t_val, "x": x_val, "y": y_val, "theta": th_val, "frame": frame_id}
                )
        else:
            raise ValueError("Unsupported trajectory JSON structure")
    else:
        raise ValueError("Unsupported trajectory JSON structure")

    for item in iterable:
        if append_event(item):
            continue
        if isinstance(item, dict):
            points.append(item)

    for item in events_raw:
        append_event(item)

    events.sort(key=lambda e: e["t"])
    return points, events


def build_stop_intervals(stop_events):
    """Pair STOP_START/STOP_END events into ordered intervals."""
    intervals = []
    start_evt = None
    for evt in sorted(stop_events, key=lambda e: e["t"]):
        if evt["event"] == "STOP_START":
            start_evt = evt
        elif evt["event"] == "STOP_END" and start_evt is not None:
            intervals.append(
                {
                    "start": start_evt,
                    "end": evt,
                    "duration": max(0.0, evt["t"] - start_evt["t"]),
                }
            )
            start_evt = None
    return intervals


def build_timeline_segments(times, intervals):
    """Build MOVING/STOPPED segments across the trajectory time span."""
    if not times:
        return []
    start_time, end_time = min(times), max(times)
    segments = []
    current = start_time
    for interval in intervals:
        seg_start = max(start_time, interval["start"]["t"])
        seg_end = min(end_time, interval["end"]["t"])
        if seg_end <= seg_start:
            continue
        if seg_start > current:
            segments.append({"state": "MOVING", "start": current, "end": seg_start})
        segments.append({"state": "STOPPED", "start": seg_start, "end": seg_end})
        current = seg_end
    if current < end_time:
        segments.append({"state": "MOVING", "start": current, "end": end_time})
    for seg in segments:
        seg["duration"] = seg["end"] - seg["start"]
    return segments


def is_stopped_at(time_value, intervals):
    """Return True if time_value falls within any STOP interval."""
    for interval in intervals:
        if interval["start"]["t"] <= time_value <= interval["end"]["t"]:
            return True
    return False


def detect_teleports(points, v_threshold):
    """Detect position jumps where implied speed exceeds v_threshold."""
    teleports = []
    for i in range(1, len(points)):
        curr = points[i]
        prev = points[i - 1]
        dt = float(curr.get("t", 0.0)) - float(prev.get("t", 0.0))
        if dt <= MIN_DT:
            continue
        dx = float(curr.get("x", 0.0)) - float(prev.get("x", 0.0))
        dy = float(curr.get("y", 0.0)) - float(prev.get("y", 0.0))
        speed = math.hypot(dx, dy) / dt
        if speed > v_threshold:
            teleports.append(
                {
                    "event": "TELEPORT",
                    "t": float(curr.get("t", 0.0)),
                    "x": float(curr.get("x", 0.0)),
                    "y": float(curr.get("y", 0.0)),
                    "frame": curr.get("frame", i),
                    "speed": speed,
                }
            )
    return teleports


def compute_quality_metrics(points, stop_intervals, teleports):
    """Compute aggregate tracking quality metrics."""
    if not points:
        return {
            "total_duration": 0.0,
            "total_distance": 0.0,
            "avg_speed": 0.0,
            "max_speed": 0.0,
            "stopped_time": 0.0,
            "moving_time": 0.0,
            "stop_count": 0,
            "teleport_count": len(teleports),
            "percent_stopped": 0.0,
            "percent_moving": 0.0,
        }

    times = [float(p.get("t", 0.0)) for p in points]
    xs = [float(p.get("x", 0.0)) for p in points]
    ys = [float(p.get("y", 0.0)) for p in points]

    total_duration = max(times) - min(times) if len(times) > 1 else 0.0
    total_distance = 0.0
    speeds_local = []

    for i in range(len(points) - 1):
        dx = xs[i + 1] - xs[i]
        dy = ys[i + 1] - ys[i]
        dt = times[i + 1] - times[i]
        if dt <= MIN_DT:
            continue
        step_dist = math.hypot(dx, dy)
        total_distance += step_dist
        speeds_local.append(step_dist / dt)

    avg_speed = (total_distance / total_duration) if total_duration > 0 else 0.0
    max_speed = max(speeds_local) if speeds_local else 0.0
    stopped_time = sum(iv["duration"] for iv in stop_intervals)
    moving_time = max(0.0, total_duration - stopped_time)

    return {
        "total_duration": total_duration,
        "total_distance": total_distance,
        "avg_speed": avg_speed,
        "max_speed": max_speed,
        "stopped_time": stopped_time,
        "moving_time": moving_time,
        "stop_count": len(stop_intervals),
        "teleport_count": len(teleports),
        "percent_stopped": (stopped_time / total_duration * 100.0) if total_duration > 0 else 0.0,
        "percent_moving": (moving_time / total_duration * 100.0) if total_duration > 0 else 0.0,
    }


def detect_behavioral_anomalies(points, stop_intervals, teleports):
    """Detect behavioral anomalies using stops, teleports, and orientation changes."""
    anomalies = []

    # A) TELEPORT soon after STOP_END
    stop_end_times = [iv["end"]["t"] for iv in stop_intervals]
    for tp in teleports:
        t_tp = tp.get("t", 0.0)
        for end_t in stop_end_times:
            delta = t_tp - end_t
            if 0.0 <= delta <= STOP_TELEPORT_WINDOW:
                anomalies.append(
                    {
                        "event": "ANOMALY_STOP_TELEPORT",
                        "t": t_tp,
                        "x": tp.get("x", 0.0),
                        "y": tp.get("y", 0.0),
                        "details": {
                            "delta_t": delta,
                            "stop_end": end_t,
                            "teleport_speed": tp.get("speed"),
                        },
                    }
                )
                break

    # B) Excessive short stops in a time window
    short_stops = [iv for iv in stop_intervals if iv["duration"] < EXCESSIVE_STOP_SHORT_DURATION]
    short_stops.sort(key=lambda iv: iv["start"]["t"])
    i = 0
    while i < len(short_stops):
        window_start = short_stops[i]["start"]["t"]
        j = i
        while j < len(short_stops) and (short_stops[j]["start"]["t"] - window_start) <= EXCESSIVE_STOP_WINDOW:
            j += 1
        count = j - i
        if count > EXCESSIVE_STOP_COUNT_THRESHOLD:
            ref_iv = short_stops[j - 1]
            anomalies.append(
                {
                    "event": "ANOMALY_EXCESSIVE_STOPS",
                    "t": ref_iv["end"]["t"],
                    "x": ref_iv["end"]["x"],
                    "y": ref_iv["end"]["y"],
                    "details": {
                        "count": count,
                        "window": EXCESSIVE_STOP_WINDOW,
                        "threshold": EXCESSIVE_STOP_COUNT_THRESHOLD,
                    },
                }
            )
        i += 1

    # C) Orientation flip between frames
    for i in range(1, len(points)):
        prev_th = float(points[i - 1].get("theta", 0.0))
        th = float(points[i].get("theta", 0.0))
        dtheta = math.atan2(math.sin(th - prev_th), math.cos(th - prev_th))
        if abs(dtheta) > ORIENTATION_FLIP_THRESHOLD:
            anomalies.append(
                {
                    "event": "ANOMALY_ORIENTATION_FLIP",
                    "t": float(points[i].get("t", 0.0)),
                    "x": float(points[i].get("x", 0.0)),
                    "y": float(points[i].get("y", 0.0)),
                    "details": {"delta_theta": dtheta},
                }
            )

    anomalies.sort(key=lambda e: e["t"])
    return anomalies


def merge_events(existing, new):
    """Merge two event lists, deduplicating by (event, t, x, y)."""
    merged = []
    seen = set()
    for src in (existing, new):
        for evt in src:
            key = (
                evt.get("event"),
                round(float(evt.get("t", 0.0)), 6),
                round(float(evt.get("x", 0.0)), 4),
                round(float(evt.get("y", 0.0)), 4),
            )
            if key in seen:
                continue
            seen.add(key)
            merged.append(evt)
    merged.sort(key=lambda e: e.get("t", 0.0))
    return merged


def build_augmented_payload(raw_data, points, events):
    """Create a normalized trajectory payload with top-level points/events."""
    payload = dict(raw_data) if isinstance(raw_data, dict) else {}
    payload["points"] = points
    payload["events"] = events
    return payload


def format_anomaly_description(evt):
    """Human-readable anomaly description."""
    details = evt.get("details") or {}
    evt_type = evt.get("event", "")
    if evt_type == "ANOMALY_STOP_TELEPORT":
        delta_t = details.get("delta_t", 0.0)
        speed = details.get("teleport_speed")
        parts = [f"Teleport {delta_t:.2f}s after STOP_END"]
        if speed is not None:
            parts.append(f"speed={speed:.2f} m/s")
        return ", ".join(parts)
    if evt_type == "ANOMALY_EXCESSIVE_STOPS":
        count = details.get("count")
        window = details.get("window")
        threshold = details.get("threshold")
        return f"{count} short stops (<{EXCESSIVE_STOP_SHORT_DURATION:.1f}s) in {window}s (>{threshold})"
    if evt_type == "ANOMALY_ORIENTATION_FLIP":
        dtheta = details.get("delta_theta")
        if dtheta is None:
            return "Orientation flip"
        return f"|Δtheta|={abs(dtheta):.2f} rad ({math.degrees(abs(dtheta)):.1f}°)"
    return evt_type


def plot_stop_annotations(ax, stop_events, stop_intervals):
    """Overlay stop start/end markers and duration labels on a matplotlib axis."""
    if not stop_events:
        return
    start_points = [(e["x"], e["y"]) for e in stop_events if e["event"] == "STOP_START"]
    end_points = [(e["x"], e["y"]) for e in stop_events if e["event"] == "STOP_END"]

    if start_points:
        xs, ys = zip(*start_points)
        ax.scatter(xs, ys, c="red", s=45, label="STOP_START", zorder=5, edgecolors="k")
    if end_points:
        xs, ys = zip(*end_points)
        ax.scatter(xs, ys, c="green", s=45, label="STOP_END", zorder=5, edgecolors="k")

    for interval in stop_intervals:
        sx, sy = interval["start"]["x"], interval["start"]["y"]
        ex, ey = interval["end"]["x"], interval["end"]["y"]
        mid_x = 0.5 * (sx + ex)
        mid_y = 0.5 * (sy + ey)
        ax.text(
            mid_x,
            mid_y,
            f"{interval['duration']:.1f}s",
            fontsize=8,
            color="black",
            bbox=dict(facecolor="white", alpha=0.75, edgecolor="none"),
            zorder=6,
        )


def plot_event_markers(ax, events, event_type):
    """Scatter helper for teleports/anomalies on matplotlib axes."""
    style = EVENT_STYLE_MPL.get(event_type)
    if not style or not events:
        return
    xs = [e.get("x", 0.0) for e in events]
    ys = [e.get("y", 0.0) for e in events]
    ax.scatter(
        xs,
        ys,
        s=style["size"],
        c=style["color"],
        marker=style["marker"],
        label=style["label"],
        linewidths=style.get("linewidth", 1.0),
        zorder=6,
    )


def add_plotly_events(fig, events, event_type, row, col):
    """Add plotly scatter for teleports/anomalies."""
    style = EVENT_STYLE_PLOTLY.get(event_type)
    if not style or not events:
        return
    hover_text = []
    for evt in events:
        base = (
            f"{event_type}<br>"
            f"t: {evt.get('t', 0.0):.2f}s<br>"
            f"x: {evt.get('x', 0.0):.3f} m<br>"
            f"y: {evt.get('y', 0.0):.3f} m"
        )
        if event_type == "TELEPORT":
            spd = evt.get("speed")
            speed_part = f"<br>speed: {spd:.2f} m/s" if spd is not None else ""
            hover_text.append(base + speed_part)
        else:
            hover_text.append(base + "<br>" + format_anomaly_description(evt))
    fig.add_trace(
        go.Scatter(
            x=[e.get("x", 0.0) for e in events],
            y=[e.get("y", 0.0) for e in events],
            mode="markers",
            marker=dict(color=style["color"], symbol=style["symbol"], size=style["size"]),
            name=style.get("name", event_type),
            hoverinfo="text",
            text=hover_text,
        ),
        row=row,
        col=col,
    )


# === Paths ===
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# Lifted package up one level, so project root is two levels above
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
LOG_DIR = os.path.join(PROJECT_ROOT, "tracker_logs")
TRAJECTORY_JSON = os.path.join(LOG_DIR, "trajectory.json")
OUTPUT_PNG = os.path.join(LOG_DIR, "trajectory_interactive.png")
REPORT_HTML = os.path.join(LOG_DIR, "report.html")

os.makedirs(LOG_DIR, exist_ok=True)


HTML_INTERACTIVE = os.path.join(LOG_DIR, "trajectory_interactive.html")
HTML_ANIMATED = os.path.join(LOG_DIR, "trajectory_animated.html")
PNG_MAIN = os.path.join(LOG_DIR, "trajectory_interactive.png")
PNG_HEATMAP = os.path.join(LOG_DIR, "trajectory_heatmap.png")
PNG_TIME = os.path.join(LOG_DIR, "trajectory_time_colored.png")
PNG_SPEED = os.path.join(LOG_DIR, "trajectory_speed_colored.png")




parser = argparse.ArgumentParser(description="Interactive trajectory viewer")
parser.add_argument(
    "--save",
    action="store_true",
    help="Save interactive trajectory plot to tracker_logs",
)
parser.add_argument(
    "--html",
    action="store_true",
    help="Save interactive trajectory to HTML (with hover)",
)
parser.add_argument(
    "--html-anim",
    action="store_true",
    help="Save animated interactive HTML with fixed scale and open in browser",
)
parser.add_argument(
    "--heatmap",
    action="store_true",
    help="Show and/or save trajectory heatmap (visit density)",
)
parser.add_argument(
    "--time-color",
    action="store_true",
    help="Show trajectory colored by time",
)
parser.add_argument(
    "--speed-color",
    action="store_true",
    help="Show trajectory colored by linear speed",
)
parser.add_argument(
    "--all",
    action="store_true",
    help="Generate everything: PNGs + HTMLs + report",
)

parser.add_argument(
    "--report",
    action="store_true",
    help="Generate a single HTML report (dashboard) combining all outputs",
)



args = parser.parse_args()


if args.report and not args.all:
    args.all = True

if args.all:
    args.save = True
    args.heatmap = True
    args.time_color = True
    args.speed_color = True
    args.html = True
    args.html_anim = True
    args.report = True


# === Load data ===
if not os.path.exists(TRAJECTORY_JSON):
    raise FileNotFoundError(f"Trajectory file not found: {TRAJECTORY_JSON}")

with open(TRAJECTORY_JSON) as f:
    raw_data = json.load(f)

points, events = split_points_and_events(raw_data)
trajectory_points = points
if not points:
    raise ValueError("No trajectory points found in trajectory.json")

stop_events = [e for e in events if e["event"] in ("STOP_START", "STOP_END")]
stop_intervals = build_stop_intervals(stop_events)

existing_teleports = [e for e in events if e["event"] == "TELEPORT"]
teleports_detected = detect_teleports(points, TELEPORT_SPEED_THRESHOLD)
teleports_for_detection = merge_events(existing_teleports, teleports_detected)
anomalies_detected = detect_behavioral_anomalies(points, stop_intervals, teleports_for_detection)

events = merge_events(events, teleports_detected + anomalies_detected)
stop_events = [e for e in events if e["event"] in ("STOP_START", "STOP_END")]
teleports = [e for e in events if e["event"] == "TELEPORT"]
anomaly_events = [e for e in events if str(e.get("event", "")).startswith("ANOMALY_")]
anomalies_by_type = {}
for evt in anomaly_events:
    anomalies_by_type.setdefault(evt["event"], []).append(evt)

stop_intervals = build_stop_intervals(stop_events)
timeline_segments = build_timeline_segments([p.get("t", 0.0) for p in points], stop_intervals)
start_events = [e for e in stop_events if e["event"] == "STOP_START"]
end_events = [e for e in stop_events if e["event"] == "STOP_END"]
interval_by_start = {iv["start"]["t"]: iv for iv in stop_intervals}
interval_by_end = {iv["end"]["t"]: iv for iv in stop_intervals}

x = [p.get("x", 0.0) for p in points]
y = [p.get("y", 0.0) for p in points]
t = [p.get("t", 0.0) for p in points]
theta = [p.get("theta", 0.0) for p in points]
frame = [p.get("frame", idx) for idx, p in enumerate(points)]

quality_metrics = compute_quality_metrics(points, stop_intervals, teleports)

print(f"[INFO] Loaded {len(points)} trajectory points from {TRAJECTORY_JSON}")
if stop_intervals:
    total_stopped = sum(iv["duration"] for iv in stop_intervals)
    print(f"[INFO] Detected {len(stop_intervals)} stop intervals (stopped {total_stopped:.2f}s)")
print(f"[INFO] Teleport detector threshold: {TELEPORT_SPEED_THRESHOLD:.2f} m/s; detected {len(teleports)}")
if anomaly_events:
    counts_by_type = {}
    for evt in anomaly_events:
        counts_by_type[evt["event"]] = counts_by_type.get(evt["event"], 0) + 1
    summary_parts = [f"{k}={v}" for k, v in counts_by_type.items()]
    print(f"[INFO] Behavioral anomalies: {', '.join(summary_parts)}")
print(
    "[INFO] Tracking quality — duration: {:.2f}s, distance: {:.2f} m, avg speed: {:.3f} m/s, max speed: {:.3f} m/s".format(
        quality_metrics["total_duration"],
        quality_metrics["total_distance"],
        quality_metrics["avg_speed"],
        quality_metrics["max_speed"],
    )
)

augmented_payload = build_augmented_payload(raw_data, trajectory_points, events)
with open(TRAJECTORY_JSON, "w") as f:
    json.dump(augmented_payload, f, indent=2)
print(f"[INFO] trajectory.json updated with {len(events)} events (including teleports/anomalies)")

speeds = []

for i in range(len(x) - 1):
    dx = x[i + 1] - x[i]
    dy = y[i + 1] - y[i]
    dt = t[i + 1] - t[i]

    if dt > MIN_DT:
        v = math.hypot(dx, dy) / dt
    else:
        v = 0.0

    speeds.append(v)

speeds = np.array(speeds)


ARROW_LEN = 0.1  # длина вектора (м)
HEAD_LEN = 0.04  # длина наконечника (м)
HEAD_W = 0.03  # ширина наконечника (м)


xmin, xmax = min(x), max(x)
ymin, ymax = min(y), max(y)

padding = 0.05 * max(xmax - xmin, ymax - ymin)

x_range = [xmin - padding, xmax + padding]
y_range = [ymin - padding, ymax + padding]

t_min = min(t)
t_max = max(t)
t_span = (t_max - t_min) if t_max != t_min else 1.0
t_axis_max = t_max if t_max > t_min else t_min + t_span



if args.html:
    fig_html = make_subplots(
        rows=2,
        cols=1,
        row_heights=[0.7, 0.3],
        vertical_spacing=0.08,
        subplot_titles=("Robot trajectory", "Timeline"),
    )

    fig_html.add_trace(
        go.Scatter(
            x=x,
            y=y,
            mode="lines+markers",
            name="trajectory",
            marker=dict(size=6),
            customdata=list(zip(t, frame)),
            hovertemplate=(
                "t = %{customdata[0]:.2f} s<br>"
                "x = %{x:.3f} m<br>"
                "y = %{y:.3f} m<br>"
                "frame = %{customdata[1]}<extra></extra>"
            ),
        ),
        row=1,
        col=1,
    )

    if start_events:
        hover_text = []
        for evt in start_events:
            interval = interval_by_start.get(evt["t"])
            if interval:
                hover_text.append(
                    "STOP_START<br>"
                    f"start: {interval['start']['t']:.2f}s<br>"
                    f"end: {interval['end']['t']:.2f}s<br>"
                    f"duration: {interval['duration']:.2f}s"
                )
            else:
                hover_text.append(f"STOP_START<br>t: {evt['t']:.2f}s")

        fig_html.add_trace(
            go.Scatter(
                x=[e["x"] for e in start_events],
                y=[e["y"] for e in start_events],
                mode="markers",
                marker=dict(size=10, color="red", symbol="circle"),
                name="STOP_START",
                text=hover_text,
                hoverinfo="text",
            ),
            row=1,
            col=1,
        )

    if end_events:
        hover_text = []
        for evt in end_events:
            interval = interval_by_end.get(evt["t"])
            if interval:
                hover_text.append(
                    "STOP_END<br>"
                    f"start: {interval['start']['t']:.2f}s<br>"
                    f"end: {interval['end']['t']:.2f}s<br>"
                    f"duration: {interval['duration']:.2f}s"
                )
            else:
                hover_text.append(f"STOP_END<br>t: {evt['t']:.2f}s")

        fig_html.add_trace(
            go.Scatter(
                x=[e["x"] for e in end_events],
                y=[e["y"] for e in end_events],
                mode="markers",
                marker=dict(size=10, color="green", symbol="square"),
                name="STOP_END",
                text=hover_text,
                hoverinfo="text",
            ),
            row=1,
            col=1,
        )

    if teleports:
        add_plotly_events(fig_html, teleports, "TELEPORT", 1, 1)

    for evt_type, evt_list in anomalies_by_type.items():
        add_plotly_events(fig_html, evt_list, evt_type, 1, 1)

    timeline = timeline_segments if timeline_segments else [
        {"state": "MOVING", "start": t_min, "end": t_axis_max, "duration": t_span}
    ]
    for seg in timeline:
        color = "red" if seg["state"] == "STOPPED" else "green"
        fig_html.add_trace(
            go.Scatter(
                x=[seg["start"], seg["end"]],
                y=[0, 0],
                mode="lines",
                line=dict(color=color, width=10),
                hovertemplate=(
                    f"{seg['state']}<br>"
                    "start: %{x[0]:.2f}s<br>"
                    "end: %{x[1]:.2f}s<br>"
                    f"duration: {seg['duration']:.2f}s<extra></extra>"
                ),
                showlegend=False,
            ),
            row=2,
            col=1,
        )

    fig_html.update_layout(
        title="Robot trajectory",
        xaxis_title="X (m)",
        yaxis_title="Y (m)",
        xaxis=dict(range=x_range, fixedrange=True),
        yaxis=dict(range=y_range, scaleanchor="x", fixedrange=True),
        xaxis2=dict(title="Time (s)", range=[t_min, t_axis_max], fixedrange=True),
        yaxis2=dict(visible=False, showgrid=False),
        height=900,
    )

    html_path = os.path.join(LOG_DIR, "trajectory_interactive.html")
    fig_html.write_html(html_path)
    print(f"[INFO] Interactive HTML saved to {html_path}")
    if not args.report and not args.all:
        webbrowser.open(f"file://{html_path}")


if args.html_anim:
    frames = []

    for i in range(len(x)):
        x0, y0 = x[i], y[i]
        th = theta[i]
        state_color = "red" if is_stopped_at(t[i], stop_intervals) else "green"

        # Конец линии (чуть не до самой вершины, чтобы не выглядело странно)
        x1 = x0 + (ARROW_LEN - HEAD_LEN) * math.cos(th)
        y1 = y0 + (ARROW_LEN - HEAD_LEN) * math.sin(th)

        # Вершина наконечника (tip) будет на полной длине ARROW_LEN
        tip_x = x0 + ARROW_LEN * math.cos(th)
        tip_y = y0 + ARROW_LEN * math.sin(th)

        tri_x, tri_y = arrow_head_triangle(tip_x, tip_y, th, HEAD_LEN, HEAD_W)

        frames.append(
            go.Frame(
                data=[
                    # trace 1: точка робота
                    go.Scatter(
                        x=[x0],
                        y=[y0],
                        mode="markers",
                        marker=dict(size=12, color=state_color),
                        customdata=[[t[i], frame[i]]],
                        hovertemplate=(
                            "t = %{customdata[0]:.2f} s<br>"
                            "x = %{x:.3f} m<br>"
                            "y = %{y:.3f} m<br>"
                            "frame = %{customdata[1]}<extra></extra>"
                        ),
                    ),
                    # trace 2: линия стрелки
                    go.Scatter(
                        x=[x0, x1],
                        y=[y0, y1],
                        mode="lines",
                        line=dict(color=state_color, width=3),
                        hoverinfo="skip",
                    ),
                    # trace 3: наконечник как заполненный треугольник
                    go.Scatter(
                        x=tri_x,
                        y=tri_y,
                        mode="lines",
                        fill="toself",
                        line=dict(color=state_color, width=1),
                        fillcolor=state_color,
                        hoverinfo="skip",
                    ),
                ],
                traces=[1, 2, 3],
                name=str(i),
            )
        )

    initial_color = "red" if is_stopped_at(t[0], stop_intervals) else "green"
    fig_html = go.Figure(
        data=[
            # 0 — траектория
            go.Scatter(
                x=x,
                y=y,
                mode="lines",
                line=dict(color="gray", width=2),
                name="trajectory",
            ),
            # 1 — точка робота
            go.Scatter(
                x=[x[0]],
                y=[y[0]],
                mode="markers",
                marker=dict(size=12, color=initial_color),
                name="robot",
            ),
            # 2 — линия ориентации
            go.Scatter(
                x=[x[0], x[0] + ARROW_LEN * math.cos(theta[0])],
                y=[y[0], y[0] + ARROW_LEN * math.sin(theta[0])],
                mode="lines",
                line=dict(color=initial_color, width=3),
                name="orientation",
            ),
            # 3 — наконечник
            go.Scatter(
                x=[x[0] + ARROW_LEN * math.cos(theta[0])],
                y=[y[0] + ARROW_LEN * math.sin(theta[0])],
                mode="markers",
                marker=dict(
                    size=12,
                    symbol="triangle-up",
                    angle=math.degrees(theta[0]),
                    angleref="up",
                    color=initial_color,
                ),
                hoverinfo="skip",
                name="arrow_head",
            ),
        ],
        frames=frames,
    )

    if start_events:
        start_hover = []
        for evt in start_events:
            interval = interval_by_start.get(evt["t"])
            if interval:
                start_hover.append(
                    "STOP_START<br>"
                    f"start: {interval['start']['t']:.2f}s<br>"
                    f"end: {interval['end']['t']:.2f}s<br>"
                    f"duration: {interval['duration']:.2f}s"
                )
            else:
                start_hover.append(f"STOP_START<br>t: {evt['t']:.2f}s")
        fig_html.add_trace(
            go.Scatter(
                x=[e["x"] for e in start_events],
                y=[e["y"] for e in start_events],
                mode="markers",
                marker=dict(size=10, color="red", symbol="circle"),
                name="STOP_START",
                hoverinfo="text",
                text=start_hover,
            )
        )
    if end_events:
        end_hover = []
        for evt in end_events:
            interval = interval_by_end.get(evt["t"])
            if interval:
                end_hover.append(
                    "STOP_END<br>"
                    f"start: {interval['start']['t']:.2f}s<br>"
                    f"end: {interval['end']['t']:.2f}s<br>"
                    f"duration: {interval['duration']:.2f}s"
                )
            else:
                end_hover.append(f"STOP_END<br>t: {evt['t']:.2f}s")
        fig_html.add_trace(
            go.Scatter(
                x=[e["x"] for e in end_events],
                y=[e["y"] for e in end_events],
                mode="markers",
                marker=dict(size=10, color="green", symbol="square"),
                name="STOP_END",
                hoverinfo="text",
                text=end_hover,
            )
        )

    fig_html.update_layout(
        title="Robot trajectory (animated with orientation)",
        xaxis=dict(range=x_range, fixedrange=True),
        yaxis=dict(range=y_range, scaleanchor="x", fixedrange=True),
        updatemenus=[
            {
                "type": "buttons",
                "showactive": False,
                "buttons": [
                    {
                        "label": "▶ Play",
                        "method": "animate",
                        "args": [
                            None,
                            {
                                "frame": {"duration": 50, "redraw": True},
                                "fromcurrent": True,
                            },
                        ],
                    },
                    {
                        "label": "⏸ Pause",
                        "method": "animate",
                        "args": [
                            [None],
                            {"frame": {"duration": 0}, "mode": "immediate"},
                        ],
                    },
                ],
            }
        ],
    )

    html_path = os.path.join(LOG_DIR, "trajectory_animated.html")
    fig_html.write_html(html_path)
    print(f"[INFO] Animated HTML saved to {html_path}")
    if not args.report and not args.all:
        webbrowser.open(f"file://{html_path}")

if args.heatmap:
    fig_hm, ax_hm = plt.subplots()

    # количество бинов — можно подбирать
    bins = 60

    heatmap, xedges, yedges = np.histogram2d(x, y, bins=bins)

    # транспонируем, т.к. imshow ожидает [row, col]
    heatmap = heatmap.T

    im = ax_hm.imshow(
        heatmap,
        origin="lower",
        extent=[xedges[0], xedges[-1], yedges[0], yedges[-1]],
        cmap="hot",
        aspect="equal",
    )

    ax_hm.set_title("Trajectory heatmap (visit density)")
    ax_hm.set_xlabel("X (m)")
    ax_hm.set_ylabel("Y (m)")
    ax_hm.grid(False)

    cbar = plt.colorbar(im, ax=ax_hm)
    cbar.set_label("Visits count")

    # поверх heatmap — траектория
    ax_hm.plot(x, y, color="cyan", linewidth=1.5, label="trajectory")
    ax_hm.legend()

    # сохранение
    output_hm = os.path.join(LOG_DIR, "trajectory_heatmap.png")
    fig_hm.savefig(output_hm, dpi=200, bbox_inches="tight")
    print(f"[INFO] Heatmap saved to {output_hm}")
    if not args.report and not args.all:
        plt.show()
    
if args.time_color:
    fig_tc, ax_tc = plt.subplots()

    # Формируем сегменты линии
    pts_arr = np.array([x, y]).T.reshape(-1, 1, 2)
    segments = np.concatenate([pts_arr[:-1], pts_arr[1:]], axis=1)

    lc = LineCollection(
        segments,
        cmap="viridis",
        norm=plt.Normalize(t_min, t_max),
    )
    lc.set_array(np.array(t[:-1]))
    lc.set_linewidth(3)

    ax_tc.add_collection(lc)

    # Начало и конец
    ax_tc.scatter(x[0], y[0], c="green", s=80, label="start")
    ax_tc.scatter(x[-1], y[-1], c="red", s=80, label="end")
    plot_stop_annotations(ax_tc, stop_events, stop_intervals)
    plot_event_markers(ax_tc, teleports, "TELEPORT")
    for evt_type, evt_list in anomalies_by_type.items():
        plot_event_markers(ax_tc, evt_list, evt_type)

    ax_tc.set_title("Trajectory colored by time")
    ax_tc.set_xlabel("X (m)")
    ax_tc.set_ylabel("Y (m)")
    ax_tc.axis("equal")
    ax_tc.grid(True)

    # Цветовая шкала
    cbar = plt.colorbar(lc, ax=ax_tc)
    cbar.set_label("Time (s)")

    # Ограничим область как в других режимах
    ax_tc.set_xlim(x_range)
    ax_tc.set_ylim(y_range)

    # Сохранение
    output_tc = os.path.join(LOG_DIR, "trajectory_time_colored.png")
    fig_tc.savefig(output_tc, dpi=200, bbox_inches="tight")
    print(f"[INFO] Time-colored trajectory saved to {output_tc}")
    if not args.report and not args.all:
        plt.show()
    
if args.speed_color:
    fig_sc, ax_sc = plt.subplots()

    pts_arr = np.array([x, y]).T.reshape(-1, 1, 2)
    segments = np.concatenate([pts_arr[:-1], pts_arr[1:]], axis=1)

    lc = LineCollection(
        segments,
        cmap="plasma",
        norm=plt.Normalize(speeds.min(), speeds.max()),
    )
    lc.set_array(speeds)
    lc.set_linewidth(3)

    ax_sc.add_collection(lc)

    # начало / конец
    ax_sc.scatter(x[0], y[0], c="green", s=80, label="start")
    ax_sc.scatter(x[-1], y[-1], c="red", s=80, label="end")
    plot_stop_annotations(ax_sc, stop_events, stop_intervals)
    plot_event_markers(ax_sc, teleports, "TELEPORT")
    for evt_type, evt_list in anomalies_by_type.items():
        plot_event_markers(ax_sc, evt_list, evt_type)

    ax_sc.set_title("Trajectory colored by speed")
    ax_sc.set_xlabel("X (m)")
    ax_sc.set_ylabel("Y (m)")
    ax_sc.axis("equal")
    ax_sc.grid(True)

    # цветовая шкала
    cbar = plt.colorbar(lc, ax=ax_sc)
    cbar.set_label("Speed (m/s)")

    ax_sc.set_xlim(x_range)
    ax_sc.set_ylim(y_range)

    output_sc = os.path.join(LOG_DIR, "trajectory_speed_colored.png")
    fig_sc.savefig(output_sc, dpi=200, bbox_inches="tight")
    print(f"[INFO] Speed-colored trajectory saved to {output_sc}")
    if not args.report and not args.all:
        plt.show()


# --- график ---
fig, ax = plt.subplots()
line, = ax.plot(x, y, "-b", label="trajectory")
scatter = ax.scatter(x, y, s=15, c="blue")

ax.scatter(x[0], y[0], c="green", s=80, label="start")
ax.scatter(x[-1], y[-1], c="red", s=80, label="end")
plot_stop_annotations(ax, stop_events, stop_intervals)
plot_event_markers(ax, teleports, "TELEPORT")
for evt_type, evt_list in anomalies_by_type.items():
    plot_event_markers(ax, evt_list, evt_type)

ax.set_title("Robot trajectory")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.axis("equal")
ax.grid(True)
ax.legend()

# --- интерактивные подсказки ---
cursor = mplcursors.cursor(scatter, hover=True)


@cursor.connect("add")
def on_add(sel):
    i = sel.index
    sel.annotation.set(
        text=(
            f"t = {t[i]:.2f} s\n"
            f"x = {x[i]:.3f} m\n"
            f"y = {y[i]:.3f} m\n"
            f"frame = {frame[i]}"
        )
    )
    sel.annotation.get_bbox_patch().set(alpha=0.9)
    
def build_report_html(points, stop_intervals, timeline_segments, teleports, anomalies, quality_metrics, output_path):
    """Compose a single-page HTML report with trajectory, quality, and anomaly analysis."""
    xs = [p.get("x", 0.0) for p in points]
    ys = [p.get("y", 0.0) for p in points]
    ts = [p.get("t", 0.0) for p in points]

    n = len(xs)
    t0, t1 = (min(ts), max(ts)) if ts else (0.0, 0.0)
    duration = quality_metrics.get("total_duration", (t1 - t0) if ts else 0.0)
    stop_count = quality_metrics.get("stop_count", len(stop_intervals))
    total_stopped = sum(iv["duration"] for iv in stop_intervals)
    teleport_count = quality_metrics.get("teleport_count", len(teleports))
    anomaly_count = len(anomalies)
    v_avg = quality_metrics.get("avg_speed", 0.0)
    v_max = quality_metrics.get("max_speed", 0.0)
    total_distance = quality_metrics.get("total_distance", 0.0)
    moving_time = quality_metrics.get("moving_time", 0.0)
    percent_stopped = quality_metrics.get("percent_stopped", 0.0)
    percent_moving = quality_metrics.get("percent_moving", 0.0)

    def rel(p):
        return os.path.basename(p)

    def render_timeline():
        if not timeline_segments:
            return '<div class="small">No stop intervals detected.</div>'
        span = duration if duration > 0 else 1.0
        segs = []
        for seg in timeline_segments:
            width_pct = max(1.5, 100.0 * seg["duration"] / span)
            css_class = "stop" if seg["state"] == "STOPPED" else "move"
            segs.append(f'<div class="seg {css_class}" style="width:{width_pct:.2f}%"></div>')
        return '<div class="timeline">' + "".join(segs) + "</div>"

    rows_html = "".join(
        "<tr>"
        f"<td>{idx}</td>"
        f"<td>{iv['start']['t']:.2f}</td>"
        f"<td>{iv['end']['t']:.2f}</td>"
        f"<td>{iv['duration']:.2f}</td>"
        f"<td>{iv['start']['x']:.3f}</td>"
        f"<td>{iv['start']['y']:.3f}</td>"
        "</tr>"
        for idx, iv in enumerate(stop_intervals, 1)
    )
    if not rows_html:
        rows_html = '<tr><td colspan="6">No stop intervals detected.</td></tr>'

    teleport_rows = "".join(
        "<tr>"
        f"<td>{idx}</td>"
        f"<td>{tp.get('t', 0.0):.2f}</td>"
        f"<td>{tp.get('x', 0.0):.3f}</td>"
        f"<td>{tp.get('y', 0.0):.3f}</td>"
        f"<td>{tp.get('speed', 0.0) if tp.get('speed') is not None else 0.0:.2f}</td>"
        f"<td>{tp.get('frame', '-')}</td>"
        "</tr>"
        for idx, tp in enumerate(teleports, 1)
    )
    if not teleport_rows:
        teleport_rows = '<tr><td colspan="6">No teleports detected.</td></tr>'

    anomaly_rows = "".join(
        "<tr>"
        f"<td>{evt.get('event', '')}</td>"
        f"<td>{evt.get('t', 0.0):.2f}</td>"
        f"<td>{evt.get('x', 0.0):.3f}</td>"
        f"<td>{evt.get('y', 0.0):.3f}</td>"
        f"<td>{format_anomaly_description(evt)}</td>"
        "</tr>"
        for evt in anomalies
    )
    if not anomaly_rows:
        anomaly_rows = '<tr><td colspan="5">No anomalies detected.</td></tr>'

    quality_cards = f"""
      <div class="stats">
        <div class="item"><strong>{duration:.2f}s</strong><span>Total duration</span></div>
        <div class="item"><strong>{total_distance:.2f} m</strong><span>Total distance</span></div>
        <div class="item"><strong>{v_avg:.3f} m/s</strong><span>Avg speed</span></div>
        <div class="item"><strong>{v_max:.3f} m/s</strong><span>Max speed</span></div>
        <div class="item"><strong>{total_stopped:.2f}s</strong><span>Stopped time</span></div>
        <div class="item"><strong>{moving_time:.2f}s</strong><span>Moving time</span></div>
        <div class="item"><strong>{stop_count}</strong><span>Stops</span></div>
        <div class="item"><strong>{teleport_count}</strong><span>Teleports</span></div>
        <div class="item"><strong>{percent_stopped:.1f}%</strong><span>Percent stopped</span></div>
        <div class="item"><strong>{percent_moving:.1f}%</strong><span>Percent moving</span></div>
      </div>
    """

    html = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>Robot Trajectory Report</title>
  <style>
    body {{
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Arial, sans-serif;
      margin: 24px;
      color: #111;
    }}
    h1 {{ margin: 0 0 8px; }}
    .meta {{ color: #444; margin-bottom: 20px; }}
    .grid {{
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 16px;
      align-items: start;
    }}
    .card {{
      border: 1px solid #ddd;
      border-radius: 10px;
      padding: 14px;
      background: #fff;
    }}
    table {{
      width: 100%;
      border-collapse: collapse;
      margin-top: 10px;
      font-size: 13px;
    }}
    th, td {{
      border: 1px solid #eee;
      padding: 8px;
      text-align: center;
    }}
    th {{
      background: #fafafa;
    }}
    .card h2 {{
      font-size: 16px;
      margin: 0 0 10px;
    }}
    iframe {{
      width: 100%;
      height: 520px;
      border: 0;
      border-radius: 8px;
      background: #fafafa;
    }}
    img {{
      width: 100%;
      border-radius: 8px;
      border: 1px solid #eee;
    }}
    .timeline {{
      display: flex;
      height: 20px;
      border-radius: 8px;
      overflow: hidden;
      border: 1px solid #e0e0e0;
      margin-bottom: 8px;
    }}
    .timeline .seg {{
      height: 100%;
    }}
    .timeline .move {{
      background: #9ad18b;
    }}
    .timeline .stop {{
      background: #e57373;
    }}
    .wide {{
      grid-column: 1 / -1;
    }}
    .small {{
      font-size: 13px;
      color: #555;
      line-height: 1.45;
    }}
    .stats {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(140px, 1fr));
      gap: 12px;
      margin: 8px 0 6px;
    }}
    .stats .item {{
      padding: 10px 12px;
      border-radius: 8px;
      background: #f7f7f7;
      border: 1px solid #eee;
    }}
    .stats .item strong {{
      display: block;
      font-size: 18px;
    }}
    .stats .item span {{
      color: #555;
      font-size: 13px;
    }}
    code {{
      background: #f6f6f6;
      padding: 2px 6px;
      border-radius: 6px;
    }}
  </style>
</head>
<body>
  <h1>Robot Trajectory Report</h1>
  <div class="meta">
    Points: <b>{n}</b> &nbsp;|&nbsp;
    Duration: <b>{duration:.2f}s</b> &nbsp;|&nbsp;
    Avg speed: <b>{v_avg:.3f} m/s</b> &nbsp;|&nbsp;
    Max speed: <b>{v_max:.3f} m/s</b> &nbsp;|&nbsp;
    Teleports: <b>{teleport_count}</b> &nbsp;|&nbsp;
    Anomalies: <b>{anomaly_count}</b>
  </div>

  <div class="grid">
    <div class="card">
      <h2>Interactive trajectory (hover)</h2>
      <iframe src="{rel(HTML_INTERACTIVE)}"></iframe>
      <div class="small">Source: <code>{rel(HTML_INTERACTIVE)}</code></div>
    </div>

    <div class="card">
      <h2>Animated pose (orientation)</h2>
      <iframe src="{rel(HTML_ANIMATED)}"></iframe>
      <div class="small">Source: <code>{rel(HTML_ANIMATED)}</code></div>
    </div>

    <div class="card">
      <h2>Heatmap (visit density)</h2>
      <img src="{rel(PNG_HEATMAP)}" alt="heatmap"/>
      <div class="small">Source: <code>{rel(PNG_HEATMAP)}</code></div>
    </div>

    <div class="card">
      <h2>Colored by time</h2>
      <img src="{rel(PNG_TIME)}" alt="time-colored"/>
      <div class="small">Source: <code>{rel(PNG_TIME)}</code></div>
    </div>

    <div class="card">
      <h2>Colored by speed</h2>
      <img src="{rel(PNG_SPEED)}" alt="speed-colored"/>
      <div class="small">Source: <code>{rel(PNG_SPEED)}</code></div>
    </div>

    <div class="card">
      <h2>Base PNG (matplotlib)</h2>
      <img src="{rel(PNG_MAIN)}" alt="base-png"/>
      <div class="small">Source: <code>{rel(PNG_MAIN)}</code></div>
    </div>

    <div class="card wide">
      <h2>Tracking Quality Summary</h2>
      {quality_cards}
    </div>

    <div class="card wide">
      <h2>Stop Analysis</h2>
      <div class="stats">
        <div class="item"><strong>{stop_count}</strong><span>Stops</span></div>
        <div class="item"><strong>{total_stopped:.2f}s</strong><span>Total stopped time</span></div>
        <div class="item"><strong>{duration:.2f}s</strong><span>Trajectory duration</span></div>
      </div>
      <table>
        <thead>
          <tr>
            <th>#</th><th>Start (s)</th><th>End (s)</th><th>Duration (s)</th><th>X</th><th>Y</th>
          </tr>
        </thead>
        <tbody>
          {rows_html}
        </tbody>
      </table>
    </div>

    <div class="card wide">
      <h2>Teleports</h2>
      <table>
        <thead>
          <tr>
            <th>#</th><th>Time (s)</th><th>X</th><th>Y</th><th>Speed (m/s)</th><th>Frame</th>
          </tr>
        </thead>
        <tbody>
          {teleport_rows}
        </tbody>
      </table>
    </div>

    <div class="card wide">
      <h2>Behavioral Anomalies</h2>
      <table>
        <thead>
          <tr><th>Type</th><th>Time (s)</th><th>X</th><th>Y</th><th>Description</th></tr>
        </thead>
        <tbody>
          {anomaly_rows}
        </tbody>
      </table>
    </div>

    <div class="card wide">
      <h2>Timeline</h2>
      {render_timeline()}
      <div class="small">Green = MOVING, Red = STOPPED</div>
    </div>
  </div>
</body>
</html>
"""
    with open(output_path, "w", encoding="utf-8") as f:
        f.write(html)



if args.save:
    fig.savefig(OUTPUT_PNG, dpi=200, bbox_inches="tight")
    print(f"[INFO] Trajectory plot saved to {OUTPUT_PNG}")

# Показываем matplotlib только если был хотя бы один matplotlib-режим
if (not args.html and not args.html_anim and not args.heatmap \
	and not args.time_color and not args.speed_color and \
	not args.report):
    plt.show()


if args.report:
    build_report_html(
        trajectory_points,
        stop_intervals,
        timeline_segments,
        teleports,
        anomaly_events,
        quality_metrics,
        REPORT_HTML,
    )
    print(f"[INFO] Report saved to {REPORT_HTML}")


if args.report and not getattr(args, "no_open", False):
    webbrowser.open(f"file://{REPORT_HTML}")
