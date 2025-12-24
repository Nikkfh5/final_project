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
    Split raw trajectory JSON into trajectory points and stop events.
    Events with type/event STOP_START/STOP_END are normalized to a common shape.
    """
    points = []
    events = []

    events_raw = []
    iterable = []
    if isinstance(data, list):
        iterable = data
    elif isinstance(data, dict):
        events_raw = data.get("events", [])
        iterable = data.get("points", [])
        if all(k in data for k in ["times", "x", "y", "theta"]):
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
        if not isinstance(item, dict):
            continue
        evt_type = item.get("event") or item.get("type")
        if evt_type in ("STOP_START", "STOP_END"):
            events.append(
                {
                    "event": evt_type,
                    "t": float(item.get("t", 0.0)),
                    "x": float(item.get("x", 0.0)),
                    "y": float(item.get("y", 0.0)),
                }
            )
            continue
        points.append(item)

    for item in events_raw:
        if not isinstance(item, dict):
            continue
        evt_type = item.get("event") or item.get("type")
        if evt_type in ("STOP_START", "STOP_END"):
            events.append(
                {
                    "event": evt_type,
                    "t": float(item.get("t", 0.0)),
                    "x": float(item.get("x", 0.0)),
                    "y": float(item.get("y", 0.0)),
                }
            )

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

points, stop_events = split_points_and_events(raw_data)
trajectory_points = points
if not points:
    raise ValueError("No trajectory points found in trajectory.json")

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

print(f"[INFO] Loaded {len(points)} trajectory points from {TRAJECTORY_JSON}")
if stop_intervals:
    total_stopped = sum(iv["duration"] for iv in stop_intervals)
    print(f"[INFO] Detected {len(stop_intervals)} stop intervals (stopped {total_stopped:.2f}s)")

speeds = []

for i in range(len(x) - 1):
    dx = x[i + 1] - x[i]
    dy = y[i + 1] - y[i]
    dt = t[i + 1] - t[i]

    if dt > 1e-6:
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
    
def build_report_html(points, stop_intervals, timeline_segments, output_path):
    """Compose a single-page HTML report with trajectory and stop analysis."""
    xs = [p.get("x", 0.0) for p in points]
    ys = [p.get("y", 0.0) for p in points]
    ts = [p.get("t", 0.0) for p in points]
    thetas = [p.get("theta", 0.0) for p in points]

    n = len(xs)
    t0, t1 = (min(ts), max(ts)) if ts else (0.0, 0.0)
    duration = (t1 - t0) if ts else 0.0
    stop_count = len(stop_intervals)
    total_stopped = sum(iv["duration"] for iv in stop_intervals)

    speeds_local = []
    for i in range(n - 1):
        dt = ts[i + 1] - ts[i]
        if dt > 1e-6:
            speeds_local.append(math.hypot(xs[i + 1] - xs[i], ys[i + 1] - ys[i]) / dt)
    v_avg = float(np.mean(speeds_local)) if speeds_local else 0.0
    v_max = float(np.max(speeds_local)) if speeds_local else 0.0

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
    Max speed: <b>{v_max:.3f} m/s</b>
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
      <h2>Timeline</h2>
      {render_timeline()}
      <div class="small">Green = MOVING, Red = STOPPED</div>
    </div>

    <div class="card wide">
      <h2>Anomalies (planned)</h2>
      <div class="small">
        Not implemented yet. Planned detectors:
        <ul>
          <li>Stops (speed below threshold for N seconds)</li>
          <li>Teleports (position jump above threshold between frames)</li>
          <li>Marker swap / orientation flip (theta discontinuity)</li>
          <li>Tracking loss intervals (gaps, interpolation usage)</li>
        </ul>
      </div>
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
    build_report_html(trajectory_points, stop_intervals, timeline_segments, REPORT_HTML)
    print(f"[INFO] Report saved to {REPORT_HTML}")


if args.report and not getattr(args, "no_open", False):
    webbrowser.open(f"file://{REPORT_HTML}")
