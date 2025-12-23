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
    data = json.load(f)


x = [p["x"] for p in data]
y = [p["y"] for p in data]
t = [p["t"] for p in data]
theta = [p["theta"] for p in data]
frame = [p["frame"] for p in data]

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
t_norm = [(ti - t_min) / (t_max - t_min) for ti in t]



if args.html:
    fig_html = go.Figure()

    fig_html.add_trace(
        go.Scatter(
            x=x,
            y=y,
            mode="lines+markers",
            marker=dict(size=6),
            customdata=list(zip(t, frame)),
            hovertemplate=(
                "t = %{customdata[0]:.2f} s<br>"
                "x = %{x:.3f} m<br>"
                "y = %{y:.3f} m<br>"
                "frame = %{customdata[1]}<extra></extra>"
            ),
        )
    )

    fig_html.update_layout(
        title="Robot trajectory",
        xaxis_title="X (m)",
        yaxis_title="Y (m)",
        xaxis=dict(range=x_range, fixedrange=True),
        yaxis=dict(range=y_range, scaleanchor="x", fixedrange=True),
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
                        marker=dict(size=12, color="red"),
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
                        line=dict(color="red", width=3),
                        hoverinfo="skip",
                    ),
                    # trace 3: наконечник как заполненный треугольник
                    go.Scatter(
                        x=tri_x,
                        y=tri_y,
                        mode="lines",
                        fill="toself",
                        line=dict(color="red", width=1),
                        fillcolor="red",
                        hoverinfo="skip",
                    ),
                ],
                traces=[1, 2, 3],
                name=str(i),
            )
        )

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
                marker=dict(size=12, color="red"),
                name="robot",
            ),
            # 2 — линия ориентации
            go.Scatter(
                x=[x[0], x[0] + ARROW_LEN * math.cos(theta[0])],
                y=[y[0], y[0] + ARROW_LEN * math.sin(theta[0])],
                mode="lines",
                line=dict(color="red", width=3),
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
                    color="red",
                ),
                hoverinfo="skip",
                name="arrow_head",
            ),
        ],
        frames=frames,
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
    points = np.array([x, y]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

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

    points = np.array([x, y]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

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
points = ax.scatter(x, y, s=15, c="blue")

ax.scatter(x[0], y[0], c="green", s=80, label="start")
ax.scatter(x[-1], y[-1], c="red", s=80, label="end")

ax.set_title("Robot trajectory")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.axis("equal")
ax.grid(True)
ax.legend()

# --- интерактивные подсказки ---
cursor = mplcursors.cursor(points, hover=True)


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
    
    
def build_report_html(data, output_path):
    # basic stats
    xs = [p["x"] for p in data]
    ys = [p["y"] for p in data]
    ts = [p["t"] for p in data]
    thetas = [p.get("theta", 0.0) for p in data]

    n = len(xs)
    t0, t1 = (min(ts), max(ts)) if ts else (0.0, 0.0)
    duration = (t1 - t0) if ts else 0.0

    # speed estimate
    speeds_local = []
    for i in range(n - 1):
        dt = ts[i + 1] - ts[i]
        if dt > 1e-6:
            speeds_local.append(math.hypot(xs[i+1] - xs[i], ys[i+1] - ys[i]) / dt)
    v_avg = float(np.mean(speeds_local)) if speeds_local else 0.0
    v_max = float(np.max(speeds_local)) if speeds_local else 0.0

    # relative links (report in same folder)
    def rel(p): 
        return os.path.basename(p)

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
    .wide {{
      grid-column: 1 / -1;
    }}
    .small {{
      font-size: 13px;
      color: #555;
      line-height: 1.45;
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
    build_report_html(data, REPORT_HTML)
    print(f"[INFO] Report saved to {REPORT_HTML}")


if args.report and not getattr(args, "no_open", False):
    webbrowser.open(f"file://{REPORT_HTML}")
