import plotly.graph_objects as go
import webbrowser
import sys
import json
import os
import argparse
import matplotlib.pyplot as plt
import mplcursors
import math


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
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))
LOG_DIR = os.path.join(PROJECT_ROOT, "tracker_logs")
TRAJECTORY_JSON = os.path.join(LOG_DIR, "trajectory.json")
OUTPUT_PNG = os.path.join(LOG_DIR, "trajectory_interactive.png")


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

args = parser.parse_args()


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

ARROW_LEN = 0.1  # длина вектора (м)
HEAD_LEN = 0.04  # длина наконечника (м)
HEAD_W = 0.03  # ширина наконечника (м)


xmin, xmax = min(x), max(x)
ymin, ymax = min(y), max(y)

padding = 0.05 * max(xmax - xmin, ymax - ymin)

x_range = [xmin - padding, xmax + padding]
y_range = [ymin - padding, ymax + padding]


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
    webbrowser.open(f"file://{html_path}")


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


if args.save:
    fig.savefig(OUTPUT_PNG, dpi=200, bbox_inches="tight")
    print(f"[INFO] Trajectory plot saved to {OUTPUT_PNG}")

if not (args.html or args.html_anim):
    plt.show()
