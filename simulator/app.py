#!/usr/bin/env python3
"""Tkinter GUI for MataTurboPi simulator."""

from __future__ import annotations

import math
import tkinter as tk
from tkinter import ttk

try:
    from simulator.core.sim_state import load_state, reset_state, save_state, state_path
except Exception:
    from core.sim_state import load_state, reset_state, save_state, state_path

CANVAS_W = 900
CANVAS_H = 620
SCALE = 130.0


class SimApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("MataTurboPi Simulator")

        frame = ttk.Frame(root, padding=8)
        frame.pack(fill="both", expand=True)

        top = ttk.Frame(frame)
        top.pack(fill="x")

        self.status = ttk.Label(top, text="")
        self.status.pack(side="left", padx=(0, 16))

        ttk.Button(top, text="Reset Robot", command=self.reset_robot).pack(side="left")

        self.canvas = tk.Canvas(frame, width=CANVAS_W, height=CANVAS_H, bg="#f4f6f8", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True, pady=(8, 0))

        self.poll_ms = 60
        self.draw()

    def reset_robot(self):
        st = reset_state()
        st["trace"] = []
        save_state(st)

    def world_to_canvas(self, x: float, y: float):
        cx = CANVAS_W * 0.5 + x * SCALE
        cy = CANVAS_H * 0.5 - y * SCALE
        return cx, cy

    def draw_grid(self):
        self.canvas.delete("grid")
        step = int(SCALE)
        for x in range(0, CANVAS_W, step):
            self.canvas.create_line(x, 0, x, CANVAS_H, fill="#dde3ea", tags="grid")
        for y in range(0, CANVAS_H, step):
            self.canvas.create_line(0, y, CANVAS_W, y, fill="#dde3ea", tags="grid")
        self.canvas.create_line(CANVAS_W * 0.5, 0, CANVAS_W * 0.5, CANVAS_H, fill="#c4ccd5", width=2, tags="grid")
        self.canvas.create_line(0, CANVAS_H * 0.5, CANVAS_W, CANVAS_H * 0.5, fill="#c4ccd5", width=2, tags="grid")

    def draw_trace(self, trace):
        self.canvas.delete("trace")
        if len(trace) < 2:
            return
        pts = []
        for item in trace:
            x, y = self.world_to_canvas(float(item.get("x", 0.0)), float(item.get("y", 0.0)))
            pts.extend([x, y])
        self.canvas.create_line(*pts, fill="#85a3c2", width=2, smooth=True, tags="trace")

    def draw_robot(self, st):
        self.canvas.delete("robot")
        r = st["robot"]
        x = float(r["x"])
        y = float(r["y"])
        heading = math.radians(float(r["heading_deg"]))

        cx, cy = self.world_to_canvas(x, y)
        body_w = 0.30 * SCALE
        body_h = 0.22 * SCALE

        local = [(-body_w / 2, -body_h / 2), (body_w / 2, -body_h / 2), (body_w / 2, body_h / 2), (-body_w / 2, body_h / 2)]
        pts = []
        for px, py in local:
            wx = px * math.cos(heading) - py * math.sin(heading)
            wy = px * math.sin(heading) + py * math.cos(heading)
            pts.extend([cx + wx, cy + wy])

        self.canvas.create_polygon(*pts, fill="#ffcc66", outline="#b9892d", width=2, tags="robot")

        fx = cx + math.cos(heading) * body_w * 0.6
        fy = cy + math.sin(heading) * body_w * 0.6
        self.canvas.create_line(cx, cy, fx, fy, fill="#2c3e50", width=3, arrow="last", tags="robot")

        cam = st["camera"]
        yaw_norm = (int(cam.get("yaw", 1500)) - 1500) / 500.0
        pitch_norm = (int(cam.get("pitch", 1500)) - 1500) / 500.0
        gx = cx + yaw_norm * 26.0
        gy = cy - body_h * 0.20 + pitch_norm * 18.0
        self.canvas.create_oval(gx - 10, gy - 10, gx + 10, gy + 10, fill="#2d3436", outline="#111", tags="robot")

        eyes = st["eyes"]
        l = eyes.get("left", [0, 0, 0])
        rr = eyes.get("right", [0, 0, 0])
        lc = "#%02x%02x%02x" % (int(l[0]), int(l[1]), int(l[2]))
        rc = "#%02x%02x%02x" % (int(rr[0]), int(rr[1]), int(rr[2]))

        ex = cx + math.cos(heading) * body_w * 0.40
        ey = cy + math.sin(heading) * body_w * 0.40
        off = 16
        self.canvas.create_oval(ex - 7, ey - off - 7, ex + 7, ey - off + 7, fill=lc, outline="#222", tags="robot")
        self.canvas.create_oval(ex - 7, ey + off - 7, ex + 7, ey + off + 7, fill=rc, outline="#222", tags="robot")

    def draw(self):
        st = load_state()
        self.draw_grid()
        self.draw_trace(st.get("trace", []))
        self.draw_robot(st)
        robot = st["robot"]
        self.status.config(
            text=(
                f"state: {state_path()} | x={robot['x']:.2f} y={robot['y']:.2f} "
                f"heading={robot['heading_deg']:.1f} | last={st.get('last_command', '')}"
            )
        )
        self.root.after(self.poll_ms, self.draw)


def main():
    root = tk.Tk()
    SimApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
