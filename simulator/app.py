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
PANEL_W = 280


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

        body = ttk.Frame(frame)
        body.pack(fill="both", expand=True, pady=(8, 0))

        self.canvas = tk.Canvas(body, width=CANVAS_W, height=CANVAS_H, bg="#f4f6f8", highlightthickness=0)
        self.canvas.pack(side="left", fill="both", expand=True)

        self.panel = ttk.Frame(body, width=PANEL_W, padding=(10, 8))
        self.panel.pack(side="right", fill="y")
        self.panel.pack_propagate(False)

        self._build_side_panel()

        self.poll_ms = 60
        self.draw()

    def _build_side_panel(self):
        title = ttk.Label(self.panel, text="Robot Status", font=("TkDefaultFont", 12, "bold"))
        title.pack(anchor="w", pady=(0, 8))

        lights_card = ttk.LabelFrame(self.panel, text="Lights", padding=(8, 8))
        lights_card.pack(fill="x", pady=(0, 10))

        row_l = ttk.Frame(lights_card)
        row_l.pack(fill="x", pady=(0, 6))
        ttk.Label(row_l, text="Left").pack(side="left")
        self.left_color_chip = tk.Canvas(row_l, width=20, height=20, highlightthickness=1, highlightbackground="#999")
        self.left_color_chip.pack(side="right")

        row_r = ttk.Frame(lights_card)
        row_r.pack(fill="x")
        ttk.Label(row_r, text="Right").pack(side="left")
        self.right_color_chip = tk.Canvas(row_r, width=20, height=20, highlightthickness=1, highlightbackground="#999")
        self.right_color_chip.pack(side="right")

        self.left_rgb_label = ttk.Label(lights_card, text="Left RGB: (0, 0, 0)")
        self.left_rgb_label.pack(anchor="w", pady=(8, 0))
        self.right_rgb_label = ttk.Label(lights_card, text="Right RGB: (0, 0, 0)")
        self.right_rgb_label.pack(anchor="w", pady=(2, 0))

        cam_card = ttk.LabelFrame(self.panel, text="Camera Position", padding=(8, 8))
        cam_card.pack(fill="x", pady=(0, 10))
        self.cam_yaw_label = ttk.Label(cam_card, text="Yaw: 1500")
        self.cam_yaw_label.pack(anchor="w")
        self.cam_pitch_label = ttk.Label(cam_card, text="Pitch: 1500")
        self.cam_pitch_label.pack(anchor="w", pady=(2, 0))
        self.cam_yaw_norm_label = ttk.Label(cam_card, text="Yaw offset: +0.00")
        self.cam_yaw_norm_label.pack(anchor="w", pady=(2, 0))
        self.cam_pitch_norm_label = ttk.Label(cam_card, text="Pitch offset: +0.00")
        self.cam_pitch_norm_label.pack(anchor="w", pady=(2, 0))
        self.cam_axis_label = ttk.Label(cam_card, text="Direction: Center")
        self.cam_axis_label.pack(anchor="w", pady=(2, 6))
        self.cam_model = tk.Canvas(
            cam_card,
            width=240,
            height=165,
            bg="#f8fafc",
            highlightthickness=1,
            highlightbackground="#d2dae2",
        )
        self.cam_model.pack(fill="x")

        sensors_card = ttk.LabelFrame(self.panel, text="Sensors (Upcoming)", padding=(8, 8))
        sensors_card.pack(fill="x")
        ttk.Label(sensors_card, text="Placeholder for line / distance / vision inputs").pack(anchor="w")
        self.last_cmd_label = ttk.Label(self.panel, text="Last command: -")
        self.last_cmd_label.pack(anchor="w", pady=(10, 0))

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

    def _rgb_to_hex(self, rgb):
        return "#%02x%02x%02x" % (int(rgb[0]), int(rgb[1]), int(rgb[2]))

    def _proj3(self, x: float, y: float, z: float, ox: float, oy: float):
        sx = ox + x - z * 0.58
        sy = oy - y - z * 0.36
        return sx, sy

    def _draw_camera_model(self, yaw_offset: float, pitch_offset: float, left_hex: str, right_hex: str):
        c = self.cam_model
        c.delete("all")
        ox = 122.0
        oy = 124.0

        # Platform cube (pseudo-3D robot top).
        front = [(-48, -8, 0), (48, -8, 0), (48, 20, 0), (-48, 20, 0)]
        top = [(-48, 20, 0), (48, 20, 0), (48, 20, 22), (-48, 20, 22)]
        side = [(48, -8, 0), (48, 20, 0), (48, 20, 22), (48, -8, 22)]
        for poly, fill, outline in (
            (front, "#d4b06c", "#9a7e47"),
            (top, "#f2d292", "#9a7e47"),
            (side, "#c99f53", "#8e703d"),
        ):
            pts = []
            for x, y, z in poly:
                sx, sy = self._proj3(x, y, z, ox, oy)
                pts.extend([sx, sy])
            c.create_polygon(*pts, fill=fill, outline=outline, width=1.5)

        # Eyes on robot front.
        lcx, lcy = self._proj3(-22, 10, 2, ox, oy)
        rcx, rcy = self._proj3(22, 10, 2, ox, oy)
        c.create_oval(lcx - 7, lcy - 7, lcx + 7, lcy + 7, fill=left_hex, outline="#2d3436")
        c.create_oval(rcx - 7, rcy - 7, rcx + 7, rcy + 7, fill=right_hex, outline="#2d3436")

        # Gimbal base.
        bx, by = self._proj3(0, 26, 10, ox, oy)
        c.create_oval(bx - 13, by - 9, bx + 13, by + 9, fill="#506d85", outline="#2f4152")

        # Camera lens position responds to yaw/pitch.
        lx = bx + yaw_offset * 34.0
        ly = by + pitch_offset * 24.0
        lens_r = 10
        c.create_line(bx, by, lx, ly, fill="#2f3e46", width=3)
        c.create_oval(lx - lens_r, ly - lens_r, lx + lens_r, ly + lens_r, fill="#273238", outline="#10161b")
        c.create_oval(lx - 4, ly - 4, lx + 4, ly + 4, fill="#8ac6ff", outline="")

        # Direction indicator in panel space.
        ax0, ay0 = 198, 28
        c.create_text(ax0, ay0 - 10, text="View Dir", fill="#546e7a", font=("TkDefaultFont", 8))
        c.create_line(ax0, ay0, ax0 + yaw_offset * 26.0, ay0 + pitch_offset * 18.0, arrow="last", width=2, fill="#1976d2")

        c.create_text(36, 148, text="Robot body", fill="#6b7280", anchor="w", font=("TkDefaultFont", 8))

    def update_side_panel(self, st):
        eyes = st.get("eyes", {})
        left = eyes.get("left", [0, 0, 0])
        right = eyes.get("right", [0, 0, 0])
        left_hex = self._rgb_to_hex(left)
        right_hex = self._rgb_to_hex(right)

        self.left_color_chip.delete("all")
        self.left_color_chip.create_rectangle(0, 0, 22, 22, fill=left_hex, outline=left_hex)
        self.right_color_chip.delete("all")
        self.right_color_chip.create_rectangle(0, 0, 22, 22, fill=right_hex, outline=right_hex)

        self.left_rgb_label.config(text=f"Left RGB: ({int(left[0])}, {int(left[1])}, {int(left[2])})")
        self.right_rgb_label.config(text=f"Right RGB: ({int(right[0])}, {int(right[1])}, {int(right[2])})")

        cam = st.get("camera", {})
        yaw = int(cam.get("yaw", 1500))
        pitch = int(cam.get("pitch", 1500))
        yaw_offset = (yaw - 1500) / 500.0
        pitch_offset = (pitch - 1500) / 500.0
        self.cam_yaw_label.config(text=f"Yaw: {yaw}")
        self.cam_pitch_label.config(text=f"Pitch: {pitch}")
        self.cam_yaw_norm_label.config(text=f"Yaw offset: {yaw_offset:+.2f}")
        self.cam_pitch_norm_label.config(text=f"Pitch offset: {pitch_offset:+.2f}")
        lr = "Right" if yaw_offset > 0.12 else ("Left" if yaw_offset < -0.12 else "Center")
        ud = "Down" if pitch_offset > 0.12 else ("Up" if pitch_offset < -0.12 else "Level")
        self.cam_axis_label.config(text=f"Direction: {lr} / {ud}")
        self._draw_camera_model(yaw_offset, pitch_offset, left_hex, right_hex)

        self.last_cmd_label.config(text=f"Last command: {st.get('last_command', '-')}")

    def draw(self):
        st = load_state()
        self.draw_grid()
        self.draw_trace(st.get("trace", []))
        self.draw_robot(st)
        self.update_side_panel(st)
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
