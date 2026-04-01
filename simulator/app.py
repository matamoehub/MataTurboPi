#!/usr/bin/env python3
"""Tkinter GUI for MataTurboPi simulator."""

from __future__ import annotations

import math
import time
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
        self._syncing_controls = False
        self._anim_t0 = time.monotonic()

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
            height=200,
            bg="#f8fafc",
            highlightthickness=1,
            highlightbackground="#d2dae2",
        )
        self.cam_model.pack(fill="x")

        controls_card = ttk.LabelFrame(self.panel, text="Manual Controls", padding=(8, 8))
        controls_card.pack(fill="x", pady=(10, 10))

        ttk.Label(controls_card, text="Camera Yaw").pack(anchor="w")
        self.cam_yaw_var = tk.IntVar(value=1500)
        self.cam_yaw_scale = ttk.Scale(
            controls_card,
            from_=1000,
            to=2000,
            variable=self.cam_yaw_var,
            command=self._on_camera_slider,
        )
        self.cam_yaw_scale.pack(fill="x")

        ttk.Label(controls_card, text="Camera Pitch").pack(anchor="w", pady=(6, 0))
        self.cam_pitch_var = tk.IntVar(value=1500)
        self.cam_pitch_scale = ttk.Scale(
            controls_card,
            from_=1000,
            to=2000,
            variable=self.cam_pitch_var,
            command=self._on_camera_slider,
        )
        self.cam_pitch_scale.pack(fill="x")

        ttk.Label(controls_card, text="Eyes R / G / B").pack(anchor="w", pady=(8, 0))
        self.eye_r_var = tk.IntVar(value=0)
        self.eye_g_var = tk.IntVar(value=0)
        self.eye_b_var = tk.IntVar(value=0)
        for label, var in (("R", self.eye_r_var), ("G", self.eye_g_var), ("B", self.eye_b_var)):
            row = ttk.Frame(controls_card)
            row.pack(fill="x", pady=(2, 0))
            ttk.Label(row, text=label, width=2).pack(side="left")
            ttk.Scale(row, from_=0, to=255, variable=var, command=self._on_eye_slider).pack(side="left", fill="x", expand=True)

        sensors_card = ttk.LabelFrame(self.panel, text="Sensors (Upcoming)", padding=(8, 8))
        sensors_card.pack(fill="x")
        self.course_label = ttk.Label(sensors_card, text="Course: default")
        self.course_label.pack(anchor="w")
        self.sonar_label = ttk.Label(sensors_card, text="Sonar: -")
        self.sonar_label.pack(anchor="w", pady=(2, 0))
        self.collision_label = ttk.Label(sensors_card, text="Collision: none")
        self.collision_label.pack(anchor="w", pady=(2, 0))
        self.last_cmd_label = ttk.Label(self.panel, text="Last command: -")
        self.last_cmd_label.pack(anchor="w", pady=(10, 0))

    def _on_camera_slider(self, _value):
        if self._syncing_controls:
            return
        st = load_state()
        st["camera"]["yaw"] = int(float(self.cam_yaw_var.get()))
        st["camera"]["pitch"] = int(float(self.cam_pitch_var.get()))
        st["last_command"] = "panel_camera"
        save_state(st)

    def _on_eye_slider(self, _value):
        if self._syncing_controls:
            return
        r = int(float(self.eye_r_var.get()))
        g = int(float(self.eye_g_var.get()))
        b = int(float(self.eye_b_var.get()))
        st = load_state()
        st["eyes"]["left"] = [r, g, b]
        st["eyes"]["right"] = [r, g, b]
        st["last_command"] = "panel_eyes"
        save_state(st)

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

    def draw_obstacles(self, st):
        self.canvas.delete("obstacle")
        course = st.get("course", {})
        for obs in course.get("obstacles", []):
            x = float(obs.get("x", 0.0))
            y = float(obs.get("y", 0.0))
            r = float(obs.get("radius_m", 0.04)) * SCALE
            px, py = self.world_to_canvas(x, y)
            color_name = str(obs.get("color_name", obs.get("color", "cup"))).strip().lower()
            palette = {
                "red": ("#d94a4a", "#7d1f1f", "#f6d7d7"),
                "green": ("#42a35a", "#1f5a2d", "#d9f1df"),
                "blue": ("#3f7dd9", "#23497a", "#d9e6f8"),
            }
            outer_fill, outer_outline, inner_fill = palette.get(color_name, ("#c86a3a", "#7a3f1e", "#f3d7c3"))
            self.canvas.create_oval(
                px - r,
                py - r,
                px + r,
                py + r,
                fill=outer_fill,
                outline=outer_outline,
                width=2,
                tags="obstacle",
            )
            self.canvas.create_oval(
                px - r * 0.45,
                py - r * 0.45,
                px + r * 0.45,
                py + r * 0.45,
                fill=inner_fill,
                outline="",
                tags="obstacle",
            )
            self.canvas.create_text(px, py + r + 12, text=str(obs.get("id", "cup")), fill="#374151", tags="obstacle")

    def _local_to_canvas(self, cx: float, cy: float, heading: float, lx: float, ly: float):
        wx = lx * math.cos(heading) - ly * math.sin(heading)
        wy = lx * math.sin(heading) + ly * math.cos(heading)
        return cx + wx, cy - wy

    def _local_poly(self, cx: float, cy: float, heading: float, points):
        out = []
        for lx, ly in points:
            px, py = self._local_to_canvas(cx, cy, heading, lx, ly)
            out.extend([px, py])
        return out

    def _draw_wheel(self, cx: float, cy: float, heading: float, lx: float, ly: float):
        px, py = self._local_to_canvas(cx, cy, heading, lx, ly)
        r = 18
        self.canvas.create_oval(px - r, py - r, px + r, py + r, fill="#f4c62e", outline="#b08913", width=2, tags="robot")
        self.canvas.create_oval(px - 8, py - 8, px + 8, py + 8, fill="#2a2f33", outline="#111417", tags="robot")
        # Roller hint lines for mecanum look.
        for ang in (heading + math.radians(35), heading - math.radians(35)):
            dx = math.cos(ang) * 13
            dy = math.sin(ang) * 13
            self.canvas.create_line(px - dx, py + dy, px + dx, py - dy, fill="#22282d", width=2, tags="robot")

    def draw_robot(self, st):
        self.canvas.delete("robot")
        r = st["robot"]
        x = float(r["x"])
        y = float(r["y"])
        heading = math.radians(float(r["heading_deg"]))

        cx, cy = self.world_to_canvas(x, y)
        # Main black body: 2:1 rectangle (length:width) as requested.
        body_w = 0.34 * SCALE
        body_l = body_w * 2.0

        # Wheels
        wheel_dx = body_w * 0.70
        wheel_dy = body_l * 0.42
        self._draw_wheel(cx, cy, heading, -wheel_dx, +wheel_dy)
        self._draw_wheel(cx, cy, heading, +wheel_dx, +wheel_dy)
        self._draw_wheel(cx, cy, heading, -wheel_dx, -wheel_dy)
        self._draw_wheel(cx, cy, heading, +wheel_dx, -wheel_dy)

        # Main black rectangular chassis.
        base_poly = [
            (-body_w * 0.50, -body_l * 0.50),
            (+body_w * 0.50, -body_l * 0.50),
            (+body_w * 0.50, +body_l * 0.50),
            (-body_w * 0.50, +body_l * 0.50),
        ]
        self.canvas.create_polygon(
            *self._local_poly(cx, cy, heading, base_poly),
            fill="#1d2329",
            outline="#0f1317",
            width=2,
            tags="robot",
        )

        # Upper deck inset.
        deck_poly = [
            (-body_w * 0.38, -body_l * 0.18),
            (+body_w * 0.38, -body_l * 0.18),
            (+body_w * 0.38, +body_l * 0.24),
            (-body_w * 0.38, +body_l * 0.24),
        ]
        self.canvas.create_polygon(
            *self._local_poly(cx, cy, heading, deck_poly),
            fill="#2a3138",
            outline="#11161b",
            width=2,
            tags="robot",
        )

        # Front bumper plate.
        bumper_poly = [
            (-body_w * 0.68, +body_l * 0.40),
            (+body_w * 0.68, +body_l * 0.40),
            (+body_w * 0.68, +body_l * 0.56),
            (-body_w * 0.68, +body_l * 0.56),
        ]
        self.canvas.create_polygon(
            *self._local_poly(cx, cy, heading, bumper_poly),
            fill="#20262d",
            outline="#0e1216",
            width=2,
            tags="robot",
        )

        cam = st["camera"]
        yaw_norm = (int(cam.get("yaw", 1500)) - 1500) / 500.0
        pitch_norm = (int(cam.get("pitch", 1500)) - 1500) / 500.0
        # Camera mast sits above the front section.
        mast_x, mast_y = self._local_to_canvas(cx, cy, heading, 0.0, body_l * 0.34)
        self.canvas.create_rectangle(
            mast_x - 9, mast_y - 24, mast_x + 9, mast_y + 6, fill="#2a3138", outline="#101418", tags="robot"
        )

        # Camera head and moving lens.
        # Push camera further forward so it reads like a "nose".
        head_x, head_y = self._local_to_canvas(cx, cy, heading, yaw_norm * 16.0, body_l * 0.58 + pitch_norm * 10.0)
        self.canvas.create_rectangle(
            head_x - 14, head_y - 12, head_x + 14, head_y + 12, fill="#252c33", outline="#0f1317", tags="robot"
        )
        lens_x, lens_y = self._local_to_canvas(cx, cy, heading, yaw_norm * 16.0, body_l * 0.64 + pitch_norm * 10.0)
        self.canvas.create_line(head_x, head_y, lens_x, lens_y, fill="#2f3942", width=2, tags="robot")
        self.canvas.create_oval(lens_x - 6, lens_y - 6, lens_x + 6, lens_y + 6, fill="#74b9ff", outline="#1b4f72", tags="robot")

        eyes = st["eyes"]
        l = eyes.get("left", [0, 0, 0])
        rr = eyes.get("right", [0, 0, 0])
        lc = "#%02x%02x%02x" % (int(l[0]), int(l[1]), int(l[2]))
        rc = "#%02x%02x%02x" % (int(rr[0]), int(rr[1]), int(rr[2]))

        # Sonar "eyes" at front.
        lcx, lcy = self._local_to_canvas(cx, cy, heading, -body_w * 0.20, body_l * 0.24)
        rcx, rcy = self._local_to_canvas(cx, cy, heading, +body_w * 0.20, body_l * 0.24)
        for ex, ey, col in ((lcx, lcy, lc), (rcx, rcy, rc)):
            self.canvas.create_oval(ex - 9, ey - 9, ex + 9, ey + 9, fill=col, outline="#cfd8dc", width=2, tags="robot")
            self.canvas.create_oval(ex - 3, ey - 3, ex + 3, ey + 3, fill="#101820", outline="", tags="robot")

        # Forward arrow.
        fx0, fy0 = self._local_to_canvas(cx, cy, heading, 0.0, body_l * 0.05)
        fx1, fy1 = self._local_to_canvas(cx, cy, heading, 0.0, body_l * 0.68)
        self.canvas.create_line(fx0, fy0, fx1, fy1, fill="#2c3e50", width=3, arrow="last", tags="robot")

    def _rgb_to_hex(self, rgb):
        return "#%02x%02x%02x" % (int(rgb[0]), int(rgb[1]), int(rgb[2]))

    def _mix_rgb(self, rgb, boost: float):
        f = max(0.0, min(1.0, float(boost)))
        r = int(rgb[0] + (255 - int(rgb[0])) * f)
        g = int(rgb[1] + (255 - int(rgb[1])) * f)
        b = int(rgb[2] + (255 - int(rgb[2])) * f)
        return r, g, b

    def _proj3(self, x: float, y: float, z: float, ox: float, oy: float):
        sx = ox + x - z * 0.58
        sy = oy - y - z * 0.36
        return sx, sy

    def _draw_camera_model(
        self,
        yaw_offset: float,
        pitch_offset: float,
        left_rgb,
        right_rgb,
        motion_level: float,
        phase: float,
    ):
        c = self.cam_model
        c.delete("all")
        left_hex = self._rgb_to_hex(left_rgb)
        right_hex = self._rgb_to_hex(right_rgb)
        # Local helper for concentric sonar rings.
        def sonar(x: float, y: float, lit_hex: str):
            c.create_oval(x - 18, y - 18, x + 18, y + 18, fill="#dce3ea", outline="#8f9aa5", width=1.5)
            c.create_oval(x - 14, y - 14, x + 14, y + 14, fill="#1f2429", outline="#3f474f")
            c.create_oval(x - 10, y - 10, x + 10, y + 10, fill="#2b3138", outline="#555f69")
            c.create_oval(x - 7, y - 7, x + 7, y + 7, fill=lit_hex, outline="")
            c.create_oval(x - 3, y - 3, x + 3, y + 3, fill="#0c1014", outline="")

        # Wheels (front-on/straight like reference photo).
        def wheel(x: float, y: float, r: float = 18):
            c.create_oval(x - r, y - r, x + r, y + r, fill="#f2c21d", outline="#b58a08", width=2)
            c.create_oval(x - 11, y - 11, x + 11, y + 11, fill="#1b2025", outline="#13171b")
            ang = phase * (1.2 + motion_level * 3.2)
            dx = math.cos(ang) * 11
            dy = math.sin(ang) * 11
            c.create_line(x - dx, y - dy, x + dx, y + dy, fill="#2d353d", width=2)

        # Front two wheels only (to match your reference framing).
        wheel(36, 142, 20)
        wheel(204, 142, 20)

        # Front bumper / lower plate.
        c.create_polygon(40, 128, 200, 128, 184, 162, 56, 162, fill="#2a323a", outline="#14191e", width=2)
        c.create_oval(52, 145, 58, 151, fill="#2e3740", outline="")
        c.create_oval(182, 145, 188, 151, fill="#2e3740", outline="")
        pulse = 0.5 + 0.5 * math.sin(phase * 2.0)
        blue_l = "#%02x%02x%02x" % (20, 70 + int(120 * pulse), 255)
        blue_r = "#%02x%02x%02x" % (20, 70 + int(120 * (1.0 - pulse * 0.4)), 255)
        c.create_oval(68, 150, 76, 158, fill=blue_l, outline="#1b3ca1")
        c.create_oval(164, 150, 172, 158, fill=blue_r, outline="#1b3ca1")

        # Middle black body.
        c.create_rectangle(66, 88, 174, 132, fill="#1f252b", outline="#0f1317", width=2)
        c.create_rectangle(74, 74, 166, 96, fill="#232a31", outline="#11161a")

        # Sonar faceplate with two eyes.
        c.create_rectangle(78, 96, 162, 132, fill="#141a20", outline="#0e1216")
        left_glow = self._mix_rgb(left_rgb, 0.10 + 0.22 * (0.5 + 0.5 * math.sin(phase * 2.6)))
        right_glow = self._mix_rgb(right_rgb, 0.10 + 0.22 * (0.5 + 0.5 * math.sin(phase * 2.6 + 0.8)))
        sonar(98, 114, self._rgb_to_hex(left_glow))
        sonar(142, 114, self._rgb_to_hex(right_glow))
        for sx in (82, 158):
            c.create_oval(sx - 2.5, 99.5, sx + 2.5, 104.5, fill="#8d98a3", outline="")
            c.create_oval(sx - 2.5, 124.5, sx + 2.5, 129.5, fill="#8d98a3", outline="")

        # Camera mount mast.
        c.create_rectangle(114, 66, 126, 82, fill="#171d23", outline="#0e1216")

        # Pan-tilt camera bracket + board responding to yaw/pitch.
        # Accentuated movement so left/right/up/down is easy to read.
        cam_cx = 120 + yaw_offset * 56.0
        cam_cy = 52 + pitch_offset * 34.0
        c.create_rectangle(cam_cx - 24, cam_cy - 17, cam_cx + 24, cam_cy + 17, fill="#1b2127", outline="#0e1318", width=2)
        c.create_line(cam_cx - 24, cam_cy + 14, cam_cx - 34, cam_cy + 22, fill="#2a323a", width=3)
        c.create_line(cam_cx + 24, cam_cy + 14, cam_cx + 34, cam_cy + 22, fill="#2a323a", width=3)
        c.create_oval(cam_cx - 11, cam_cy - 11, cam_cx + 11, cam_cy + 11, fill="#0f1317", outline="#2f3942", width=2)
        c.create_oval(cam_cx - 4, cam_cy - 4, cam_cx + 4, cam_cy + 4, fill="#7dc3ff", outline="")
        for px in (-20, 20):
            c.create_oval(cam_cx + px - 2.5, cam_cy - 13.5, cam_cx + px + 2.5, cam_cy - 8.5, fill="#949faa", outline="")
            c.create_oval(cam_cx + px - 2.5, cam_cy + 8.5, cam_cx + px + 2.5, cam_cy + 13.5, fill="#949faa", outline="")

        # Direction arrow.
        ax0, ay0 = 208, 24
        c.create_text(ax0, ay0 - 10, text="View Dir", fill="#546e7a", font=("TkDefaultFont", 8))
        c.create_line(ax0, ay0, ax0 + yaw_offset * 34.0, ay0 + pitch_offset * 22.0, arrow="last", width=3, fill="#1976d2")

        c.create_text(12, 187, text="Front view (animated)", fill="#6b7280", anchor="w", font=("TkDefaultFont", 8))

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
        robot = st.get("robot", {})
        motion_level = min(
            1.0,
            (
                abs(float(robot.get("vx", 0.0))) * 2.8
                + abs(float(robot.get("vy", 0.0))) * 2.8
                + abs(float(robot.get("omega_deg_s", 0.0))) / 240.0
            ),
        )
        phase = time.monotonic() - self._anim_t0
        self._draw_camera_model(yaw_offset, pitch_offset, left, right, motion_level, phase)

        self._syncing_controls = True
        try:
            self.cam_yaw_var.set(yaw)
            self.cam_pitch_var.set(pitch)
            self.eye_r_var.set(int(left[0]))
            self.eye_g_var.set(int(left[1]))
            self.eye_b_var.set(int(left[2]))
        finally:
            self._syncing_controls = False

        self.last_cmd_label.config(text=f"Last command: {st.get('last_command', '-')}")
        course = st.get("course", {})
        self.course_label.config(text=f"Course: {course.get('title', course.get('id', 'default'))}")
        sonar = st.get("sonar", {})
        distance_cm = sonar.get("distance_cm")
        if distance_cm is None:
            self.sonar_label.config(text="Sonar: no obstacle in range")
        else:
            self.sonar_label.config(text=f"Sonar: {float(distance_cm):.1f} cm")
        collision = st.get("collision", {})
        if collision.get("active"):
            self.collision_label.config(text=f"Collision: {collision.get('message', 'Collision')}")
        else:
            self.collision_label.config(text="Collision: none")

    def draw(self):
        st = load_state()
        self.draw_grid()
        self.draw_trace(st.get("trace", []))
        self.draw_obstacles(st)
        self.draw_robot(st)
        self.update_side_panel(st)
        robot = st["robot"]
        status_text = (
            f"state: {state_path()} | x={robot['x']:.2f} y={robot['y']:.2f} "
            f"heading={robot['heading_deg']:.1f} | last={st.get('last_command', '')}"
        )
        if st.get("collision", {}).get("active"):
            status_text += " | Collision"
            self.status.config(text=status_text, foreground="#b00020")
        else:
            self.status.config(text=status_text, foreground="")
        self.root.after(self.poll_ms, self.draw)


def main():
    root = tk.Tk()
    SimApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
