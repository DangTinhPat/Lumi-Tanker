#!/usr/bin/env python3
"""2-D playback of the v3.3 two-body/LegPlant sim (v3_tier_tests TIER_ANIM).

Each CSV is one run (TR_ANIM=<file> TIER_ANIM=1 v3_tier_tests); every run
gets its own panel, played in sync, with a body-pitch strip chart below.

  python3 tools/tier_sim_view.py old.csv:"Ban cu" new.csv:"Ban moi"

Space: pause, R: restart, +/-: playback speed.
"""
import argparse
import bisect
import csv
import math
import os
import tkinter as tk

SCALE = 950.0          # px per metre
PANEL_H = 330
PLOT_H = 170
WIDTH = 1100
BODY_BOX = [(-0.11, -0.035), (0.11, -0.035), (0.11, 0.065), (-0.11, 0.065)]
WHEEL_R = 0.04
COLORS = ["#d9534f", "#2a7de1", "#3c9d5d", "#9b59b6"]
LIGHT = ["#f6d3d2", "#d2e3f8", "#d5ecdc", "#e8d9ef"]


def load(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    return [{k: float(v) for k, v in r.items()} for r in rows]


class Viewer:
    def __init__(self, root, runs):
        self.root = root
        self.runs = runs
        self.n = min(len(r["rows"]) for r in runs)
        self.dt = runs[0]["rows"][1]["t"] - runs[0]["rows"][0]["t"]
        self.i = 0.0
        self.speed = 0.5
        self.paused = False
        h = PANEL_H * len(runs) + PLOT_H + 30
        self.c = tk.Canvas(root, width=WIDTH, height=h, bg="#fbfbf8", highlightthickness=0)
        self.c.pack()
        self.status = tk.Label(root, anchor="w", font=("DejaVu Sans Mono", 10))
        self.status.pack(fill="x")
        root.bind("<space>", lambda e: self.toggle())
        root.bind("r", lambda e: self.restart())
        root.bind("<plus>", lambda e: self.set_speed(self.speed * 2))
        root.bind("<KP_Add>", lambda e: self.set_speed(self.speed * 2))
        root.bind("<minus>", lambda e: self.set_speed(self.speed / 2))
        root.bind("<KP_Subtract>", lambda e: self.set_speed(self.speed / 2))
        self.plot_top = PANEL_H * len(runs) + 10
        self.plot_lim = max(4.0, min(12.0, math.ceil(max(abs(math.degrees(r['body'])) for run in runs for r in run['rows'][:self.n]))))
        self.tick()

    def toggle(self):
        self.paused = not self.paused

    def restart(self):
        self.i = 0.0

    def set_speed(self, s):
        self.speed = min(max(s, 0.0625), 2.0)

    def to_px(self, x, z, cam_x, top):
        return (WIDTH / 2 + (x - cam_x) * SCALE, top + PANEL_H - 25 - z * SCALE)

    def draw_panel(self, k, run, row):
        c = self.c
        top = k * PANEL_H
        color = COLORS[k % len(COLORS)]
        cam = row["axle_x"]
        c.create_rectangle(0, top, WIDTH, top + PANEL_H, fill="#fbfbf8", outline="#ddd")
        # Floor: the terrain profile when the run has one (<csv>.ground; the
        # profile is under the axle, so draw it one wheel radius below the
        # axle path), else flat; ticks every 5 cm show the motion.
        terrain = run.get("terrain")
        xl, xr = cam - WIDTH / 2 / SCALE, cam + WIDTH / 2 / SCALE
        if terrain:
            xs, hs = terrain
            i0 = max(bisect.bisect_left(xs, xl) - 1, 0)
            i1 = min(bisect.bisect_right(xs, xr) + 1, len(xs))
            pts = []
            for i in range(i0, i1):
                pts.extend(self.to_px(xs[i], hs[i], cam, top))
            if len(pts) >= 4:
                c.create_line(pts, fill="#555", width=2)
                bottom = top + PANEL_H
                poly = [pts[0], bottom] + pts + [pts[-2], bottom]
                c.create_polygon(poly, fill="#e6e1d6", outline="")
                c.create_line(pts, fill="#555", width=2)
        else:
            gy = self.to_px(0, row["ground"], cam, top)[1]
            c.create_line(0, gy, WIDTH, gy, fill="#555", width=2)
        x = math.floor(xl / 0.05) * 0.05
        while x < xr:
            px = self.to_px(x, 0, cam, top)[0]
            gy = self.to_px(0, self.ground_at(run, x, row), cam, top)[1]
            c.create_line(px, gy, px - 8, gy + 8, fill="#aaa")
            x += 0.05
        # Level reference through the hip.
        hx, hz = self.to_px(row["hip_x"], row["hip_z"], cam, top)
        c.create_line(hx - 170, hz, hx + 170, hz, fill="#bbb", dash=(4, 4))
        # Body box: hip origin, pitch = deviation from level (0 = parallel).
        a = row["body"]
        ca, sa = math.cos(a), math.sin(a)
        pts = []
        for lx, lz in BODY_BOX:
            wx = row["hip_x"] + ca * lx + sa * lz
            wz = row["hip_z"] - sa * lx + ca * lz
            pts.extend(self.to_px(wx, wz, cam, top))
        c.create_polygon(pts, fill=run["light"], outline=color, width=3)
        # Wheel, shin, thigh (drawn over the body).
        ax, az = self.to_px(row["axle_x"], row["axle_z"], cam, top)
        r = WHEEL_R * SCALE
        c.create_oval(ax - r, az - r, ax + r, az + r, outline="#333", width=3, fill="#e8e8e8")
        spin = row["axle_x"] / WHEEL_R
        c.create_line(ax, az, ax + r * math.cos(spin), az + r * math.sin(spin), fill="#333", width=2)
        kx, kz = self.to_px(row["knee_x"], row["knee_z"], cam, top)
        c.create_line(ax, az, kx, kz, fill="#444", width=7, capstyle="round")
        c.create_line(kx, kz, hx, hz, fill="#666", width=9, capstyle="round")
        for px, pz in ((hx, hz), (kx, kz), (ax, az)):
            c.create_oval(px - 5, pz - 5, px + 5, pz + 5, fill="#fff", outline="#222", width=2)
        # CoM line (what the wheels balance): axle -> CoM.
        mx, mz = self.to_px(row["com_x"], row["com_z"], cam, top)
        c.create_line(ax, az, mx, mz, fill="#e69500", width=2, dash=(6, 3))
        c.create_oval(mx - 6, mz - 6, mx + 6, mz + 6, fill="#e69500", outline="")
        # Hand press on the body edge (vertical, from above for f < 0).
        if abs(row.get("press", 0.0)) > 0.01:
            lx, lz = row["press_x"], BODY_BOX[2][1]
            px, pz = self.to_px(row["hip_x"] + ca * lx + sa * lz, row["hip_z"] - sa * lx + ca * lz, cam, top)
            d = 1 if row["press"] < 0 else -1
            c.create_line(px, pz - d * 90, px, pz - d * 4, arrow="last", width=6, fill="#c0392b")
            c.create_text(px, pz - d * 104, text=f"ấn {abs(row['press']):.0f} N", fill="#c0392b",
                          font=("DejaVu Sans", 11, "bold"))
        # Push arrow.
        if abs(row["push"]) > 0.01:
            d = 1 if row["push"] > 0 else -1
            c.create_line(hx - d * 140, hz - 10, hx - d * 20, hz - 10, arrow="last", width=5, fill="#c0392b")
            c.create_text(hx - d * 80, hz - 26, text=f"đẩy {row['push']:+.0f} N", fill="#c0392b",
                          font=("DejaVu Sans", 11, "bold"))
        # Labels.
        c.create_text(14, top + 16, anchor="w", text=run["label"], fill=color,
                      font=("DejaVu Sans", 14, "bold"))
        c.create_text(14, top + 42, anchor="w", font=("DejaVu Sans Mono", 11), fill="#222",
                      text=(f"thân lệch {math.degrees(row['body']):+5.1f}°   "
                            f"CoM nghiêng {math.degrees(row['lean']):+5.1f}°   "
                            f"hông lắc {math.degrees(row['hip'] - run['hip0']):+5.1f}°   "
                            f"gối {math.degrees(row['knee'] - run['knee0']):+5.1f}°"))
        c.create_text(14, top + 62, anchor="w", font=("DejaVu Sans Mono", 11), fill="#222",
                      text=f"tốc độ {row['v']:+.2f} m/s (lệnh {row['cmd_v']:+.2f})")

    @staticmethod
    def ground_at(run, x, row):
        terrain = run.get("terrain")
        if not terrain:
            return row["ground"]
        xs, hs = terrain
        i = min(max(bisect.bisect_left(xs, x), 1), len(xs) - 1)
        a = (x - xs[i - 1]) / max(xs[i] - xs[i - 1], 1e-9)
        return hs[i - 1] + a * (hs[i] - hs[i - 1])

    def draw_plot(self, idx):
        c = self.c
        top = self.plot_top
        h = PLOT_H - 20
        left, right = 60, WIDTH - 20
        c.create_rectangle(left, top, right, top + h, outline="#999", fill="#fff")
        lim = self.plot_lim  # degrees
        mid = top + h / 2
        for deg in (-lim, -lim / 2, 0, lim / 2, lim):
            y = mid - deg / lim * h / 2
            c.create_line(left, y, right, y, fill="#eee" if deg else "#bbb")
            c.create_text(left - 6, y, anchor="e", text=f"{deg:+.0f}°", font=("DejaVu Sans", 9))
        c.create_text(left + 6, top + 10, anchor="w", text="thân lệch khỏi ngang (độ)",
                      font=("DejaVu Sans", 10, "bold"))
        tmax = self.runs[0]["rows"][self.n - 1]["t"]
        for k, run in enumerate(self.runs):
            rows = run["rows"]
            pts = []
            step = 4
            for j in range(0, self.n, step):
                x = left + rows[j]["t"] / tmax * (right - left)
                d = max(min(math.degrees(rows[j]["body"]), lim), -lim)
                pts.extend((x, mid - d / lim * h / 2))
            if len(pts) >= 4:
                c.create_line(pts, fill=COLORS[k % len(COLORS)], width=2)
        # Shade drives / pushes from the first run's command.
        rows = self.runs[0]["rows"]
        x = left + rows[idx]["t"] / tmax * (right - left)
        c.create_line(x, top, x, top + h, fill="#000", width=2)
        for j in range(0, self.n, 5):
            if abs(rows[j]["push"]) > 0.01 or abs(rows[j].get("press", 0.0)) > 0.01:
                xx = left + rows[j]["t"] / tmax * (right - left)
                c.create_line(xx, top + h - 6, xx, top + h, fill="#c0392b", width=3)
            elif abs(rows[j]["cmd_v"]) > 0.01:
                xx = left + rows[j]["t"] / tmax * (right - left)
                c.create_line(xx, top + h - 4, xx, top + h, fill="#888", width=2)

    def tick(self):
        if not self.paused:
            self.i += self.speed * 0.033 / self.dt
            if self.i >= self.n:
                self.i = 0.0
        idx = int(self.i)
        self.c.delete("all")
        for k, run in enumerate(self.runs):
            self.draw_panel(k, run, run["rows"][idx])
        self.draw_plot(idx)
        t = self.runs[0]["rows"][idx]["t"]
        self.status.config(text=f" t = {t:5.2f} s   tốc độ phát x{self.speed:g}   "
                                f"[Space] dừng/chạy  [R] lại từ đầu  [+/-] nhanh/chậm"
                                + ("   (đang dừng)" if self.paused else ""))
        self.root.after(33, self.tick)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+", help="file.csv[:label]")
    ap.add_argument("--title", default="TAP v3.3 - mô phỏng 2D")
    args = ap.parse_args()
    runs = []
    for spec in args.runs:
        path, _, label = spec.partition(":")
        rows = load(path)
        # Joint angles relative to standing still at the crouch height
        # (just before the first drive): the legs' own swing.
        ref = next((r for r in rows if abs(r["cmd_v"]) > 1e-3 or abs(r.get("press", 0.0)) > 1e-3
                    or abs(r["push"]) > 1e-3), rows[-1])
        ref = rows[max(rows.index(ref) - 1, 0)]
        k = len(runs) % len(COLORS)
        terrain = None
        if os.path.exists(path + ".ground"):
            g = load(path + ".ground")
            # Profile under the axle -> floor surface one wheel radius lower.
            terrain = ([r["x"] for r in g], [r["h"] for r in g])
        runs.append({"rows": rows, "label": label or path, "hip0": ref["hip"], "knee0": ref["knee"],
                     "light": LIGHT[k], "terrain": terrain})
    root = tk.Tk()
    root.title(args.title)
    Viewer(root, runs)
    root.mainloop()


if __name__ == "__main__":
    main()
