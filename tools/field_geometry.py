#!/usr/bin/env python3
"""BIOBUZZ field geometry (Competition Manual TU09, section 9 figures; field CAD is the official version, +-1 in) and what the
CELL's pentagon opening does to a shot. Numbers and where each one comes from: docs/FIELD.md.

  python -I tools/field_geometry.py window X Y [--goal RED_CLOSE] [--angle 72] [--exit-height 11.5]
        the speeds (as a ratio of the speed that hits the opening's center) that go in from turret position X, Y
  python -I tools/field_geometry.py map [--goal RED_CLOSE] [-o map.png]
        the same over the field: how much faster / slower than a center hit still scores, and the approach angle

Frame: Pedro inches, (0,0) the audience-left corner, x toward blue, y away from the audience, z up from the tiles.
"""
import argparse
import math
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SHOOTER = os.path.join(ROOT, 'TeamCode', 'src', 'main', 'java', 'org', 'firstinspires', 'ftc', 'teamcode', 'Constants', 'ShooterConstants.java')
FIELD = os.path.join(ROOT, 'TeamCode', 'src', 'main', 'java', 'org', 'firstinspires', 'ftc', 'teamcode', 'Constants', 'FieldConstants.java')

G = 386.09
POLLEN_DIAMETER = 2.8
HIVE_TILT_DEG = 30.0                  # Fig 9-10
PIVOT_HEIGHT = 43.95                  # 9.6.1
OPENING_BOTTOM_Z = 53.5               # Fig 9-10
OPENING_TOP_Z = 65.6                  # Fig 9-10
CELL_DEPTH = 12.04                    # Fig 9-9 (text: ~12 in)
# the opening, looking into the CELL: flat bottom 20 in, straight sides to 7.61 in, roof up to a peak 14 in above the bottom (Fig 9-11)
PENTAGON = [(-10.0, 0.0), (10.0, 0.0), (10.0, 7.61), (0.0, 14.0), (-10.0, 7.61)]
AIM_H = 7.61                          # the aim point (FieldConstants goals): top middle of the pentagon's rectangle, under the roof


def field_goals():
    src = open(FIELD, encoding='utf-8').read()
    out = {}
    for side in ('RED', 'BLUE'):
        for cell in ('CLOSE', 'FAR'):
            x = float(re.search(r'%s_GOAL_%s_X\s*=\s*([-0-9.]+)' % (side, cell), src).group(1))
            y = float(re.search(r'%s_GOAL_%s_Y\s*=\s*([-0-9.]+)' % (side, cell), src).group(1))
            out[side + '_' + cell] = (x, y)
    return out


def shot_constants():
    src = open(SHOOTER, encoding='utf-8').read()
    angle = float(re.search(r'SHOT_LAUNCH_ANGLE_DEGREES\s*=\s*([0-9.]+)', src).group(1))
    rise = float(re.search(r'SHOT_RISE_INCHES\s*=\s*([0-9.]+)', src).group(1))
    return angle, rise


class Cell:
    """One CELL tipped up. CLOSE cells (y > 72) open toward +y, FAR ones toward -y; the opening leans back 30 deg, peak toward the pivot."""

    def __init__(self, name, x, y):
        self.name = name
        out = 1.0 if y > 72 else -1.0
        t = math.radians(HIVE_TILT_DEG)
        self.u = (1.0, 0.0, 0.0)                                    # across the opening (field x)
        self.h = (0.0, -out * math.sin(t), math.cos(t))             # up the opening, toward the peak
        self.n = (0.0, out * math.cos(t), math.sin(t))              # out of the opening
        cz = OPENING_BOTTOM_Z + AIM_H * math.cos(t)
        self.center = (x, y, cz)
        self.bottom = (x, y + out * AIM_H * math.sin(t), OPENING_BOTTOM_Z)

    def local(self, p):
        d = [p[k] - self.bottom[k] for k in range(3)]
        return dot(d, self.u), dot(d, self.h), dot(d, self.n)


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def inside(u, h, margin):
    """ball center at (u, h) on the opening: inside the pentagon by at least margin (the ball's radius) from every edge"""
    n = len(PENTAGON)
    for k in range(n):
        x0, y0 = PENTAGON[k]
        x1, y1 = PENTAGON[(k + 1) % n]
        ex, ey = x1 - x0, y1 - y0
        # inward normal of a counterclockwise polygon
        nx, ny = -ey, ex
        length = math.hypot(nx, ny)
        if ((u - x0) * nx + (h - y0) * ny) / length < margin:
            return False
    return True


def crossing(cell, turret, exit_height, angle_deg, speed):
    """where the ball (no drag) first crosses the opening's plane coming in from the front: (u, h, s) or None"""
    cx, cy, _ = cell.center
    dx, dy = cx - turret[0], cy - turret[1]
    reach = math.hypot(dx, dy)
    wx, wy = dx / reach, dy / reach
    a = math.radians(angle_deg)
    vh, vz = speed * math.cos(a), speed * math.sin(a)
    prev = None
    s = 0.0
    step = 0.05
    while s < reach + 40:
        t = s / vh
        p = (turret[0] + wx * s, turret[1] + wy * s, exit_height + vz * t - 0.5 * G * t * t)
        if p[2] < 0:
            return None
        side = cell.local(p)[2]
        if prev is not None and prev > 0 >= side:
            f = prev / (prev - side)
            sp = s - step + f * step
            t = sp / vh
            q = (turret[0] + wx * sp, turret[1] + wy * sp, exit_height + vz * t - 0.5 * G * t * t)
            u, h, _ = cell.local(q)
            return u, h, sp
        prev = side
        s += step
    return None


def center_speed(cell, turret, exit_height, angle_deg):
    cx, cy, cz = cell.center
    s = math.hypot(cx - turret[0], cy - turret[1])
    a = math.radians(angle_deg)
    rise = s * math.tan(a) - (cz - exit_height)
    if rise <= 0:
        return float('nan')
    return math.sqrt(G * s * s / (2 * math.cos(a) ** 2 * rise))


def window(cell, turret, exit_height=None, angle_deg=None, radius=POLLEN_DIAMETER / 2):
    """(lo, hi): ratios of the aim-point-hit speed that still go in (ball clear of every edge). NaN if none.
    Also the approach angle off straight-on (deg) and the center-hit speed (in/s)."""
    if angle_deg is None or exit_height is None:
        a, rise = shot_constants()
        angle_deg = a if angle_deg is None else angle_deg
        exit_height = cell.center[2] - rise if exit_height is None else exit_height
    v0 = center_speed(cell, turret, exit_height, angle_deg)
    cx, cy, _ = cell.center
    approach = math.degrees(math.atan2(abs(cx - turret[0]), abs(cy - turret[1])))
    if not v0 == v0:
        return float('nan'), float('nan'), approach, v0

    def ok(r):
        c = crossing(cell, turret, exit_height, angle_deg, v0 * r)
        return c is not None and inside(c[0], c[1], radius)
    if not ok(1.0):
        return float('nan'), float('nan'), approach, v0

    def edge(direction):
        lo, hi = 1.0, 1.0 + direction * 0.4
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            if ok(mid):
                lo = mid
            else:
                hi = mid
        return lo
    return edge(-1), edge(1), approach, v0


def cells():
    return {k: Cell(k, x, y) for k, (x, y) in field_goals().items()}


def cmd_window(a):
    cell = cells()[a.goal]
    lo, hi, approach, v0 = window(cell, (a.x, a.y), a.exit_height, a.angle)
    d = math.hypot(cell.center[0] - a.x, cell.center[1] - a.y)
    print('%s from (%.1f, %.1f): %.1f in away, %.0f deg off straight-on, center hit at %.0f in/s' % (a.goal, a.x, a.y, d, approach, v0))
    print('goes in from %.1f%% slow to %.1f%% fast' % (100 * (1 - lo), 100 * (hi - 1)))
    for r in (lo, 1.0, hi):
        c = crossing(cell, (a.x, a.y), a.exit_height or cell.center[2] - shot_constants()[1], a.angle or shot_constants()[0], v0 * r)
        if c:
            print('  x%.3f crosses the opening %.1f in across, %.1f in up (peak 14, shoulders 7.61)' % (r, c[0], c[1]))


def cmd_map(a):
    from PIL import Image, ImageDraw, ImageFont
    cell = cells()[a.goal]
    out = 1 if cell.center[1] > 72 else -1
    scale = 6
    im = Image.new('RGB', (144 * scale, 144 * scale + 40), 'white')
    d = ImageDraw.Draw(im)
    f = ImageFont.load_default(size=13)
    worst = 0.08
    for gx in range(2, 144, 4):
        for gy in range(2, 144, 4):
            if (gy - 72) * out < 8:
                continue
            lo, hi, approach, _ = window(cell, (gx, gy))
            if lo != lo:
                col = (120, 120, 120)
            else:
                # green = room long, red = little room long
                room = max(0.0, min(1.0, (hi - 1) / worst))
                col = (int(255 * (1 - room)), int(200 * room), 40)
            x0, y0 = gx * scale - 2 * scale, (144 - gy) * scale - 2 * scale
            d.rectangle((x0, y0, x0 + 4 * scale - 1, y0 + 4 * scale - 1), fill=col)
            if lo == lo and gx % 12 == 2 and gy % 12 == 2:
                d.text((x0 + 1, y0 + 2), '+%.0f' % (100 * (hi - 1)), fill='white', font=f)
                d.text((x0 + 1, y0 + 12), '-%.0f' % (100 * (1 - lo)), fill='white', font=f)
    cx, cy, _ = cell.center
    d.rectangle((cx * scale - 10 * scale, (144 - cy) * scale - 3, cx * scale + 10 * scale, (144 - cy) * scale + 3), fill='black')
    d.text((6, 144 * scale + 4), '%s: %% faster (top) / slower (bottom) than a center hit that still goes in. Red = little room long. Grey = no way in.' % a.goal, fill='black', font=f)
    d.text((6, 144 * scale + 20), 'audience at the bottom; launch %.0f deg; no drag' % shot_constants()[0], fill='black', font=f)
    im.save(a.o)
    print(a.o)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('window')
    s.add_argument('x', type=float)
    s.add_argument('y', type=float)
    s.add_argument('--goal', default='RED_CLOSE')
    s.add_argument('--angle', type=float)
    s.add_argument('--exit-height', type=float)
    s.set_defaults(f=cmd_window)
    s = sub.add_parser('map')
    s.add_argument('--goal', default='RED_CLOSE')
    s.add_argument('-o', default='cell_window_map.png')
    s.set_defaults(f=cmd_map)
    a = p.parse_args()
    a.f(a)


if __name__ == '__main__':
    main()
