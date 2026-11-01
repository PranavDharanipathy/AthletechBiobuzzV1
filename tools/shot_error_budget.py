#!/usr/bin/env python3
"""Shot error budget (docs/PHYSICS_ERROR_BUDGET.md). Standard library only; run: python3 tools/shot_error_budget.py

Error budget for the POLLEN shooter: sensitivity of the opening-plane crossing to each input.

Geometry (DEV_NOTES 10-31 CAD, tools/pollen_shot_envelope.py, docs/SHOOTER_DESIGN.md):
  exit point relative to turret axis (in): x = 2.57 - 2.82 sin(th), z = 9.30 + 2.82 cos(th)
  target opening centre z = 59.55 in, opening plane 60 deg from horizontal facing the robot
  (normal 30 deg above horizontal), 14 in tall (in-plane), 20 in wide. Ball r = 1.4 in.
Model: gravity + quadratic drag + Magnus lift (constant Cd, Cl), RK4.
"""
import math

IN = 0.0254
G = 9.80665
RHO = 1.2
R_BALL = 1.4 * IN
AREA = math.pi * R_BALL ** 2
ZT = 59.55 * IN
S3 = math.sqrt(3) / 2

def exit_point(th):
    return (2.57 - 2.82 * math.sin(th)) * IN, (9.30 + 2.82 * math.cos(th)) * IN

def fly(d, th, v, m=0.030, cd=0.5, cl=0.0, dh=0.0, plane=True):
    """Return (s, incidence_deg, tof, descent_deg) where s = in-plane height of the crossing (m)."""
    kd = 0.5 * RHO * cd * AREA / m
    kl = 0.5 * RHO * cl * AREA / m
    x, z = exit_point(th)
    z += dh
    vx, vz = v * math.cos(th), v * math.sin(th)
    def sd(x, z):  # signed distance past the plane (positive = beyond)
        return (x - d) * S3 - (z - ZT) * 0.5 if plane else x - d
    def acc(vx, vz):
        s = math.hypot(vx, vz)
        return -kd * s * vx - kl * s * vz, -G - kd * s * vz + kl * s * vx
    t, h = 0.0, 0.002
    while t < 3:
        st = (x, z, vx, vz)
        def f(s):
            ax, az = acc(s[2], s[3]); return (s[2], s[3], ax, az)
        k1 = f(st)
        k2 = f(tuple(st[i] + h / 2 * k1[i] for i in range(4)))
        k3 = f(tuple(st[i] + h / 2 * k2[i] for i in range(4)))
        k4 = f(tuple(st[i] + h * k3[i] for i in range(4)))
        n = tuple(st[i] + h / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) for i in range(4))
        if sd(x, z) <= 0 < sd(n[0], n[1]):
            a = -sd(x, z) / (sd(n[0], n[1]) - sd(x, z))
            cx, cz, cvx, cvz = (st[i] + a * (n[i] - st[i]) for i in range(4))
            s = (cx - d) * 0.5 + (cz - ZT) * S3  # in-plane "up" coordinate
            sp = math.hypot(cvx, cvz)
            inc = math.degrees(math.acos(max(-1, min(1, (cvx * S3 - cvz * 0.5) / sp))))
            return s, inc, t + a * h, math.degrees(math.atan2(-cvz, cvx))
        if n[1] < 0 or n[2] <= 0:
            return None
        x, z, vx, vz = n
        t += h
    return None

def solve_v(d, th, **kw):
    lo, hi = 2.0, 20.0
    if fly(d, th, hi, **kw) is None or fly(d, th, hi, **kw)[0] < 0:
        return None
    for _ in range(50):
        mid = (lo + hi) / 2
        r = fly(d, th, mid, **kw)
        if r is None or r[0] < 0: lo = mid
        else: hi = mid
    return (lo + hi) / 2

def margin_in(inc):
    # half the opening seen along the plane, less the ball radius stretched by the incidence
    c = math.cos(math.radians(inc))
    return 7.0 - 1.4 / c if c > 0.2 else -99

def table(m=0.030, cd=0.5, cl=0.0):
    print(f"\n=== m={m*1000:.0f} g  Cd={cd}  Cl={cl}  (turret-axis horizontal distance) ===")
    print(" d_m  hood  v_m/s  rpm@eta.5  tof_s  inc  desc  marg_in | s per: 1%v  1deg  0.5in_h  1in_d | Cd+.15 Cl+.2 m-30%")
    for d in (1.5, 2.0, 2.5, 3.0, 3.5):
        best = None
        for k in range(0, 47):
            th = math.radians(31 + 0.5 * k)
            v = solve_v(d, th, m=m, cd=cd, cl=cl)
            if v is None: continue
            s, inc, tof, desc = fly(d, th, v, m=m, cd=cd, cl=cl)
            mg = margin_in(inc)
            if best is None or mg > best[0]:
                best = (mg, th, v, inc, tof, desc)
        if best is None:
            print(f" {d:.1f}  unreachable in 31-54 deg"); continue
        mg, th, v, inc, tof, desc = best
        def ds(**kw):
            base = dict(m=m, cd=cd, cl=cl); base.update(kw.pop('model', {}))
            vv = kw.pop('v', v); tt = kw.pop('th', th); dd = kw.pop('d', d)
            r = fly(dd, tt, vv, **base, **kw)
            return float('nan') if r is None else r[0] / IN
        rpm = 60 * v / (0.5 * math.pi * 0.072)
        print(f" {d:.1f}  {math.degrees(th):4.1f}  {v:5.2f}  {rpm:6.0f}    {tof:4.2f}  {inc:4.0f}  {desc:4.0f}  {mg:5.2f}  |"
              f"  {ds(v=v*1.01):5.2f} {ds(th=th+math.radians(1)):5.2f}  {ds(dh=0.5*IN):5.2f}  {ds(d=d-IN):5.2f} |"
              f"  {ds(model=dict(cd=cd+.15)):5.2f}  {ds(model=dict(cl=cl+.2)):5.2f}  {ds(model=dict(m=m*0.7)):5.2f}")

def extras():
    # vacuum (PollenPhysics) vs drag model: same point target crossing a vertical plane at the opening centre
    for d, th in ((2.0, 54), (3.0, 53), (3.5, 50)):
        t = math.radians(th)
        x0, z0 = exit_point(t)
        dx = d - x0
        vvac = math.sqrt(G*dx*dx/(2*math.cos(t)**2*(dx*math.tan(t)-(ZT-z0))))
        for m in (0.020, 0.030, 0.050):
            vd = solve_v(d, t, m=m, plane=False)
            s = fly(d, t, vvac, m=m, plane=False)
            print(f"d={d} th={th} m={m*1000:.0f}g  v_vac={vvac:.3f}  v_drag={vd:.3f}  ({100*(vd/vvac-1):+.1f}%)  vacuum speed lands {s[0]/IN if s else float('nan'):+.1f} in (vertical)")
    # flywheel energy per shot and recovery bound
    J = 2e-4; rpm = 3700; w = rpm*2*math.pi/60
    for m in (0.02, 0.03, 0.05):
        v = 0.5*w*0.036
        for k, name in ((1+2/3, 'hollow'), (1.4, 'solid')):
            E = k*0.5*m*v*v; Ew = 0.5*J*w*w
            dw = 1 - math.sqrt(1-E/Ew)
            print(f"m={m*1000:.0f}g {name}: ball KE+spin {E:.2f} J of wheel {Ew:.1f} J -> wheel drops {100*dw:.1f}%")
    tau_stall, wfree = 0.144, 6000*2*math.pi/60
    tau = tau_stall*(1-w/wfree) - 0.01
    print(f"torque headroom at {rpm} rpm, 12 V: {tau:.3f} N m -> accel {tau/J:.0f} rad/s^2; 4% dip recovers in {0.04*w/(tau/J)*1000:.0f} ms (saturated)")
    # velocity quantization, 28 CPR
    for dt in (0.015, 0.025, 0.040):
        cps = rpm/60*28
        n = cps*dt
        print(f"dt={dt*1000:.0f} ms: {n:.0f} counts/loop, sigma(v)={100*0.408/n:.2f}%, sigma(v_k-v_k-1)={0.707/dt:.0f} ticks/s vs stability gate 20; 2 ms timestamp slip -> {100*0.002/dt:.1f}% velocity error")

if __name__ == "__main__":
    for m in (0.020, 0.030, 0.050):
        table(m=m)
    table(m=0.030, cl=0.2)
    # lateral window from geometry
    print("\nLateral allowance (frontal): half-width 10 - 1.4 = 8.6 in -> yaw tolerance deg:")
    for d in (1.5, 2.0, 2.5, 3.0, 3.5):
        print(f"  d={d} m  {math.degrees(math.atan(8.6*IN/d)):.2f} deg; at 45 deg azimuth {math.degrees(math.atan((10*math.cos(math.pi/4)-1.4)*IN/d)):.2f} deg")
    extras()
