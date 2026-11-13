"""Fits the sim CR-servo turret model to every turret log (autotune runs, SHT sessions, match logs), and exports
real-match traces for the Sim's TurretTuning.
  python -I tools/turret_fit.py fit [iterations]
  python -I tools/turret_fit.py export
"""
import csv, glob, os, sys, json
import numpy as np

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tuning-data")
TPD = 68.27


def load_csv(path):
    with open(path, newline='') as f:
        r = csv.reader(f)
        head = next(r)
        rows = [x for x in r if len(x) == len(head)]
    cols = {h: np.array([float(v) if v not in ('', 'NaN', 'nan') else np.nan for v in col]) for h, col in zip(head, zip(*rows))} if rows else {}
    return cols


def runs():
    out = []
    for d in sorted(glob.glob(os.path.join(ROOT, 'turret-autotune', '*'))):
        if os.path.exists(os.path.join(d, 'IGNORED')):
            continue
        c = load_csv(os.path.join(d, 'samples.csv'))
        if not c:
            continue
        omega = c.get('omega', np.zeros_like(c['t']))
        out.append(dict(name=os.path.basename(d), t=c['t'], x=c['position'], v=c['velocity'], u=0.5 * (c['left_volts'] + c['right_volts']), omega=omega))
    for d in glob.glob(os.path.join(ROOT, 'sessions', '*')):
        c = load_csv(os.path.join(d, 'loop.csv'))
        out.append(dict(name='sht-' + os.path.basename(d), t=c['t'], x=c['tu_pos'], v=c['tu_vel'], u=c['tu_power'] * c['tu_volts_comp'], omega=c['omega']))
    for d in glob.glob(os.path.join(ROOT, 'match-logs', '*')):
        c = load_csv(os.path.join(d, 'loop.csv'))
        out.append(dict(name='match-' + os.path.basename(d), t=c['t'], x=c['tu_pos'], v=c['tu_vel'], u=c['tu_power'] * 12.6, omega=c['omega']))
    return out


HORIZON = 0.35
H = 0.002


def windows(rs, every=0.25, active_only=True):
    """starts where something is happening (volts or motion), each window: start state + (time, volts) list + measured positions"""
    W = []
    for r in rs:
        t, x, v, u, om = r['t'], r['x'], r['v'], r['u'], r['omega']
        ok = np.isfinite(t) & np.isfinite(x) & np.isfinite(u)
        t, x, v, u, om = t[ok], x[ok], np.nan_to_num(v[ok]), u[ok], np.nan_to_num(om[ok])
        alpha = np.gradient(om, t) if len(t) > 3 else np.zeros_like(t)
        vb = np.zeros_like(x)
        vb[2:] = (x[2:] - x[:-2]) / np.maximum(t[2:] - t[:-2], 1e-3)
        last = -1e9
        for k in range(2, len(t)):
            if t[k] - last < every:
                continue
            j = np.searchsorted(t, t[k] + HORIZON)
            if j >= len(t) or j - k < 6:
                continue
            seg = slice(k - 1, j + 1)
            if active_only and np.max(np.abs(u[seg])) < 0.3 and np.ptp(x[seg]) < 0.3 * TPD:
                continue
            if np.max(np.diff(t[seg])) > 0.15:
                continue
            last = t[k]
            W.append(dict(run=r['name'], t=t[seg] - t[k], x=x[seg], v=vb[k], rest=abs(x[k] - x[k - 2]) < 1, u=u[seg], alpha=alpha[seg]))
    return W


def pack(W):
    n = len(W)
    steps = int(HORIZON / H) + 2
    grid = np.arange(steps) * H
    # volts in force at time s (zero-order hold, applied from its sample time), robot alpha, measured positions on the grid
    U = np.zeros((n, 400))
    UT = np.full((n, 400), 1e9)
    A = np.zeros((n, steps))
    X0 = np.zeros(n)
    V0 = np.zeros(n)
    meas_idx, meas_val, meas_t = [], [], []
    for i, w in enumerate(W):
        m = len(w['t'])
        U[i, :m] = w['u']
        UT[i, :m] = w['t']
        A[i] = np.interp(grid, w['t'], w['alpha'])
        X0[i] = w['x'][1]
        V0[i] = w['v']
        for k in range(2, m):
            if 0 < w['t'][k] <= HORIZON:
                meas_idx.append(i)
                meas_t.append(w['t'][k])
                meas_val.append(w['x'][k] - w['x'][1])
    return dict(U=U, UT=UT, A=A, X0=X0, V0=V0, grid=grid, mi=np.array(meas_idx), mt=np.array(meas_t), mv=np.array(meas_val), n=n)


def simulate(P, D):
    gain, tau, dead, kc, ks, sd, acc, top, coast, couple = P
    n, grid = D['n'], D['grid']
    U, UT = D['U'], D['UT']
    w = D['V0'].copy()
    x = np.zeros(n)
    push = np.full(n, np.nan)
    out = np.zeros((n, len(grid)))
    for s, now in enumerate(grid):
        # volts sent at time <= now - dead
        idx = np.sum(UT <= (now - dead), axis=1) - 1
        u = np.where(idx >= 0, U[np.arange(n), np.maximum(idx, 0)], U[:, 0])
        rest = np.abs(w) < 1
        below = rest & (np.abs(u) <= ks)
        push = np.where(rest & ~below & np.isnan(push), now, push)
        waiting = rest & ~below & (now - push < sd - dead)
        w = np.where(rest & ~below & ~waiting, np.sign(u) * 1.5, w)
        moving = ~(below | waiting)
        push = np.where(moving & ~rest, np.nan, push)
        push = np.where(below, np.nan, push)
        ra = couple * D['A'][:, s] * (-180 / np.pi) * TPD
        if coast <= 0:
            wanted = np.clip(gain * (u - kc * np.sign(w)), -top, top)
            rate = (wanted - w) / tau - ra
            cap = np.where(rate * w >= 0, acc, 3 * acc)
            nxt = w + H * np.clip(rate, -cap, cap)
        else:
            au = np.abs(u)
            wanted = np.where(au > kc, np.sign(u) * np.minimum(top, gain * (au - kc)), 0)
            rev = np.sign(u) != np.sign(w)
            up = np.abs(wanted) >= np.abs(w)
            r_rev = np.clip((wanted - w) / tau, -3 * acc, 3 * acc)
            r_up = np.clip((wanted - w) / tau, -acc, acc)
            r_coast = -np.sign(w) * np.minimum(coast, np.abs(w - wanted) / H)
            rate = np.where(au <= 0.1, -np.sign(w) * np.minimum(coast, np.abs(w) / H), np.where(rev, r_rev, np.where(up, r_up, r_coast)))
            nxt = w + H * (rate - ra)
        stopflip = (np.sign(nxt) != np.sign(w)) & ((np.abs(u) <= ks) | (np.sign(u) == np.sign(w)))
        nxt = np.where(stopflip, 0, nxt)
        nxt = np.where(moving, nxt, 0)
        w = nxt
        x = x + w * H
        out[:, s] = x
    return out


def cost(P, D):
    if P[0] <= 1000 or P[1] <= 0.005 or P[2] < 0 or P[3] < 0 or P[4] < P[3] or P[5] < P[2] or P[6] <= 1000 or P[7] <= 1000 or not (0 <= P[9] <= 1.5):
        return 1e18
    X = simulate(P, D)
    pred = X[D['mi'], np.minimum((D['mt'] / H).astype(int), X.shape[1] - 1)]
    e = (pred - D['mv']) / TPD
    return float(np.mean(np.minimum(e * e, 400)))


def nelder(f, x0, scale, iters=250):
    n = len(x0)
    pts = [np.array(x0, float)] + [np.array(x0, float) + np.eye(n)[i] * scale[i] for i in range(n)]
    vals = [f(p) for p in pts]
    for it in range(iters):
        o = np.argsort(vals)
        pts = [pts[i] for i in o]
        vals = [vals[i] for i in o]
        c = np.mean(pts[:-1], axis=0)
        r = c + (c - pts[-1])
        fr = f(r)
        if fr < vals[0]:
            e = c + 2 * (c - pts[-1])
            fe = f(e)
            pts[-1], vals[-1] = (e, fe) if fe < fr else (r, fr)
        elif fr < vals[-2]:
            pts[-1], vals[-1] = r, fr
        else:
            k = c + 0.5 * (pts[-1] - c)
            fk = f(k)
            if fk < vals[-1]:
                pts[-1], vals[-1] = k, fk
            else:
                for i in range(1, n + 1):
                    pts[i] = pts[0] + 0.5 * (pts[i] - pts[0])
                    vals[i] = f(pts[i])
    i = int(np.argmin(vals))
    return pts[i], vals[i]


NAMES = ['gain', 'tau', 'dead', 'kinetic', 'breakaway', 'start_delay', 'max_accel', 'max_speed', 'coast_decel', 'robot_coupling']

def export():

    out = os.path.join(ROOT, '..', 'Sim', 'out', 'turret-traces')
    os.makedirs(out, exist_ok=True)
    cols = [('time', 't'), ('loop', 'loop_ms'), ('pose', 'omega'), ('shooter', 'aim_raw'), ('shooter', 'auto_aim'), ('shooter', 'locked'), ('shooter', 'aim_rate')]
    for d in sorted(glob.glob(os.path.join(ROOT, 'robot-logs', '*TeleOp'))):
        ch = [l.rstrip('\n').split('\t') for l in open(os.path.join(d, 'channels.tsv'))][1:]
        idx = {(c[1], c[2]): int(c[0]) for c in ch}
        if any(c not in idx for c in cols):
            continue
        a = np.fromfile(os.path.join(d, 'frames.bin'), '<f8')
        a = a[:len(a) // len(ch) * len(ch)].reshape(-1, len(ch))
        if len(a) < 2000 or np.nanpercentile(np.abs(a[:, idx[('pose', 'omega')]]), 90) < 20:
            continue
        np.savetxt(os.path.join(out, os.path.basename(d) + '.csv'), np.column_stack([a[:, idx[c]] for c in cols]), delimiter=',', header=','.join(c[1] for c in cols), comments='', fmt='%.6g')
        print('exported', os.path.basename(d))


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == 'export':
        export()
        sys.exit()
    if len(sys.argv) > 1 and sys.argv[1] == 'fit':
        sys.argv.pop(1)
    rs = runs()
    for r in rs:
        print('%-36s %6d samples, %5.0f s, |u|>0.3 V %4.0f%%' % (r['name'], len(r['t']), np.nanmax(r['t']) - np.nanmin(r['t']), 100 * np.mean(np.abs(np.nan_to_num(r['u'])) > 0.3)))
    W = windows(rs)
    rng = np.random.default_rng(1)
    rng.shuffle(W)
    train, test = W[::2], W[1::2]
    print('windows', len(W))
    Dtr, Dte = pack(train), pack(test)
    base = [11374.8, 0.046, 0.0361, 1.011, 1.163, 0.0828, 118456, 26233, 0, 1.0]
    base_coast = base[:8] + [71702, 1.0]
    for label, x0 in (('autotuner model, linear', base), ('autotuner model, coast', base_coast)):
        print('%-28s train %.3f test %.3f deg^2' % (label, cost(x0, Dtr), cost(x0, Dte)))
    best = None
    for label, x0 in (('linear', base), ('coast', base_coast)):
        scale = [2000, 0.02, 0.015, 0.25, 0.25, 0.03, 40000, 6000, 20000 if x0[8] > 0 else 0, 0.3]
        p, v = nelder(lambda q: cost(q, Dtr), x0, scale, iters=int(sys.argv[1]) if len(sys.argv) > 1 else 200)
        te = cost(p, Dte)
        print('fit %-6s train %.3f test %.3f :: %s' % (label, v, te, ', '.join('%s=%.4g' % kv for kv in zip(NAMES, p))))
        if best is None or te < best[1]:
            best = (p, te, label)
    by = {}
    for w in test:
        by.setdefault(w['run'], []).append(w)
    for run, ws in sorted(by.items()):
        D = pack(ws)
        print('  %-36s %4d windows: autotuner %.3f, fit %.3f deg^2' % (run, len(ws), cost(base_coast if best[2] == 'coast' else base, D), cost(best[0], D)))
    json.dump(dict(zip(NAMES, map(float, best[0])), model=best[2]), open(os.path.join(ROOT, '..', 'Sim', 'out', 'turret_fit.json'), 'w'), indent=1)
