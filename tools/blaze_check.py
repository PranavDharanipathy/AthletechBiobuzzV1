#!/usr/bin/env python3
"""Is Blaze reliable? Reads a robot log (tools/robot_log.py pull) and compares it with a Blaze-off run.

  python -I tools/blaze_check.py [RUN] [--base RUN]

RUN defaults to the newest run, --base to the newest earlier run of a TeleOp with BlazeConstants.ENABLED false.
Covers: Blaze's hub and Pinpoint feeds, faults, loop times, command-to-motion lag (turret, drive), pose resets and
jumps, turret hold and tracking, intake, and a verdict.
"""
import argparse
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOCAL = os.path.join(ROOT, 'tuning-data', 'robot-logs')
TPD = 68.27


class Run:
    def __init__(self, name):
        self.name = name
        d = os.path.join(LOCAL, name)
        rows = [l.rstrip('\n').split('\t') for l in open(os.path.join(d, 'channels.tsv'), encoding='utf-8')][1:]
        self.index = {r[1] + '.' + r[2]: int(r[0]) for r in rows}
        n = len(rows)
        a = np.fromfile(os.path.join(d, 'frames.bin'), '<f8')
        self.a = a[:len(a) // n * n].reshape(-1, n)
        self.meta = {}
        for l in open(os.path.join(d, 'meta.tsv'), encoding='utf-8'):
            p = l.rstrip('\n').split('\t')
            if len(p) >= 2:
                self.meta[p[0]] = p[1]
        self.events = []
        path = os.path.join(d, 'events.tsv')
        if os.path.exists(path):
            for l in list(open(path, encoding='utf-8'))[1:]:
                p = l.rstrip('\n').split('\t')
                if len(p) >= 3:
                    self.events.append((float(p[0]), p[1], p[2]))
        self.t = self.c('time.t')

    def has(self, k):
        return k in self.index and np.isfinite(self.a[:, self.index[k]]).any()

    def c(self, k):
        return self.a[:, self.index[k]] if k in self.index else np.full(len(self.a), np.nan)

    def blaze(self):
        return self.meta.get('BlazeConstants.ENABLED')


def runs():
    return sorted(r for r in os.listdir(LOCAL) if os.path.exists(os.path.join(LOCAL, r, 'frames.bin')))


def pick(name, default):
    if not name:
        return default
    hits = [r for r in runs() if name in r]
    if not hits:
        sys.exit('no run like %r' % name)
    return hits[-1]


def q(x, p):
    x = x[np.isfinite(x)]
    return np.percentile(x, p) if len(x) else np.nan


def lag(r, x, y, valid, top=0.8):
    #seconds from x to y at the best correlation, on a 10 ms grid
    t = r.t
    m = valid & np.isfinite(x) & np.isfinite(y) & np.isfinite(t)
    if m.sum() < 200:
        return np.nan, np.nan
    g = np.arange(t[m][0], t[m][-1], 0.01)
    xs = np.interp(g, t[m], x[m])
    ys = np.interp(g, t[m], y[m])
    xs -= xs.mean()
    ys -= ys.mean()
    best = (np.nan, -2)
    for k in range(0, int(top / 0.01)):
        a, b = xs[:len(xs) - k], ys[k:]
        if a.std() == 0 or b.std() == 0:
            continue
        cc = np.corrcoef(a, b)[0, 1]
        if cc > best[1]:
            best = (k * 0.01, cc)
    return best


def report(r):
    out = {}
    t = r.t
    dt = np.diff(t)
    dur = t[-1] - t[0] if len(t) > 1 else 0
    print('=== %s  (%.0f s, %d loops, Blaze %s, motor accel %s, Pinpoint accel %s)' % (r.name, dur, len(t), r.blaze(), r.meta.get('BlazeConstants.MOTOR_ACCELERATION'), r.meta.get('BlazeConstants.PINPOINT_ACCELERATION')))

    lm = r.c('loop.loop_ms')
    out['loop50'], out['loop95'] = q(lm, 50), q(lm, 95)
    print('loop ms   p50 %.1f  p95 %.1f  p99 %.1f  max %.0f   loops > 100 ms: %d' % (q(lm, 50), q(lm, 95), q(lm, 99), np.nanmax(lm), np.sum(lm > 100)))

    if r.has('blaze.open'):
        print('-- Blaze')
        print('open %.0f%% of loops, running %.0f%%' % (100 * np.nanmean(r.c('blaze.open')), 100 * np.nanmean(r.c('blaze.running'))))
        for hub, key in (('Control Hub', 'ch'), ('Expansion Hub', 'eh')):
            age = r.c('blaze.%s_age_ms' % key)
            live = r.c('blaze.%s_live' % key)
            upd = r.c('blaze.%s_updates' % key)
            if not np.isfinite(age).any():
                print('%-13s not fed by Blaze' % hub)
                continue
            rate = (upd[-1] - upd[np.isfinite(upd)][0]) / dur if dur else np.nan
            out[key + '_stale'] = 1 - np.nanmean(live)
            out[key + '_age95'] = q(age, 95)
            print('%-13s updates %.0f Hz   age ms p50 %.1f p95 %.1f max %.0f   stale (normal reads) %.1f%% of loops' % (hub, rate, q(age, 50), q(age, 95), np.nanmax(age), 100 * out[key + '_stale']))
        pa = r.c('pose.blaze_age_ms')
        if np.isfinite(pa).any():
            sdk = r.c('pose.blaze_sdk')
            pu = r.c('blaze.pinpoint_updates')
            out['pp_age95'], out['pp_sdk'] = q(pa, 95), np.nanmean(sdk)
            print('Pinpoint      samples %.0f Hz   age ms p50 %.1f p95 %.1f max %.0f   SDK fallback %.1f%% of loops' % ((pu[-1] - pu[np.isfinite(pu)][0]) / dur, q(pa, 50), q(pa, 95), np.nanmax(pa), 100 * out['pp_sdk']))
        else:
            print('Pinpoint      not fed by Blaze')
        faults = [e for e in r.events if e[1] == 'blaze' and not e[2].endswith('=')]
        out['faults'] = len(faults)
        for e in faults:
            print('FAULT at %.2f s: %s' % (e[0], e[2]))
        if not faults:
            print('faults: none')

    print('-- lag (command -> motion, best correlation)')
    tv, rv = r.c('turret.volts'), r.c('turret.raw_velocity')
    l, cc = lag(r, tv, rv, np.abs(tv) > 0.05)
    out['turret_lag'] = l
    print('turret volts -> encoder speed   %.0f ms  (corr %.2f)' % (1000 * l, cc))
    h = np.radians(r.c('pose.heading'))
    fwd = r.c('pose.vx') * np.cos(h) + r.c('pose.vy') * np.sin(h)
    cf = r.c('drive.cmd_forward')
    l, cc = lag(r, cf, fwd, np.isfinite(cf))
    out['drive_lag'] = l
    print('drive forward stick -> speed    %.0f ms  (corr %.2f)' % (1000 * l, cc))
    ct = r.c('drive.cmd_turn')
    l, cc = lag(r, -ct, r.c('pose.omega'), np.isfinite(ct))
    l2, cc2 = lag(r, ct, r.c('pose.omega'), np.isfinite(ct))
    if cc2 > cc:
        l, cc = l2, cc2
    out['turn_lag'] = l
    print('drive turn stick -> turn rate   %.0f ms  (corr %.2f)' % (1000 * l, cc))

    print('-- pose')
    x, y, hd = r.c('pose.x'), r.c('pose.y'), r.c('pose.heading')
    zero = (x == 0) & (y == 0) & (hd == 0)
    nonzero_before = np.maximum.accumulate(~zero & np.isfinite(x))
    resets = np.sum(zero[1:] & ~zero[:-1] & nonzero_before[:-1])
    reloc = [e[0] for e in r.events if 'relocaliz' in e[2]]
    step = np.hypot(np.diff(x), np.diff(y))
    jumps = [(t[k + 1], step[k]) for k in range(len(step)) if step[k] > 6 and not any(0 <= t[k + 1] - e < 0.3 for e in reloc) and not zero[k + 1]]
    out['pose_resets'], out['pose_jumps'] = resets, len(jumps)
    print('resets to (0,0,0): %d   jumps > 6 in per loop (not relocalize): %d   relocalizes: %d   loops at (0,0,0): %.0f%%' % (resets, len(jumps), len(reloc), 100 * zero.mean()))
    for tj, s in jumps[:5]:
        print('  jump %.1f in at %.2f s' % (s, tj))

    print('-- turret')
    tg, e, v = r.c('turret.target'), r.c('turret.error'), r.c('turret.volts')
    if np.isfinite(tg).sum() > 50:
        dtg = np.abs(np.gradient(tg, t))
        hold = (dtg < 3) & (np.abs(e) < 8)
        move = (dtg >= 3) & np.isfinite(e)
        ht = hold.sum() * np.nanmedian(dt)
        vv = v[hold]
        flips = np.sum(vv[1:] * vv[:-1] < 0) / ht if ht > 0 else np.nan
        out['hold95'], out['hold_flips'] = q(np.abs(e[hold]), 95), flips
        print('holding (%.0f s): |err| p50 %.2f p95 %.2f deg   speed rms %.0f deg/s   volt flips %.1f/s' % (ht, q(np.abs(e[hold]), 50), q(np.abs(e[hold]), 95), np.sqrt(np.nanmean(r.c('turret.velocity')[hold] ** 2)), flips))
        print('tracking (%.0f s): |err| p50 %.2f p95 %.2f deg   on target %.0f%%' % (move.sum() * np.nanmedian(dt), q(np.abs(e[move]), 50), q(np.abs(e[move]), 95), 100 * np.nanmean(r.c('turret.on_target')[move])))
        if np.nanmax(r.c('turret.encoder_frozen')) > 0:
            print('ENCODER CUTOFF tripped')

    print('-- intake')
    st, iv, tvv = r.c('intake.state'), r.c('intake_motor.velocity'), r.c('intake.target_velocity')
    on = st == 1
    if on.any():
        start = np.zeros(len(t))
        s0 = np.nan
        for k in range(len(t)):
            if on[k] and (k == 0 or not on[k - 1]):
                s0 = t[k]
            start[k] = t[k] - s0 if on[k] else np.nan
        settled = on & (start > 0.6)
        tgt = np.where(np.isfinite(tvv), tvv, float(r.meta.get('IntakeConstants.INTAKE_VELOCITY', 2300)))
        low = settled & (iv < 0.5 * tgt)
        out['intake_low'] = low.sum() / max(1, settled.sum())
        print('held %.0f s: speed p50 %.0f ticks/s   below half speed %.1f%% of the time (after 0.6 s)' % (on.sum() * np.nanmedian(dt), q(iv[settled], 50), 100 * out['intake_low']))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run', nargs='?')
    ap.add_argument('--base')
    a = ap.parse_args()
    names = runs()
    run = pick(a.run, names[-1])
    if a.base:
        base = pick(a.base, None)
    else:
        base = None
        for n in reversed(names[:names.index(run)]):
            if 'TeleOp' in n:
                rr = Run(n)
                if rr.blaze() == 'false' and len(rr.t) > 2000:
                    base = n
                    break
    R = report(Run(run))
    B = report(Run(base)) if base else None
    print()
    print('=== verdict: %s vs %s' % (run, base))
    if Run(run).blaze() != 'true':
        print('NOTE: Blaze was off in this run, nothing to judge')
    bad = []
    if R.get('faults'):
        bad.append('Blaze reported %d faults' % R['faults'])
    for k, name in (('ch', 'Control Hub'), ('eh', 'Expansion Hub')):
        if R.get(k + '_stale', 0) > 0.05:
            bad.append('%s data stale %.0f%% of loops' % (name, 100 * R[k + '_stale']))
    if R.get('pp_sdk', 0) > 0.1:
        bad.append('Pinpoint fell back to the SDK %.0f%% of loops' % (100 * R['pp_sdk']))
    if R.get('pp_age95', 0) > 50:
        bad.append('Pinpoint sample p95 %.0f ms old' % R['pp_age95'])
    if R.get('pose_resets') and (not B or R['pose_resets'] > B.get('pose_resets', 0)):
        bad.append('pose reset to (0,0,0) %d times' % R['pose_resets'])
    elif R.get('pose_resets'):
        print('pose reset to (0,0,0) %d times, but %d without Blaze too: the Pinpoint itself (power, wiring), not Blaze' % (R['pose_resets'], B['pose_resets']))
    if B:
        for k, name in (('turret_lag', 'turret'), ('drive_lag', 'drive'), ('turn_lag', 'turn')):
            if np.isfinite(R.get(k, np.nan)) and np.isfinite(B.get(k, np.nan)) and R[k] - B[k] > 0.04:
                bad.append('%s lag %.0f ms vs %.0f ms without Blaze' % (name, 1000 * R[k], 1000 * B[k]))
        if R['loop50'] > B['loop50'] * 1.1:
            bad.append('median loop %.1f ms vs %.1f ms without Blaze (slower)' % (R['loop50'], B['loop50']))
        if R.get('hold95', 0) > 1.5 * B.get('hold95', np.inf):
            bad.append('turret hold p95 %.1f deg vs %.1f deg without Blaze' % (R['hold95'], B['hold95']))
        if R.get('intake_low', 0) > B.get('intake_low', 0) + 0.1:
            bad.append('intake under half speed %.0f%% vs %.0f%% without Blaze' % (100 * R['intake_low'], 100 * B['intake_low']))
        if R['loop50'] < B['loop50'] * 0.9:
            print('loops faster with Blaze: %.1f ms vs %.1f ms median' % (R['loop50'], B['loop50']))
    print('\n'.join('UNRELIABLE: ' + b for b in bad) if bad else 'nothing wrong found with Blaze in this run')


if __name__ == '__main__':
    main()
