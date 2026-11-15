#!/usr/bin/env python3
"""Robot logs -> the flywheel's physics -> the Sim's wheel (tuning-data/flywheel-fit/plant.properties).

Runs FlywheelObserver's model (Python copy) over every logged loop with the volts that were sent (flywheel.power x the supply)
and fits the wheel so the hub's readings surprise it least: accel per volt, how fast it coasts with the power cut, kv, and how
late the hub's reading is. The same pass gives each ball's drop, the ball timing, the battery and the loop times. The Sim's
FlywheelBursts reads the file (FlywheelBursts.Config.fitted), so FlywheelTuning tries gains on the wheel these logs show.

  python -I tools/flywheel_fit.py fit [RUN ...]       default: every run in tuning-data/robot-logs with the flywheel spinning
  python -I tools/flywheel_fit.py fit RUN --no-save   print only

Steps in docs/VIDEO_SHOT_REVIEW.md ("Flywheel gains from the logs").
"""
import argparse
import math
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True
import robot_log  # noqa: E402

ROOT = os.path.dirname(HERE)
OUT = os.path.join(ROOT, 'tuning-data', 'flywheel-fit', 'plant.properties')
CONSTANTS = os.path.join(ROOT, 'TeamCode', 'src', 'main', 'java', 'org', 'firstinspires', 'ftc', 'teamcode', 'Constants', 'FlywheelConstants.java')
STEP = 0.01


def constant(name, default):
    src = open(CONSTANTS, encoding='utf-8').read()
    m = re.search(r'\b' + name + r'\s*=\s*([-0-9.eE]+)', src)
    return float(m.group(1)) if m else default


def load(name):
    run = robot_log.Run(robot_log.find(name))
    need = ('time.t', 'flywheel.raw_velocity', 'flywheel.power', 'flywheel.target')
    if not all(run.has(c) for c in need):
        return None
    t = np.array(run.col('time.t'))
    raw = np.array(run.col('flywheel.raw_velocity'))
    power = np.array(run.col('flywheel.power'))
    target = np.array(run.col('flywheel.target'))
    supply = np.array(run.col('flywheel.volts')) if run.has('flywheel.volts') else np.array(run.col('power.battery_v'))
    battery = np.array(run.col('power.battery_v')) if run.has('power.battery_v') else supply
    clear = np.array(run.col('blocker.clear')) if run.has('blocker.clear') else np.zeros(len(t))
    ok = np.isfinite(t) & np.isfinite(raw) & np.isfinite(power) & np.isfinite(supply)
    if ok.sum() < 200 or np.nanmax(target) <= 0:
        return None
    # a ball can hit while the blocker is open and until 0.6 s after it shuts (balls still on the way)
    feeding = np.zeros(len(t), dtype=bool)
    last_open = -1e9
    for k in range(len(t)):
        if clear[k] == 1:
            last_open = t[k]
        feeding[k] = t[k] - last_open <= 0.6
    return {'name': os.path.basename(run.folder), 't': t, 'raw': raw, 'volts': power * supply, 'target': target, 'battery': battery,
            'feeding': feeding, 'clear': clear, 'ok': ok}


def smooth_sign(x, width):
    return math.tanh(x / width) if width > 0 else (1.0 if x > 0 else -1.0 if x < 0 else 0.0)


def observe(d, p, keep=False):
    """FlywheelObserver over one run. Returns (sum of squared surprises, count) on loops with no ball near, and with keep, the
    contacts: (time, drop it took, feeding window start)."""
    A, kv, ks, coast, lag = p['accel'], p['kv'], p['ks'], p['coast'], p['lag']
    gain, load_gain, contact_drop = p['gain'], p['load_gain'], p['contact_drop']
    t, raw, volts, target, feeding, ok = d['t'], d['raw'], d['volts'], d['target'], d['feeding'], d['ok']
    times, speeds = [], []
    v = load = 0.0
    u = 0.0
    last = None
    sq = 0.0
    n = 0
    contact_t = -1e9
    contacts = []
    drop = None
    diverged = None
    prev_raw = None
    for k in range(len(t)):
        if not ok[k]:
            continue
        tk = float(t[k])
        if last is None or tk <= last or tk - last > 0.3 or target[k] <= 0:
            v = float(raw[k])
            times, speeds = [tk], [v]
            last = tk
            u = float(volts[k])
            continue
        dt = tk - last
        last = tk
        steps = max(1, int(math.ceil(dt / STEP)))
        h = dt / steps
        for _ in range(steps):
            net = u - kv * v - ks * smooth_sign(v, 35) - load
            v += A * max(net, -coast) * h
        times.append(tk)
        speeds.append(v)
        if len(times) > 16:
            del times[0]
            del speeds[0]
        # the estimate at tk - lag
        tp = tk - lag
        past = speeds[0]
        for j in range(len(times) - 1, 0, -1):
            if times[j - 1] <= tp:
                span = times[j] - times[j - 1]
                f = (tp - times[j - 1]) / span if span > 0 else 1
                past = speeds[j - 1] + min(1, max(0, f)) * (speeds[j] - speeds[j - 1])
                break
        inn = float(raw[k]) - past
        fell = prev_raw is None or float(raw[k]) <= prev_raw - p.get('contact_fall', 0)
        prev_raw = float(raw[k])
        if feeding[k] and inn < -contact_drop and fell:
            corr = inn
            if tk - lag - contact_t > 0.12:
                if keep and drop is not None:
                    contacts[-1][1] = drop
                contact_t = tk - lag - 0.02
                drop = 0.0
                if keep:
                    contacts.append([contact_t, float('nan')])
        else:
            corr = gain * inn
            if tk - lag - contact_t > 0.12:
                load = min(1.5, max(-1.5, load - load_gain * inn))
                if feeding[k] or abs(inn) < 150:
                    sq += inn * inn
                    n += 1
        v += corr
        for j in range(len(speeds)):
            speeds[j] += corr
        if drop is not None:
            if tk - lag - contact_t <= 0.12:
                drop -= corr
            else:
                if keep:
                    contacts[-1][1] = drop
                drop = None
        # far off for long (a jam, a reseed on the robot): start over
        if abs(inn) > 150 and not feeding[k]:
            diverged = tk if diverged is None else diverged
            if tk - diverged > 0.25:
                v = float(raw[k])
                times, speeds = [tk], [v]
                diverged = None
        else:
            diverged = None
        u = float(volts[k])
    return sq, n, contacts


def cost(runs, p):
    sq = n = 0
    for d in runs:
        a, b, _ = observe(d, p)
        sq += a
        n += b
    return math.sqrt(sq / max(1, n)), n


def golden(f, lo, hi, iters=12):
    g = (math.sqrt(5) - 1) / 2
    a, b = lo, hi
    c, e = b - g * (b - a), a + g * (b - a)
    fc, fe = f(c), f(e)
    for _ in range(iters):
        if fc < fe:
            b, e, fe = e, c, fc
            c = b - g * (b - a)
            fc = f(c)
        else:
            a, c, fc = c, e, fe
            e = a + g * (b - a)
            fe = f(e)
    return (c, fc) if fc < fe else (e, fe)


def cmd_fit(a):
    names = a.runs or (sorted(os.listdir(robot_log.LOCAL)) if os.path.isdir(robot_log.LOCAL) else [])
    runs = []
    for name in names:
        d = load(name)
        if d is not None and np.sum(d['target'] > 0) > 300:
            runs.append(d)
    if not runs:
        sys.exit('no runs with the flywheel spinning')
    print('%d run(s): %s' % (len(runs), ', '.join(d['name'] for d in runs)))
    code = {'accel': constant('FLYWHEEL_ACCEL_PER_VOLT', 165), 'kv': 0.00475, 'ks': 0.42, 'coast': constant('FLYWHEEL_COAST_VOLTS', 0.7),
            'lag': constant('FLYWHEEL_OBSERVER_LAG', 0.035), 'gain': constant('FLYWHEEL_OBSERVER_GAIN', 0.35),
            'load_gain': constant('FLYWHEEL_OBSERVER_LOAD_GAIN', 0.0005), 'contact_drop': constant('FLYWHEEL_CONTACT_DROP', 45)}
    base, n = cost(runs, code)
    print('the code\'s wheel (accel %.0f, coast %.2f V, kv %.5f, lag %.3f s): surprise %.2f ticks/s rms over %d loops' % (code['accel'], code['coast'], code['kv'], code['lag'], base, n))
    p = dict(code)
    bounds = {'accel': (80, 300), 'coast': (0.1, 3.0), 'kv': (0.0040, 0.0055), 'lag': (0.0, 0.09)}
    for rnd in range(a.passes):
        for key in ('accel', 'coast', 'lag', 'kv'):
            lo, hi = bounds[key]

            def f(x):
                q = dict(p)
                q[key] = x
                return cost(runs, q)[0]
            x, fx = golden(f, lo, hi)
            p[key] = x
        print('  pass %d: accel %.0f, coast %.2f V, kv %.5f, lag %.3f s -> %.2f ticks/s rms' % (rnd + 1, p['accel'], p['coast'], p['kv'], p['lag'], cost(runs, p)[0]))
    fit, _ = cost(runs, p)

    drops, gaps, firsts, dts, start_v, end_v, sags = [], [], [], [], [], [], []
    for d in runs:
        _, _, contacts = observe(d, p, keep=True)
        t, clear = d['t'], d['clear']
        opens = [float(t[k]) for k in range(1, len(t)) if clear[k] == 1 and clear[k - 1] != 1]
        prev = None
        for ct, dr in contacts:
            if dr == dr and 20 < dr < 400:
                drops.append(dr)
            if prev is not None and 0.08 < ct - prev < 0.6:
                gaps.append(ct - prev)
            before = [o for o in opens if o <= ct]
            if before and 0.05 < ct - before[-1] < 0.6 and (prev is None or prev < before[-1]):
                firsts.append(ct - before[-1])
            prev = ct
        dt = np.diff(t[d['ok']])
        dts.extend(dt[(dt > 0.005) & (dt < 0.3)].tolist())
        spin = (d['target'] > 0) & np.isfinite(d['battery'])
        bv, tv = d['battery'][spin], t[spin]
        if len(bv) > 100:
            start_v.append(float(np.median(bv[tv < tv[0] + 30])))
            end_v.append(float(np.median(bv[tv > tv[-1] - 30])))
            sags.append(float(np.median(bv) - np.percentile(bv, 2)))
    dts = np.array(dts)
    out = {
        'accel': p['accel'], 'coast': p['coast'], 'kv': p['kv'], 'ks': p['ks'],
        # the hub's 50 ms count stands for the speed ~25 ms back; whatever lag is left is how late it's read
        'readAgeMax': min(0.03, max(0.0, 2 * (p['lag'] - 0.025))),
        'ballDrop': float(np.median(drops)) if drops else 95, 'ballDropSpread': float(np.std(drops)) if len(drops) > 3 else 15,
        'firstContact': float(np.median(firsts)) if firsts else 0.24, 'interval': float(np.median(gaps)) if gaps else 0.3,
        'intervalSpread': float(np.std(gaps)) if len(gaps) > 3 else 0.03,
        'loopMedian': float(np.median(dts)), 'loopSpread': float(np.std(np.log(dts))),
        'startVolts': float(np.median(start_v)) if start_v else 13.3, 'endVolts': float(np.median(end_v)) if end_v else 11.0,
    }
    print('fitted wheel: accel %.0f ticks/s^2 per V (code %.0f), coast %.2f V (code %.2f), kv %.5f (code %.5f), lag %.3f s (code %.3f): surprise %.2f -> %.2f ticks/s rms' % (
        p['accel'], code['accel'], p['coast'], code['coast'], p['kv'], code['kv'], p['lag'], code['lag'], base, fit))
    print('balls: %d drops, median %.0f ticks/s (spread %.0f); first %.3f s after opening (%d), then every %.3f s (%d)' % (
        len(drops), out['ballDrop'], out['ballDropSpread'], out['firstContact'], len(firsts), out['interval'], len(gaps)))
    print('battery while spinning: %.2f V at the start, %.2f V at the end, dips %.2f V; loops median %.1f ms (log spread %.2f)' % (
        out['startVolts'], out['endVolts'], float(np.median(sags)) if sags else float('nan'), 1000 * out['loopMedian'], out['loopSpread']))
    worth = base - fit > 0.05 * base
    print('-> the observer\'s own constants: %s' % (
        'FLYWHEEL_ACCEL_PER_VOLT %.0f, FLYWHEEL_COAST_VOLTS %.2f, FLYWHEEL_OBSERVER_LAG %.3f would surprise it %.0f%% less' % (p['accel'], p['coast'], p['lag'], 100 * (base - fit) / base)
        if worth else 'already fit the logs (within 5%)'))
    if not a.no_save:
        os.makedirs(os.path.dirname(OUT), exist_ok=True)
        with open(OUT, 'w', encoding='utf-8', newline='\n') as f:
            f.write('# tools/flywheel_fit.py from %s\n' % ', '.join(d['name'] for d in runs))
            f.write('# surprise %.2f ticks/s rms (the code\'s wheel %.2f); %d ball drops, %d gaps\n' % (fit, base, len(drops), len(gaps)))
            f.write('# wheelFit=1: the fitted wheel (accel, coast, kv, readAgeMax) beat the code\'s by 5%+, so the Sim uses it; 0 = only the balls, battery and loops\n')
            f.write('wheelFit=%d\n' % (1 if worth else 0))
            for k, v in out.items():
                f.write('%s=%.6g\n' % (k, v))
        print('wrote', OUT)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('fit')
    s.add_argument('runs', nargs='*')
    s.add_argument('--passes', type=int, default=2)
    s.add_argument('--no-save', action='store_true')
    s.set_defaults(f=cmd_fit)
    a = p.parse_args()
    a.f(a)


if __name__ == '__main__':
    main()
