#!/usr/bin/env python3
"""Read the robot's recordings (TeamCode Utility/Recording): /sdcard/FIRST/data/RobotLog/<time>-<OpMode>/
   frames.bin (float64 rows), channels.tsv (columns), events.tsv, meta.tsv.

  python tools/robot_log.py pull                 copy every run off the robot (adb) into tuning-data/robot-logs
  python tools/robot_log.py list                 runs here, newest last
  python tools/robot_log.py channels RUN         the columns
  python tools/robot_log.py events RUN           swings, relocalizations, toggles, status changes
  python tools/robot_log.py meta RUN [filter]    constants, hardware, start (filter = substring)
  python tools/robot_log.py csv RUN [-c turret.error,pose.omega] [-o out.csv]
  python tools/robot_log.py summary RUN          the review: loop, shots, turret, flywheel, swings

RUN = a folder, a run name, or 'last' (newest). The build also pulls runs whenever a robot is on adb.
"""
import argparse
import array
import math
import os
import statistics
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOCAL = os.path.join(ROOT, 'tuning-data', 'robot-logs')
REMOTE = '/sdcard/FIRST/data/RobotLog'


def adb():
    sdk = os.environ.get('ANDROID_HOME') or os.path.join(os.environ.get('LOCALAPPDATA', ''), 'Android', 'Sdk')
    exe = os.path.join(sdk, 'platform-tools', 'adb.exe' if os.name == 'nt' else 'adb')
    return exe if os.path.isfile(exe) else 'adb'


def find(run):
    if os.path.isdir(run):
        return run
    runs = sorted(os.listdir(LOCAL)) if os.path.isdir(LOCAL) else []
    if run == 'last' and runs:
        return os.path.join(LOCAL, runs[-1])
    hits = [r for r in runs if run in r]
    if not hits:
        sys.exit('no run like %r in %s' % (run, LOCAL))
    return os.path.join(LOCAL, hits[-1])


class Run:
    def __init__(self, folder):
        self.folder = folder
        self.channels = []
        with open(os.path.join(folder, 'channels.tsv'), encoding='utf-8') as f:
            next(f)
            for line in f:
                p = line.rstrip('\n').split('\t')
                self.channels.append((p[1] + '.' + p[2], p[3], p[4] if len(p) > 4 else ''))
        self.index = {c[0]: k for k, c in enumerate(self.channels)}
        data = array.array('d')
        path = os.path.join(folder, 'frames.bin')
        n = len(self.channels)
        size = os.path.getsize(path) // 8 // n * n
        with open(path, 'rb') as f:
            data.fromfile(f, size)
        if sys.byteorder != 'little':
            data.byteswap()
        self.data = data
        self.rows = size // n
        self.n = n

    def has(self, name):
        return name in self.index

    def col(self, name):
        k = self.index[name]
        return self.data[k::self.n]

    def events(self):
        out = []
        path = os.path.join(self.folder, 'events.tsv')
        if os.path.isfile(path):
            with open(path, encoding='utf-8') as f:
                next(f)
                for line in f:
                    p = line.rstrip('\n').split('\t')
                    out.append((float(p[0]), p[1], p[2], p[3] if len(p) > 3 else ''))
        return out

    def meta(self):
        out = {}
        path = os.path.join(self.folder, 'meta.tsv')
        if os.path.isfile(path):
            with open(path, encoding='utf-8') as f:
                next(f)
                for line in f:
                    p = line.rstrip('\n').split('\t', 1)
                    if len(p) == 2:
                        out[p[0]] = p[1]
        return out


def finite(x):
    return x == x and not math.isinf(x)


def pct(values, p):
    v = sorted(x for x in values if finite(x))
    return v[min(len(v) - 1, int(p * (len(v) - 1)))] if v else float('nan')


def cmd_pull(_):
    os.makedirs(LOCAL, exist_ok=True)
    out = subprocess.run([adb(), 'shell', 'ls', REMOTE], capture_output=True, text=True)
    if out.returncode != 0:
        sys.exit('adb: ' + out.stderr.strip())
    for name in out.stdout.split():
        if os.path.isfile(os.path.join(LOCAL, name, 'meta.tsv')):
            local = os.path.getsize(os.path.join(LOCAL, name, 'frames.bin')) if os.path.isfile(os.path.join(LOCAL, name, 'frames.bin')) else -1
            remote = subprocess.run([adb(), 'shell', 'stat', '-c', '%s', REMOTE + '/' + name + '/frames.bin'], capture_output=True, text=True).stdout.strip()
            if remote.isdigit() and int(remote) == local:
                continue
        subprocess.run([adb(), 'pull', REMOTE + '/' + name, LOCAL], capture_output=True)
        print('pulled', name)


def cmd_list(_):
    for r in sorted(os.listdir(LOCAL)) if os.path.isdir(LOCAL) else []:
        path = os.path.join(LOCAL, r, 'frames.bin')
        print(r, '%.1f MB' % (os.path.getsize(path) / 1e6) if os.path.isfile(path) else '')


def cmd_channels(a):
    run = Run(find(a.run))
    for name, unit, what in run.channels:
        print('%-34s %-10s %s' % (name, unit, what))
    print('%d channels, %d rows' % (run.n, run.rows))


def cmd_events(a):
    for t, source, what, value in Run(find(a.run)).events():
        print('%8.3f  %-8s %s %s' % (t, source, what, '' if value in ('NaN', '') else value))


def cmd_meta(a):
    for k, v in Run(find(a.run)).meta().items():
        if not a.filter or a.filter.lower() in k.lower():
            print('%s = %s' % (k, v))


def cmd_csv(a):
    run = Run(find(a.run))
    names = a.c.split(',') if a.c else [c[0] for c in run.channels]
    cols = [run.col(n) for n in names]
    out = open(a.o, 'w', encoding='utf-8') if a.o else sys.stdout
    out.write(','.join(names) + '\n')
    for i in range(run.rows):
        out.write(','.join(repr(c[i]) for c in cols) + '\n')
    if a.o:
        out.close()
        print('wrote', a.o)


def cmd_summary(a):
    run = Run(find(a.run))
    meta = run.meta()
    t = run.col('time.t')
    print('%s: %s, %d rows, %.0f s, %d channels' % (os.path.basename(run.folder), meta.get('opmode', '?'), run.rows, t[-1] if run.rows else 0, run.n))
    print('start:', meta.get('start_from', '?'))
    if run.has('loop.loop_ms'):
        lm = run.col('loop.loop_ms')
        print('loop: median %.1f ms, p95 %.1f, p99 %.1f, worst %.1f' % (pct(lm, .5), pct(lm, .95), pct(lm, .99), pct(lm, 1)))
    if run.has('power.battery_v'):
        b = run.col('power.battery_v')
        print('battery: %.2f V at start, lowest %.2f V' % (next((x for x in b if finite(x) and x > 0), float('nan')), pct([x for x in b if x > 0], 0)))
    events = run.events()
    kinds = {}
    for e in events:
        kinds[e[2].split(' to')[0].split('=')[0]] = kinds.get(e[2].split(' to')[0].split('=')[0], 0) + 1
    if kinds:
        print('events:', ', '.join('%s x%d' % kv for kv in sorted(kinds.items())))

    if run.has('turret.error'):
        err = run.col('turret.error')
        om = run.col('pose.omega') if run.has('pose.omega') else None
        aiming = run.col('shooter.auto_aim') if run.has('shooter.auto_aim') else None
        locked = run.col('shooter.locked') if run.has('shooter.locked') else None
        usable = run.col('shooter.aim_usable') if run.has('shooter.aim_usable') else None
        print('turret error by how fast the robot turns (aiming, inside the travel):')
        for lo, hi in ((0, 10), (10, 60), (60, 150), (150, 1e9)):
            s = [abs(err[i]) for i in range(run.rows) if finite(err[i]) and (om is None or lo <= abs(om[i]) < hi)
                 and (aiming is None or aiming[i] == 1) and (locked is None or locked[i] == 0) and (usable is None or usable[i] == 1)]
            if s:
                print('  %4d-%-5s deg/s: %5d rows, median %.1f deg, p90 %.1f' % (lo, '' if hi > 1e8 else int(hi), len(s), pct(s, .5), pct(s, .9)))
        target = run.col('turret.target')
        pos = run.col('turret.position')
        settle = []
        i = 0
        while i < run.rows - 1:
            j = i
            while j < run.rows and abs(target[j] - target[i]) < 0.7 and t[j] - t[i] < 1.2:
                j += 1
            if j < run.rows and t[j] - t[i] >= 1.2:
                k = i
                while k > 0 and t[i] - t[k] < 1.0:
                    k -= 1
                if abs(target[k] - target[i]) > 5 and abs(target[i] - pos[i]) > 2:
                    row = []
                    for dt in (0.25, 0.5, 1.0):
                        m = i
                        while m < run.rows and t[m] - t[i] < dt:
                            m += 1
                        row.append(abs(target[min(m, run.rows - 1)] - pos[min(m, run.rows - 1)]))
                    settle.append(row)
                    i = j
                    continue
            i += 1
        if settle:
            print('after the aim stops moving (%d times): off %.1f / %.1f / %.1f deg (median) at 0.25 / 0.5 / 1 s' % (len(settle), *[statistics.median(r[k] for r in settle) for k in range(3)]))

    if run.has('blocker.clear'):
        clear = run.col('blocker.clear')
        shots = [i for i in range(1, run.rows) if clear[i] == 1 and clear[i - 1] == 0]
        print('shots: %d' % len(shots))
        for i in shots:
            def v(name):
                return run.col(name)[i] if run.has(name) else float('nan')
            j = i
            aim = run.col('shooter.aim') if run.has('shooter.aim') else None
            while aim is not None and j > 0 and abs(aim[j] - aim[i]) < 2:
                j -= 1
            k = i
            while k < run.rows and clear[k] == 1:
                k += 1
            print('  %6.1f s  turret off %5.1f deg (%s)  aim still %.1f s  flywheel %5.0f / %5.0f  robot %3.0f in/s %4.0f deg/s  open %.2f s  dist %3.0f in' % (
                t[i], v('turret.error'), 'on target' if v('turret.on_target') == 1 else 'NOT on target', t[i] - t[j] if aim is not None else float('nan'),
                v('flywheel.velocity'), v('flywheel.target'), v('pose.speed'), v('pose.omega'), t[min(k, run.rows - 1)] - t[i], v('shooter.distance')))

    if run.has('flywheel.error') and run.has('shooter.on'):
        fe = run.col('flywheel.error')
        on = run.col('shooter.on')
        s = [abs(fe[i]) for i in range(run.rows) if on[i] == 1 and finite(fe[i])]
        if s:
            print('flywheel while on: error median %.0f ticks/s, p90 %.0f' % (pct(s, .5), pct(s, .9)))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='cmd', required=True)
    sub.add_parser('pull').set_defaults(f=cmd_pull)
    sub.add_parser('list').set_defaults(f=cmd_list)
    for name, f in (('channels', cmd_channels), ('events', cmd_events), ('summary', cmd_summary)):
        s = sub.add_parser(name)
        s.add_argument('run')
        s.set_defaults(f=f)
    s = sub.add_parser('meta')
    s.add_argument('run')
    s.add_argument('filter', nargs='?')
    s.set_defaults(f=cmd_meta)
    s = sub.add_parser('csv')
    s.add_argument('run')
    s.add_argument('-c')
    s.add_argument('-o')
    s.set_defaults(f=cmd_csv)
    a = p.parse_args()
    a.f(a)


if __name__ == '__main__':
    main()
