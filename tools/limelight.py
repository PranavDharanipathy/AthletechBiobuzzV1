#!/usr/bin/env python3
"""Talk to the Limelight 3A from the laptop through its REST API (port 5807). Standard library only.

Reach it one of two ways:
  - Limelight plugged into the laptop by USB-C: host limelight.local
  - Limelight on the Control Hub, laptop on the robot's Wi-Fi, "Limelight Lab" OpMode running (it forwards
    ports 5800-5809): host 192.168.43.1
With no --host it tries both (plus the USB addresses) and uses the first that answers.

Pipelines and SnapScripts live in TeamCode/src/main/assets/limelight as p<index>_<name>.json / p<index>_<name>.py,
so they're versioned in git, ship in the APK (the Lab OpMode can upload them), and push from here.

  python tools/limelight.py find                         which host answers, plus status
  python tools/limelight.py status | results [--raw]
  python tools/limelight.py watch [--hz 10]              one live line: pipeline, valid, tx, ty, ta, fps, python out
  python tools/limelight.py switch 9
  python tools/limelight.py get [filter] [--index N]     pipeline settings (current pipeline by default)
  python tools/limelight.py set exposure=1200 sensor_gain=15 [--save]   live change; --save writes it to disk
  python tools/limelight.py sweep exposure 300 3000 300 [--settle 0.6] [--samples 15]   restores the value after
  python tools/limelight.py record 20 [--out tools/out/x.csv]           log results while you move the target
  python tools/limelight.py inputs 100 120 60 130 255 255 1 0           llrobot for SnapScripts
  python tools/limelight.py pull 0|all                   camera pipeline(s) -> assets json
  python tools/limelight.py push [files...] [--yes]      assets -> camera (backs up the camera's pipelines first)
  python tools/limelight.py snap NAME | snaps
  python tools/limelight.py urls                         web UI / stream / REST links
Setting names: run `get` to see the real keys before `set` or `sweep`.
"""
import argparse
import csv
import datetime
import json
import math
import os
import re
import socket
import sys
import time
import urllib.error
import urllib.request

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ASSETS = os.path.join(ROOT, 'TeamCode', 'src', 'main', 'assets', 'limelight')
OUT = os.path.join(ROOT, 'tools', 'out')
HOSTS = ['192.168.43.1', 'limelight.local', '172.29.0.1', '172.28.0.1']
REST = 5807
ASSET = re.compile(r'p(\d)_.+\.(py|json)$')


class Limelight:
    def __init__(self, host, timeout=3.0):
        self.host = host
        self.timeout = timeout

    def url(self, path):
        return 'http://%s:%d%s' % (self.host, REST, path)

    def request(self, method, path, body=None, content_type='application/json'):
        data = None
        if body is not None:
            data = body if isinstance(body, bytes) else body.encode('utf-8')
        req = urllib.request.Request(self.url(path), data=data, method=method)
        if data is not None:
            req.add_header('Content-Type', content_type)
        with urllib.request.urlopen(req, timeout=self.timeout) as r:
            raw = r.read()
        text = str(raw, 'utf-8', 'replace').strip()
        if not text:
            return None
        try:
            return json.loads(text)
        except ValueError:
            return text

    def get(self, path):
        return self.request('GET', path)

    def post(self, path, body=None, content_type='application/json'):
        return self.request('POST', path, body if body is not None else '', content_type)

    def status(self):
        return self.get('/status')

    def results(self):
        r = self.get('/results')
        if isinstance(r, dict) and 'Results' in r and isinstance(r['Results'], dict):
            return r['Results']
        return r if isinstance(r, dict) else {}

    def current_index(self):
        s = self.status()
        if isinstance(s, dict) and 'pipelineIndex' in s:
            return int(s['pipelineIndex'])
        return int(self.results().get('pID', 0))

    def pipeline(self, index):
        return self.get('/pipeline-atindex?index=%d' % index)

    def update(self, settings, save=False):
        return self.post('/update-pipeline' + ('?flush=1' if save else ''), json.dumps(settings))


def answers(host, timeout):
    try:
        socket.getaddrinfo(host, REST)
        s = Limelight(host, timeout).status()
        return isinstance(s, dict)
    except Exception:
        return False


def connect(args):
    hosts = [args.host] if args.host else ([os.environ['LIMELIGHT_HOST']] if os.environ.get('LIMELIGHT_HOST') else HOSTS)
    for h in hosts:
        if answers(h, 1.5):
            return Limelight(h, args.timeout)
    sys.exit('No Limelight answered on %s (port %d). USB: plug it into the laptop. Robot: join its Wi-Fi and run '
             'the "Limelight Lab" OpMode.' % (', '.join(hosts), REST))


def num(v, digits=2):
    try:
        return ('%.' + str(digits) + 'f') % float(v)
    except (TypeError, ValueError):
        return '-'


def summary(r, s=None):
    py = r.get('PythonOut') or []
    parts = [
        'p%s' % r.get('pID', '?'),
        'valid' if r.get('v') == 1 else 'none ',
        'tx %7s' % num(r.get('tx')),
        'ty %7s' % num(r.get('ty')),
        'ta %6s' % num(r.get('ta'), 3),
        'lat %5s ms' % num((r.get('cl') or 0) + (r.get('tl') or 0), 1),
    ]
    for key, label in (('Retro', 'color'), ('Fiducial', 'tags'), ('Detector', 'det')):
        if r.get(key):
            parts.append('%s %d' % (label, len(r[key])))
    det = r.get('Detector') or []
    if det:
        best = max(det, key=lambda d: d.get('conf', 0))
        parts.append('%s %.2f' % (best.get('class', '?'), best.get('conf', 0)))
    if s:
        parts.append('%s fps' % num(s.get('fps'), 0))
        parts.append('%s C' % num(s.get('temp'), 0))
    if py:
        parts.append('py ' + ' '.join(num(v) for v in py[:8]))
    return '  '.join(parts)


def parse_value(text):
    try:
        return json.loads(text)
    except ValueError:
        return text


def asset_files(names=None):
    if names:
        return [os.path.abspath(n) for n in names]
    if not os.path.isdir(ASSETS):
        return []
    return [os.path.join(ASSETS, n) for n in sorted(os.listdir(ASSETS)) if ASSET.match(n)]


def stamp():
    return datetime.datetime.now().strftime('%Y%m%d_%H%M%S')


def cmd_find(args):
    hosts = [args.host] if args.host else HOSTS
    found = None
    for h in hosts:
        ok = answers(h, 1.5)
        print('%-16s %s' % (h, 'answers' if ok else '-'))
        if ok and not found:
            found = h
    if found:
        print(json.dumps(Limelight(found).status(), indent=1))


def cmd_status(args):
    print(json.dumps(connect(args).status(), indent=1))


def cmd_results(args):
    ll = connect(args)
    r = ll.results()
    print(json.dumps(r, indent=1) if args.raw else summary(r, ll.status()))


def cmd_watch(args):
    ll = connect(args)
    period = 1.0 / max(args.hz, 0.5)
    s, s_time = None, 0
    try:
        while True:
            now = time.time()
            if now - s_time > 1:
                s, s_time = ll.status(), now
            line = summary(ll.results(), s)
            sys.stdout.write('\r' + line[:180].ljust(180))
            sys.stdout.flush()
            time.sleep(max(0, period - (time.time() - now)))
    except KeyboardInterrupt:
        print()


def cmd_switch(args):
    ll = connect(args)
    ll.post('/pipeline-switch?index=%d' % args.index)
    for _ in range(20):
        if ll.current_index() == args.index:
            print('on pipeline %d' % args.index)
            return
        time.sleep(0.1)
    print('switch sent, camera still reports pipeline %d' % ll.current_index())


def cmd_get(args):
    ll = connect(args)
    index = args.index if args.index is not None else ll.current_index()
    p = ll.pipeline(index)
    if not isinstance(p, dict):
        print(p)
        return
    print('pipeline %d' % index)
    for k in sorted(p):
        if args.filter and args.filter.lower() not in k.lower():
            continue
        v = p[k]
        text = json.dumps(v)
        print('  %-32s %s' % (k, text if len(text) < 100 else text[:97] + '...'))


def cmd_set(args):
    ll = connect(args)
    settings = {}
    for pair in args.pairs:
        if '=' not in pair:
            sys.exit('expected key=value, got ' + pair)
        k, v = pair.split('=', 1)
        settings[k] = parse_value(v)
    ll.update(settings, args.save)
    print(('saved ' if args.save else 'live (not saved) ') + json.dumps(settings))


def sample(ll, n, period=0.05):
    rows = []
    for _ in range(n):
        rows.append(ll.results())
        time.sleep(period)
    return rows


def stats(rows):
    valid = [r for r in rows if r.get('v') == 1]
    out = {'valid': len(valid) / max(len(rows), 1)}
    for key in ('tx', 'ty', 'ta'):
        vals = [float(r.get(key, 0)) for r in valid]
        if vals:
            mean = sum(vals) / len(vals)
            out[key] = mean
            out[key + '_sd'] = math.sqrt(sum((v - mean) ** 2 for v in vals) / len(vals))
    return out


def cmd_sweep(args):
    ll = connect(args)
    index = ll.current_index()
    before = ll.pipeline(index)
    original = before.get(args.key) if isinstance(before, dict) else None
    if original is None:
        print('warning: %s not in pipeline %d settings (run `get`), sweeping anyway' % (args.key, index))
    values = []
    v = args.start
    while (args.step > 0 and v <= args.stop + 1e-9) or (args.step < 0 and v >= args.stop - 1e-9):
        values.append(round(v, 6))
        v += args.step
    print('%-10s %6s %8s %7s %8s %7s %8s' % (args.key, 'valid', 'tx', 'tx_sd', 'ty', 'ty_sd', 'ta'))
    try:
        for v in values:
            ll.update({args.key: int(v) if float(v).is_integer() else v})
            time.sleep(args.settle)
            s = stats(sample(ll, args.samples))
            print('%-10s %5.0f%% %8s %7s %8s %7s %8s' % ('%g' % v, 100 * s['valid'], num(s.get('tx')), num(s.get('tx_sd')),
                                                     num(s.get('ty')), num(s.get('ty_sd')), num(s.get('ta'), 3)))
    finally:
        if original is not None:
            ll.update({args.key: original})
            print('restored %s = %s (live, not saved)' % (args.key, json.dumps(original)))


def cmd_record(args):
    ll = connect(args)
    os.makedirs(OUT, exist_ok=True)
    path = args.out or os.path.join(OUT, 'limelight_record_%s.csv' % stamp())
    rows = []
    end = time.time() + args.seconds
    print('recording %.0f s, move the target around now' % args.seconds)
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'pipeline', 'valid', 'tx', 'ty', 'ta', 'latency_ms', 'blobs', 'tags', 'detections', 'python'])
        t0 = time.time()
        while time.time() < end:
            r = ll.results()
            rows.append(r)
            w.writerow(['%.3f' % (time.time() - t0), r.get('pID'), r.get('v'), r.get('tx'), r.get('ty'), r.get('ta'),
                        (r.get('cl') or 0) + (r.get('tl') or 0), len(r.get('Retro') or []), len(r.get('Fiducial') or []),
                        len(r.get('Detector') or []), ' '.join(num(v, 3) for v in (r.get('PythonOut') or []))])
            sys.stdout.write('\r' + summary(r)[:150].ljust(150))
            sys.stdout.flush()
            time.sleep(1.0 / args.hz)
    print()
    s = stats(rows)
    print('%d samples -> %s' % (len(rows), path))
    print('valid %.0f%%  tx %s (sd %s)  ty %s (sd %s)  ta %s' % (100 * s['valid'], num(s.get('tx')), num(s.get('tx_sd')),
                                                              num(s.get('ty')), num(s.get('ty_sd')), num(s.get('ta'), 3)))
    drops, run = 0, 0
    for r in rows:
        if r.get('v') == 1:
            run = 0
        else:
            run += 1
            if run == 1:
                drops += 1
    print('target lost %d time(s)' % drops)


def cmd_inputs(args):
    ll = connect(args)
    ll.post('/update-pythoninputs', json.dumps(args.values))
    print('llrobot = %s' % args.values)


def cmd_pull(args):
    ll = connect(args)
    os.makedirs(ASSETS, exist_ok=True)
    indexes = range(10) if args.index == 'all' else [int(args.index)]
    for i in indexes:
        p = ll.pipeline(i)
        if not isinstance(p, dict):
            print('pipeline %d: nothing' % i)
            continue
        existing = [n for n in os.listdir(ASSETS) if n.startswith('p%d_' % i) and n.endswith('.json')]
        name = existing[0] if existing else 'p%d_%s.json' % (i, re.sub(r'[^a-z0-9]+', '_', str(p.get('desc', 'pipeline')).lower()).strip('_') or 'pipeline')
        with open(os.path.join(ASSETS, name), 'w', newline='\n') as f:
            json.dump(p, f, indent=1, sort_keys=True)
            f.write('\n')
        print('pipeline %d -> %s' % (i, os.path.relpath(os.path.join(ASSETS, name), ROOT)))


def cmd_push(args):
    ll = connect(args)
    files = asset_files(args.files)
    if not files:
        sys.exit('nothing to push in ' + ASSETS)
    plan = []
    for path in files:
        m = ASSET.match(os.path.basename(path))
        if not m:
            sys.exit('%s: name it p<index>_<name>.py or .json' % path)
        plan.append((int(m.group(1)), m.group(2), path))
    plan.sort(key=lambda p: (p[1] != 'json', p[0]))
    for index, kind, path in plan:
        print('  pipeline %d <- %s' % (index, os.path.relpath(path, ROOT)))
    if not args.yes and input('Overwrite those pipelines on the camera? [y/N] ').strip().lower() != 'y':
        print('cancelled')
        return
    backup = os.path.join(OUT, 'limelight_backup_%s' % stamp())
    os.makedirs(backup, exist_ok=True)
    for index in sorted(set(p[0] for p in plan)):
        p = ll.pipeline(index)
        with open(os.path.join(backup, 'p%d.json' % index), 'w') as f:
            json.dump(p, f, indent=1, sort_keys=True)
    print('backed up camera pipelines to %s' % os.path.relpath(backup, ROOT))
    current = ll.current_index()
    for index, kind, path in plan:
        with open(path, 'rb') as f:
            body = f.read()
        if kind == 'json':
            json.loads(str(body, 'utf-8'))
            ll.post('/upload-pipeline?index=%d' % index, body)
        else:
            ll.post('/upload-python?index=%d' % index, body, 'text/plain')
        print('  uploaded %s' % os.path.basename(path))
    ll.post('/pipeline-switch?index=%d' % current)
    ll.post('/reload-pipeline')
    print('reloaded pipeline %d' % current)


def cmd_snap(args):
    connect(args).post('/capture-snapshot?snapname=%s' % urllib.request.quote(args.name))
    print('snapshot ' + args.name)


def cmd_snaps(args):
    print(json.dumps(connect(args).get('/snapshotmanifest'), indent=1))


def cmd_urls(args):
    ll = connect(args)
    print('web UI   http://%s:5801' % ll.host)
    print('stream   http://%s:5800' % ll.host)
    print('REST     http://%s:%d/results' % (ll.host, REST))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', help='Limelight or bridge address (default: try %s)' % ', '.join(HOSTS))
    ap.add_argument('--timeout', type=float, default=3.0)
    sub = ap.add_subparsers(dest='cmd')
    sub.required = True
    sub.add_parser('find').set_defaults(fn=cmd_find)
    sub.add_parser('status').set_defaults(fn=cmd_status)
    p = sub.add_parser('results')
    p.add_argument('--raw', action='store_true')
    p.set_defaults(fn=cmd_results)
    p = sub.add_parser('watch')
    p.add_argument('--hz', type=float, default=10)
    p.set_defaults(fn=cmd_watch)
    p = sub.add_parser('switch')
    p.add_argument('index', type=int)
    p.set_defaults(fn=cmd_switch)
    p = sub.add_parser('get')
    p.add_argument('filter', nargs='?')
    p.add_argument('--index', type=int)
    p.set_defaults(fn=cmd_get)
    p = sub.add_parser('set')
    p.add_argument('pairs', nargs='+')
    p.add_argument('--save', action='store_true')
    p.set_defaults(fn=cmd_set)
    p = sub.add_parser('sweep')
    p.add_argument('key')
    p.add_argument('start', type=float)
    p.add_argument('stop', type=float)
    p.add_argument('step', type=float)
    p.add_argument('--settle', type=float, default=0.6)
    p.add_argument('--samples', type=int, default=15)
    p.set_defaults(fn=cmd_sweep)
    p = sub.add_parser('record')
    p.add_argument('seconds', type=float)
    p.add_argument('--hz', type=float, default=20)
    p.add_argument('--out')
    p.set_defaults(fn=cmd_record)
    p = sub.add_parser('inputs')
    p.add_argument('values', type=float, nargs='+')
    p.set_defaults(fn=cmd_inputs)
    p = sub.add_parser('pull')
    p.add_argument('index')
    p.set_defaults(fn=cmd_pull)
    p = sub.add_parser('push')
    p.add_argument('files', nargs='*')
    p.add_argument('--yes', action='store_true')
    p.set_defaults(fn=cmd_push)
    p = sub.add_parser('snap')
    p.add_argument('name')
    p.set_defaults(fn=cmd_snap)
    sub.add_parser('snaps').set_defaults(fn=cmd_snaps)
    sub.add_parser('urls').set_defaults(fn=cmd_urls)
    args = ap.parse_args()
    if args.cmd == 'sweep' and args.step == 0:
        sys.exit('step must not be 0')
    try:
        args.fn(args)
    except urllib.error.HTTPError as e:
        sys.exit('HTTP %d from %s' % (e.code, e.url))
    except urllib.error.URLError as e:
        sys.exit('could not reach the Limelight: %s' % e.reason)


if __name__ == '__main__':
    main()
