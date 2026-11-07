#!/usr/bin/env python3
"""Read Temporal Log sessions (TeamCode/Utility/TemporalLog): time -> every read, write, variable change and decision.

  python tools/temporal_log.py pull                          adb pull every session into temporal-logs/
  python tools/temporal_log.py summary  SESSION              what's in it: loops, loop time, busiest keys, drops
  python tools/temporal_log.py csv      SESSION [-o out.csv] [--grep REGEX] [--kinds read,write,var,...] [--from S] [--to S]
  python tools/temporal_log.py decisions SESSION [--grep REGEX] [--from S] [--to S]
  python tools/temporal_log.py keys     SESSION [--grep REGEX]

SESSION is a session folder (events.bin + keys.tsv), or "latest" for the newest one in temporal-logs/.
csv columns: t_s, kind, name, value, label, duration_us, flag (cached read / redundant write).
decisions = every change of a boolean or enum variable (shooter on, intake reverse, blocker open, turret route swing, ...),
plus the phase marks, as "12.345 s  teleop.shooter.shooterToggle -> true".
"""
import argparse
import csv
import os
import re
import struct
import subprocess
import sys

RECORD = struct.Struct("<qiid")
FLAG = 1 << 30
REMOTE = "/sdcard/FIRST/data/TemporalLog"
LOCAL = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "temporal-logs")


def adb():
    sdk = os.environ.get("ANDROID_HOME") or os.path.join(os.environ.get("LOCALAPPDATA", ""), "Android", "Sdk")
    exe = os.path.join(sdk, "platform-tools", "adb.exe" if os.name == "nt" else "adb")
    return exe if os.path.exists(exe) else "adb"


def resolve(session):
    if session != "latest":
        return session
    if not os.path.isdir(LOCAL):
        sys.exit("no temporal-logs/ yet: run pull first")
    runs = sorted(d for d in os.listdir(LOCAL) if os.path.isfile(os.path.join(LOCAL, d, "events.bin")))
    if not runs:
        sys.exit("no sessions in temporal-logs/")
    return os.path.join(LOCAL, runs[-1])


def load_keys(session):
    keys = {}
    with open(os.path.join(session, "keys.tsv"), encoding="utf-8") as f:
        next(f, None)
        for line in f:
            parts = line.rstrip("\n").split("\t")
            if len(parts) < 3 or not parts[0].isdigit():
                continue
            labels = {}
            if len(parts) > 3 and parts[3]:
                for item in parts[3].split("|"):
                    k, _, v = item.partition("=")
                    if k.lstrip("-").isdigit():
                        labels[int(k)] = v
            keys[int(parts[0])] = (parts[1], parts[2], labels)
    return keys


def records(session):
    """every record as (t_s, key, aux, value), sorted by time (Blaze's background reads land a loop late in the file)"""
    out = []
    with open(os.path.join(session, "events.bin"), "rb") as f:
        data = f.read()
    usable = len(data) - len(data) % RECORD.size
    for t, key, aux, value in RECORD.iter_unpack(data[:usable]):
        out.append((t * 1e-9, key, aux, value))
    out.sort(key=lambda r: r[0])
    return out


def label(labels, value):
    if value != value:
        return ""
    if labels and float(value).is_integer():
        return labels.get(int(value), "")
    return ""


def fmt(v):
    if v != v:
        return "NaN"
    return repr(int(v)) if float(v).is_integer() and abs(v) < 1e15 else "%.9g" % v


def selected(rows, keys, grep, kinds, t0, t1):
    rx = re.compile(grep) if grep else None
    for t, key, aux, value in rows:
        kind, name, labels = keys.get(key, ("?", "key%d" % key, {}))
        if kinds and kind not in kinds:
            continue
        if rx and not rx.search(name):
            continue
        if t0 is not None and t < t0:
            continue
        if t1 is not None and t > t1:
            continue
        yield t, kind, name, labels, aux, value


def cmd_pull(args):
    os.makedirs(LOCAL, exist_ok=True)
    listing = subprocess.run([adb(), "shell", "ls", REMOTE], capture_output=True, text=True)
    if listing.returncode != 0:
        sys.exit("adb couldn't list %s: %s" % (REMOTE, listing.stderr.strip()))
    for name in listing.stdout.split():
        if os.path.exists(os.path.join(LOCAL, name, "events.bin")):
            continue
        print("pulling", name)
        subprocess.run([adb(), "pull", REMOTE + "/" + name, os.path.join(LOCAL, name)], check=False)


def cmd_csv(args):
    session = resolve(args.session)
    keys = load_keys(session)
    kinds = set(args.kinds.split(",")) if args.kinds else None
    out = open(args.out, "w", newline="", encoding="utf-8") if args.out else sys.stdout
    w = csv.writer(out)
    w.writerow(["t_s", "kind", "name", "value", "label", "duration_us", "flag"])
    for t, kind, name, labels, aux, value in selected(records(session), keys, args.grep, kinds, args.t0, args.t1):
        io = kind in ("read", "write")
        w.writerow(["%.6f" % t, kind, name, fmt(value), label(labels, value), aux & (FLAG - 1) if io else "", (1 if aux & FLAG else 0) if io else (aux if kind == "loop" else "")])
    if args.out:
        out.close()
        print("wrote", args.out)


def cmd_decisions(args):
    session = resolve(args.session)
    keys = load_keys(session)
    last = {}
    for t, kind, name, labels, aux, value in selected(records(session), keys, args.grep, {"var", "phase"}, args.t0, args.t1):
        if kind == "phase":
            print("%10.3f s  == %s" % (t, name))
            continue
        if not labels:
            continue
        before = last.get(name)
        last[name] = value
        if before is None:
            continue
        print("%10.3f s  %s -> %s" % (t, name, label(labels, value) or fmt(value)))


def cmd_summary(args):
    session = resolve(args.session)
    keys = load_keys(session)
    rows = records(session)
    if not rows:
        sys.exit("empty")
    counts = {}
    loop_dt = []
    io_us = {}
    for t, key, aux, value in rows:
        counts[key] = counts.get(key, 0) + 1
        kind = keys.get(key, ("?",))[0]
        if kind == "loop" and value == value and aux > 0:
            loop_dt.append(value)
        if kind in ("read", "write"):
            io_us.setdefault(key, []).append(aux & (FLAG - 1))
    span = rows[-1][0] - rows[0][0]
    print("session  ", session)
    print("records  ", len(rows), "over %.1f s" % span, "(%d keys)" % len(keys))
    if loop_dt:
        loop_dt.sort()
        q = lambda p: loop_dt[min(len(loop_dt) - 1, int(p * len(loop_dt)))] * 1000
        print("loops    ", len(loop_dt) + 1, "loop ms: median %.1f  p95 %.1f  p99 %.1f  max %.1f" % (q(0.5), q(0.95), q(0.99), loop_dt[-1] * 1000))
    by_kind = {}
    for key, n in counts.items():
        kind = keys.get(key, ("?",))[0]
        by_kind[kind] = by_kind.get(kind, 0) + n
    print("by kind  ", ", ".join("%s %d" % kv for kv in sorted(by_kind.items(), key=lambda kv: -kv[1])))
    print("\nslowest hardware calls (mean us, count):")
    slow = sorted(((sum(v) / len(v), len(v), keys[k][1]) for k, v in io_us.items() if k in keys), reverse=True)[:15]
    for mean, n, name in slow:
        print("  %8.0f  %7d  %s" % (mean, n, name))
    print("\nbusiest keys:")
    for key, n in sorted(counts.items(), key=lambda kv: -kv[1])[:20]:
        print("  %8d  %s" % (n, keys.get(key, ("?", "key%d" % key))[1]))
    for key, (kind, name, _) in keys.items():
        if name in ("temporal.dropped_records", "blaze.dropped_samples") and counts.get(key):
            print("\nWARNING:", name, "was logged: some records were lost")


def cmd_keys(args):
    session = resolve(args.session)
    rx = re.compile(args.grep) if args.grep else None
    for key, (kind, name, labels) in sorted(load_keys(session).items()):
        if rx and not rx.search(name):
            continue
        print("%6d  %-6s %s%s" % (key, kind, name, ("  {" + ", ".join("%d=%s" % kv for kv in sorted(labels.items())) + "}") if labels else ""))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("pull").set_defaults(fn=cmd_pull)
    for name, fn in (("csv", cmd_csv), ("decisions", cmd_decisions), ("summary", cmd_summary), ("keys", cmd_keys)):
        s = sub.add_parser(name)
        s.add_argument("session")
        s.add_argument("--grep")
        if name in ("csv", "decisions"):
            s.add_argument("--from", dest="t0", type=float)
            s.add_argument("--to", dest="t1", type=float)
        if name == "csv":
            s.add_argument("-o", "--out")
            s.add_argument("--kinds", help="comma list of read, write, var, loop, blaze, phase, meta")
        s.set_defaults(fn=fn)
    args = p.parse_args()
    args.fn(args)


if __name__ == "__main__":
    main()
