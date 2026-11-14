#!/usr/bin/env python3
"""Match video + robot logs -> which balls scored -> small flywheel table / flywheel tuning changes.

A phone video (any spot, any phone, variable frame rate is fine) is lined up with the robot's recordings
(tuning-data/robot-logs/<run>/, every OpMode writes one, the Temporal Log TeleOp too). Each ball is found in the log
from its flywheel dip, shown in the video by its own timestamps, marked score/short/long/left/right by whoever watches
it, and every marked ball from every review fits a correction to FLYWHEEL_VELOCITIES. The steps are in docs/VIDEO_SHOT_REVIEW.md.

  python -I tools/shot_video.py new NAME VIDEO RUN [RUN ...]    start tuning-data/video-reviews/NAME (RUN as robot_log.py takes it)
  python -I tools/shot_video.py balls NAME                       balls found in each run's log (+ video time once synced)
  python -I tools/shot_video.py audio NAME                       sound onsets (launch thumps) + a picture against the log's balls
  python -I tools/shot_video.py motion NAME --roi X0,Y0,X1,Y1    how much the picture changes in a box (the goal) over time
  python -I tools/shot_video.py sheet NAME --from S --to S [--fps 4] [--roi ...]   grid of frames, each labeled with its time
  python -I tools/shot_video.py frame NAME S [--roi ...]          one frame, full size
  python -I tools/shot_video.py sync NAME --run RUN (--events T,T,... | --audio | --anchor VIDEO_S=LOG)
                                                                  LOG = seconds, 'move' (first drive), 'ball:N', 'open:N'
  python -I tools/shot_video.py shot NAME BALL [--roi ...] [--before 0.25] [--after 1.6] [--step 1]   every frame of one shot
  python -I tools/shot_video.py mark NAME BALL OUTCOME [--conf 1] [--note ...]     score|short|long|left|right|rim|blocked|none|unknown
  python -I tools/shot_video.py fit [NAME ...] [--apply] [--max-step 25]          all marked balls -> table change + flywheel checks
  python -I tools/shot_video.py report NAME                      writes the review's REPORT.md

Times: video seconds are what a player shows (first frame = 0). video = log + offset, one offset per run.
"""
import argparse
import csv
import datetime
import json
import math
import os
import re
import subprocess
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True
import robot_log  # noqa: E402

ROOT = os.path.dirname(HERE)
REVIEWS = os.path.join(ROOT, 'tuning-data', 'video-reviews')
SHOOTER = os.path.join(ROOT, 'TeamCode', 'src', 'main', 'java', 'org', 'firstinspires', 'ftc', 'teamcode', 'Constants', 'ShooterConstants.java')
OUTCOMES = ('score', 'short', 'long', 'left', 'right', 'rim', 'blocked', 'none', 'unknown')
FIELDS = ['ball', 'run', 'log_t', 'video_t', 'burst', 'in_burst', 'x', 'y', 'heading', 'turret_x', 'turret_y', 'distance', 'goal_distance', 'cell',
          'target', 'speed_at_contact', 'raw_at_contact', 'dip', 'lowest', 'target_minus_speed', 'battery', 'robot_speed', 'omega', 'turret_error', 'aim_ready',
          'outcome', 'conf', 'note']


def ffmpeg(name):
    return name + ('.exe' if os.name == 'nt' else '')


def folder(name):
    return os.path.join(REVIEWS, name)


def load(name):
    path = os.path.join(folder(name), 'review.json')
    if not os.path.isfile(path):
        sys.exit('no review %r (make it with: new %s VIDEO RUN)' % (name, name))
    with open(path, encoding='utf-8') as f:
        return json.load(f)


def save(name, review):
    with open(os.path.join(folder(name), 'review.json'), 'w', encoding='utf-8') as f:
        json.dump(review, f, indent=2)


def font(size):
    try:
        return ImageFont.load_default(size=size)
    except TypeError:
        return ImageFont.load_default()


# ---------- video ----------

def probe(path):
    out = subprocess.run([ffmpeg('ffprobe'), '-v', 'error', '-print_format', 'json', '-show_format', '-show_streams', path], capture_output=True, text=True)
    if out.returncode != 0:
        sys.exit('ffprobe: ' + out.stderr.strip())
    info = json.loads(out.stdout)
    video = next((s for s in info['streams'] if s.get('codec_type') == 'video'), None)
    audio = next((s for s in info['streams'] if s.get('codec_type') == 'audio'), None)
    if video is None:
        sys.exit('no video stream in ' + path)
    rotation = 0
    for side in video.get('side_data_list', []):
        if 'rotation' in side:
            rotation = int(side['rotation'])
    if 'rotate' in video.get('tags', {}):
        rotation = int(video['tags']['rotate'])
    w, h = int(video['width']), int(video['height'])
    if abs(rotation) % 180 == 90:
        w, h = h, w
    # frame times straight from the packets: phones change frame rate mid-video
    pts = subprocess.run([ffmpeg('ffprobe'), '-v', 'error', '-select_streams', 'v:0', '-show_entries', 'packet=pts_time', '-of', 'csv=p=0', path], capture_output=True, text=True).stdout.split()
    times = np.sort(np.array([float(p) for p in pts if p and p != 'N/A']))
    start = float(video.get('start_time', 0) or 0)
    gaps = np.diff(times) if len(times) > 1 else np.array([0.0])
    tags = info['format'].get('tags', {})
    return {
        'path': os.path.abspath(path),
        'width': w,
        'height': h,
        'rotation': rotation,
        'start_time': start,
        'duration': float(info['format'].get('duration', 0)),
        'frames': int(len(times)),
        'fps_median': round(1 / float(np.median(gaps)), 3) if len(times) > 1 else None,
        'frame_gap_ms': {'min': round(1000 * float(gaps.min()), 2), 'median': round(1000 * float(np.median(gaps)), 2), 'max': round(1000 * float(gaps.max()), 2)},
        'variable_frame_rate': bool(len(times) > 1 and gaps.max() - gaps.min() > 0.002),
        'audio': audio is not None,
        'created': tags.get('creation_time') or tags.get('com.apple.quicktime.creationdate'),
    }


def roi_filter(review, roi, width):
    v = review['video']
    parts = []
    if roi:
        x0, y0, x1, y1 = [float(s) for s in roi.split(',')]
        cw, ch = int((x1 - x0) * v['width']) // 2 * 2, int((y1 - y0) * v['height']) // 2 * 2
        parts.append('crop=%d:%d:%d:%d' % (cw, ch, int(x0 * v['width']), int(y0 * v['height'])))
    if width:
        parts.append('scale=%d:-2' % width)
    return parts


def grab(review, start, end, fps=None, roi=None, width=None):
    """(video time, PIL image) for every frame shown in [start, end], or sampled at fps."""
    v = review['video']
    filters = roi_filter(review, roi, width)
    if fps:
        filters.insert(0, 'fps=%g' % fps)
    filters.append('showinfo')
    t0 = max(0.0, start)
    cmd = [ffmpeg('ffmpeg'), '-v', 'info', '-hide_banner', '-ss', '%.4f' % (t0 + v['start_time']), '-copyts', '-i', v['path'], '-to', '%.4f' % (end + v['start_time']),
           '-vf', ','.join(filters), '-fps_mode', 'passthrough', '-an', '-f', 'image2pipe', '-c:v', 'ppm', '-']
    out = subprocess.run(cmd, capture_output=True)
    times = [float(m) - v['start_time'] for m in re.findall(rb'pts_time:\s*([-0-9.]+)', out.stderr)]
    images = []
    data = out.stdout
    k = 0
    while k < len(data):
        m = re.match(rb'P6\s+(\d+)\s+(\d+)\s+(\d+)\s', data[k:k + 64])
        if not m:
            break
        w, h = int(m.group(1)), int(m.group(2))
        body = k + m.end()
        images.append(Image.frombytes('RGB', (w, h), data[body:body + w * h * 3]))
        k = body + w * h * 3
    pairs = [(t, im) for t, im in zip(times, images) if start - 1e-3 <= t <= end + 1e-3]
    if not pairs and out.returncode != 0:
        sys.exit('ffmpeg: ' + out.stderr.decode(errors='replace')[-800:])
    return pairs


def label(im, text, size):
    d = ImageDraw.Draw(im)
    f = font(size)
    box = d.textbbox((0, 0), text, font=f)
    d.rectangle((0, 0, box[2] + 8, box[3] + 6), fill=(0, 0, 0))
    d.text((4, 2), text, fill=(255, 255, 0), font=f)


def tile(frames, cols, path):
    if not frames:
        sys.exit('no frames there')
    w, h = frames[0][1].size
    rows = (len(frames) + cols - 1) // cols
    sheet = Image.new('RGB', (cols * w, rows * h), (40, 40, 40))
    for k, (text, im) in enumerate(frames):
        im = im.copy()
        label(im, text, max(12, w // 22))
        sheet.paste(im, ((k % cols) * w, (k // cols) * h))
    sheet.save(path)
    return path


# ---------- log ----------

def find_balls(run, min_dip=40, below=25):
    """Each ball from the flywheel's dip: speed falls >= min_dip from where it was and ends below the target.
    Looks from each blocker open to 0.8 s after it closes (balls still in the path land after the close)."""
    t = np.array(run.col('time.t'))
    v = np.array(run.col('flywheel.velocity'))
    raw = np.array(run.col('flywheel.raw_velocity')) if run.has('flywheel.raw_velocity') else v
    target = np.array(run.col('flywheel.target'))
    n = len(t)
    if run.has('blocker.clear'):
        clear = np.array(run.col('blocker.clear'))
        windows = []
        i = 1
        while i < n:
            if clear[i] == 1 and clear[i - 1] != 1:
                j = i
                while j < n and clear[j] == 1:
                    j += 1
                end = t[min(j, n - 1)] + 0.8
                windows.append((i, int(np.searchsorted(t, end))))
                i = j
            i += 1
    else:
        windows = [(1, n)]
    balls = []
    last = -1e9
    for burst, (a, b) in enumerate(windows):
        b = min(b, n - 1)
        k = max(a, 1)
        in_burst = 0
        while k < b:
            # a local low that is a real ball
            if v[k] <= v[k - 1] and v[k] <= v[k + 1] and target[k] > 0 and v[k] < target[k] - below:
                back = k
                while back > max(a - 8, 1) and v[back - 1] >= v[back] and t[k] - t[back - 1] < 0.35:
                    back -= 1
                lowraw = raw[back:k + 1].min()
                if v[back] - v[k] >= min_dip and lowraw <= target[k] - 20 and raw[max(0, back - 1):back + 1].max() - lowraw >= 30 and t[back] - last > 0.12:
                    in_burst += 1
                    balls.append({'row': back, 'low_row': k, 'burst': burst + 1, 'in_burst': in_burst, 'log_t': float(t[back]), 'speed': float(v[back]),
                                  'raw': contact_speed(t, raw, back), 'dip': float(v[back] - v[k]), 'lowest': float(v[k]), 'target': float(target[back])})
                    last = t[back]
            k += 1
    return balls


def contact_speed(t, raw, i, span=0.15):
    """The encoder's own speed at the contact (a line through its last 0.15 s: it steps 20 ticks/s), shown next to the filtered one."""
    js = [j for j in range(max(0, i - 12), i + 1) if t[i] - t[j] <= span + 1e-6]
    if len(js) < 3:
        return float(raw[i])
    x = np.array([t[j] - t[i] for j in js])
    y = np.array([raw[j] for j in js])
    slope, at = np.polyfit(x, y, 1)
    return float(at)


def first_move(run, speed=3.0):
    t = run.col('time.t')
    s = run.col('pose.speed')
    for i in range(len(t)):
        if s[i] == s[i] and s[i] > speed:
            return float(t[i])
    return None


def opens(run):
    t = run.col('time.t')
    c = run.col('blocker.clear')
    return [float(t[i]) for i in range(1, len(t)) if c[i] == 1 and c[i - 1] != 1]


def ball_rows(name, review):
    rows = []
    for r, entry in enumerate(review['runs']):
        run = robot_log.Run(entry['folder'])
        offset = entry.get('offset')

        def col(c, i):
            return float(run.col(c)[i]) if run.has(c) else float('nan')
        for k, b in enumerate(find_balls(run)):
            i = b['row']
            gx, gy, tx, ty = col('shooter.goal_x', i), col('shooter.goal_y', i), col('shooter.turret_x', i), col('shooter.turret_y', i)
            rows.append({
                'ball': 'R%dB%02d' % (r + 1, k + 1), 'run': entry['name'], 'log_t': '%.3f' % b['log_t'],
                'video_t': '%.3f' % (b['log_t'] + offset) if offset is not None else '',
                'burst': b['burst'], 'in_burst': b['in_burst'],
                'x': '%.1f' % col('pose.x', i), 'y': '%.1f' % col('pose.y', i), 'heading': '%.0f' % col('pose.heading', i),
                'turret_x': '%.1f' % tx, 'turret_y': '%.1f' % ty,
                'distance': '%.2f' % col('shooter.shot_distance', i), 'goal_distance': '%.2f' % math.hypot(gx - tx, gy - ty),
                'cell': 'CLOSE' if col('pose.y', i) > 72 else 'FAR',
                'target': '%.0f' % b['target'], 'speed_at_contact': '%.0f' % b['speed'], 'raw_at_contact': '%.0f' % b['raw'],
                'dip': '%.0f' % b['dip'], 'lowest': '%.0f' % b['lowest'], 'target_minus_speed': '%.0f' % (b['target'] - b['speed']),
                'battery': '%.2f' % col('power.battery_v', i), 'robot_speed': '%.1f' % col('pose.speed', i), 'omega': '%.0f' % col('pose.omega', i),
                'turret_error': '%.1f' % col('turret.error', i), 'aim_ready': '%.0f' % col('blocker.aim_ready', i),
                'outcome': '', 'conf': '', 'note': ''})
    return rows


def shots_path(name):
    return os.path.join(folder(name), 'shots.csv')


def read_shots(name):
    path = shots_path(name)
    if not os.path.isfile(path):
        return []
    with open(path, encoding='utf-8', newline='') as f:
        return list(csv.DictReader(f))


def write_shots(name, rows):
    with open(shots_path(name), 'w', encoding='utf-8', newline='') as f:
        w = csv.DictWriter(f, FIELDS)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, '') for k in FIELDS})


def refresh_shots(name, review):
    """Re-find balls (new offsets give new video times) but keep every mark already made."""
    old = {(r['run'], round(float(r['log_t']), 2)): r for r in read_shots(name)}
    rows = ball_rows(name, review)
    for r in rows:
        prev = old.get((r['run'], round(float(r['log_t']), 2)))
        if prev:
            for k in ('outcome', 'conf', 'note'):
                r[k] = prev.get(k, '')
    write_shots(name, rows)
    return rows


# ---------- commands ----------

def cmd_new(a):
    os.makedirs(folder(a.name), exist_ok=True)
    video = probe(a.video)
    runs = []
    for spec in a.runs:
        path = robot_log.find(spec)
        meta = robot_log.Run(path).meta()
        runs.append({'name': os.path.basename(path), 'folder': os.path.abspath(path), 'opmode': meta.get('opmode', '?'), 'started': meta.get('started', '?'), 'offset': None})
    review = {'name': a.name, 'made': datetime.datetime.now().isoformat(timespec='seconds'), 'video': video, 'runs': runs, 'table': current_table()}
    save(a.name, review)
    rows = refresh_shots(a.name, review)
    print(json.dumps(video, indent=2))
    for r in runs:
        print('run %s (%s), robot clock says it started %s' % (r['name'], r['opmode'], r['started']))
    if video['variable_frame_rate']:
        print('variable frame rate: every time here comes from the frame timestamps, never frame number / fps')
    print('%d balls in the logs -> %s' % (len(rows), shots_path(a.name)))


def cmd_balls(a):
    review = load(a.name)
    rows = refresh_shots(a.name, review)
    for r in rows:
        print('%s  log %8s  video %8s  burst %3s #%s  dist %6s  target %5s  speed %5s (raw %5s)  dip %4s  turret err %5s  robot %4s in/s  %s %s' % (
            r['ball'], r['log_t'], r['video_t'] or '-', r['burst'], r['in_burst'], r['distance'], r['target'], r['speed_at_contact'], r['raw_at_contact'],
            r['dip'], r['turret_error'], r['robot_speed'], r['outcome'], r['note']))
    for k, run in enumerate(review['runs']):
        print('run R%d %s offset %s' % (k + 1, run['name'], 'not synced' if run.get('offset') is None else '%+.3f s (%s)' % (run['offset'], run.get('how', ''))))


def audio_onsets(review):
    v = review['video']
    if not v['audio']:
        sys.exit('this video has no sound')
    rate = 16000
    out = subprocess.run([ffmpeg('ffmpeg'), '-v', 'error', '-i', v['path'], '-vn', '-ac', '1', '-ar', str(rate), '-f', 'f32le', '-'], capture_output=True)
    x = np.frombuffer(out.stdout, dtype=np.float32).astype(np.float64)
    hop, size = 160, 1024
    frames = (len(x) - size) // hop
    if frames < 10:
        sys.exit('no sound decoded')
    window = np.hanning(size)
    idx = np.arange(size)[None, :] + hop * np.arange(frames)[:, None]
    spec = np.abs(np.fft.rfft(x[idx] * window, axis=1))
    freqs = np.fft.rfftfreq(size, 1 / rate)
    band = (freqs > 80) & (freqs < 5000)
    logspec = np.log1p(100 * spec[:, band])
    flux = np.maximum(0, np.diff(logspec, axis=0)).sum(axis=1)
    flux = np.concatenate([[0], flux])
    times = np.arange(frames) * hop / rate + size / 2 / rate
    # judge each onset against its own surroundings (crowd, music and motors change loudness over a match)
    k = int(1.5 * rate / hop)
    pad = np.pad(flux, k, mode='edge')
    med = np.array([np.median(pad[i:i + 2 * k + 1]) for i in range(0, len(flux), 4)])
    med = np.repeat(med, 4)[:len(flux)]
    mad = np.array([np.median(np.abs(pad[i:i + 2 * k + 1] - np.median(pad[i:i + 2 * k + 1]))) for i in range(0, len(flux), 4)])
    mad = np.repeat(mad, 4)[:len(flux)] + 1e-9
    z = (flux - med) / (1.4826 * mad)
    peaks = []
    gap = int(0.08 * rate / hop)
    for i in range(1, len(z) - 1):
        if z[i] > 6 and z[i] >= z[i - 1] and z[i] >= z[i + 1] and z[i] == z[max(0, i - gap):i + gap + 1].max():
            peaks.append((onset(x, rate, float(times[i])), float(z[i])))
    return times, z, peaks


def onset(x, rate, near):
    """Where the sound actually starts: the 1 ms loudness first passes half its peak within 60 ms of the spectral guess."""
    a, b = max(0, int((near - 0.06) * rate)), min(len(x), int((near + 0.06) * rate))
    if b - a < 32:
        return near
    k = max(1, rate // 1000)
    env = np.convolve(np.abs(x[a:b]), np.ones(k) / k, mode='same')
    base = np.median(env[:len(env) // 4])
    top = env.max()
    if top <= 2 * base:
        return near
    return (a + int(np.argmax(env > base + 0.5 * (top - base)))) / rate


def cmd_audio(a):
    review = load(a.name)
    times, z, peaks = audio_onsets(review)
    path = os.path.join(folder(a.name), 'audio.tsv')
    with open(path, 'w', encoding='utf-8') as f:
        f.write('video_t\tstrength\n')
        for t, s in peaks:
            f.write('%.3f\t%.1f\n' % (t, s))
    print('%d onsets -> %s' % (len(peaks), path))
    pic = timeline(a.name, review, times, np.minimum(z, 40), 'sound onset strength (z)', 'audio')
    print('picture ->', pic)


def timeline(name, review, times, values, title, tag):
    """Signal over the whole video (rows of 30 s) with the log's balls drawn on as red ticks once synced."""
    span = 30.0
    total = float(times[-1]) if len(times) else review['video']['duration']
    rows = int(math.ceil(total / span)) or 1
    w, h = 1800, 130
    im = Image.new('RGB', (w, rows * h + 30), (255, 255, 255))
    d = ImageDraw.Draw(im)
    f = font(14)
    d.text((6, 6), title + '   (red = balls in the log at the synced offset, blue = sync events)', fill=(0, 0, 0), font=f)
    top = float(np.nanmax(values)) if len(values) else 1
    balls = [float(r['video_t']) for r in read_shots(name) if r['video_t']]
    events = review.get('sync_events', [])
    for r in range(rows):
        y0 = 30 + r * h
        t0 = r * span
        d.text((4, y0 + 2), '%.0f s' % t0, fill=(80, 80, 80), font=f)
        for s in range(31):
            x = 60 + s * (w - 70) / span
            d.line((x, y0 + h - 18, x, y0 + h - 12), fill=(150, 150, 150))
            if s % 5 == 0:
                d.text((x - 6, y0 + h - 12), '%d' % (t0 + s), fill=(120, 120, 120), font=font(11))
        sel = (times >= t0) & (times < t0 + span)
        pts = [(60 + (t - t0) * (w - 70) / span, y0 + h - 20 - (val / top) * (h - 30)) for t, val in zip(times[sel], values[sel])]
        if len(pts) > 1:
            d.line(pts, fill=(0, 0, 0), width=1)
        for b in balls:
            if t0 <= b < t0 + span:
                x = 60 + (b - t0) * (w - 70) / span
                d.line((x, y0 + 4, x, y0 + h - 20), fill=(220, 0, 0), width=2)
        for e in events:
            if t0 <= e < t0 + span:
                x = 60 + (e - t0) * (w - 70) / span
                d.line((x, y0 + 4, x, y0 + 20), fill=(0, 0, 230), width=3)
    path = os.path.join(folder(name), tag + '.png')
    im.save(path)
    return path


def cmd_motion(a):
    review = load(a.name)
    v = review['video']
    fps = a.fps
    filters = ['fps=%g' % fps] + roi_filter(review, a.roi, 96) + ['format=gray']
    out = subprocess.run([ffmpeg('ffmpeg'), '-v', 'error', '-i', v['path'], '-vf', ','.join(filters), '-an', '-f', 'rawvideo', '-'], capture_output=True)
    probe_im = grab(review, 1.0, 1.2, fps=5, roi=a.roi, width=96)
    if not probe_im:
        sys.exit('could not read the box')
    w, h = probe_im[0][1].size
    x = np.frombuffer(out.stdout, dtype=np.uint8)
    frames = len(x) // (w * h)
    x = x[:frames * w * h].reshape(frames, h, w).astype(np.float32)
    act = np.concatenate([[0], np.abs(np.diff(x, axis=0)).mean(axis=(1, 2))])
    times = np.arange(frames) / fps
    path = os.path.join(folder(a.name), 'motion.tsv')
    with open(path, 'w', encoding='utf-8') as f:
        f.write('video_t\tchange\n')
        for t, s in zip(times, act):
            f.write('%.3f\t%.2f\n' % (t, s))
    review['motion_roi'] = a.roi
    save(a.name, review)
    print('motion in %s -> %s, picture -> %s' % (a.roi, path, timeline(a.name, review, times, act, 'picture change in ' + a.roi, 'motion')))


def cmd_sheet(a):
    review = load(a.name)
    frames = grab(review, a.start, a.end, fps=a.fps, roi=a.roi, width=a.width)
    shots = read_shots(a.name)
    out = []
    for t, im in frames:
        text = 'v %.2f' % t
        for r in shots:
            if r['video_t'] and -0.1 <= t - float(r['video_t']) <= 1.5:
                text += '  %s+%.2f' % (r['ball'], t - float(r['video_t']))
                break
        out.append((text, im))
    path = os.path.join(folder(a.name), 'frames', 'sheet_%07.2f_%07.2f%s.png' % (a.start, a.end, '_roi' if a.roi else ''))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    print(tile(out, a.cols, path), '(%d frames)' % len(out))


def cmd_frame(a):
    review = load(a.name)
    frames = grab(review, a.t, a.t + 0.2, roi=a.roi, width=a.width)
    if not frames:
        sys.exit('no frame at %.3f' % a.t)
    t, im = frames[0]
    label(im, 'v %.3f' % t, max(14, im.size[0] // 40))
    path = os.path.join(folder(a.name), 'frames', 'frame_%08.3f%s.png' % (t, '_roi' if a.roi else ''))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    im.save(path)
    print(path)


def log_time(run, spec):
    if spec == 'move':
        t = first_move(run)
        if t is None:
            sys.exit('the robot never moves in this run')
        return t
    if spec.startswith('ball:'):
        return find_balls(run)[int(spec[5:]) - 1]['log_t']
    if spec.startswith('open:'):
        return opens(run)[int(spec[5:]) - 1]
    return float(spec)


def match(video, log, tol):
    """Best offset (video = log + offset) by voting: every video/log pair proposes one, the one most others agree with wins."""
    best = None
    tried = []
    for v in video:
        for l in log:
            off = v - l
            used = set()
            pairs = []
            for vv in video:
                k = int(np.argmin([abs(vv - (ll + off)) if j not in used else 1e9 for j, ll in enumerate(log)]))
                if abs(vv - (log[k] + off)) <= tol:
                    used.add(k)
                    pairs.append((vv, log[k]))
            if pairs:
                refined = float(np.median([p[0] - p[1] for p in pairs]))
                rms = float(np.sqrt(np.mean([(p[0] - p[1] - refined) ** 2 for p in pairs])))
                tried.append((len(pairs), -rms, refined, pairs))
    tried.sort(key=lambda x: (x[0], x[1]), reverse=True)
    if not tried:
        return None, None
    best = tried[0]
    other = next((x for x in tried[1:] if abs(x[2] - best[2]) > 3 * tol), None)
    return best, other


def cmd_sync(a):
    review = load(a.name)
    entry = next((r for r in review['runs'] if a.run in r['name']), None)
    if entry is None:
        sys.exit('no run like %r in this review' % a.run)
    run = robot_log.Run(entry['folder'])
    if a.anchor:
        vt, spec = a.anchor.split('=')
        lt = log_time(run, spec)
        entry['offset'] = float(vt) - lt
        entry['how'] = 'anchor video %s = log %s (%.3f)' % (vt, spec, lt)
        print('offset %+.3f s from the anchor' % entry['offset'])
    else:
        if a.audio:
            _, _, peaks = audio_onsets(review)
            video = [t for t, s in peaks]
            tol = a.tol or 0.06
        else:
            video = [float(s) for s in a.events.split(',')]
            tol = a.tol or 0.12
        log = [b['log_t'] for b in find_balls(run)]
        if a.window:
            lo, hi = [float(s) for s in a.window.split(',')]
            video = [t for t in video if lo <= t <= hi]
        best, other = match(video, log, tol)
        if best is None:
            sys.exit('nothing lines up')
        n, neg, off, pairs = best
        print('offset %+.3f s: %d of %d video events land on balls in the log (%d balls), spread %.0f ms' % (off, n, len(video), len(log), -neg * 1000))
        for vv, ll in pairs:
            print('   video %8.3f  <->  log %8.3f   (%+.0f ms)' % (vv, ll, 1000 * (vv - ll - off)))
        if other:
            print('next best: %+.3f s with %d matches' % (other[2], other[0]))
        sure = n >= 3 and (other is None or other[0] <= n - 2)
        print('sure' if sure else 'NOT sure: add more events, or an anchor, and check it in the pictures')
        entry['offset'] = off
        entry['how'] = '%s, %d matches, spread %.0f ms%s' % ('sound' if a.audio else 'marked events', n, -neg * 1000, '' if sure else ', not sure')
        review['sync_events'] = video if not a.audio else []
    save(a.name, review)
    rows = refresh_shots(a.name, review)
    print('%d balls now have video times (%s)' % (sum(1 for r in rows if r['video_t']), shots_path(a.name)))
    if review['video']['audio'] and os.path.isfile(os.path.join(folder(a.name), 'audio.png')):
        times, z, _ = audio_onsets(review)
        print('redrawn with the balls on it:', timeline(a.name, review, times, np.minimum(z, 40), 'sound onset strength (z)', 'audio'))
    # the other runs of the same robot boot share its clock: say where they should land
    for other in review['runs']:
        if other is entry or other.get('offset') is not None:
            continue
        try:
            d = (parse_started(other['started']) - parse_started(entry['started'])).total_seconds()
            print('  %s started %.0f s after this one by the robot clock: its offset should be about %+.1f s (+-1.5)' % (other['name'], d, entry['offset'] + d))
        except ValueError:
            pass


def parse_started(s):
    return datetime.datetime.strptime(re.sub(r' [A-Z]{2,5} ', ' ', s), '%a %b %d %H:%M:%S %Y')


def cmd_shot(a):
    review = load(a.name)
    rows = read_shots(a.name)
    r = next((x for x in rows if x['ball'] == a.ball), None)
    if r is None or not r['video_t']:
        sys.exit('no synced ball %s (sync first)' % a.ball)
    t = float(r['video_t'])
    frames = grab(review, t - a.before, t + a.after, roi=a.roi, width=a.width)[::a.step]
    out = [('v %.3f  %+.0f ms' % (ft, 1000 * (ft - t)), im) for ft, im in frames]
    path = os.path.join(folder(a.name), 'frames', 'shot_%s%s.png' % (a.ball, '_roi' if a.roi else ''))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    print(tile(out, a.cols, path), '(%d frames)' % len(out))
    print('%s: distance %s in, speed at contact %s (target %s), turret error %s deg, robot %s in/s, ball %s of burst %s' % (
        a.ball, r['distance'], r['speed_at_contact'], r['target'], r['turret_error'], r['robot_speed'], r['in_burst'], r['burst']))


def cmd_mark(a):
    if a.outcome not in OUTCOMES:
        sys.exit('outcome must be one of ' + ', '.join(OUTCOMES))
    rows = read_shots(a.name)
    hit = False
    for r in rows:
        if r['ball'] in a.ball.split(','):
            r['outcome'] = a.outcome
            r['conf'] = '%g' % a.conf
            r['note'] = a.note or r['note']
            hit = True
    if not hit:
        sys.exit('no ball ' + a.ball)
    write_shots(a.name, rows)
    print('marked', a.ball, a.outcome)


# ---------- fit ----------

def current_table(path=SHOOTER):
    src = open(path, encoding='utf-8').read()

    def lst(name):
        m = re.search(name + r'\s*=\s*new ArrayList<>\(Arrays\.asList\(([^)]*)\)\)', src)
        return [float(s) for s in m.group(1).split(',')]
    deg = int(re.search(r'FLYWHEEL_FIT_DEGREE\s*=\s*(\d+)', src).group(1))
    tol = float(re.search(r'SHOT_MISS_TOLERANCE_INCHES\s*=\s*([0-9.]+)', src).group(1))
    return {'distances': lst('FLYWHEEL_DISTANCES'), 'velocities': lst('FLYWHEEL_VELOCITIES'), 'degree': deg, 'miss_tolerance_inches': tol}


def poly(table):
    d, v = np.array(table['distances']), np.array(table['velocities'])
    deg = max(0, min(table['degree'], 3, len(d) - 1))
    c = np.polyfit(d, v, deg)
    return lambda x: np.polyval(c, x)


def phi(x):
    return 0.5 * (1 + np.vectorize(math.erf)(x / math.sqrt(2)))


def cmd_fit(a):
    names = a.names or sorted(n for n in os.listdir(REVIEWS) if os.path.isfile(os.path.join(REVIEWS, n, 'shots.csv'))) if os.path.isdir(REVIEWS) else []
    shots = []
    for n in names:
        for r in read_shots(n):
            if r['outcome']:
                r['review'] = n
                shots.append(r)
    if not shots:
        sys.exit('no marked balls yet')
    table = current_table()
    line = poly(table)
    counts = {}
    for r in shots:
        counts[r['outcome']] = counts.get(r['outcome'], 0) + 1
    print('%d marked balls from %d review(s): %s' % (len(shots), len(names), ', '.join('%s %d' % kv for kv in sorted(counts.items()))))

    used = [r for r in shots if r['outcome'] in ('score', 'short', 'long', 'rim') and finite(r['distance']) and finite(r['speed_at_contact'])]
    d = np.array([float(r['distance']) for r in used])
    v = np.array([float(r['speed_at_contact']) for r in used])
    wt = np.array([(float(r['conf']) if r['conf'] else 1.0) * (0.5 if finite(r['robot_speed']) and float(r['robot_speed']) > 4 else 1.0) for r in used])
    out = [r['outcome'] for r in used]
    # speed per inch of range from the table, so the scoring window in speed matches the goal's size in inches
    xs = np.linspace(min(table['distances']), max(table['distances']), 50)
    slope = max(1.0, float(np.median(np.abs(np.gradient(line(xs), xs)))))
    w = a.window if a.window else slope * table['miss_tolerance_inches']
    sigma = a.sigma
    print('speed table now: %s at %s in (fit degree %d), %.1f ticks/s per inch -> scoring window +-%.0f ticks/s, ball-to-ball spread %.0f' % (
        table['velocities'], table['distances'], table['degree'], slope, w, sigma))

    if len(used):
        dm = float(np.average(d, weights=wt))
        A = np.arange(-80, 80.01, 0.5)
        B = np.arange(-4, 4.001, 0.05)
        AA, BB = np.meshgrid(A, B, indexing='ij')
        ll = -0.5 * (AA / a.prior_offset) ** 2 - 0.5 * (BB / a.prior_slope) ** 2
        for k in range(len(used)):
            need = line(d[k]) + AA + BB * (d[k] - dm)
            m = v[k] - need
            if out[k] == 'short':
                p = phi((-w - m) / sigma)
            elif out[k] == 'long':
                p = phi((m - w) / sigma)
            elif out[k] == 'rim':
                p = phi((-w - m) / sigma) + phi((m - w) / sigma) + 0.3 * (phi((w - m) / sigma) - phi((-w - m) / sigma))
            else:
                p = phi((w - m) / sigma) - phi((-w - m) / sigma)
            ll += wt[k] * np.log(np.maximum(p, 1e-12))
        post = np.exp(ll - ll.max())
        post /= post.sum()
        ma = float((post * AA).sum())
        mb = float((post * BB).sum())
        pa = post.sum(axis=1)
        cdf = np.cumsum(pa)
        lo, hi = float(A[np.searchsorted(cdf, 0.1)]), float(A[np.searchsorted(cdf, 0.9)])
        print('balls in the fit: %d (score/short/long/rim), distances %.1f-%.1f in, middle %.1f' % (len(used), d.min(), d.max(), dm))
        print('the speed that scores is %+.1f ticks/s from the table at %.1f in (80%%: %+.0f..%+.0f), and changes %+.2f ticks/s per inch more than the table' % (ma, dm, lo, hi, mb))
        new = []
        for dk, vk in zip(table['distances'], table['velocities']):
            change = ma + mb * (dk - dm)
            change = max(-a.max_step, min(a.max_step, change))
            if hi - lo > 2 * w + 20 or abs(change) < a.min_change:
                change = 0.0
            new.append(round(vk + change))
        far = [dk for dk in table['distances'] if dk < d.min() - 6 or dk > d.max() + 6]
        print('proposed FLYWHEEL_VELOCITIES: %s  (was %s)' % (new, [round(x) for x in table['velocities']]))
        if far:
            print('  points at %s in have no balls near them: their change comes from the line through the balls that were seen' % far)
        if new == [round(x) for x in table['velocities']]:
            print('  no change: the balls agree with the table or are too few to say')
        elif a.apply:
            apply_table(new)
            with open(os.path.join(REVIEWS, 'CHANGES.md'), 'a', encoding='utf-8') as f:
                f.write('- %s, from %s (%d balls, %.1f-%.1f in): FLYWHEEL_VELOCITIES %s -> %s (scoring speed %+.1f ticks/s at %.1f in, 80%% %+.0f..%+.0f, slope %+.2f/in)\n' % (
                    datetime.date.today().isoformat(), ', '.join(names), len(used), d.min(), d.max(), [round(x) for x in table['velocities']], new, ma, dm, lo, hi, mb))
            print('wrote', SHOOTER, 'and', os.path.join(REVIEWS, 'CHANGES.md'))
    side = [r for r in shots if r['outcome'] in ('left', 'right')]
    if side:
        print('left/right misses (aim, not flywheel): %d; turret error at contact %s' % (len(side), ', '.join('%s %s' % (r['ball'], r['turret_error']) for r in side)))

    # flywheel tuning: is it back to speed when each ball hits?
    print('flywheel at contact (target - speed; + = slow), by ball in the burst:')
    for k in sorted(set(int(r['in_burst']) for r in shots)):
        group = [r for r in shots if int(r['in_burst']) == k and finite(r['target_minus_speed'])]
        if not group:
            continue
        gaps = [float(r['target_minus_speed']) for r in group]
        res = {}
        for r in group:
            res[r['outcome']] = res.get(r['outcome'], 0) + 1
        print('  ball %d: %2d balls, slow by %+.0f median (worst %+.0f), %s' % (k, len(group), float(np.median(gaps)), max(gaps), ', '.join('%s %d' % kv for kv in sorted(res.items()))))
    short = [float(r['target_minus_speed']) for r in shots if r['outcome'] == 'short' and finite(r['target_minus_speed'])]
    good = [float(r['target_minus_speed']) for r in shots if r['outcome'] == 'score' and finite(r['target_minus_speed'])]
    if short and good and np.median(short) - np.median(good) > 0.5 * w:
        print('  -> short balls hit the wheel %.0f ticks/s slower than scoring ones: that is the flywheel not recovering, not the table.' % (np.median(short) - np.median(good)))
        print('     Small steps: FLYWHEEL_PRE_KICK_VOLTS +0.3 V (pushes just before each ball), or a tighter SHOT_GATE_HOLD_BAND, or longer FLYWHEEL_BALL_INTERVAL feed spacing.')


def apply_table(new):
    src = open(SHOOTER, encoding='utf-8').read()
    body = ', '.join('%.1f' % x for x in new)
    src2 = re.sub(r'(FLYWHEEL_VELOCITIES\s*=\s*new ArrayList<>\(Arrays\.asList\()[^)]*(\)\))', lambda m: m.group(1) + body + m.group(2), src, count=1)
    with open(SHOOTER, 'w', encoding='utf-8', newline='') as f:
        f.write(src2)


def finite(s):
    try:
        x = float(s)
    except (TypeError, ValueError):
        return False
    return x == x and not math.isinf(x)


def cmd_report(a):
    review = load(a.name)
    rows = read_shots(a.name)
    lines = ['# Video review %s' % a.name, '', 'Video: `%s` (%.0f s, %s fps median, %s)' % (
        os.path.basename(review['video']['path']), review['video']['duration'], review['video']['fps_median'], 'variable frame rate' if review['video']['variable_frame_rate'] else 'constant frame rate'), '']
    for k, r in enumerate(review['runs']):
        lines.append('- R%d `%s` (%s): offset %s' % (k + 1, r['name'], r['opmode'], 'not synced' if r.get('offset') is None else '%+.3f s, %s' % (r['offset'], r.get('how', ''))))
    lines += ['', '| ball | video s | dist in | target | speed at contact | turret err | robot in/s | outcome | note |', '|---|---|---|---|---|---|---|---|---|']
    for r in rows:
        lines.append('| %s | %s | %s | %s | %s | %s | %s | %s | %s |' % (r['ball'], r['video_t'], r['distance'], r['target'], r['speed_at_contact'], r['turret_error'], r['robot_speed'], r['outcome'], r['note']))
    path = os.path.join(folder(a.name), 'REPORT.md')
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')
    print(path)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('new')
    s.add_argument('name')
    s.add_argument('video')
    s.add_argument('runs', nargs='+')
    s.set_defaults(f=cmd_new)
    for n, f in (('balls', cmd_balls), ('audio', cmd_audio), ('report', cmd_report)):
        s = sub.add_parser(n)
        s.add_argument('name')
        s.set_defaults(f=f)
    s = sub.add_parser('motion')
    s.add_argument('name')
    s.add_argument('--roi', required=True)
    s.add_argument('--fps', type=float, default=30)
    s.set_defaults(f=cmd_motion)
    s = sub.add_parser('sheet')
    s.add_argument('name')
    s.add_argument('--from', dest='start', type=float, required=True)
    s.add_argument('--to', dest='end', type=float, required=True)
    s.add_argument('--fps', type=float, default=4)
    s.add_argument('--roi')
    s.add_argument('--width', type=int, default=480)
    s.add_argument('--cols', type=int, default=4)
    s.set_defaults(f=cmd_sheet)
    s = sub.add_parser('frame')
    s.add_argument('name')
    s.add_argument('t', type=float)
    s.add_argument('--roi')
    s.add_argument('--width', type=int)
    s.set_defaults(f=cmd_frame)
    s = sub.add_parser('sync')
    s.add_argument('name')
    s.add_argument('--run', required=True)
    s.add_argument('--events')
    s.add_argument('--audio', action='store_true')
    s.add_argument('--anchor')
    s.add_argument('--window', help='only video events between A,B s')
    s.add_argument('--tol', type=float)
    s.set_defaults(f=cmd_sync)
    s = sub.add_parser('shot')
    s.add_argument('name')
    s.add_argument('ball')
    s.add_argument('--roi')
    s.add_argument('--width', type=int, default=360)
    s.add_argument('--before', type=float, default=0.25)
    s.add_argument('--after', type=float, default=1.6)
    s.add_argument('--step', type=int, default=1)
    s.add_argument('--cols', type=int, default=6)
    s.set_defaults(f=cmd_shot)
    s = sub.add_parser('mark')
    s.add_argument('name')
    s.add_argument('ball', help='one ball or several: R1B03,R1B04')
    s.add_argument('outcome')
    s.add_argument('--conf', type=float, default=1)
    s.add_argument('--note')
    s.set_defaults(f=cmd_mark)
    s = sub.add_parser('fit')
    s.add_argument('names', nargs='*')
    s.add_argument('--apply', action='store_true')
    s.add_argument('--max-step', type=float, default=25)
    s.add_argument('--min-change', type=float, default=3)
    s.add_argument('--window', type=float, help='scoring window, +- ticks/s (default: table slope x SHOT_MISS_TOLERANCE_INCHES)')
    s.add_argument('--sigma', type=float, default=10, help='ball-to-ball speed spread, ticks/s')
    s.add_argument('--prior-offset', type=float, default=25)
    s.add_argument('--prior-slope', type=float, default=1)
    s.set_defaults(f=cmd_fit)
    a = p.parse_args()
    if a.cmd == 'sync' and not (a.events or a.audio or a.anchor):
        sys.exit('sync needs --events, --audio or --anchor')
    a.f(a)


if __name__ == '__main__':
    main()
