#!/usr/bin/env python3
"""Measure settling after a height move, using recorded telemetry only.

Robot: crouch_stop_check.py ~/tap_logs/tap_*.log --json report.json
Plant: crouch_stop_check.py --sim trace.csv --json report.json
No device connection. Events inferred from height are candidates, not proof
that no operator touched the robot. Incomplete windows remain censored.
"""
import argparse
import csv
import json
import math
import re
import statistics
from pathlib import Path

KV = re.compile(r'(\w+)=(\[[^\]]*\]|\S+)')

def number(x):
    try:
        v = float(x)
        return v if math.isfinite(v) else None
    except (ValueError, TypeError):
        return None

def percentile(xs, p):
    xs = sorted(x for x in xs if x is not None)
    if not xs:
        return None
    k = (len(xs)-1)*p
    lo = int(k)
    return xs[lo] + (xs[min(lo+1, len(xs)-1)]-xs[lo])*(k-lo)

def read_robot(path):
    segments, rows = [], []
    active_drive = False
    with open(path, errors='replace') as f:
        for line in f:
            if line.startswith('#'):
                continue
            drive = re.search(r'>>\s*DRIVE\s+(\S+)\s+(\S+)', line)
            if drive:
                values = [number(x) for x in drive.groups()]
                active_drive = any(x is not None and abs(x)>1e-4 for x in values)

            if 'CONTROL mode=' in line or 'FAULT' in line or 'state=Fault' in line:
                if rows: segments.append(rows); rows = []
            if 'TR t=' not in line:
                continue
            kv = dict(KV.findall(line))
            r = {k: number(kv.get(src)) for k, src in
                 [('t','t'),('h','h'),('tvr','tvr'),('hip','dhip'),('lean','tv'),('rate','rate')]}
            if r['t'] is None or r['h'] is None:
                continue
            r['t'] *= .001  # AppV3 TR t is device milliseconds.
            r['drive_active'] = active_drive
            if rows and (r['t'] <= rows[-1]['t'] or r['t']-rows[-1]['t'] > .25):
                segments.append(rows); rows = []
            rows.append(r)
    if rows: segments.append(rows)
    return segments

def read_sim(path):
    rows = []
    for r in csv.DictReader(open(path, newline='')):
        rows.append(dict(t=float(r['t']), h=1000*(float(r['hip_z'])-float(r['axle_z'])),
                         tvr=number(r.get('tvr')), hip=number(r.get('trunk_hip')),
                         lean=number(r.get('lean')), rate=number(r.get('rate'))))
    return [rows]

def stats(rows):
    def vals(k): return [r[k] for r in rows if r[k] is not None]
    result = {}
    for k in ['tvr','hip','rate']:
        xs = vals(k)
        result[k+'_pp'] = max(xs)-min(xs) if xs else None
        result[k+'_rms'] = math.sqrt(statistics.fmean(x*x for x in xs)) if xs else None
    xs = vals('tvr')
    # Zero crossings of centered rate delimit half cycles. Short excursions
    # below a robust noise threshold cannot manufacture growth ratios.
    if len(xs) < 12:
        result.update(half_cycle_growth=None, frequency_hz=None, half_cycles=0)
        return result
    center = statistics.median(xs)
    floor = max(.02, .15*percentile([abs(x-center) for x in xs], .99))
    peaks, peak_times = [], []
    sign, peak, peak_t = 0, 0., 0.
    for r in rows:
        if r['tvr'] is None: continue
        v = r['tvr']-center
        s = 1 if v > floor else -1 if v < -floor else 0
        if s and sign and s != sign:
            peaks.append(peak); peak_times.append(peak_t); peak = 0.
        if s: sign = s
        if abs(v) > peak: peak, peak_t = abs(v), r['t']
    ratios = [b/a for a,b in zip(peaks,peaks[1:]) if a >= floor]
    intervals = [b-a for a,b in zip(peak_times,peak_times[1:]) if .08 <= b-a <= 1.]
    result.update(half_cycle_growth=statistics.median(ratios) if len(ratios)>=3 else None,
                  frequency_hz=.5/statistics.median(intervals) if len(intervals)>=3 else None,
                  half_cycles=len(peaks))
    return result

def events(rows, horizon=3.):
    if len(rows) < 10: return []
    moving_since, moving_last, last_event = None, None, -1e9
    result = []
    for i,r in enumerate(rows):
        # 0.2 s finite difference on height suppresses telemetry quantization.
        j = i
        while j > 0 and r['t']-rows[j]['t'] < .2: j -= 1
        dt = r['t']-rows[j]['t']
        if dt < .15: continue
        speed = (r['h']-rows[j]['h'])/dt
        if abs(speed) >= 4.:
            if moving_since is None: moving_since = r['t']
            moving_last = r['t']
        elif moving_since is not None and moving_last is not None:
            if r['t']-moving_last < .25: continue
            if moving_last-moving_since < .3:
                moving_since = moving_last = None; continue
            if r['t']-last_event < horizon:
                continue
            window = [x for x in rows[i:] if x['t'] <= r['t']+horizon]
            complete = bool(window) and window[-1]['t'] >= r['t']+horizon-.12
            result.append(dict(t_stop_s=r['t'], height_mm=r['h'],
                               observed_s=window[-1]['t']-r['t'] if window else 0.,
                               complete=complete, drive_active=any(x.get('drive_active',False) for x in window),
                               classification='height_stop_candidate',
                               **stats(window)))
            last_event = r['t']; moving_since = moving_last = None
    return result

def analyze(path, sim=False):
    segments = read_sim(path) if sim else read_robot(path)
    es = [e for rows in segments for e in events(rows)]
    # Conservative candidate threshold pool: long flat-height, low-lean
    # windows without an active DRIVE command. Hand contact remains unknown.
    pool = []
    for rows in segments:
        for i in range(0,len(rows),250):
            window=rows[i:i+250]
            if len(window)<200 or window[-1]['t']-window[0]['t']<5: continue
            if max(r['h'] for r in window)-min(r['h'] for r in window)>2: continue
            if any(r.get('drive_active',False) for r in window): continue
            if any(r['lean'] is None or abs(r['lean'])>.1 for r in window): continue
            pool.extend(window)
    thresholds = {k: (1.5*percentile([abs(r[k]) for r in pool if r[k] is not None],.99)
                       if pool and any(r[k] is not None for r in pool) else None)
                  for k in ['lean','tvr','rate']}
    return dict(candidate_thresholds=thresholds, threshold_samples=len(pool),
                threshold_caveat='heuristic selection; operator contact not excluded',
                path=str(path), source='plant_scoring_only' if sim else 'robot_telemetry',
                events=es, complete_events=sum(e['complete'] for e in es))

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('logs', nargs='*'); ap.add_argument('--sim', action='append', default=[])
    ap.add_argument('--json', type=Path)
    args=ap.parse_args()
    if not args.logs and not args.sim: ap.error('provide a robot log or --sim CSV')
    reports=[analyze(p) for p in args.logs]+[analyze(p,True) for p in args.sim]
    for report in reports:
        print(report['path'], 'complete events:',report['complete_events'])
        for e in report['events']:
            print(json.dumps(e,ensure_ascii=False))
    if args.json:
        args.json.parent.mkdir(parents=True,exist_ok=True)
        args.json.write_text(json.dumps(reports,indent=2,allow_nan=False)+'\n')
    return 0
if __name__=='__main__': raise SystemExit(main())
