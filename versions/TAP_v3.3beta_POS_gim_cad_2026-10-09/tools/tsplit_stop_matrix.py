#!/usr/bin/env python3
"""Run the host-only TIER_STOP matrix and retain failed/censored trials."""
import argparse,csv,hashlib,json,os,re,subprocess,statistics
from pathlib import Path
from crouch_stop_check import analyze,stats

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--runner',type=Path,required=True);ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--split',type=int,choices=[0,1],default=0)
    ap.add_argument('--quick',action='store_true')
    ap.add_argument('--modes',default='0,1,2')
    ap.add_argument('--brake',type=float)
    args=ap.parse_args()
    args.out.mkdir(parents=True,exist_ok=True);results=[]
    hs=[.084,.090,.095,.100,.110] if not args.quick else [.095]
    for h in hs:
      for variant in range(6):
       for mode in [int(x) for x in args.modes.split(',')]:
        name=f'h{int(h*1000)}_v{variant}_m{mode}_s{args.split}'
        path=args.out/(name+'.csv');env=os.environ.copy()
        env.update(TIER_STOP='1',STOP_H=str(h),STOP_VARIANT=str(variant),STOP_MODE=str(mode),
                   TR_HRATE='.020',TR_HEXP='1.5',TR_TVGLOW='.7',TR_ANIM=str(path),TR_SPLIT=str(args.split))
        if args.brake is not None: env['STOP_BRAKE']=str(args.brake)
        run=subprocess.run([str(args.runner.resolve())],env=env,capture_output=True,text=True)
        args.out.joinpath(name+'.txt').write_text(run.stdout+run.stderr)
        rows=list(csv.DictReader(path.open())) if path.exists() else []
        tail=[r for r in rows if float(r['t'])>=22]
        status=run.stdout.strip().splitlines()[-1] if run.stdout.strip() else ''
        a=analyze(path,True) if rows else {'events':[]}
        height=statistics.mean((float(r['hip_z'])-float(r['axle_z']))*1000 for r in tail) if tail else None
        result=dict(h_mm=h*1000,variant=variant,mode=mode,split=args.split,returncode=run.returncode,
                    status=status,actual_h_mm=height,events=a['events'])
        if tail:
            normalized=[dict(t=float(r['t']),hip=float(r['trunk_hip']),tvr=float(r['tvr']),rate=float(r['rate'])) for r in tail]
            result['tail']=stats(normalized)
            result['tail_actual_hip_pp']=max(float(r['hip']) for r in tail)-min(float(r['hip']) for r in tail)
        results.append(result)
        print(name,status,flush=True)
    args.out.joinpath('summary.json').write_text(json.dumps(dict(runner_sha256=hashlib.sha256(args.runner.read_bytes()).hexdigest(),
                                                              runs=results),indent=2)+'\n')
if __name__=='__main__':main()
