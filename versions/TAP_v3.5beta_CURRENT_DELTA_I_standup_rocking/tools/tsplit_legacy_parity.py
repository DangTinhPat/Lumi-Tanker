#!/usr/bin/env python3
"""Replay identical sensor/plant streams through old and flag-off runners.

CSV strings and stdout must match exactly for all pre-existing fields. This
checks the emitted trace precision, not unreported float bits.
"""
import argparse,csv,json,os,subprocess
from pathlib import Path

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--baseline',type=Path,required=True);p.add_argument('--candidate',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    cases={'tall':{},'low':{'DIAG_H':'.084'},'drive_brake':{'DIAG_DRIVE':'.55'},
           'push':{'DIAG_H':'.110','DIAG_F':'4'},'held':{'DIAG_HELD':'8'},
           'soft_hip':{'DIAG_H':'.095','DIAG_BLH':'.001','DIAG_GK':'20','DIAG_GKK':'150'},
           'noisy':{'DIAG_H':'.095','DIAG_HVN':'.12','DIAG_KVN':'.12'}}
    results=[]
    for name,extra in cases.items():
        outputs=[];streams=[]
        for tag,runner in [('baseline',a.baseline),('flag_off',a.candidate)]:
            dest=a.out/(name+'_'+tag+'.csv');env=os.environ.copy()
            env.update(TIER_DIAG='1',DIAG_DUR='26',TR_ANIM=str(dest),TR_SPLIT='0');env.update(extra)
            r=subprocess.run([str(runner.resolve())],env=env,capture_output=True,text=True)
            outputs.append((r.returncode,r.stdout,r.stderr))
            streams.append(list(csv.DictReader(dest.open())))
        old,new=streams
        same=len(old)==len(new) and all(all(y.get(k)==v for k,v in x.items()) for x,y in zip(old,new))
        same=same and outputs[0]==outputs[1]
        results.append(dict(case=name,identical=same,rows=len(old)))
    a.out.joinpath('summary.json').write_text(json.dumps(results,indent=2)+'\n')
    for r in results:print(r)
    return 0 if all(r['identical'] for r in results) else 1
if __name__=='__main__':raise SystemExit(main())
