"""Sweep every operator action against the standing acceptance criteria.

Drives the native runner the way the operator GUI does - DRIVE re-sent at
50 Hz - because a single-shot command does not reproduce operator behaviour,
and a bug that only appears under the repeated command has already slipped
past single-shot tests here.

CAVEAT on the drift column: it is the peak-to-peak range of `wheel_position`,
which is only meaningful for actions that are NOT supposed to translate (yaw
in place, dance, standing). For actions that travel on purpose it reports the
intended distance, not drift - judging those needs deviation-from-commanded
-path or post-stop settling instead.

Usage: python3 tools/action_sweep.py <runner> <model>
"""
import selectors, subprocess, sys, time
def F(l): return {p.split("=",1)[0]:p.split("=",1)[1] for p in l.split("|")[1:] if "=" in p}
def run(runner, model, seed, script, t_end):
    p=subprocess.Popen([runner,"--model",model,"--rate","3000","--sensor-rate","500",
        "--realtime","--seed",str(seed)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1)
    sel=selectors.DefaultSelector(); sel.register(p.stdout,selectors.EVENT_READ)
    def send(c):
        try: p.stdin.write(c+"\n"); p.stdin.flush()
        except OSError: pass
    st=[]; drive=(0.0,0.0); last=0.0; done=set(); dl=time.monotonic()+t_end+30
    try:
        while time.monotonic()<dl:
            if not sel.select(0.5): continue
            line=p.stdout.readline().rstrip("\n")
            if line.startswith("READY|"): send("ENABLE")
            elif line.startswith("STATE|"):
                s=F(line); st.append(s); t=float(s["t"])
                for i,(at,kind,arg) in enumerate(script):
                    if t>=at and i not in done and s.get("mode")!="Disabled":
                        done.add(i)
                        if kind=="drive": drive=arg
                        else: send(f"HEIGHT {arg}")
                if t-last>=0.02:
                    send(f"DRIVE {drive[0]:.6g} {drive[1]:.6g}"); last=t
                if t>=t_end: break
    finally:
        if p.poll() is None:
            try: p.stdin.close()
            except OSError: pass
            p.terminate(); p.wait(timeout=5)
    return st
def judge(name, st):
    fell=any(s["mode"]=="Fallen" for s in st)
    w=[s for s in st if float(s["t"])>=3.0]
    if not w: return f"  {name:34s} NO DATA"
    pk=max(abs(float(s["pitch"])) for s in w)
    pos=[float(s["wheel_position"]) for s in w]
    return (f"  {name:34s} fall={'YES' if fell else 'no ':3s} "
            f"peakPitch={pk:6.3f} rad  drift={100*(max(pos)-min(pos)):6.1f} cm")
if __name__=="__main__":
    R,M=sys.argv[1],sys.argv[2]
    SC={
     "yaw in place":            ([(3.0,"drive",(0.0,0.6))], 9.0),
     "yaw while driving":       ([(3.0,"drive",(0.5,0.6))], 9.0),
     "stop from 1.0 m/s":       ([(3.0,"drive",(1.0,0.0)),(6.0,"drive",(0.0,0.0))], 10.0),
     "height up while driving": ([(3.0,"drive",(0.6,0.0)),(4.5,"height",0.20)], 10.0),
     "height down while driving":([(3.0,"drive",(0.6,0.0)),(4.5,"height",0.12)], 10.0),
     "dance: yaw + height cycle":([(3.0,"drive",(0.0,0.5)),(4.0,"height",0.20),
                                   (6.0,"height",0.13),(8.0,"height",0.20),(10.0,"height",0.13)], 13.0),
    }
    for seed in (6814,1234):
        print(f"--- seed {seed} ---")
        for name,(script,te) in SC.items():
            print(judge(name, run(R,M,seed,script,te)))
