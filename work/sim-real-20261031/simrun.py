"""Replay a real run's applied commands in MuJoCo and write the same fields as robot.state."""
import sys, json, math, numpy as np
sys.path.insert(0, sys.argv[3]); from simpolicy import Sim, HOME, axang
real=[json.loads(l)['params'] for l in open(sys.argv[2]) if '"robot.state"' in l]
sim=Sim(sys.argv[1]); sim.reset()
out=open(sys.argv[4],'w')
bias=np.radians([float(sys.argv[5]),float(sys.argv[6]),0]) if len(sys.argv)>6 else np.zeros(3)
# settle 2 s on the floor at HOME under the policy before the replay, as the real run is
# already standing when it is handed over
for _ in range(100): sim.step((0,0,0),bias,1.0)
for r in real:
    driving=r['policy']!='held'
    cmd=r['move']['applied'] if driving else [0,0,0]
    scale=0.9 if math.hypot(*cmd)>1e-3 else 1.0
    g=sim.step(cmd,bias,scale) if driving else sim.step((0,0,0),bias,1.0)
    q=list(sim.d.qpos[sim.qa]); j=q[:9]+[0.0]+q[9:]
    t=list(sim.d.ctrl); tt=t[:9]+[0.0]+t[9:]
    out.write(json.dumps(dict(policy=r['policy'],cmd=cmd,gravity=list(g),gravity_true=list(sim.gravity_true()),
        gyro=list(axang(bias)@sim.d.sensordata[sim.gy:sim.gy+3]),joints=j,targets=tt,
        xy=list(sim.d.qpos[:2]),fallen=bool(sim.gravity_true()[2]>-0.5)))+'\n')
