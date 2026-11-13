import sys, numpy as np
sys.path.insert(0, sys.argv[2]); from simpolicy import Sim, stand, tilt
import math
sim=Sim(sys.argv[1])
def walk(bias, vx, seconds=20, push=0.0, seed=0):
    rng=np.random.default_rng(seed); sim.reset(); b=np.radians([bias[0],bias[1],0])
    true=[]; fell=None
    for t in range(int(seconds*50)):
        cmd=(vx if t>50 else 0.0,0,0)
        if push and t%100==50: sim.d.qvel[0:2]+=rng.normal(0,push,2)
        sim.step(cmd,b,scale=0.9); gt=sim.gravity_true()
        if gt[2]<-0.5: true.append(gt)
        elif fell is None: fell=t/50; break
    x=sim.d.qpos[0]
    return fell, np.mean(true,0), x
print('行走 20 s，动作缩放 0.9；每 2 s 随机推一下（0.15 m/s），5 个随机种子')
for b in [(0,0),(-1.8,4.79),(1.8,-4.79),(0,8),(0,-8)]:
    for vx in (0.0,0.15,-0.15):
        res=[walk(b,vx,push=0.15,seed=s) for s in range(5)]
        falls=sum(r[0] is not None for r in res)
        pitch=np.mean([tilt(r[1])[0] for r in res])
        print(f'bias X{b[0]:+.2f} Y{b[1]:+.2f} vx {vx:+.2f}: 摔倒 {falls}/5  平均真实前倾 {pitch:+5.2f}°  前进 {np.mean([r[2] for r in res]):+.2f} m')
