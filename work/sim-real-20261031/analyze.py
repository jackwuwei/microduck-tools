import sys, json, math, numpy as np
def tilt(g): return (math.degrees(math.atan2(g[0],-g[2])), math.degrees(math.atan2(-g[1],-g[2])))
fr=[json.loads(l)['params'] for l in open(sys.argv[1]) if '"robot.state"' in l]
print(len(fr),'frames; t', round(fr[0]['t'],2),'→',round(fr[-1]['t'],2))
pol=[f['policy'] for f in fr]; i0=next((i for i,p in enumerate(pol) if p!='held'),None)
print('policy 序列:', [(p, pol.count(p)) for p in dict.fromkeys(pol)], '接管于帧', i0)
for i in range(0,len(fr),25):
    f=fr[i]; g=f['safety']['gravity']; p,r=tilt(g)
    print(f"t={f['t']-fr[0]['t']:5.2f} {f['policy']:6} 前倾 {p:+6.1f} 右倾 {r:+6.1f} fallen={f['safety']['fallen']} limp={f['safety']['limp']} gain={f['safety']['gain']} odom={np.round(f['odom']['position'][:2],3).tolist()} gyro={np.round(f['imu']['gyro'],2).tolist()}")
