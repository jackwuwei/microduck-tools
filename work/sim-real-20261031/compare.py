import sys, json, math, numpy as np
sys.path.insert(0,'/Users/wuwei/workspace/microduck-tools/tools/imu-calibration'); import imu_math as m
def tilt(g): return np.array([math.degrees(math.atan2(g[0],-g[2])), math.degrees(math.atan2(-g[1],-g[2]))])
K=m.Model()
L=['L髋转','L髋侧','L髋前后','L膝','L踝','颈','头俯','头转','头侧','嘴','R髋转','R髋侧','R髋前后','R膝','R踝']
def summ(rows, real):
    rows=[r for r in rows if r['policy']!='held'][50:]   # skip the first second after handover
    J=np.degrees(np.mean([r['joints'] for r in rows],0)); T=np.degrees(np.mean([r['targets'] for r in rows],0))
    g=np.mean([r['safety']['gravity'] if real else r['gravity'] for r in rows],0)
    kin=tilt(m.expected_gravity_from_angles(K,list(np.radians(J)))[0])
    out=dict(seen=tilt(g),kin=kin,J=J,T=T,err=J-T)
    if not real: out['true']=tilt(np.mean([r['gravity_true'] for r in rows],0))
    return out
real=summ([json.loads(l)['params'] for l in open(sys.argv[1]) if '"robot.state"' in l],True)
sims={f:summ([json.loads(l) for l in open(f)],False) for f in sys.argv[2:]}
print(f"{'':22}{'策略看到的重力':>16}{'运动学推出':>14}{'仿真真值':>12}")
print(f"{'实机(新安装)':22}{real['seen'][0]:+7.2f}/{real['seen'][1]:+6.2f}  {real['kin'][0]:+6.2f}/{real['kin'][1]:+6.2f}")
for f,s in sims.items(): print(f"{f.split('sim-stand-')[1][:-6]:22}{s['seen'][0]:+7.2f}/{s['seen'][1]:+6.2f}  {s['kin'][0]:+6.2f}/{s['kin'][1]:+6.2f}  {s['true'][0]:+6.2f}/{s['true'][1]:+6.2f}")
idx=[1,2,3,4,11,12,13,14]
print('\n关节实测（°）      '+' '.join(f'{L[i]:>7}' for i in idx))
print('实机              '+' '.join(f'{real["J"][i]:+7.2f}' for i in idx))
for f,s in sims.items(): print(f'{f.split("sim-stand-")[1][:-6]:18}'+' '.join(f'{s["J"][i]:+7.2f}' for i in idx))
print('\n跟踪误差 实测−目标  '+' '.join(f'{L[i]:>7}' for i in idx))
print('实机              '+' '.join(f'{real["err"][i]:+7.2f}' for i in idx))
for f,s in sims.items(): print(f'{f.split("sim-stand-")[1][:-6]:18}'+' '.join(f'{s["err"][i]:+7.2f}' for i in idx))
