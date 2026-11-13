import sys, json, math, numpy as np, mujoco
sys.path.insert(0,'/Users/wuwei/workspace/microduck-tools/tools/imu-calibration')
import imu_math as m
D='/Users/wuwei/workspace/microduck-simulator/app/public/robot/mjlab/'
spec=mujoco.MjSpec.from_file(D+'robot_allcollisions.xml'); spec.meshdir=D+'meshes'
spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE,size=[2,2,0.1],name='floor')
model=spec.compile(); data=mujoco.MjData(model)
kp=30.0
model.actuator_gainprm[:,0]=kp; model.actuator_biasprm[:,1]=-kp; model.actuator_biasprm[:,2]=-0.5
model.actuator_forcerange[:]=[-80,80]; model.actuator_forcelimited[:]=1
acts=[model.actuator(i).name for i in range(model.nu)]
qa={n:model.joint(n).qposadr[0] for n in acts}
def settle(angles):
    mujoco.mj_resetData(model,data); data.qpos[2]=0.13
    for n in acts: data.qpos[qa[n]]=angles[n]
    data.ctrl[:]=[angles[n] for n in acts]; mujoco.mj_forward(model,data)
    tb=model.body('trunk_base').id; x0=data.xpos[tb][:2].copy()
    for k in range(int(3/model.opt.timestep)):
        # A hand on the trunk: horizontal force only, applied at the trunk's centre of mass,
        # so it stops the robot tipping over without setting its attitude.
        data.xfrc_applied[tb,:2]=-60*(data.xpos[tb][:2]-x0)-6*data.cvel[tb][3:5]
        mujoco.mj_step(model,data)
    R=data.site('imu').xmat.reshape(3,3); g=R.T@np.array([0,0,-1.0])
    err=max(abs(data.qpos[qa[n]]-angles[n]) for n in acts)
    feet={f:math.degrees(math.acos(min(1,data.site(f).xmat.reshape(3,3)[2,2]))) for f in ('left_foot','right_foot')}
    return g, math.degrees(err), feet
def deg(g): t=m.tilt_deg(g); return f'{t[0]:+6.2f}/{t[1]:+6.2f}'
OLD=m.MOUNTS['zero3']; NEW=(0.676852027,-0.011079456,0.735952325,-0.011079456)
C=m.mul(NEW,m.conj(OLD))     # new = C ⊗ old, so g_new = C·g_old
model_k=m.Model()
print('样本            | 仿真IMU真值 | 运动学(脚站点) | 实机IMU旧安装 | 实机−仿真 | 运动学−仿真 | 关节跟随误差 | 脚站点z倾角L/R')
for f in sys.argv[1:]:
    sv=json.load(open(f)); new=sv['mount_name']=='custom'
    for i,c in enumerate(sv['captures']):
        if 'joints' in c: ang=c['joints']
        else: ang=[m.counts_to_radians(p,t) for p,t in zip(c['positions'],sv['zero_trim'])]
        angles=dict(zip(m.JOINT_NAMES,ang))
        g,err,feet=settle(angles)
        meas=c['measured']
        old=m.rotate_inverse(C,meas) if new else meas
        kin=m.expected_gravity_from_angles(model_k,ang)[0]
        dr=[a-b for a,b in zip(m.tilt_deg(old),m.tilt_deg(g))]; dk=[a-b for a,b in zip(m.tilt_deg(kin),m.tilt_deg(g))]
        tag=('验证' if new else ('站立' if 'joints' in c else '松力'))+f'#{i} 朝向{c["heading"]}'
        print(f'{tag:14}| {deg(g)} | {deg(kin)} | {deg(old)} | {dr[0]:+5.2f}/{dr[1]:+5.2f} | {dk[0]:+5.2f}/{dk[1]:+5.2f} | {err:.2f}° | {feet["left_foot"]:.1f}/{feet["right_foot"]:.1f}')
