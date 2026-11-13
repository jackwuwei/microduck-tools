import sys, math, numpy as np, mujoco
sys.path.insert(0, '/Users/wuwei/workspace/microduck-tools/tools/imu-calibration')
import imu_math as m
D='/Users/wuwei/workspace/microduck-simulator/app/public/robot/mjlab/'
spec=mujoco.MjSpec.from_file(D+'robot_allcollisions.xml'); spec.meshdir=D+'meshes'
spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[2,2,0.1], name='floor')
model=spec.compile(); data=mujoco.MjData(model)
kp=20.0
model.actuator_gainprm[:,0]=kp; model.actuator_biasprm[:,1]=-kp; model.actuator_biasprm[:,2]=-0.5
model.actuator_forcerange[:]=[-50,50]; model.actuator_forcelimited[:]=1
names=[model.actuator(i).name for i in range(model.nu)]
HOME=dict(zip(m.JOINT_NAMES, m.HOME))
def run(pose, seconds=3.0):
    mujoco.mj_resetData(model,data)
    for n,v in pose.items():
        j=model.joint(n) if n in [model.joint(i).name for i in range(model.njnt)] else None
        if j is not None: data.qpos[j.qposadr[0]]=v
    data.qpos[2]=0.2; mujoco.mj_forward(model,data)
    data.ctrl[:]=[pose[n] for n in names]
    for _ in range(int(seconds/model.opt.timestep)): mujoco.mj_step(model,data)
    imu=data.site('imu'); R=imu.xmat.reshape(3,3)
    g_true=R.T@np.array([0,0,-1.0])
    q={model.joint(i).name:data.qpos[model.joint(i).qposadr[0]] for i in range(1,model.njnt)}
    ang=[q.get(n,0.0) for n in m.JOINT_NAMES]
    exp,dis,(l,r)=m.expected_gravity_from_angles(m.Model(), ang)
    feet={f:math.degrees(math.acos(min(1,data.site(f).xmat.reshape(3,3)[2,2]))) for f in ('left_foot','right_foot')}
    return g_true,exp,dis,l,r,ang,feet
print('timestep',model.opt.timestep,'nu',model.nu)
for label,pose in (('HOME',HOME),('HOME 髋侧摆 0',{**HOME,'left_hip_roll':0.0,'right_hip_roll':0.0})):
    g,e,d,l,r,ang,feet=run(pose)
    print(f'{label}: 仿真IMU真值 前倾/右倾 {m.tilt_deg(g)[0]:+.2f}/{m.tilt_deg(g)[1]:+.2f}  我的运动学 {m.tilt_deg(e)[0]:+.2f}/{m.tilt_deg(e)[1]:+.2f}  双脚差 {math.degrees(d):.2f}  脚站点z与竖直夹角 L{feet["left_foot"]:.2f} R{feet["right_foot"]:.2f}')
    print('   hip roll', [round(math.degrees(a),2) for a in (ang[1],ang[11])], 'hip pitch',[round(math.degrees(a),2) for a in (ang[2],ang[12])],'knee',[round(math.degrees(a),2) for a in (ang[3],ang[13])],'ankle',[round(math.degrees(a),2) for a in (ang[4],ang[14])])
