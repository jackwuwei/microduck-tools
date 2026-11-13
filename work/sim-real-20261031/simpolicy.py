"""velstand.onnx in MuJoCo with robotd's observation layout; optional IMU bias on the obs."""
import sys, math, numpy as np, mujoco, onnxruntime as ort
D='/Users/wuwei/workspace/microduck-simulator/app/public/robot/mjlab/'
NAMES=["left_hip_yaw","left_hip_roll","left_hip_pitch","left_knee","left_ankle","neck_pitch","head_pitch","head_yaw","head_roll","right_hip_yaw","right_hip_roll","right_hip_pitch","right_knee","right_ankle"]
HOME=np.array([0,-0.08726646259971647,-0.457924,-0.004940,0.452984,0.3490658503988659,0.3490658503988659,0,0,0,0.08726646259971647,0.457924,0.004940,-0.452984])
def axang(v):
    a=np.linalg.norm(v)
    if a<1e-12: return np.eye(3)
    k=v/a; K=np.array([[0,-k[2],k[1]],[k[2],0,-k[0]],[-k[1],k[0],0]])
    return np.eye(3)+math.sin(a)*K+(1-math.cos(a))*K@K
class Sim:
    def __init__(s, policy, seed=0):
        spec=mujoco.MjSpec.from_file(D+'robot_allcollisions.xml'); spec.meshdir=D+'meshes'
        spec.option.timestep=0.005
        spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE,size=[5,5,0.1],name='floor')
        s.m=spec.compile(); s.d=mujoco.MjData(s.m)
        s.qa=[s.m.joint(n).qposadr[0] for n in NAMES]; s.da=[s.m.joint(n).dofadr[0] for n in NAMES]
        s.gy=s.m.sensor('imu_ang_vel').adr[0]; s.trunk=s.m.body('trunk_base').id
        s.sess=ort.InferenceSession(policy); s.inp=s.sess.get_inputs()[0].name
        s.rng=np.random.default_rng(seed)
    def reset(s):
        mujoco.mj_resetData(s.m,s.d); s.d.qpos[2]=0.12; s.d.qpos[3:7]=[1,0,0,0]
        for a,v in zip(s.qa,HOME): s.d.qpos[a]=v
        s.d.ctrl[:]=HOME; mujoco.mj_forward(s.m,s.d); s.last=np.zeros(14,np.float32)
    def gravity_true(s):
        R=s.d.xmat[s.trunk].reshape(3,3); return R.T@np.array([0,0,-1.0])
    def step(s, cmd=(0,0,0), bias=np.zeros(3), scale=1.0):
        g=axang(bias)@s.gravity_true()     # robotd's gravity = C·g; a bias is a wrong C
        w=axang(bias)@s.d.sensordata[s.gy:s.gy+3]
        q=s.d.qpos[s.qa]; v=s.d.qvel[s.da]
        c=np.zeros(13); c[:3]=cmd
        obs=np.concatenate([w,g,q-HOME,v,s.last,c]).astype(np.float32)[None]
        a=s.sess.run(None,{s.inp:obs})[0][0]; s.last=a.astype(np.float32)
        # robotd's first-order low-pass on targets: 0.7 legs, 0.5 head (as trained)
        alpha=np.full(14,0.7); alpha[5:9]=0.5
        s.d.ctrl[:]=alpha*(HOME+scale*a)+(1-alpha)*s.d.ctrl
        for _ in range(4): mujoco.mj_step(s.m,s.d)
        return g
def tilt(g): return (math.degrees(math.atan2(g[0],-g[2])), math.degrees(math.atan2(-g[1],-g[2])))
def stand(sim, bias_deg=(0,0), seconds=10, cmd=(0,0,0)):
    sim.reset(); b=np.radians([bias_deg[0],bias_deg[1],0])
    true=[];obs=[];q=[]; fell=None
    for t in range(int(seconds*50)):
        g=sim.step(cmd,b); gt=sim.gravity_true()
        if gt[2]>-0.5 and fell is None: fell=t/50
        if t>=100: true.append(gt); obs.append(g); q.append(sim.d.qpos[sim.qa].copy())
    return fell, np.mean(true,0), np.mean(obs,0), np.mean(q,0)
if __name__=='__main__':
    sim=Sim(sys.argv[1])
    print('IMU 偏差注入（X 横滚, Y 俯仰）→ 机身真实前倾/右倾、策略看到的、关节（左髋前后/左膝/左踝/右髋前后/右膝/右踝）')
    for b in [(0,0),(0,5),(0,-5),(0,8),(0,-8),(3,0),(-3,0)]:
        fell,gt,go,q=stand(sim,b)
        j=np.degrees(q[[2,3,4,11,12,13]])
        print(f'bias X{b[0]:+} Y{b[1]:+}: 摔倒 {fell}  真实 {tilt(gt)[0]:+6.2f}/{tilt(gt)[1]:+6.2f}  策略看到 {tilt(go)[0]:+6.2f}/{tilt(go)[1]:+6.2f}  关节 {np.round(j,1)}')
