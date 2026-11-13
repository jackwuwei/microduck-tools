#!/usr/bin/env python3
"""Body IMU mount calibration: compare the IMU's gravity with the one the joint angles imply.

stand    robotd holds the robot at HOME on a table; read joints and gravity from robotctl monitor
capture  servos limp, both soles held flat on a table; read the bus directly (robotd stopped)
solve    recompute the correction from a saved survey
"""
import argparse
import json
import math
import os
from pathlib import Path
import shlex
import subprocess
import sys
import time

import imu_math

ROOT = Path(__file__).resolve().parents[2]
SAMPLES, SAMPLE_INTERVAL = 10, .05
MAX_POSITION_SPREAD = 8         # counts, as the zero capture
MAX_GRAVITY_SPREAD_DEG = .5
MAX_FEET_DISAGREEMENT_DEG = 3.0
MIN_PER_HEADING = 3
MAX_JOINT_SPREAD = math.radians(.6)
MONITOR_SECONDS, MONITOR_HZ = 1.5, 50


class Rejected(ValueError):
    """This pose is not usable; the bus session stays open."""


def parse_mount(text):
    if text in imu_math.MOUNTS:
        return text, imu_math.MOUNTS[text]
    try:
        q = tuple(float(v) for v in text.split(','))
    except ValueError:
        q = ()
    if len(q) != 4 or abs(math.sqrt(sum(c*c for c in q)) - 1) > 1e-3:
        raise argparse.ArgumentTypeError('安装四元数应为 zero3、beta 或 w,x,y,z 单位四元数')
    return 'custom', imu_math.normalise(q)


def take(bus, model, trims, mount, heading, max_feet=MAX_FEET_DISAGREEMENT_DEG):
    readings = []
    for i in range(SAMPLES):
        readings.append(bus.snapshot(torque=i == 0))
        if i + 1 < SAMPLES:
            time.sleep(SAMPLE_INTERVAL)
    on = [motor for motor, t in zip(imu_math.JOINT_IDS, readings[0]['torque']) if t]
    if on:
        raise Rejected(f'舵机扭矩开启：ID {on}；标定要求全部松力')
    spread = [max(col) - min(col) for col in zip(*(r['positions'] for r in readings))]
    moving = [motor for motor, s in zip(imu_math.JOINT_IDS, spread) if s > MAX_POSITION_SPREAD]
    if moving:
        raise Rejected(f'关节尚未静止：ID {moving}；扶稳后重试')
    decoded = [imu_math.decode_block(r['imu'], mount) for r in readings]
    if any(g is None for _, g in decoded):
        raise Rejected('IMU 尚未输出姿态（SFLP 四元数为空）；上电后稍等再试')
    measured = imu_math.normalise([sum(c)/SAMPLES for c in zip(*(g for _, g in decoded))])
    wobble = max(math.degrees(imu_math.angle_between(g, measured)) for _, g in decoded)
    if wobble > MAX_GRAVITY_SPREAD_DEG:
        raise Rejected(f'IMU 读数在晃（{wobble:.2f}°）；扶稳后重试')
    positions = [sum(col)/SAMPLES for col in zip(*(r['positions'] for r in readings))]
    expected, feet, (left, right) = imu_math.expected_gravity(model, positions, trims)
    feet = math.degrees(feet)
    if feet > max_feet:
        (lp, lr), (rp, rr) = imu_math.tilt_deg(left), imu_math.tilt_deg(right)
        mp, mr = imu_math.tilt_deg(measured)
        raise Rejected(f'两只脚算出的机身姿态相差 {feet:.1f}°；两脚掌要同时平贴桌面'
                       f'（左脚推出 前倾 {lp:+.1f}° 右倾 {lr:+.1f}°，右脚 前倾 {rp:+.1f}° 右倾 {rr:+.1f}°，'
                       f'IMU 前倾 {mp:+.1f}° 右倾 {mr:+.1f}°）')
    gyro = [math.degrees(sum(c)/SAMPLES) for c in zip(*(w for w, _ in decoded))]
    return dict(time=time.time(), heading=heading, positions=positions,
                imu=[r['imu'] for r in readings], measured=measured, expected=expected,
                feet_disagreement_deg=feet, gyro_dps=gyro)


def monitor_frames(host, seconds=MONITOR_SECONDS, hz=MONITOR_HZ):
    """robot.state frames from `robotctl monitor --json` over SSH, for about `seconds`."""
    remote = shlex.join(['timeout', str(seconds), 'robotctl', 'monitor', '--json', '--hz', str(hz)])
    run = subprocess.run(['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', host, remote],
                         capture_output=True, text=True, timeout=seconds + 15)
    frames = []
    for line in run.stdout.splitlines():
        try:
            message = json.loads(line)
        except ValueError:
            continue
        if message.get('method') == 'robot.state':
            frames.append(message['params'])
    if not frames:
        raise ValueError('robotctl monitor 没有输出状态；robotd 在运行吗？' + run.stderr.strip()[-300:])
    return frames


def take_stand(frames, model, heading, max_feet=MAX_FEET_DISAGREEMENT_DEG):
    """One capture from robotd's own joints and gravity while it holds the robot still."""
    if len(frames) < 10:
        raise Rejected(f'只收到 {len(frames)} 帧状态，至少需要 10 帧')
    if any(f['safety']['limp'] or f['safety']['fallen'] for f in frames):
        raise Rejected('robotd 报告卸力或摔倒；先执行 robotctl robot init 站到 HOME')
    joints = [f['joints'] for f in frames]
    spread = [max(c) - min(c) for c in zip(*joints)]
    moving = [m for m, s in zip(imu_math.JOINT_IDS, spread) if s > MAX_JOINT_SPREAD]
    if moving:
        raise Rejected(f'关节在动：ID {moving}；松手、等它站稳后重试')
    gravities = [imu_math.normalise(f['safety']['gravity']) for f in frames]
    measured = imu_math.normalise([sum(c)/len(frames) for c in zip(*gravities)])
    wobble = max(math.degrees(imu_math.angle_between(g, measured)) for g in gravities)
    if wobble > MAX_GRAVITY_SPREAD_DEG:
        raise Rejected(f'IMU 读数在晃（{wobble:.2f}°）；等它站稳后重试')
    radians = [sum(c)/len(frames) for c in zip(*joints)]
    expected, feet, (left, right) = imu_math.expected_gravity_from_angles(model, radians)
    feet = math.degrees(feet)
    if feet > max_feet:
        (lp, lr), (rp, rr) = imu_math.tilt_deg(left), imu_math.tilt_deg(right)
        raise Rejected(f'两只脚算出的机身姿态相差 {feet:.1f}°（左脚 前倾 {lp:+.1f}° 右倾 {lr:+.1f}°，'
                       f'右脚 前倾 {rp:+.1f}° 右倾 {rr:+.1f}°）；看看是否一只脚没踩实')
    gyro = [math.degrees(sum(c)/len(frames)) for c in zip(*(f['imu']['gyro'] for f in frames))]
    return dict(time=time.time(), source='robotd', heading=heading, joints=radians,
                measured=measured, expected=expected, feet_disagreement_deg=feet, gyro_dps=gyro)


def describe(capture):
    m, e = imu_math.tilt_deg(capture['measured']), imu_math.tilt_deg(capture['expected'])
    return (f'朝向 {capture["heading"]:>3}°  IMU 前倾 {m[0]:+6.2f}° 右倾 {m[1]:+6.2f}°  '
            f'运动学 前倾 {e[0]:+6.2f}° 右倾 {e[1]:+6.2f}°  '
            f'差 {m[0]-e[0]:+5.2f}° / {m[1]-e[1]:+5.2f}°  双脚差 {capture["feet_disagreement_deg"]:.1f}°')


def report(survey):
    captures = survey['captures']
    counts = {h: sum(c['heading'] == h for c in captures) for h in (0, 180)}
    if not captures:
        print('还没有样本。')
        return None
    result = imu_math.solve(captures, survey['mount'])
    survey['result'] = result
    rx, ry = result['correction_deg']
    print(f'\n样本：朝向 0° {counts[0]} 个，180° {counts[180]} 个')
    for heading, (x, y) in result['per_heading_deg'].items():
        print(f'  朝向 {heading:>3}° 单独解：绕 X {x:+.2f}°，绕 Y {y:+.2f}°')
    if result['table_tilt_deg'] is None:
        print('  只有一个朝向：桌面倾斜会混进结果。把鸭子原地掉头 180° 再采几组可以抵消。')
    else:
        tx, ty = result['table_tilt_deg']
        print(f'  桌面倾斜（已抵消）：约 {math.hypot(tx, ty):.2f}°')
    if min(counts.values()) < MIN_PER_HEADING:
        print(f'  建议每个朝向至少 {MIN_PER_HEADING} 个样本，姿势各不相同。')
    print(f'修正 C：绕机身 X（横滚）{rx:+.2f}°，绕机身 Y（俯仰）{ry:+.2f}°；'
          f'即 IMU 原本多报前倾 {ry:+.2f}°、右倾 {rx:+.2f}°')
    residuals = result['residuals_deg']
    print(f'修正后残差：平均 {sum(residuals)/len(residuals):.2f}°，最大 {max(residuals):.2f}°')
    w, x, y, z = result['mount']
    print('\n写进 robotd（duck-control/src/imu.rs），在 body_imu_mount 里替换原安装四元数：\n')
    print(f'    /// Measured {time.strftime("%Y-%m-%d")} with microduck-tools imu-calibration: '
          f'{survey["mount_name"]} mount corrected by {rx:+.2f}° about X, {ry:+.2f}° about Y.')
    print(f'    pub const MEASURED_MOUNT: [f64; 4] = [{w:.9f}, {x:.9f}, {y:.9f}, {z:.9f}];\n')
    return result


def save(survey, path):
    tmp = path.with_suffix('.tmp')
    with tmp.open('w', encoding='utf-8') as file:
        json.dump(survey, file, ensure_ascii=False, indent=2)
        file.flush()
        os.fsync(file.fileno())
    tmp.replace(path)


def capture_command(args):
    from imu_bus import ImuLocalBus, ImuSshBus
    trims = imu_math.read_zero_trim(args.zero_trim)
    model = imu_math.Model(args.mjcf)
    mount = args.mount[1]
    if args.serial:
        bus, connection = ImuLocalBus(args.serial, args.baud, True), f'电脑串口 {args.serial}'
    else:
        bus = ImuSshBus(args.ssh, args.remote_serial, args.baud, True,
                        remote_python=args.remote_python, pythonpath=args.remote_pythonpath)
        connection = f'SSH {args.ssh} → {args.remote_serial}'
    survey, path = new_survey(args, connection, dict(zero_trim_source=str(args.zero_trim), zero_trim=trims))
    survey['max_feet_deg'] = args.max_feet_deg
    print(f'连接 {connection} …（robotd 必须已停止，舵机松力）', flush=True)
    bus.open()
    print('双手扶住机身，两只脚掌同时平贴水平桌面，每次换一个姿势（蹲低、站高、前后左右倾一点）。')
    try:
        interact(survey, path, lambda heading: take(bus, model, trims, mount, heading, args.max_feet_deg))
    finally:
        bus.close()


def new_survey(args, connection, extra):
    args.out_dir.mkdir(parents=True, exist_ok=True)
    mount_name, mount = args.mount
    if args.resume:
        survey = json.loads(args.resume.read_text(encoding='utf-8'))
        if survey['mount'] != list(mount) or any(survey.get(k) != v for k, v in extra.items() if not k.endswith('_source')):
            raise ValueError('续采记录的安装四元数或其他参数和本次不同；请新建记录')
        print(f'续采 {args.resume}：已有 {len(survey["captures"])} 个样本')
        return survey, args.resume
    path = args.out_dir / f'imu-survey-{time.strftime("%Y%m%d-%H%M%S")}.json'
    return dict(version=1, created_at=time.time(), connection=connection, mount_name=mount_name,
                mount=mount, mjcf=str(args.mjcf), captures=[], result=None, **extra), path


def stand_command(args):
    model = imu_math.Model(args.mjcf)
    survey, path = new_survey(args, f'robotctl monitor via SSH {args.ssh}', dict(source='robotd'))
    survey['max_feet_deg'] = args.max_feet_deg
    print('robotd 保持 HOME，鸭子自己站在水平桌面上，手离开机身；静止后采集。')
    print('机身重力取自 robotd，安装四元数须与 robotd 当前部署的一致（--mount）。')
    interact(survey, path, lambda heading: take_stand(monitor_frames(args.ssh), model, heading, args.max_feet_deg))


def interact(survey, path, take_one):
    heading = 0
    try:
        print('回车 采集 · t 原地掉头 180° 后切换朝向 · u 撤销上一个 · s 解算 · q 保存退出')
        while True:
            command = input(f'[朝向 {heading}°，已采 {len(survey["captures"])}] > ').strip().lower()
            if command == 'q':
                break
            if command == 't':
                heading = 180 - heading
                print(f'朝向切换为 {heading}°：把鸭子在桌上原地转 180°。')
            elif command == 'u':
                if survey['captures']:
                    survey['captures'].pop()
                    print('已撤销上一个样本。')
            elif command == 's':
                report(survey)
            elif command == '':
                try:
                    capture = take_one(heading)
                except Rejected as error:
                    print(f'未采集：{error}')
                    continue
                survey['captures'].append(capture)
                print(describe(capture))
            else:
                print('未知命令')
            save(survey, path)
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        if survey['captures']:
            report(survey)
        save(survey, path)
        print(f'记录已保存：{path}')


def solve_command(args):
    report(json.loads(args.survey.read_text(encoding='utf-8')))


def parse(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    commands = parser.add_subparsers(dest='command', required=True)
    capture = commands.add_parser('capture', help='连接机器人并采集')
    transport = capture.add_mutually_exclusive_group()
    transport.add_argument('--ssh', metavar='HOST', default='microduck', help='SSH 目标，默认 microduck')
    transport.add_argument('--serial', help='电脑本地串口，例如 /dev/cu.usbserial-...')
    capture.add_argument('--remote-serial', default='/dev/ttyS2')
    capture.add_argument('--remote-python', default='python3')
    capture.add_argument('--remote-pythonpath', action='append', metavar='PATH')
    capture.add_argument('--baud', type=int, default=1_000_000)
    capture.add_argument('--zero-trim', type=Path, required=True,
                         help='robotd 当前部署的 ZERO_TRIM_COUNTS：model.rs 或工作台的 *-trims.txt')
    stand = commands.add_parser('stand', help='robotd 带力站 HOME，从 robotctl monitor 采集')
    stand.add_argument('--ssh', metavar='HOST', default='microduck', help='SSH 目标，默认 microduck')
    for sub in (capture, stand):
        sub.add_argument('--mount', type=parse_mount, default=parse_mount('zero3'),
                         help='robotd 当前的安装四元数：zero3（默认）、beta 或 w,x,y,z')
        sub.add_argument('--mjcf', type=Path, default=imu_math.MJCF)
        sub.add_argument('--out-dir', type=Path, default=ROOT / 'work/imu-surveys')
        sub.add_argument('--resume', type=Path, metavar='SURVEY', help='接着一份已保存的记录继续采')
        sub.add_argument('--max-feet-deg', type=float, default=MAX_FEET_DISAGREEMENT_DEG,
                         help=f'两只脚推出的姿态最多相差多少度，默认 {MAX_FEET_DISAGREEMENT_DEG}')
    solve = commands.add_parser('solve', help='用已保存的记录重新解算')
    solve.add_argument('survey', type=Path)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse(argv)
    try:
        dict(capture=capture_command, stand=stand_command, solve=solve_command)[args.command](args)
    except (OSError, ValueError) as error:
        print(f'错误：{error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
