# microduck-tools

[简体中文](README.md) | **English**

A zero-calibration workbench for Microduck's XL330 servos, plus the matching V3 calibration tray. The workbench runs on your computer. By default it uses the controller's servo port over SSH; a USB serial adapter connected directly to the computer also works.

| Folder | Contents |
| --- | --- |
| `tools/xl330-calibration/` | Workbench: web page, backend, 3D renderer, tests |
| `assets/` | 3D model, photos, screenshots |
| `CAD/` | V3 tray STEP / STL |
| `patches/` | robotd software zero-trim patch |
| `work/` | Locally generated backups and surveys (not committed) |

## Contents

- [Photos and workbench](#workbench)
- [Install and connect](#connection)
- [Calibration procedure](#calibration-flow)
- [Apply to robotd](#apply-calibration)
- [Appendix: EEPROM writes](#eeprom)
- [Power and tests](#offline-tests)
- [Tray printing](#cad-printing)
- [Sources and licenses](#licenses)

<a id="workbench"></a>

## Photos and workbench

The robot lies on its back in the tray, with the head, torso and feet each resting in their own area. Photo backgrounds were blurred with AI assistance.

![Side photo of the robot in the calibration tray](assets/photos/robot-on-tray-side.jpg)

| Tray, top view | Robot placement, top view |
| --- | --- |
| ![Top photo of the calibration tray showing the split seam and two keys](assets/photos/calibration-tray-top.jpg) | ![Top photo of the robot on its back in the tray](assets/photos/robot-on-tray-top.jpg) |

![English calibration workbench: joint selection, highlighted servo and calibration panel](assets/screenshots/calibration-en.jpg)

- Supports 15 **XL330-M288-T servos (model 1200) in position mode 3**.
- Writes only the Drive Mode direction bit, Homing Offset and Min/Max Position Limit in EEPROM. It never enables torque, sends goal positions, stops other programs or changes the IMU.
- Backs up the original registers to `work/xl330-calibration-backups/` on every connection.
- The 3D model highlights the selected servo and shows parts hidden by the shell as translucent. Drag to rotate; double-click to reset.
- Switch between 中文 and English at the top. Diagnostics from external systems stay in their original language.

Offline demo (no hardware needed):

```bash
./start-calibration.sh --demo
```

Then open <http://127.0.0.1:18092/>.

<a id="connection"></a>

## Install and connect

Install dependencies once from the repository root on your computer (macOS / Linux):

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
```

Pick one connection method. The baud rate defaults to **1000000**, and hardware runs use the **Microduck 2S profile (6.6–8.2 V)** by default; about 7.9–8.0 V is normal.

| Method | Path | Command |
| --- | --- | --- |
| **SSH (default)** | Computer → SSH `microduck` → controller `/dev/ttyS2` → servos | `./start-calibration.sh` |
| USB direct | Computer → USB serial adapter → servos | `./start-calibration.sh --serial COMPUTER_PORT` |

<a id="ssh-connection"></a>

### SSH (default)

Nothing is deployed on the controller and no port forwarding is needed. While connected, the workbench runs a temporary serial helper in the controller's memory over SSH. The helper exits and releases the port on disconnect.

1. Add an alias to `~/.ssh/config` on your computer (replace the host and user). Log in with a key or agent; the repository never stores passwords:

   ```sshconfig
   Host microduck
       HostName YOUR_ROBOT_HOST
       User microduck
   ```

   Check: `ssh -o BatchMode=yes microduck true` should succeed without prompting. Use `--ssh user@host` for another target.

2. On first use, log in to the controller (Python 3.9+) and install only the serial dependencies:

   ```bash
   python3 -m pip install --target "$HOME/.local/share/microduck-serial/deps" \
     dynamixel-sdk==4.0.5 pyserial==3.5
   ```

   The older `~/diagnostics/readlibs` folder is also searched. Use `--remote-python` to choose the interpreter and `--remote-pythonpath` (repeatable) to replace the dependency folders.

3. On the controller, make sure nothing else is using the bus; stop anything that is, using its normal procedure:

   ```bash
   pgrep -af 'robotd|observer|server.py'
   fuser -v /dev/ttyS2
   ```

   If permissions block the check, use an account that can see it. Empty output alone does not prove the port is free.

4. On your computer run `./start-calibration.sh`, open <http://127.0.0.1:18092/>, tick the readiness box and click **Check and connect**.
5. When done, click **Disconnect** on the page, press `Ctrl+C` in the terminal, then restart the controller software as usual.

<a id="usb-connection"></a>

### USB direct

You need an adapter for the DYNAMIXEL TTL half-duplex bus. The controller's ordinary USB port does not provide the servo bus.

1. With power off, connect the bus and a common ground, then restore the robot's original 2S supply. The adapter is for communication only; do not power the robot from two sources. Stop the controller from using the bus.
2. Find the port on your computer (macOS usually `/dev/cu.usbserial-…`, Linux usually `/dev/ttyUSB0`), check it is free, then start:

   ```bash
   .venv/bin/python -m serial.tools.list_ports -v
   SERIAL_PORT=/dev/cu.usbserial-YOUR_DEVICE
   lsof "$SERIAL_PORT"
   ./start-calibration.sh --serial "$SERIAL_PORT"
   ```

3. Use the browser and finish the same way as with SSH.

<a id="connection-issues"></a>

### Connection issues

- **Port 18092 in use:** `lsof -nP -iTCP:18092 -sTCP:LISTEN`. If it is an old SSH web tunnel, close it first.
- **SSH fails:** check `ssh -o BatchMode=yes microduck true`, then the controller's serial permissions, port owner and dependencies.
- **Detection fails:** follow the on-page message about model, mode, torque, temperature, bus or power.
- **Capture or validation error** (joint not still, offset out of range, …): the session stays connected and captures are kept. Fix the issue and retry.
- **Communication lost or write failed:** the workbench disconnects and never retries writes. An interrupted write may be partial; after reconnecting, check the registers against the backup taken before the failure.

<a id="calibration-flow"></a>

## Calibration procedure

This procedure captures zero only, for robotd software trims: **do not click "Set normal direction" / "Set reversed direction", capture limits, or "Write selected joint and verify".** Direction and limits come from the mechanical assembly; never change them to make a reading closer to 0.

```mermaid
flowchart TD
    A[Support the robot, servos limp<br/>stop other bus programs] --> B[Start the workbench and connect]
    B --> C[Select a joint, starting with ID 23 left knee]
    C --> D[View "Selected +10°", nudge the joint<br/>reading should increase]
    D --> E[Pose it like "Zero 0°", hold still<br/>click "Capture current as 0°"]
    E --> F[Check "Zero✓", record sample N]
    F --> G{All 14 joints done?}
    G -- No --> C
    G -- Yes --> H[Recheck large offsets, save results]
    H --> I[Disconnect, update robotd]
```

### 1. Check the baseline after connecting

- All 15 servos online and limp; a backup was created.
- "Offset": knees ID 13 and 23 are **−512**, all others **0**.

Support the robot. When checking a joint, hold the neighbouring links; hold the head for neck joints. Left and right are from the robot's perspective. **Readings in a random limp pose are not errors; pose the joint like the model's zero first.**

### 2. For each joint

1. Select the joint on the left and confirm the title and live reading changed.
2. Click **"Zero 0°"** and pose the real joint like the model. Do not use "HOME standing pose" or "Current encoder" as the zero reference.
3. Click **"Selected +10°"** and gently move the joint the way the model shows. The angle should **increase**. If it decreases or does not change, stop and check the ID, view and assembly; do not click the direction buttons.
4. Back on "Zero 0°", pose and hold the joint still, then click **"Capture current as 0°"**. If the ten samples spread more than 8 pulses you'll be asked to hold it steady and retry.
5. Check that the capture box shows `Zero: N pulse` and the joint list shows `Zero✓`. **Record N from the capture box**, not the live reading after you let go. Offset = N − 2048 (1° ≈ 11.38 pulses).

The live angle does not become 0° after capture; the trim only takes effect in robotd.

| Order | ID / joint | View | Check |
| --- | --- | --- | --- |
| 1 | 23 left knee | Side | Thigh and shin straight |
| 2 | 20 left hip yaw | Top | Toes point straight ahead |
| 3 | 21 left hip roll | Front | Leg neither splayed out nor tucked in |
| 4 | 22 left hip pitch | Side | Thigh angle relative to the torso |
| 5 | 24 left ankle | Side | Sole angle relative to the shin |
| 6–10 | 13, 10, 11, 12, 14 right leg | As left | Cross-check against the left side |
| 11 | 30 neck pitch | Side | Hold the head; neck angle relative to the torso |
| 12 | 31 head pitch | Side | Head tilt up / down |
| 13 | 32 head yaw | Top | Beak faces straight ahead |
| 14 | 33 head roll | Front | Head level |

**ID 34 beak:** the model's beak is fixed, so there is no zero reference. Close it gently and note the reading only; do not capture it. Its trim stays 0.

### 3. Save and recheck

Captures are not saved to disk automatically, and the backup does not contain them. When all joints are captured and you are still connected, save them from the repository root in another terminal:

```bash
mkdir -p work/zero-surveys
curl --fail --silent --show-error http://127.0.0.1:18092/api/state \
  -o "work/zero-surveys/zero-survey-$(date +%Y%m%d-%H%M%S).json"
```

Check that `joints[].marks.zero` has a value for all 14 non-beak joints. **Do not disconnect or reconnect before saving**: reconnecting clears captures.

Recapture joints that are off by more than about 3° or that you posed inconsistently, and confirm the capture box shows the new result. Hand posing repeats to about ±1.5°, so don't chase one or two pulses. Save again after rechecking, then disconnect.

<a id="apply-calibration"></a>

## Apply to robotd

For Pollen Robotics `microduck` **daemon-v0.17.8** (commit `942e58e3aa6dbb28e7037203fff11866122c7567`). The [patch](patches/robotd-v0.17.8-gyro-zero-trim.patch) compiles the zero offsets into `robotd`:

- `duck-control/src/model.rs`: the `ZERO_TRIM_COUNTS` array, all 0 by default.
- `duck-control/src/bus.rs`: subtracts the trim from position reads and adds it to goal writes.
- `duck-control/src/imu.rs`: a separate IMU v1.2 gyro unit fix (`count / 16.4` → °/s); keep it if your IMU uses that protocol.

On startup robotd forces Homing Offset to −512 for ID 13 and 23 and 0 for the rest, so use software trims rather than EEPROM writes. Don't keep the same offset in both EEPROM and the array, and copying a backup JSON to the controller does not apply anything.

### 1. Fill in the trims

```text
ZERO_TRIM_COUNTS[joint] = zero sample N − 2048
```

This assumes the Homing Offset baseline above. If offsets were changed, the safest fix is to restore the baseline and capture again.

The array follows `JOINT_IDS` order, **not ascending ID**. The values below are an example only; do not use them:

```rust
pub const ZERO_TRIM_COUNTS: [i32; NUM_JOINTS] = [
    -25, 16, 5, 3, 8,        // left leg: 20 21 22 23 24
    -3, -6, -3, -54, 0,      // neck, head, beak: 30 31 32 33 34
    -51, -40, -42, -22, -10, // right leg: 10 11 12 13 14
];
```

Keep ID 34 at 0 and leave `DEFAULT_POSITION` (the standing pose) unchanged. Change the `zero_trims_are_neutral_by_default` test to check your robot's measured values.

### 2. Build and deploy

On your computer, check out the pinned baseline and apply the patch (don't overwrite an existing folder or apply the patch twice):

```bash
TOOLS_ROOT="$PWD"
git clone https://github.com/pollen-robotics/microduck.git ../microduck-runtime
cd ../microduck-runtime
git switch -c calibration-v0.17.8 942e58e3aa6dbb28e7037203fff11866122c7567
git apply --check "$TOOLS_ROOT/patches/robotd-v0.17.8-gyro-zero-trim.patch"
git apply "$TOOLS_ROOT/patches/robotd-v0.17.8-gyro-zero-trim.patch"
```

After filling in the trims, run `cargo test -p duck-control`, then build and install following the upstream [development build and deployment guide](https://github.com/pollen-robotics/microduck/blob/942e58e3aa6dbb28e7037203fff11866122c7567/docs/robot/dev-push.md), e.g. `scripts/dev-push.sh microduck`. Deploying restarts services, so **disconnect the workbench first**.

### 3. Verify and roll back

1. Confirm systemd runs the new binary (`systemctl cat robotd`), logs are clean and the IMU is ready.
2. With the robot unmoved, switching from an untrimmed build should change feedback by `−trim × 2π / 4096`.
3. Posed limp at mechanical zero, feedback is close to 0; moving in the model's positive direction increases it.
4. Check that policy targets plus trims stay inside the EEPROM position limits.

To roll back, restore the previous array, rebuild and reinstall.

<a id="eeprom"></a>

## Appendix: EEPROM writes (optional)

Use only if you deliberately calibrate in EEPROM and the controller will not overwrite these registers. After capturing zero and both limits, click "Preview register changes" to review, then write:

```text
new Homing Offset = old Homing Offset + 2048 − zero sample
```

- The offset must stay within ±1024, otherwise reinstall the horn. Limits must be within 0–4095 and surround 2048. Wraps are never hidden with modulo arithmetic.
- The direction buttons change only the Drive Mode direction bit, and write and read back immediately.
- You can restore the selected joint to its pre-connection registers, or from an older backup. **Keep the backup folder** and make sure a backup comes from the same robot.
- On a write or read-back error the workbench disconnects and the write may be partial. After reconnecting, restore from the backup taken before the failure, not the one created on reconnect.

<a id="offline-tests"></a>

## Power and tests

**2S supply:** hardware runs default to `--microduck-2s`, which requires 6.6–8.2 V and allows (and displays) the input-voltage alert bit 0x01. Other hardware errors, torque on or temperature ≥ 60°C stop operation. Shutdown and voltage limits are never modified. Source: [Microduck official design §3.2](https://github.com/pollen-robotics/microduck/blob/main/docs/design/robotd-design.md#32-state-out).

**Offline tests** (simulated registers; not a substitute for checking real hardware):

```bash
python3 -m unittest discover -s tools/xl330-calibration -p 'test_*.py' -v
node --test tools/xl330-calibration/test_i18n.cjs
```

<a id="cad-printing"></a>

## Tray printing

| Part | STEP | STL | Qty |
| --- | --- | --- | ---: |
| Upper half | [head.step](CAD/head.step) | [head.stl](CAD/head.stl) | 1 |
| Lower half with NP-F550 clearance fix | [legs_npf550_r1.step](CAD/legs_npf550_r1.step) | [legs_npf550_r1.stl](CAD/legs_npf550_r1.stl) | 1 |
| Butterfly key | [key.step](CAD/key.step) | [key.stl](CAD/key.stl) | 2 |

Units are mm. Print flat side down at 100% scale with a 0.4 mm nozzle, PETG and Bambu's default 0.20 mm Standard process. If you already have the upper half and keys, print only the lower half.

Hold the robot with soft hook-and-loop straps, avoiding the battery, clips and cables; remove it before motion tests. Fit was checked against the standard NP-F550 battery shape; 25–26 mm thick batteries may not fit. Test the 0.4 mm clearance physically. The tray is not a precision angle gauge.

Slicer estimates (Bambu Studio 02.08.02.61 CLI, P1S, Bambu PETG Basic, 2026-10-27; not measured on a real print):

| Plate | Parts | Filament | Estimated time incl. startup |
| --- | --- | ---: | ---: |
| 1 | `head` ×1 + `key` ×2 | 78.30 g | 2 h 17 min |
| 2 | `legs_npf550_r1` ×1 | 46.20 g | 1 h 44 min |
| **Full set** | **4 parts** | **124.50 g** | **4 h 1 min** |

Have on hand: 55 g for the lower half only, 140 g for the full set.

<a id="licenses"></a>

## Sources and licenses

**Original software in this repository is licensed under [Apache-2.0](LICENSE)**, including original code in `tools/`, launcher scripts and tests. Patches to Apache-2.0 upstream software are provided under Apache-2.0 with attribution retained. This license does not cover robot geometry, CAD / STL / STEP, photos or third-party dependencies.

| Content | Source and terms |
| --- | --- |
| Robot reference geometry, UI robot model and derived fixtures | Pollen Robotics [`microduck_rl`](https://github.com/pollen-robotics/microduck_rl/tree/d424a0c899f6b33cbd3daeb279913134349c0b63), commit `d424a0c899f6b33cbd3daeb279913134349c0b63`. The pinned README says CC BY-SA-NC for model files without a version number; no additional version or commercial permission is implied. See the [pinned upstream README](https://github.com/pollen-robotics/microduck_rl/blob/d424a0c899f6b33cbd3daeb279913134349c0b63/README.md#license). Software Apache-2.0 does not cover mechanical models. |
| XL330 workbench and 3D renderer | Original code under [Apache-2.0](LICENSE). The interaction takes inspiration from [kissqy/Microduck-kissqy](https://github.com/kissqy/Microduck-kissqy); the DYNAMIXEL Protocol 2.0 backend is an independent implementation, and no source code, calibration data or UI assets were copied. Registers follow the [official XL330 control table](https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/). |
| Controller adaptation patch | Based on the pinned Pollen Robotics [`microduck` commit](https://github.com/pollen-robotics/microduck/tree/942e58e3aa6dbb28e7037203fff11866122c7567), provided under Apache-2.0. The patch defaults to zero trims; fill in your own measurements. |
| Photos | Robot and tray photos supplied by the user for this project's documentation; code and model licenses do not apply to them automatically. |
| 3D-printed calibration fixture | The V3 zero tray takes inspiration from 星猫's XGO Duck neutral tray (`R7_X2D04_PETG_slice_reference.gcode.3mf`, CC BY-NC-SA 4.0), borrowing only the on-back positioning approach. This design and its derived geometry keep attribution and non-commercial terms under CC BY-NC-SA 4.0, without extending the upstream model license. |

This is an unofficial personal research reproduction and is not an official Pollen Robotics release.
