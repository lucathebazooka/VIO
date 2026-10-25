# Kalibr runbook (Mac host, Pi records)

Run Kalibr on the MacBook Air M5. Record bags on the Pi. The image is pulled from
Docker Hub — **there is no build step**. All flags below were checked against
`kalibr/master` sources and `rosbags` 0.9.23, not from memory.

Working dir on the Mac (outside git — bags are multi-GB):

```bash
mkdir -p ~/kalibr/w && cd ~/kalibr/w
```

Everything in `~/kalibr/w` gets bind-mounted to `/data` in the container, and Kalibr
writes its results next to the `--bag` path, so all outputs land back in `~/kalibr/w`.

---

## 0. Prerequisites (once)

### 0.1 Image

```bash
docker pull prehensile/kalibr:arm64        # ~1.17 GB, single arch linux/arm64
```

Do **not** run `./build.sh` unless you actually need to patch Kalibr: the build wants
12 GB+ RAM *inside the Docker VM*, which on a 16 GB Mac means starving macOS.

### 0.2 Docker VM memory

The container cannot use more memory than the Linux VM, and the VM default is usually
~7.75 GB.

```bash
docker info --format '{{.MemTotal}}'       # want >= 10737418240 (10 GiB)
```

If lower: Docker Desktop → Settings → Resources → Memory → 10–12 GB → Apply & Restart.
(`docker run --memory 10g` does **not** work around this on macOS.)

### 0.3 rosbags (bag conversion, no ROS install needed)

```bash
python3 -m venv ~/venvs/rosbags
~/venvs/rosbags/bin/pip install -U rosbags
~/venvs/rosbags/bin/rosbags-convert --help
```

Pure Python — deliberately used *instead of* running ROS 1 on the Mac.

---

## 1. Calibration target yaml (once)

`~/kalibr/w/april_6x6.yaml`:

```yaml
target_type: 'aprilgrid'
tagCols: 6
tagRows: 6
tagSize: 0.10667    # metres — MEASURE the printed tag edge, do not trust this
tagSpacing: 0.3     # gap as a fraction of tagSize
```

`tagSize` follows from the grid footprint Kalibr expects:

```
span = tagCols*tagSize + (tagCols-1)*tagSpacing*tagSize = 7.5 * tagSize
```

so a 0.8 m 6x6 grid at 30 % gaps gives `tagSize = 0.8/7.5 = 0.10667`. Regenerate the PDF if
you need one (runs fine headless; it writes into the mounted dir):

```bash
docker run --rm -v "$PWD:/data" prehensile/kalibr:arm64 \
  kalibr_create_target_pdf --type apriltag --nx 6 --ny 6 --tsize 0.10667 --tspace 0.3
```

Print at **100 % / "actual size"**, never "fit to page", then measure one tag edge with
calipers and edit `tagSize`. A 1 % scale error is a 1 % systematic error in every
translation in the result; the optimizer cannot absorb it.
`tagRows`/`tagCols` only matter for non-square grids (Kalibr's AprilGrid is `tagCols`
along x, `tagRows` along y) — with 6x6 you are immune to that trap.

---

## 2. Record on the Pi

Camera-only, 1–2 min, whole frame well covered, target crisp (short exposure, low gain,
good light — motion blur is unrecoverable):

```bash
ros2 bag record -o cam_calib \
  /camera/left/image_raw /camera/right/image_raw \
  --qos-profile-overrides-path qos_override.yaml
```

Combined camera+IMU, 30–60 s, slow smooth excitation on **all** axes (translate + rotate
each axis, no jerks, keep the target fully visible in both cameras at all times):

```bash
ros2 bag record -o imu_cam \
  /camera/left/image_raw /camera/right/image_raw /imu/data_raw \
  --qos-profile-overrides-path qos_override.yaml
```

Record only these topics. Every extra topic inflates the transfer and breaks the converter.

---

## 3. Transfer to the Mac

Over **Ethernet**, not Wi-Fi (Pi 5 has gigabit; stereo at 30 fps is ~60 MB/s of raw
frames, so bags run several GB).

```bash
# on the Mac
scp -r pi@192.168.2.2:~/bags/cam_calib ~/kalibr/w/
scp -r pi@192.168.2.2:~/bags/imu_cam  ~/kalibr/w/

# integrity, on both machines
(cd ~/kalibr/w && find cam_calib imu_cam -type f -print0 | sort -z | xargs -0 cat | shasum -a 256)
```

Copy, don't move — the Pi keeps the authoritative copy.

---

## 4. Convert ROS 2 → ROS 1

`--include-topic` keeps only what Kalibr needs (smaller, and no converter surprises).
Kalibr for `kalibr_calibrate_imu_camera` needs **only** `/imu/data_raw`; the
`/imu/accel` + `/imu/gyro` `Vector3Stamped` topics must stay out.

```bash
cd ~/kalibr/w
RB=~/venvs/rosbags/bin/rosbags-convert

$RB cam_calib --dst cam_calib.bag \
  --include-topic /camera/left/image_raw --include-topic /camera/right/image_raw

$RB imu_cam --dst imu_cam.bag \
  --include-topic /camera/left/image_raw --include-topic /camera/right/image_raw \
  --include-topic /imu/data_raw
```

Sanity check before spending CPU on calibration:

```bash
docker run --rm -v "$PWD:/data" --entrypoint bash prehensile/kalibr:arm64 \
  -c 'source /catkin_ws/devel/setup.bash && rosbag info /data/cam_calib.bag && rosbag info /data/imu_cam.bag'
```

Expect: two image topics at ~30 Hz, `/imu/data_raw` at ~200 Hz, no other topics.

---

## 5. Phase 3 — intrinsics + stereo extrinsics

```bash
cd ~/kalibr/w

docker run --rm -it -v "$PWD:/data" -e MPLBACKEND=Agg prehensile/kalibr:arm64 \
  xvfb-run -a kalibr_calibrate_cameras \
    --bag /data/cam_calib.bag \
    --topics /camera/left/image_raw /camera/right/image_raw \
    --models pinhole-radtan pinhole-radtan \
    --target /data/april_6x6.yaml \
    --bag-freq 10.0 \
    --dont-show-report
```

* `--bag-freq 10.0` thins 30 Hz to 10 Hz of the *best* frames (mutual-information
  selected). 3× less work, no accuracy loss.
* `--models`: keep `pinhole-radtan` as in your notes. If the residual at the frame edges
  is systematically worse than the centre, rerun with `pinhole-equi`.
* `xvfb-run -a` + `MPLBACKEND=Agg`: exits cleanly with no X11 quay. Omit both only if you
  have XQuartz and want the plots.
* No `--output` flag exists — results go next to the bag.

Outputs: `cam_calib-camchain.yaml` (the one you keep), `-report-cam.pdf`, `-results-cam.txt`.

Acceptance: reprojection RMS **< 0.5 px** for both cameras, and the report's residual
plots look like noise, not structure. Rerun with a different `--bag-from-to` window if not.

---

## 6. Phase 5 — camera–IMU extrinsics + time offset

Needs Phase 3 (`camchain`) and Phase 4 (Allan-variance `imu.yaml`) first.

```bash
cd ~/kalibr/w

docker run --rm -it -v "$PWD:/data" -e MPLBACKEND=Agg prehensile/kalibr:arm64 \
  xvfb-run -a kalibr_calibrate_imu_camera \
    --bag /data/imu_cam.bag \
    --cams /data/cam_calib-camchain.yaml \
    --imu /data/imu.yaml \
    --target /data/april_6x6.yaml \
    --timeoffset-padding 0.06 \
    --dont-show-report
```

Flag traps, all verified in the source:

* It is `--cams` (plural, one camchain file), **not** `--cam`, and there is **no
  `--topics`** — topics come from the camchain yaml.
* Time calibration is **on by default**; there is no `--time-calibration` flag to add
  (the switch is the negative `--no-time-calibration`).
* `--timeoffset-padding` defaults to **±30 ms**. Your camera stamps are ~18 ms stale by
  construction, and the estimate must fit *inside* the padding or it gets clamped, so
  0.06 gives it room. If the estimated `timeshift_cam_imu` ends up pinned at the padding
  edge, the timestamps are worse than you think.

Outputs:

| file | goes to |
| --- | --- |
| `imu_cam-camchain-imucam.yaml` | OpenVINS `estimator_config.yaml` (`T_cam_imu` → invert to `T_imu_cam`, keep `timeshift_cam_imu`) |
| `imu_cam-imu.yaml` | OpenVINS `kalibr_imu_chain.yaml` (add `model: "kalibr"`, identity `Tw`/`Ta`/`R_IMUtoGYRO`/`R_IMUtoACC`, zero `Tg`) |
| `-results-imucam.txt`, `-report-imucam.pdf` | inspection only |

```bash
grep -E 'timeshift_cam_imu|T_cam_imu' -A4 imu_cam-camchain-imucam.yaml
```

Acceptance: reprojection RMS < 0.5 px, spline fit visually smooth in the PDF, accel/gyro
residuals white and within ~3σ, estimated biases small and stable.

---

## 7. Repeat cost (why the Mac)

A full Phase 5 run on this data is minutes on the M5 vs. hours on the Pi 5, where 8 GB is
also shared with the ROS 2 stack — and calibration is inherently iterative (window choice,
target yaml, camera model, padding). The bag transfer is a one-time few-minute cost;
putting the runs on the Pi is a recurring one.

If you ever do want the Pi path, nothing is locked in: `docker pull prehensile/kalibr:arm64`
there, or `docker save prehensile/kalibr:arm64 | gzip > kalibr.tar.gz` (~1.17 GB) and
`gunzip -c kalibr.tar.gz | docker load`.

---

## 8. Troubleshooting

| symptom | cause / fix |
| --- | --- |
| `rosbag info` shows extra topics | recovery used `--exclude-topic` incompletely; re-run with `--include-topic`. |
| no images shown / "no views" | target yaml `/data` path typo, or target not detected — check `tagSize` against the print and that the *whole* grid was visible. |
| asymmetric residual, one camera only | check nothing else was recording on the Pi (frame drops); re-record with the QoS override. |
| `timeshift_cam_imu` at the padding edge | raise `--timeoffset-padding`; if it keeps growing, your camera stamping is drifting. |
| Linux OOM-killer / container dies | Docker VM memory (step 0.2). |
| Output files owned by root | add `--user "$(id -u):$(id -g)"` to `docker run`. |
| matplotlib backend crash | keep `-e MPLBACKEND=Agg` and the `xvfb-run -a` prefix. |
