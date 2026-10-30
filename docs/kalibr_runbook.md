# Kalibr runbook (Mac host, Pi records)

Run Kalibr on the MacBook Air M5. Record bags on the Pi. The image is pulled from
Docker Hub — **there is no build step**. All flags below were checked against
`kalibr/master` sources and `rosbags` 0.11.9, not from memory; the `rosrun` prefixes and
`/data` output paths were checked against `prehensile/kalibr:arm64` itself.

Working dir on the Mac (outside git — bags are multi-GB):

```bash
mkdir -p ~/kalibr/w && cd ~/kalibr/w
```

Everything in `~/kalibr/w` gets bind-mounted to `/data` in the container, and Kalibr
writes its results next to the `--bag` path, so all outputs land back in `~/kalibr/w`.

Two properties of the image that every command below depends on:

* **Nothing `kalibr_*` is on `PATH`.** The entrypoint sources `devel/setup.bash`, but that
  only puts `/opt/ros/noetic/bin` on `PATH` — the scripts live in
  `/catkin_ws/devel/lib/kalibr/` and are not exported. Always call them as
  `rosrun kalibr kalibr_...`; a bare name dies with
  `exec: kalibr_x: not found` (exit 127).
* **The entrypoint does `cd /catkin_ws`**, which overrides `docker run -w`. Anything written
  to a *relative* path lands inside the container and is deleted by `--rm`, so give every
  output an absolute `/data/...` path.

---

## 0. Prerequisites (once)

### 0.1 Image

```bash
docker pull prehensile/kalibr:arm64        # ~1.17 GB, single arch linux/arm64
docker build -t kalibr-vio:arm64 docker/kalibr   # + one fix, one layer, seconds
```

Do **not** run `./build.sh` unless you actually need to patch Kalibr: the build wants
12 GB+ RAM *inside the Docker VM*, which on a 16 GB Mac means starving macOS.

`docker/kalibr/Dockerfile` is not that build — it is one extra layer on the pulled image
that patches a single function, `CameraIntializers.stereoCalibrate`. The stock version takes
the median of its per-view baselines with `np.median(np.asmatrix(r), axis=0)`, and this
image's numpy 1.17 raises `non-broadcastable output operand with shape (1,3)` whenever a
single view's PnP left a NaN — i.e. one blurry frame out of 771 kills the whole run. The
patch averages only the finite rows. Phase 3 and 5 use `kalibr-vio:arm64`; the utility calls
(target PDF, `rosbag info`) are identical on either tag.

### 0.2 Docker VM memory

The container cannot use more memory than the Linux VM, and the VM default is usually
~7.75 GB.

```bash
docker info --format '{{.MemTotal}}'       # want >= 10737418240 (10 GiB)
```

If lower: Docker Desktop → Settings → Resources → Memory → 10–12 GB → Apply & Restart.
(`docker run --memory 10g` does **not** work around this on macOS.)

### 0.3 rosbags (bag conversion, no ROS install needed)

**Python 3.10+ is required.** Jazzy writes rosbag2 metadata **version 9**, and the last
rosbags that supports Python 3.9 (macOS's `/usr/bin/python3`) is 0.9.23, which rejects that
with `Rosbag2 version 9 not supported`. Use a Homebrew interpreter:

```bash
brew install python@3.12
/opt/homebrew/opt/python@3.12/bin/python3.12 -m venv ~/venvs/rosbags
~/venvs/rosbags/bin/pip install -U pip rosbags      # 0.11.x
~/venvs/rosbags/bin/rosbags-convert --help
```

Pure Python — deliberately used *instead of* running ROS 1 on the Mac. Note the 0.10+ CLI
change: `--src` is a required flag now, and `--include-topic` takes all its values after a
single flag (see §4).

---

## 1. Calibration target (displayed on the 24" monitor)

The target is not printed. It is displayed fullscreen on the **24" matte monitor**, face-up on a desk,
driven from the Mac over HDMI. That gives a perfectly flat, free, always-available target and the
right geometry for a downward camera (the rig moves, the target does not).

The cost is size: a square 6x6 grid is limited by the panel's height, so it shrinks from 0.8 m to
~0.3 m and you calibrate at **0.20–0.35 m** with better frame coverage per view (grid pushed into
each corner in turn).

### 1.1 Display it, then measure it (once)

Display checklist — each item is a way to quietly ruin the bag:

- Native **1920x1080, 60 Hz**, no display scaling (macOS System Settings → Displays).
- **Page-fit / letterboxed**, never "fill screen". A stretched target breaks the squareness of the
  tags, and `tagSize` can only describe a *uniform* scale.
- Pointer parked off-screen or hidden; no menu bar, dock or window chrome over the grid.
- Night Shift **off** (it applies to external displays too), auto-brightness / power-saving off,
  screensaver and display sleep off.
- Bright panel, dim-ish room; keep the camera off the specular reflection angle — glare on a tag is
  unrecoverable and invisible in Kalibr's output until detection fails.

Then measure the **outer** grid span with a ruler or calipers — outermost tag edge to outermost tag
edge (left-to-right or top-to-bottom, they are equal).

**Watch the corner squares.** The generator also draws a `tagSpacing × tagSize` square at every tag
corner, so the outermost *black ink* sticks out `0.3 × tagSize` past each tag edge: the ink spans
`(7.5 + 2·0.3) × tagSize = 8.1 × tagSize`, while the tag edges span `7.5 × tagSize`. Measure tag
edge to tag edge and divide by 7.5, or measure to the ink and divide by 8.1 — mixing the two
inflates `tagSize` by 8 %, which inflates every translation in the result by 8 % with no visible
symptom. Cross-check against the side of one single tag: it equals `tagSize` to ~1 %.

**And the axis arrows count too.** The red/green axes sit outside the ink at the bottom-left, with
their tails at `1.5 × tagSpacing × tagSize` = `0.45 × tagSize` from the grid origin — `0.15 ×
tagSize` beyond the ink. So "everything I can see on screen" spans `8.25 × tagSize`. Three
divisors, three landmarks: **7.5** tag edges, **8.1** black ink, **8.25** ink + axis tails. A
one-noticeable-object mix-up is a silent 1–10 % scale error in every translation.

```yaml
# ~/kalibr/w/april_6x6.yaml
target_type: 'aprilgrid'
tagCols: 6
tagRows: 6
tagSize: 0.0398     # metres — measured span / 7.5 (tag edges) or / 8.1 (ink); never the nominal
tagSpacing: 0.3     # gap as a fraction of tagSize
```

The `0.0398` above assumes the tag span covers the panel's full 299 mm. The generated PDF page is
~331 mm square (it includes the corner squares and the axis arrows), so **page-fit** on a 295 mm
tall panel puts ~266 mm across the tag edges and ~288 mm across the ink => ~0.0355 by either
divisor, which is exactly what the cross-check is for. Scale the viewer up only if every tag stays
clear of the panel edge; a clipped corner square or tag corner is an unrecoverable detection
failure.

Why the geometry works out:

```
tag span  = tagCols*tagSize + (tagCols-1)*tagSpacing*tagSize = 7.5 * tagSize
ink span  = tag span + 2*tagSpacing*tagSize                  = 8.1 * tagSize
axis span = ink span + tagSpacing*tagSize/2                  = 8.25 * tagSize   (arrow tails)
=> tagSize = measured_span / (7.5, 8.1 or 8.25 — whichever landmark you measured to)
```

A 24" 16:9 panel is ~531 x 299 mm of active area, so a square grid tops out at 299 mm and
`tagSize ≈ 0.0398 m`. The measurement, not the formula, is what makes it correct: it absorbs whatever
scale the viewer produced, so the grid never has to be exactly 1080 px — the only requirements are
that the tags stay square and are at least ~100 px on screen.

A 1 % scale error is a 1 % systematic error in every translation in the result; the optimiser cannot
absorb it. Regenerate the PDF at the nominal size if you need one (page size is irrelevant once you
measure):

```bash
docker run --rm -v "$PWD:/data" prehensile/kalibr:arm64 \
  rosrun kalibr kalibr_create_target_pdf /data/april_6x6 \
    --type apriltag --nx 6 --ny 6 --tsize 0.0398 --tspace 0.3
```

The filename is a plain positional argument (`output`, default `target`), not a `--flag`,
so `/data/april_6x6` writes `~/kalibr/w/april_6x6.pdf`. Without it the file is written to
`/catkin_ws/target.pdf` inside the container and lost.

`tagRows`/`tagCols` only matter for non-square grids (Kalibr's AprilGrid is `tagCols`
along x, `tagRows` along y) — with 6x6 you are immune to that trap.

### 1.2 Screen-only failure modes

**PWM banding.** The backlight may dim by pulsing, and a global-shutter camera with a very short
exposure can catch the panel mid-refresh: a horizontal seam or brightness band across the tags.
Test before trusting it — record 5 s of static target at your intended exposure and look. Fix by
raising the exposure to **>=10–20 ms** and moving the rig more slowly (longer exposure trades banding
for motion blur; there is no free lunch, so pick the shortest exposure that shows no seam).

**A 24" panel at 1080p puts ~144 px on each tag**, which is close to how many the camera samples at
0.25 m. Tag borders are several pixels thick so corner fitting survives mild aliasing, but stay at
0.25 m or further rather than pushing to 0.20 m if edges look jagged in the frames.

### 1.3 The A0 print (cross-check only, deferred)

If a print happens later: matte paper, A0, 100 %/"actual size", glued to MDF or aluminium composite
(not foam board — warp is a systematic error), `tagSize = measured_span / 7.5 ≈ 0.10667` (same
`/7.5` vs `/8.1` caveat as above). Re-run
Phase 3 against it and require `fx`/`fy` to agree within ~0.5 % of the monitor result — agreement
validates both targets, disagreement means a scale error in one of the two yamls.

---

## 2. Record on the Pi

### 2.0 Before you record anything (once per session)

Two ways to end up with a bag that looks fine and isn't.

**Disk.** 1280x800 mono8 = 1.02 MB per eye, two eyes per published frame, ~29.5 fps
=> **~60 MB/s**, ~5.4 GB per 90 s. Recording is *best-effort* (see below), so a card
that cannot sustain that drops frames with no error at all. Record to a USB 3 SSD and
prove the write rate first:

```bash
dd if=/dev/zero of=/media/pi/SSD/testfile bs=1M count=2000 conv=fdatasync; rm /media/pi/SSD/testfile
```

Want **>= 180 MB/s** (3x the recording rate). If the SD card is all you have, keep the
bags short and let `tools/bag_qc.py` decide whether they are usable.

**QoS.** `stereo_node` publishes the image topics best-effort (`qos_profile_sensor_data`),
and a reliable subscriber **cannot connect to a best-effort publisher** — so the override
file must say `best_effort`, or you record zero images. `qos_override.yaml`:

```yaml
/camera/left/image_raw:
  reliability: best_effort
  history: keep_last
  depth: 10
  durability: volatile
/camera/right/image_raw:
  reliability: best_effort
  history: keep_last
  depth: 10
  durability: volatile
/imu/data_raw:
  reliability: reliable        # that is how imu_node publishes
  history: keep_last
  depth: 400                   # ~2 s of queue, so a brief writer stall loses no IMU
  durability: volatile
```

Drop the `/imu/data_raw` block for a camera-only bag.

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

### 2.1 Camera focus — set it once, at the distance you fly at

Both barrels of this module are threaded and focus independently, and they leave the
factory set to different distances. That asymmetry is invisible in a bag (the QC passes,
the images look plausible) and shows up only as a **2–4× reprojection gap between the two
cameras** in phase 3 — plus ~4× less high-frequency energy on the soft eye
(`tools/eye_sharpness.py`). Cleaning the lenses does not move it.

Focus at the **middle of your working range, not at the calibration distance**: intrinsics
barely depend on focus, but every flight frame does. At 1.45 mm focal length and ~f/2 the
depth of field is enormous, and blur is roughly `0.35 · |1/s − 1/s₀|` px (`s` = object,
`s₀` = focus), so:

| focus `s₀` | 0.25 m | 0.5 m | 1 m | 3 m | ∞ |
|-----------|--------|-------|-----|-----|----|
| 0.25 m | 0 | 0.7 | 1.05 | 1.3 | 1.4 |
| 0.35 m | 0.4 | 0.5 | 0.65 | 0.9 | 1.0 |
| **0.50 m** | **0.7** | **0** | **0.35** | **0.6** | **0.7** |
| 1.00 m | 1.05 | 0.35 | 0 | 0.25 | 0.35 |

0.5 m keeps the whole 0.25 m→infinity range under 0.7 px. Procedure: measure 0.5 m to a
detailed surface, open the live view at full resolution (`launch_foxglove:=true`), turn one
barrel until tag/edge transitions are a single crisp step, repeat on the **same surface at
the same distance** for the other eye (matched blur matters as much as low blur), then
tighten the locking screws and re-check — they shift focus slightly. Verify with a 10 s bag
at 0.5 m: `hf` right/left should be 0.9–1.1, not 0.25.

---

## 3. Transfer to the Mac

Over **Ethernet**, not Wi-Fi (Pi 5 has gigabit; stereo at 30 fps is ~60 MB/s of raw
frames, so bags run several GB).

```bash
# on the Mac
scp -r pi@192.168.2.4:~/bags/cam_calib ~/kalibr/w/
scp -r pi@192.168.2.4:~/bags/imu_cam  ~/kalibr/w/

# integrity, on both machines
(cd ~/kalibr/w && find cam_calib imu_cam -type f -print0 | sort -z | xargs -0 cat | shasum -a 256)
```

Copy, don't move — the Pi keeps the authoritative copy.

### 3.1 QC the bag — before converting

This also validates the copy, so run it on the Mac after `scp`:

```bash
~/venvs/rosbags/bin/python tools/bag_qc.py ~/kalibr/w/cam_calib --report
```

Reads the ROS 2 bag directly (no conversion). Checks: topic inventory, per-topic rate,
header-stamp gap structure (dropped / duplicated frames), image geometry,
left/right stamp parity, and whether the recorder kept up with the disk. Exit 1 means
do not run Kalibr on this bag.

It proves the *recording* is honest. It cannot judge focus, motion blur or whether the
target ever filled both frames — that is what the Kalibr report is for.

The gap check is a diagnosis, not a verdict: `kalibr_calibrate_cameras` runs with
`--bag-freq 10.0` and discards two of every three frames on purpose, so moderate drops
are harmless for Phase 3. They matter for Phase 5 (IMU and camera excitation must cover
the same windows) and for OpenVINS in Phase 7 (it needs both eyes of each pair).

---

## 4. Convert ROS 2 → ROS 1

`--include-topic` keeps only what Kalibr needs (smaller, and no converter surprises).
Kalibr for `kalibr_calibrate_imu_camera` needs **only** `/imu/data_raw`; the
`/imu/accel` + `/imu/gyro` `Vector3Stamped` topics must stay out.

```bash
cd ~/kalibr/w
RB=~/venvs/rosbags/bin/rosbags-convert

# rosbags 0.10+: --src is a flag, and one --include-topic takes all the topics
$RB --src cam_calib --dst cam_calib.bag \
  --include-topic /camera/left/image_raw /camera/right/image_raw

$RB --src imu_cam --dst imu_cam.bag \
  --include-topic /camera/left/image_raw /camera/right/image_raw /imu/data_raw
```

A `.bag` destination selects the ROS 1 writer (`bag v2.0`); any other suffix writes a
ROS 2 bag. `--dst-storage`/`--dst-version` only apply to the ROS 2 direction.

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

# stdin carries the focal seed -- see the bullets
printf '483\n' | docker run --rm -i -v "$PWD:/data" -e MPLBACKEND=Agg \
  -e MPLCONFIGDIR=/tmp/mpl kalibr-vio:arm64 \
  rosrun kalibr kalibr_calibrate_cameras \
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
* **No `xvfb-run`.** With `MPLBACKEND=Agg` and no `--plot` / `--show-extraction` Kalibr
  never opens a display, and `xvfb-run` occasionally wedges inside Docker waiting for
  Xvfb's `SIGUSR1` readiness signal: the container sits at 0 % CPU with only `Xvfb`
  running, Kalibr never starts, and nothing is printed. `-i` (not `-it`) is there for the
  focal seed below.
* **The focal seed on stdin.** The image sets `KALIBR_MANUAL_FOCAL_LENGTH_INIT=1`, so when
  `initializeIntrinsics()` finds no usable guess it *prompts on stdin* (and reads
  `2.08e-315` from closed stdin, leaving the projection `[nan nan nan nan]`). That happens
  when no recorded view holds a **complete** 6×6 grid — the initialiser skips any image with
  a missing corner, and at 640×400 a 255 mm board only fits the 400 px height from ~0.33 m
  out, which a 0.20–0.25 m bag need never contain. Seed it with a plausible `fx` (~480 for
  this lens/resolution, or the other camera's converged value); it only has to sit in the
  right basin. A `RuntimeError: stereo initialisation: no view produced a finite pose pair`
  is the downstream symptom of getting this wrong.
* No `--output` flag exists — results go next to the bag.

Outputs: `cam_calib-camchain.yaml` (the one you keep), `-report-cam.pdf`, `-results-cam.txt`.

Acceptance: reprojection RMS **< 0.5 px** for both cameras, and the report's residual
plots look like noise, not structure. Rerun with a different `--bag-from-to` window if not.

**One camera much worse than the other is optics, not the model.** Run
`~/venvs/rosbags/bin/python tools/eye_sharpness.py ~/kalibr/w/cam_calib`: it compares the two
eyes on the same frames, so exposure and motion cancel, and separates contrast from focus
(`sharp` = edge strength / contrast, `hf` = share of energy above 1/4 Nyquist). Three runs on
the same rig, same target:

| run | `hf` right/left | left resid | right resid |
|-----|-----------------|-----------|-------------|
| 1 | 0.257 | 0.140 px | 0.643 px |
| 2 (lenses cleaned) | 0.256 | 0.117 px | 0.547 px |
| 3 (barrels refocused at 0.5 m) | **0.958** | 0.101 px | 0.102 px |

Cleaning raised sharpness on *both* eyes and left the ratio frozen to three digits: that is
optical focus, not contamination. Refocusing the right barrel took the gap from 4.6× to 1.02×
and the residual from 0.64 to 0.10 px. No `--models` choice or `--bag-from-to` window can
substitute for this — check it **before** burning a calibration run (see §2.1).

Current keeper: **`cam_calib3-camchain.yaml`** (run 3, 874 images, left 482.93/482.89 px,
right 481.91/481.62 px, baseline 64.46 mm, residuals 0.101 / 0.102 px). Runs 1 and 2 are
superseded but kept for reference.

---

## 6. Phase 4 — IMU noise (Allan variance)

Produces the four numbers Kalibr and OpenVINS need — gyro/accel noise density and random
walk — by leaving the IMU perfectly still for hours and watching how the variance of the
average grows with averaging time: white noise averages away (slope −1/2), bias random walk
does not (slope +1/2), and the flat minimum between them is the bias instability.

`allan_variance_ros` (the ROS 2 port) reads rosbag2 directly, so **no ROS 1 conversion** on
this phase: record, trim, compute, done.

### 6.1 Build (once)

`gcc` ships with the Pi image, **`g++` does not**. Without it `colcon` dies with
`CMake Error ... No CMAKE_CXX_COMPILER could be found`, which reads like a ROS problem and
is not:

```bash
sudo apt-get install -y g++ python3-matplotlib python3-scipy
cd ~/ros2_ws/src
git clone --depth 1 https://github.com/cwha0212/allan_variance_ros2.git allan_variance_ros
cd ~/ros2_ws && colcon build --packages-select allan_variance_ros && source install/setup.bash
```

The repo is named `...ros2` but the **package** is `allan_variance_ros` — that is the name
in `ros2 run`. Verified to build in 34 s on Jazzy / Pi 5 (2026-10-30).

### 6.2 Record (hours, unattended)

The IMU block in `tools/qos_override.yaml` must be **active** (it is, by default): `depth 400`
is 2 s at 200 Hz, where the rosbag2 default of 10 is 50 ms — over 11 hours one writer stall
at depth 10 silently drops samples, and missing IMU cannot be reconstructed.

Do **not** wait 30 min for thermal equilibrium before pressing record; record while the rig
warms up and cut it out afterwards (§6.3). Two reasons: the wait is dead time, and starting
early captures the drift you want to *exclude*, which is easier than guessing when it ended.

Run it in `tmux` — an SSH drop must not kill an overnight recording:

```bash
tmux new -s allan
cd ~/ros2_ws && source install/setup.bash
ros2 launch sensor_launch sensors.launch.py launch_imu:=true launch_foxglove:=false
# second pane (Ctrl-B then % to split):
ros2 bag record -o ~/ros2_ws/bags/imu_static /imu/data_raw \
  --qos-profile-overrides-path ~/ros2_ws/bags/qos_override.yaml
# Ctrl-B D detaches;  tmux attach -t allan  gets you back
```

Clock it: 200 Hz x ~400 B/msg is ~0.3 GB/h, so 11 h ≈ 3.3 GB — check `df -h ~` first, and
remember the trimmed copy needs room too (§6.3). Leave the rig in its final enclosure, lid
on, mains powered, on a surface nothing will bump.

### 6.3 Trim the stationary window

rosbag2 has no trim command, and `ros2 bag play --rate` + re-record would rewrite the very
timestamps the analysis reads. `tools/trim_bag.py` copies the window out and copies message
bytes unchanged (raw CDR, no deserialise), so `header.stamp` is bit-exact:

```bash
# raw bag -> Mac (the rosbags venv lives there), trim, trimmed bag -> Pi, drop the raw
scp -r pi@192.168.2.4:~/ros2_ws/bags/imu_static ~/kalibr/w/
~/venvs/rosbags/bin/python tools/trim_bag.py \
  ~/kalibr/w/imu_static ~/kalibr/w/imu_static_trim --skip 1800 --tail 300
scp -r ~/kalibr/w/imu_static_trim pi@192.168.2.4:~/ros2_ws/bags/
rm -rf ~/kalibr/w/imu_static                       # archived; the Pi copy too
```

`--skip 1800` drops the 30 min warm-up, `--tail 300` drops the last 5 min (you walking over
to stop it). Both are non-stationary and inflate the long-cluster deviation — exactly the
region the bias-instability number is read from.

### 6.4 Run

The config must match the *trimmed* bag: `imu_rate` is the real publish rate and
`sequence_time` is the analysed length, because the tool counts messages rather than reading
stamps (`imu_counter / imu_rate > sequence_time` ends the run). See `tools/allan_imu.yaml`.

```bash
mkdir -p ~/allan && cd ~/allan
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run allan_variance_ros allan_variance \
  ~/ros2_ws/bags/imu_static_trim ~/ros2_ws/tools/allan_imu.yaml
MPLBACKEND=Agg ros2 run allan_variance_ros analysis.py \
  --data allan_variance.csv --config ~/ros2_ws/tools/allan_imu.yaml
```

Outputs in `~/allan`: `allan_variance.csv`, `acceleration.png`, `gyro.png`, and the thing we
actually want, **`imu.yaml`**. Copy the lot to the Mac (`acceleration.png`/`gyro.png` are the
plots; `MPLBACKEND=Agg` means `plt.waitforbuttonpress` returns immediately and saves instead
of waiting for a click).

### 6.5 What good looks like

On both plots: a straight −1/2 slope at short τ, then a minimum, then a +1/2 rise. The
fitted lines (magenta = white noise, yellow = random walk) should track the curves over their
respective regions. Numbers for an MPU-6500-class part land near:

| quantity | Kalibr key | plausible | suspicious |
|---|---|---|---|
| gyro noise density | `gyroscope_noise_density` | 1e-4–5e-4 rad/s/√Hz | > 1e-3 (or < 1e-5) |
| gyro random walk | `gyroscope_random_walk` | 1e-6–1e-5 rad/s²/√Hz | > 1e-4 |
| accel noise density | `accelerometer_noise_density` | 1e-3–1e-2 m/s²/√Hz | > 1e-1 |
| accel random walk | `accelerometer_random_walk` | 1e-5–1e-4 m/s³/√Hz | > 1e-3 |

An order of magnitude outside those ranges is a hardware finding, not an analysis bug (this
is the cloned-MPU-6500 failure mode: it would show up here first). Check the two last lines
of `imu.yaml` too — `rostopic: '/imu/data_raw'` and `update_rate: 200` must be right, since
Kalibr and OpenVINS read them.

As your notes say, keep the raw numbers and plan **10–20× inflation** when tuning VIO: Allan
variance measures the sensor on a bench, and a hovering quadcopter feeds it a far nastier
signal.

---

## 7. Phase 5 — camera–IMU extrinsics + time offset

Needs Phase 3 (`camchain`) and Phase 4 (Allan-variance `imu.yaml`) first.

```bash
cd ~/kalibr/w

docker run --rm -v "$PWD:/data" -e MPLBACKEND=Agg -e MPLCONFIGDIR=/tmp/mpl \
  kalibr-vio:arm64 \
  rosrun kalibr kalibr_calibrate_imu_camera \
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

## 8. Repeat cost (why the Mac)

A full Phase 5 run on this data is minutes on the M5 vs. hours on the Pi 5, where 8 GB is
also shared with the ROS 2 stack — and calibration is inherently iterative (window choice,
target yaml, camera model, padding). The bag transfer is a one-time few-minute cost;
putting the runs on the Pi is a recurring one.

If you ever do want the Pi path, nothing is locked in: `docker pull prehensile/kalibr:arm64`
there, or `docker save prehensile/kalibr:arm64 | gzip > kalibr.tar.gz` (~1.17 GB) and
`gunzip -c kalibr.tar.gz | docker load`.

---

## 9. Troubleshooting

| symptom | cause / fix |
| --- | --- |
| `exec: kalibr_...: not found` / exit 127 | the `kalibr_*` scripts are not on `PATH` in this image — prefix with `rosrun kalibr`. |
| run "succeeds" but no output file appears | the entrypoint `cd /catkin_ws` overrode `-w`; pass an absolute `/data/...` output path. |
| `No CMAKE_CXX_COMPILER could be found` | `gcc` is installed but `g++` is not — `sudo apt-get install -y g++`. |
| `analysis.py` IndexError on `np.where(period == 10)` | the CSV has no τ = 10 s, i.e. the analysed sequence was too short for the long clusters — usually `sequence_time` disagreeing with the bag. |
| container alive, 0 % CPU, no output at all | `xvfb-run` wedged waiting for Xvfb's readiness signal — drop it, `MPLBACKEND=Agg` is enough. |
| `Initializing focal length to 2.08e-315`, projection `[nan …]` | no complete-grid view, so the prompt read closed stdin — pipe a focal seed (`printf '483\n' | docker run -i …`). |
| `ValueError: non-broadcastable output operand … (1,3) … (3,3)` | stock image's numpy 1.17 chokes on one NaN pose — use `kalibr-vio:arm64`. |
| `rosbag info` shows extra topics | recovery used `--exclude-topic` incompletely; re-run with `--include-topic`. |
| no images shown / "no views" | target yaml `/data` path typo, or target not detected — re-check `tagSize` against the **measured span / 7.5**, and that the *whole* grid was visible. |
| tags detected but RMS > 1 px, both cameras | display scaling or a glossy reflection: verify native 1080p, page-fit (not fill), no glare on the panel. |
| horizontal band/seam across tags | PWM banding on the panel — raise the exposure to >=10–20 ms and re-record (§1.2). |
| asymmetric residual, one camera only | check nothing else was recording on the Pi (frame drops); re-record with the QoS override. |
| `timeshift_cam_imu` at the padding edge | raise `--timeoffset-padding`; if it keeps growing, your camera stamping is drifting. |
| Linux OOM-killer / container dies | Docker VM memory (step 0.2). |
| Output files owned by root | add `--user "$(id -u):$(id -g)"` to `docker run`. |
| matplotlib backend crash | keep `-e MPLBACKEND=Agg` (and `-e MPLCONFIGDIR=/tmp/mpl` if it cannot write `~/.config`). |
