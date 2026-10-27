# Kalibr runbook (Mac host, Pi records)

Run Kalibr on the MacBook Air M5. Record bags on the Pi. The image is pulled from
Docker Hub — **there is no build step**. All flags below were checked against
`kalibr/master` sources and `rosbags` 0.9.23, not from memory; the `rosrun` prefixes and
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
edge (left-to-right or top-to-bottom, they are equal) — and write:

```yaml
# ~/kalibr/w/april_6x6.yaml
target_type: 'aprilgrid'
tagCols: 6
tagRows: 6
tagSize: 0.0398     # metres — MEASURED span / 7.5, do not trust the nominal value
tagSpacing: 0.3     # gap as a fraction of tagSize
```

The `0.0398` above assumes the grid spans the panel's full 299 mm. The generated PDF page is
~331 mm square around a ~299 mm grid (it includes the corner squares and the axis arrows), so
**page-fit** display on this panel yields a ~270 mm span => measure and write `~0.036` unless you
scale the viewer up to make the grid fill the screen.

Why the geometry works out:

```
span    = tagCols*tagSize + (tagCols-1)*tagSpacing*tagSize = 7.5 * tagSize
=> tagSize = measured_span / 7.5
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
(not foam board — warp is a systematic error), `tagSize = measured_span / 7.5 ≈ 0.10667`. Re-run
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
  xvfb-run -a rosrun kalibr kalibr_calibrate_cameras \
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
  xvfb-run -a rosrun kalibr kalibr_calibrate_imu_camera \
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
| `exec: kalibr_...: not found` / exit 127 | the `kalibr_*` scripts are not on `PATH` in this image — prefix with `rosrun kalibr`. |
| run "succeeds" but no output file appears | the entrypoint `cd /catkin_ws` overrode `-w`; pass an absolute `/data/...` output path. |
| `rosbag info` shows extra topics | recovery used `--exclude-topic` incompletely; re-run with `--include-topic`. |
| no images shown / "no views" | target yaml `/data` path typo, or target not detected — re-check `tagSize` against the **measured span / 7.5**, and that the *whole* grid was visible. |
| tags detected but RMS > 1 px, both cameras | display scaling or a glossy reflection: verify native 1080p, page-fit (not fill), no glare on the panel. |
| horizontal band/seam across tags | PWM banding on the panel — raise the exposure to >=10–20 ms and re-record (§1.2). |
| asymmetric residual, one camera only | check nothing else was recording on the Pi (frame drops); re-record with the QoS override. |
| `timeshift_cam_imu` at the padding edge | raise `--timeoffset-padding`; if it keeps growing, your camera stamping is drifting. |
| Linux OOM-killer / container dies | Docker VM memory (step 0.2). |
| Output files owned by root | add `--user "$(id -u):$(id -g)"` to `docker run`. |
| matplotlib backend crash | keep `-e MPLBACKEND=Agg` and the `xvfb-run -a` prefix. |
