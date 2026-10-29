"""Patch Kalibr's stereo initialiser so one failed PnP cannot kill the run.

kalibr_camera_calibration/CameraIntializers.py:32 takes the median of the
per-view baselines with

    np.median(np.asmatrix(r), axis=0).flatten().T

Two problems with that on this image's numpy 1.17.4:

  * a view whose PnP failed (blur, glare, partial grid) contributes NaN, and
    numpy's `_median_nancheck` then does `result[n] = nan` on a *matrix* result
    of shape (1,3) with a mask of shape (3,3), raising
    "non-broadcastable output operand with shape (1,3) doesn't match the
    broadcast shape (3,3)" -- one bad frame out of 771 kills the run;
  * `np.asmatrix(...).flatten()` returns a matrix, which is what makes that
    shape check fail in the first place.

Replace it with plain arrays and average only the finite rows, which is what a
median over failed estimates should have been doing. Idempotent; refuses if the
source does not look like the version this patch was written for.
"""

import sys
from pathlib import Path

TARGET = Path(
    '/catkin_ws/src/kalibr/aslam_offline_calibration/kalibr/'
    'python/kalibr_camera_calibration/CameraIntializers.py'
)

OLD = """        r_median = np.median(np.asmatrix(r), axis=0).flatten().T
        R_median = rv.parametersToRotationMatrix(r_median)
        t_median = np.median(np.asmatrix(t), axis=0).flatten().T
"""

NEW = """        # A view whose PnP failed (blur, glare, partial grid) contributes NaN,
        # and numpy's median of a matrix raises on that instead of ignoring it.
        # Average only the views that produced a finite pose pair.
        r_arr = np.asarray(r, dtype=float).reshape(-1, 3)
        t_arr = np.asarray(t, dtype=float).reshape(-1, 3)
        keep = np.isfinite(r_arr).all(axis=1) & np.isfinite(t_arr).all(axis=1)
        if not keep.any():
            raise RuntimeError(
                'stereo initialisation: no view produced a finite pose pair')
        r_median = np.median(r_arr[keep], axis=0)
        R_median = rv.parametersToRotationMatrix(r_median)
        t_median = np.median(t_arr[keep], axis=0)
"""

src = TARGET.read_text()
if NEW in src:
    print('already patched')
    sys.exit(0)

found = src.count(OLD)
if found != 1:
    sys.exit(f'expected 1 median block, found {found} -- refusing to patch')

TARGET.write_text(src.replace(OLD, NEW))
print('patched', TARGET)
