# prehensile/kalibr:arm64 plus one fix to the stereo initialiser.
#
# kalibr_camera_calibration/CameraIntializers.py takes the median of its
# per-view baselines with np.median(np.asmatrix(r), axis=0), which raises on
# this image's numpy 1.17 whenever a single view's PnP failed and left a NaN --
# i.e. one blurry frame kills a whole calibration run. See the script for the
# replacement.
#
# This is a small extra layer, NOT the from-source Kalibr build (that one wants
# 12 GB+ of RAM inside the Docker VM and is never needed for this project).
#
#   docker build -t kalibr-vio:arm64 docker/kalibr
FROM prehensile/kalibr:arm64

COPY fix_stereo_initializer.py /tmp/
RUN python3 /tmp/fix_stereo_initializer.py && rm /tmp/fix_stereo_initializer.py
