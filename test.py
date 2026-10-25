"""Split cap.read() into transfer vs JPEG decode.

CONVERT_RGB=0 makes read() return the raw MJPG bytes without decoding, so we
can time the USB transfer separately from cv2.imdecode.

    transfer ms   USB dequeue only (no decode). ~16.7 ms = camera really does
                60 fps and we are waiting for it; ~50 ms = the link is the limit
    decode ms     pure cv2.imdecode cost
    KB            compressed frame size; x60 tells you the bandwidth the
                camera needs (x2 if you want both halves as separate JPEGs)
"""
import time
import cv2

DEVICE = 0
WIDTH, HEIGHT = 2560, 800
CAPTURE_FPS = 60
WINDOW_S = 5.0
WARMUP = 10

cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
if not cap.isOpened():
    raise SystemExit('cannot open /dev/video0 - is stereo_node running?')
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
cap.set(cv2.CAP_PROP_FPS, CAPTURE_FPS)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)      # hand us raw MJPG, do not decode
print(f'convert_rgb={cap.get(cv2.CAP_PROP_CONVERT_RGB)} '
        f'{int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x'
        f'{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}')

for _ in range(WARMUP):
    cap.read()

n = 0
t_xfer = t_dec = t_split = 0.0
kb = 0.0
xfer_min, xfer_max = float('inf'), 0.0
dec_min, dec_max = float('inf'), 0.0
t_win = time.monotonic()
print('measuring - Ctrl-C to stop')

try:
    while True:
        t0 = time.monotonic()
        ok, buf = cap.read()
        t1 = time.monotonic()
        if not ok or buf is None:
            print('read failed - stopping')
            break
        kb += buf.nbytes / 1024.0

        t2 = time.monotonic()
        img = cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE)
        t3 = time.monotonic()
        if img is None:
            print('imdecode failed - stopping')
            break
        half = img.shape[1] // 2
        left, right = img[:, :half], img[:, half:]
        t4 = time.monotonic()

        t_xfer += t1 - t0
        t_dec += t3 - t2
        t_split += t4 - t3
        xfer_min = min(xfer_min, t1 - t0)
        xfer_max = max(xfer_max, t1 - t0)
        dec_min = min(dec_min, t3 - t2)
        dec_max = max(dec_max, t3 - t2)
        n += 1

        now = time.monotonic()
        win = now - t_win
        if win >= WINDOW_S:
            if n:
                print(
                    f'{n:3d} frames ({n / win:5.1f} Hz) | '
                    f'transfer {t_xfer / n * 1e3:6.1f} ms '
                    f'[{xfer_min * 1e3:5.1f}-{xfer_max * 1e3:5.1f}] | '
                    f'decode {t_dec / n * 1e3:6.1f} ms '
                    f'[{dec_min * 1e3:5.1f}-{dec_max * 1e3:5.1f}] | '
                    f'split {t_split / n * 1e3:5.2f} ms | '
                    f'{kb / n:6.0f} KB/frame'
                )
            n = 0
            t_xfer = t_dec = t_split = 0.0
            kb = 0.0
            xfer_min, xfer_max = float('inf'), 0.0
            dec_min, dec_max = float('inf'), 0.0
            t_win = now
except KeyboardInterrupt:
    pass
finally:
    cap.release()