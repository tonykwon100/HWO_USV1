# AprilTag + RTSP 카메라 구현 가이드

> 수상 드론 자율 주행 — USB Camera 기반 AprilTag 감지 및 RTSP 스트리밍

---

## 1. 카메라 캘리브레이션 (필수 선행작업)

AprilTag는 내부 파라미터 없이는 정확한 pose 추정 불가.

```bash
pip install opencv-python numpy
```

```python
# calibrate.py - 체커보드 기반
import cv2
import numpy as np
import glob

CHECKERBOARD = (9, 6)  # 내부 코너 수
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= 0.025  # 체커보드 사각형 크기 (m)

objpoints, imgpoints = [], []

for fname in glob.glob('calib_images/*.jpg'):
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
    if ret:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)

ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None)

np.save('camera_matrix.npy', K)
np.save('dist_coeffs.npy', dist)
print(f"RMS error: {ret:.4f}")  # < 1.0이면 양호
print(f"K:\n{K}")
```

**캘리브레이션 이미지 수집**

```bash
python3 -c "
import cv2
cap = cv2.VideoCapture(0)
i = 0
while True:
    ret, frame = cap.read()
    cv2.imshow('calib', frame)
    key = cv2.waitKey(1)
    if key == ord('s'):
        cv2.imwrite(f'calib_images/{i:03d}.jpg', frame)
        print(f'Saved {i}')
        i += 1
    elif key == ord('q'):
        break
"
```

---

## 2. AprilTag 라이브러리 선택

| 라이브러리 | 언어 | Pi 성능 | 정확도 | 비고 |
|---|---|---|---|---|
| **pupil-apriltags** | Python (C 바인딩) | ★★★★ | ★★★★★ | 권장 |
| apriltag (MIT) | C/Python | ★★★★ | ★★★★★ | 동일 엔진 |
| OpenCV ArUco | Python/C++ | ★★★★★ | ★★★★ | 빠름, 정밀도 약간 낮음 |
| ROS2 apriltag_ros | ROS2 | ★★★ | ★★★★★ | ROS 환경 |

```bash
pip install pupil-apriltags opencv-python
```

---

## 3. 태그 패밀리 선택

| 패밀리 | 비트 | 오탐 | 검출 거리 | 추천 용도 |
|---|---|---|---|---|
| **tag36h11** | 36 | 매우 낮음 | 중거리 | **수상드론 기본** |
| tag25h9 | 25 | 낮음 | 장거리 | 원거리 랜드마크 |
| tag16h5 | 16 | 높음 | 최장거리 | 비권장 |

**태그 크기 vs 검출 거리 (720p, f=4mm)**

| 태그 크기 | 검출 가능 거리 |
|---|---|
| 10cm | ~1.5m |
| 20cm | ~3m |
| 50cm | ~7m |
| 1m | ~15m |

수상 드론 도킹/수거 기준점으로 **20~50cm 태그** 권장.

---

## 4. AprilTag 기본 구현

```python
# apriltag_detect.py
import cv2
import numpy as np
from pupil_apriltags import Detector

K = np.load('camera_matrix.npy')
dist = np.load('dist_coeffs.npy')

TAG_SIZE = 0.15  # 태그 실물 크기 (m)

detector = Detector(
    families='tag36h11',
    nthreads=4,                # Pi 4코어 활용
    quad_decimate=2.0,         # 해상도 다운샘플 (속도↑, 정확도↓)
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

fx, fy = K[0,0], K[1,1]
cx, cy = K[0,2], K[1,2]

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.undistort(frame, K, dist)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    detections = detector.detect(
        gray,
        estimate_tag_pose=True,
        camera_params=(fx, fy, cx, cy),
        tag_size=TAG_SIZE
    )

    for d in detections:
        t = d.pose_t.flatten()
        print(f"Tag {d.tag_id}: x={t[0]:.3f} y={t[1]:.3f} z={t[2]:.3f} m")

        corners = d.corners.astype(int)
        cv2.polylines(frame, [corners], True, (0,255,0), 2)
        cv2.putText(frame, f"ID:{d.tag_id} z:{t[2]:.2f}m",
                    tuple(corners[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

    cv2.imshow('AprilTag', frame)
    if cv2.waitKey(1) == ord('q'):
        break
```

---

## 5. Pose 활용 — 드론 위치 계산

```python
import transforms3d  # pip install transforms3d

def get_drone_position(pose_R, pose_t):
    R_cam_to_drone = np.eye(3)
    t_cam_to_drone = np.array([0, 0, 0.1])  # 카메라 마운트 오프셋 (m)

    R_tag_to_cam = pose_R
    t_tag_to_cam = pose_t.flatten()

    drone_pos = -R_tag_to_cam.T @ t_tag_to_cam

    euler = transforms3d.euler.mat2euler(pose_R, axes='sxyz')
    roll, pitch, yaw = np.degrees(euler)

    return drone_pos, (roll, pitch, yaw)
```

---

## 6. RTSP 구성

### USB Camera RTSP 기본 설정

```bash
# mediamtx 설치 (권장 RTSP 서버)
wget https://github.com/bluenviron/mediamtx/releases/latest/download/mediamtx_linux_arm64v8.tar.gz
tar -xzf mediamtx_linux_arm64v8.tar.gz
```

```yaml
# mediamtx.yml
paths:
  drone:
    source: publisher
    runOnInit: >
      ffmpeg -f v4l2
      -input_format h264
      -video_size 1280x720
      -framerate 30
      -i /dev/video0
      -vcodec copy
      -f rtsp
      rtsp://localhost:$RTSP_PORT/$MTX_PATH
    runOnInitRestart: yes
```

---

## 7. AprilTag + RTSP 동시 구현

### 아키텍처

```
[USB Camera] → [캡처 스레드] → [Frame Queue] ┬→ [AprilTag 스레드]
                                              └→ [RTSP 인코딩 스레드]
```

### 방법 1: GStreamer tee (HW H.264 카메라 권장)

```bash
sudo apt install -y gstreamer1.0-rtsp gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad python3-gst-1.0 libgstrtspserver-1.0-dev
pip install pygobject
```

```python
# apriltag_rtsp_gst.py
import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer, GLib
import cv2
import numpy as np
from pupil_apriltags import Detector
import threading
import queue

Gst.init(None)

class RTSPServer:
    def __init__(self, port=8554, path="/stream"):
        self.server = GstRtspServer.RTSPServer()
        self.server.set_service(str(port))
        self.factory = GstRtspServer.RTSPMediaFactory()

        # USB 카메라 자체 H.264 인코딩 시
        pipeline = (
            "( v4l2src device=/dev/video0 "
            "! video/x-h264,width=1280,height=720,framerate=30/1 "
            "! h264parse ! rtph264pay name=pay0 pt=96 )"
        )
        self.factory.set_launch(pipeline)
        self.factory.set_shared(True)

        mounts = self.server.get_mount_points()
        mounts.add_factory(path, self.factory)
        self.server.attach(None)
        print(f"RTSP: rtsp://<pi_ip>:{port}{path}")

class AprilTagWorker(threading.Thread):
    def __init__(self, frame_queue: queue.Queue, K, dist, tag_size=0.15):
        super().__init__(daemon=True)
        self.q = frame_queue
        self.K = K
        self.dist = dist
        self.detector = Detector(
            families='tag36h11',
            nthreads=2,          # RTSP에 코어 양보
            quad_decimate=2.0,
        )
        self.fx, self.fy = K[0,0], K[1,1]
        self.cx, self.cy = K[0,2], K[1,2]
        self.tag_size = tag_size
        self.latest_detections = []
        self.lock = threading.Lock()

    def run(self):
        while True:
            try:
                frame = self.q.get(timeout=1.0)
            except queue.Empty:
                continue

            frame_ud = cv2.undistort(frame, self.K, self.dist)
            gray = cv2.cvtColor(frame_ud, cv2.COLOR_BGR2GRAY)

            detections = self.detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=(self.fx, self.fy, self.cx, self.cy),
                tag_size=self.tag_size
            )
            with self.lock:
                self.latest_detections = detections

            for d in detections:
                t = d.pose_t.flatten()
                print(f"[AprilTag] ID:{d.tag_id} "
                      f"x={t[0]:.3f} y={t[1]:.3f} z={t[2]:.3f}m")

    def get_detections(self):
        with self.lock:
            return self.latest_detections.copy()

def main():
    K = np.load('camera_matrix.npy')
    dist = np.load('dist_coeffs.npy')

    rtsp = RTSPServer(port=8554, path="/drone")

    frame_q = queue.Queue(maxsize=2)
    worker = AprilTagWorker(frame_q, K, dist)
    worker.start()

    cap = cv2.VideoCapture(2)  # RTSP가 /dev/video0 점유 시 다른 인덱스
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    loop = GLib.MainLoop()
    threading.Thread(target=loop.run, daemon=True).start()

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        if not frame_q.full():
            frame_q.put(frame)

if __name__ == '__main__':
    main()
```

### 방법 2: 단일 카메라 공유 (일반 UVC 카메라)

```python
# single_cam_share.py
import cv2, threading, queue, time, numpy as np
from pupil_apriltags import Detector
import subprocess

class SharedCamera:
    """단일 카메라 → 여러 소비자에게 프레임 분배"""
    def __init__(self, device=0, width=1280, height=720, fps=30):
        self.cap = cv2.VideoCapture(device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.subscribers: list[queue.Queue] = []
        self.running = False
        self._thread = threading.Thread(target=self._capture, daemon=True)

    def subscribe(self, maxsize=2) -> queue.Queue:
        q = queue.Queue(maxsize=maxsize)
        self.subscribers.append(q)
        return q

    def start(self):
        self.running = True
        self._thread.start()

    def _capture(self):
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                continue
            for q in self.subscribers:
                if not q.full():
                    q.put(frame)   # 느린 소비자는 drop

class FFmpegRTSPPusher:
    """FFmpeg subprocess로 RTSP 푸시 (mediamtx 필요)"""
    def __init__(self, rtsp_url, width=1280, height=720, fps=30):
        cmd = [
            'ffmpeg', '-y',
            '-f', 'rawvideo', '-vcodec', 'rawvideo',
            '-pix_fmt', 'bgr24',
            '-s', f'{width}x{height}',
            '-r', str(fps),
            '-i', '-',
            '-vcodec', 'libx264',
            '-preset', 'ultrafast',
            '-tune', 'zerolatency',
            '-pix_fmt', 'yuv420p',
            '-f', 'rtsp',
            '-rtsp_transport', 'tcp',
            rtsp_url
        ]
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)

    def push(self, frame: np.ndarray):
        try:
            self.proc.stdin.write(frame.tobytes())
        except BrokenPipeError:
            pass

    def stop(self):
        self.proc.stdin.close()
        self.proc.wait()

def apriltag_worker(frame_q: queue.Queue, K, dist):
    detector = Detector(families='tag36h11', nthreads=2, quad_decimate=2.0)
    fx, fy, cx, cy = K[0,0], K[1,1], K[0,2], K[1,2]

    while True:
        frame = frame_q.get()
        gray = cv2.cvtColor(
            cv2.undistort(frame, K, dist),
            cv2.COLOR_BGR2GRAY
        )
        detections = detector.detect(
            gray, estimate_tag_pose=True,
            camera_params=(fx, fy, cx, cy), tag_size=0.15
        )
        for d in detections:
            t = d.pose_t.flatten()
            print(f"[Tag {d.tag_id}] z={t[2]:.3f}m")

def rtsp_worker(frame_q: queue.Queue, pusher: FFmpegRTSPPusher):
    while True:
        frame = frame_q.get()
        pusher.push(frame)

def main():
    K = np.load('camera_matrix.npy')
    dist = np.load('dist_coeffs.npy')

    cam = SharedCamera(device=0, width=1280, height=720, fps=30)
    apriltag_q = cam.subscribe(maxsize=2)
    rtsp_q = cam.subscribe(maxsize=4)

    pusher = FFmpegRTSPPusher('rtsp://localhost:8554/drone', 1280, 720, 30)

    threading.Thread(target=apriltag_worker, args=(apriltag_q, K, dist), daemon=True).start()
    threading.Thread(target=rtsp_worker, args=(rtsp_q, pusher), daemon=True).start()

    cam.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pusher.stop()

if __name__ == '__main__':
    main()
```

---

## 8. 방법 비교

| | 방법 1 (GStreamer tee) | 방법 2 (SharedCamera) |
|---|---|---|
| CPU | 낮음 (HW 인코딩 활용) | 높음 (SW x264) |
| 구현 복잡도 | 높음 | 낮음 |
| USB 카메라 HW 인코딩 활용 | ✅ | ❌ |
| 단일 카메라 공유 | 제한적 | ✅ |
| 권장 상황 | HW H.264 카메라 | 일반 UVC 카메라 |

---

## 9. 성능 최적화

```python
# quad_decimate 조정 가이드
# 1.0 = 원본 해상도 (~5 fps @ 1080p)
# 2.0 = 1/4 픽셀 (~20 fps @ 720p)  ← 수상드론 권장
# 4.0 = 1/16 픽셀 (근거리 대형 태그만)
```

```bash
# Pi CPU 성능 모드
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
```

---

## 10. 수상 드론 운용 시 고려사항

```python
# 수면 반사 노이즈 대응 - CLAHE 적용
clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
gray = clahe.apply(gray)

# 다중 태그 위치 평균화 (안정성↑)
if len(detections) > 1:
    positions = [d.pose_t.flatten() for d in detections]
    avg_pos = np.mean(positions, axis=0)
```

**RTSP 네트워크 불안정 대비**
```bash
# TCP transport 강제 (WiFi 패킷 손실 대응)
ffmpeg ... -rtsp_transport tcp ...
```
