# -*- coding: utf-8 -*-
"""
Picamera2 + OpenCV 로 바닥 검은 선(Line)을 감지해서 ──▶  Arduino 로 L / R / F / B 명령 전송
Flask 로 MJPEG 실시간 스트림을 보여주며 (라즈베리파이용 경량 비동기 설계)

주요 아이디어
────────────────────────────────────────────────────
1) camera_loop ──> Picamera2 프레임 캡처 → 방향 분석 → 주기적으로 시리얼 전송
2) detect_direction() ──> ROI(관심영역) 내에서 adaptiveThreshold + contour 로 선 중심 계산
   · 선이 없으면 'B' (후진) 반환
3) send_dir() ──> 같은 방향이라도 RESEND_FRAMES 마다 한 번 더 전송 (ESC 브레이크↔후진 대응)
4) Flask 는 latest_frame 바이트를 MJPEG boundary 로 스트리밍

※ "L" 헤더 + 1 문자 명령 형태(LF/LL/LR/LB) : Arduino 스케치와 맞춤
※ 좌우 보정: 카메라 영상을 1 회 flip 한 좌표계로 통일
"""

from flask import Flask, Response
import cv2, serial, time, threading, sys
from picamera2 import Picamera2
from collections import deque
import numpy as np

# ───── 사용자 설정값 ───────────────────────────────────
SERIAL_PORT  = "/dev/ttyACM0"    # ↳ 아두이노 USB 포트 (환경에 따라 변경)
BAUDRATE     = 9600
FRAME_W, FRAME_H = 320, 240       # 카메라 해상도
BIN_THRESH   = 90                 # (adaptiveThreshold 가 못쓰는 상황 대비)
ROI_TOP_PCT  = 0.35               # ROI 시작 위치(화면 비율)
ROI_H_PCT    = 0.45               # ROI 높이(화면 비율)
MIN_LINE_AREA = 300               # 컨투어 최소 면적
SEND_GAP     = 10                 # N 프레임마다 아두이노로 명령 전송
RESEND_FRAMES= 30                 # 같은 방향도 이 프레임 주기로 재전송

# 왼쪽/오른쪽 경계 (slice 1/3, 2/3 지점)
BOUNDARY_L   = FRAME_W // 3
BOUNDARY_R   = FRAME_W * 2 // 3
# ──────────────────────────────────────────────────────

# ───── 시리얼 초기화 ──────────────────────────────────
try:
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
    print(f"[✅] Serial connected: {SERIAL_PORT} @ {BAUDRATE}")
except serial.SerialException as e:
    print(f"[❌] Serial open failed: {e}")
    ser = None

# ───── 카메라 초기화 ──────────────────────────────────
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(
    main={"format": "RGB888", "size": (FRAME_W, FRAME_H)}))
picam2.start()
time.sleep(1)       # 센서 워밍업

# ───── Flask 상태 변수 ────────────────────────────────
app = Flask(__name__)
latest_frame: bytes | None = None    # MJPEG 스트림용 최신 프레임
frame_lock  = threading.Lock()
last_dir    = 'S'     # 직전 전송한 방향(F/L/R/B/S)
frame_idx   = 0       # 전체 프레임 카운터
history     = deque(maxlen=6)  # 최근 방향 히스토리 → 다수결 노이즈 억제

# ──────────────────────────────────────────────────────
# 라인 감지 함수
# - frame_bgr  : flip 전송된 BGR 프레임(카메라 기준 좌·우 교정됨)
# - 반환 값    : 'L' / 'R' / 'F' / 'B'
# ──────────────────────────────────────────────────────

def detect_direction(frame_bgr: np.ndarray) -> str:
    # 1) 입력 프레임을 사람/차량 기준 좌·우가 맞도록 1회 좌우 반전
    frame_bgr = cv2.flip(frame_bgr, 1)

    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

    # 2) 관심영역(ROI) 설정 — 화면 하단 35% 지점부터 45% 높이 구간
    ROI_TOP = int(FRAME_H * ROI_TOP_PCT)
    ROI_H   = int(FRAME_H * ROI_H_PCT)
    roi     = gray[ROI_TOP: ROI_TOP + ROI_H, :]

    # 3) adaptiveThreshold(조명 변화 강인) → 흑백 이진 이미지
    bw = cv2.adaptiveThreshold(
            roi, 255,
            cv2.ADAPTIVE_THRESH_MEAN_C,
            cv2.THRESH_BINARY_INV,
            15, 5)

    # 4) 외곽선 추출 (OpenCV 3/4 버전 모두 호환)
    contours = cv2.findContours(bw, cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[-2]

    # 4‑1) 선이 안 보이면 후진 명령 반환
    if not contours:
        cv2.putText(frame_bgr, "B", (10, 40), cv2.FONT_HERSHEY_SIMPLEX,
                    1.2, (0, 255, 255), 3)
        return 'B'

    # 5) 가장 큰 컨투어(선) 선택 & 노이즈 필터
    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < MIN_LINE_AREA:
        return 'B'

    # 6) 선 중심 좌표(cx) 계산
    M = cv2.moments(c)
    if M['m00'] == 0:
        return 'B'
    cx = int(M['m10'] / M['m00'])

    # 7) 방향 결정: 좌·중·우 영역 기준
    if cx < BOUNDARY_L:
        direction = 'L'
    elif cx > BOUNDARY_R:
        direction = 'R'
    else:
        direction = 'F'

    # 8) 디버그 시각화(초록 원, 노란 ROI 박스, 방향 텍스트)
    y_pt = ROI_TOP + ROI_H // 2
    cv2.circle(frame_bgr, (cx, y_pt), 6, (0, 255, 0), -1)
    cv2.rectangle(frame_bgr, (0, ROI_TOP),
                  (FRAME_W, ROI_TOP + ROI_H), (0, 255, 255), 1)
    cv2.putText(frame_bgr, direction, (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3)
    return direction

# ──────────────────────────────────────────────────────
# 시리얼 전송 함수
#  - 같은 방향이라도 RESEND_FRAMES 프레임마다 재전송 (ESC 브레이크→후진 대응)
# ──────────────────────────────────────────────────────

def send_dir(dir_char: str):
    global last_dir, frame_idx

    if not ser:
        return

    need_send = (dir_char != last_dir) or (frame_idx % RESEND_FRAMES == 0)
    if need_send:
        try:
            ser.write(('L' + dir_char).encode())   # header 'L' + 1바이트 명령
            last_dir = dir_char
            print(f"[TX] → L{dir_char}")
        except Exception as e:
            print(f"[SERIAL ERR] {e}")

# ──────────────────────────────────────────────────────
# 카메라 + 분석 + MJPEG 업데이트 루프 (별도 스레드)
# ──────────────────────────────────────────────────────

def camera_loop():
    global latest_frame, frame_idx
    while True:
        try:
            # 1) 프레임 캡처 & flip 1회 (좌우 교정)
            raw   = picam2.capture_array()
            raw   = cv2.flip(raw, 1)
            frame = cv2.cvtColor(raw, cv2.COLOR_RGB2BGR)

            # 2) 방향 판단
            dir_char = detect_direction(frame)
            history.append(dir_char)
            majority = max(set(history), key=history.count)  # 다수결

            # 3) 일정 주기마다 시리얼 전송
            if frame_idx % SEND_GAP == 0:
                send_dir(majority)
            frame_idx += 1

            # 4) MJPEG 스트림용 프레임 (여기선 별도 flip 없이 그대로 사용)
            ret, buf = cv2.imencode('.jpg', frame)
            if not ret:
                continue
            with frame_lock:
                latest_frame = buf.tobytes()

        except Exception as e:
            print(f"[GEN_FRAME ERR] {e}")
            continue

# ──────────────────────────────────────────────────────
# Flask 엔드포인트: 메인 페이지 & MJPEG 스트림
# ──────────────────────────────────────────────────────
@app.route('/')
def index():
    return ("<h2>Line-Tracing Stream</h2>"
            "<img src='/video_feed' width='320'>")

@app.route('/video_feed')
def video_feed():
    def gen_frames():
        # 빈 화면용 흰 배경 JPEG 하나 생성
        blank = cv2.imencode('.jpg', 255 * np.ones((FRAME_H, FRAME_W, 3), dtype=np.uint8))[1].tobytes()
        while True:
            with frame_lock:
                frame_copy = latest_frame if latest_frame is not None else blank
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_copy + b'\r\n')
            time.sleep(0.03)  # 약 30 fps 출력
    return Response(gen_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

# ──────────────────────────────────────────────────────
# 메인 진입점: 카메라 스레드 + Flask 서버
# ──────────────────────────────────────────────────────
if __name__ == '__main__':
    print("[🚀] Starting camera thread and Flask server")
    threading.Thread(target=camera_loop, daemon=True).start()
    app.run(host='0.0.0.0', port=5000, threaded=True)
