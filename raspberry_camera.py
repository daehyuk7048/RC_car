# -*- coding: utf-8 -*-
"""
Picamera2 + OpenCV 로 바닥 검은 선(Line)을 감지해서 ──▶ Arduino 로 'P' + steerByte + thrByte 명령 전송
Flask 로 MJPEG 실시간 스트림을 보여주며 (라즈베리파이용 경량 비동기 설계)
라인 재탐색 기능 강화 및 아두이노 3바이트 프로토콜 적용, 변수명 오타 수정
ROI 위치 및 크기 재조정
"""

from flask import Flask, Response
import cv2
import serial
import time
import threading
import numpy as np
from picamera2 import Picamera2
from libcamera import Transform # hflip을 위해 import

# ───── 사용자 설정값 ───────────────────────────────────
SERIAL_PORT  = "/dev/ttyACM0" # 아두이노 시리얼 포트
BAUDRATE     = 115200         # 아두이노와 통신 속도
FRAME_W, FRAME_H = 320, 240   # 카메라 프레임 너비, 높이

# ROI (Region of Interest, 관심 영역) 설정
ROI_BOTTOM_CLEARANCE_PIXELS = 70  # 화면 맨 아래에서 범퍼 등을 피해 건너뛸 픽셀 높이
ROI_ANALYSIS_STRIP_HEIGHT   = 70  # 실제 라인 감지를 수행할 ROI의 세로 높이

BINARY_THRESHOLD_VALUE = 110      # 이미지 이진화 시 사용할 고정 임계값 (0-255 사이, 튜닝 필요)
MIN_CONTOUR_AREA_FOR_LINE = 50    # 라인으로 간주할 최소 면적 (cv2.moments의 M["m00"] 값 기준)

# 조향 결정용 center_offset 임계값 (단위: 픽셀)
OFFSET_STRONG_LEFT  = -40 # 이 값보다 작으면 강한 좌회전
OFFSET_WEAK_LEFT    = -15 # 이 값보다 작으면 약한 좌회전
OFFSET_WEAK_RIGHT   = 15  # 이 값보다 크면 약한 우회전
OFFSET_STRONG_RIGHT = 40  # 이 값보다 크면 강한 우회전

# 시리얼 명령 전송 주기 관련
SEND_GAP     = 3   # camera_loop 3회 반복마다 명령 전송 시도
RESEND_FRAMES= 20  # 마지막으로 보낸 명령과 같더라도, 이 프레임 간격마다 재전송 (주로 전진/회전 시)

# 라인 재탐색 관련 설정값 (단위: 프레임 수)
LINE_LOST_TRIGGER_FRAMES   = 15  # 이 횟수만큼 연속으로 선을 놓치면 후진 시작
REVERSE_DURATION_FRAMES    = 12  # 후진을 지속할 프레임 수
POST_REVERSE_SEARCH_FRAMES = 25  # 후진 후 정지하며 탐색할 프레임 수
SEARCH_SWEEP_FRAMES        = 8   # SEARCHING 모드에서 한쪽 방향으로 조향을 유지할 프레임 수 (좌우 까딱거림 주기)

# 스로틀 (아두이노 mapThrottleByte 함수가 이 값을 PWM µs 값으로 변환)
THROTTLE_BYTE_FORWARD_STRONG = 180 # 일반적인 상황에서의 강한 전진 값
THROTTLE_BYTE_FORWARD_WEAK   = 145 # 커브 등에서 사용할 약한 전진 값 
THROTTLE_BYTE_REVERSE_NORMAL = 0   # 후진 시 사용할 값
THROTTLE_BYTE_STOP           = 128 # 정지

# 조향 (아두이노가 이 값을 서보 각도로 변환)
# 0: 최대 좌회전, 128: 중앙, 255: 최대 우회전
STEER_BYTE_MAX_LEFT     = 0    # np.clip 함수에서 최소값으로 사용
STEER_BYTE_MAX_RIGHT    = 255  # np.clip 함수에서 최대값으로 사용
STEER_BYTE_CENTER       = 128  # 중앙 조향
STEER_BYTE_STRONG_LEFT  = 30   # OFFSET_STRONG_LEFT 조건 충족 시 사용 (0에 가깝게)
STEER_BYTE_WEAK_LEFT    = 90   # OFFSET_WEAK_LEFT 조건 충족 시 사용
STEER_BYTE_WEAK_RIGHT   = 170  # OFFSET_WEAK_RIGHT 조건 충족 시 사용
STEER_BYTE_STRONG_RIGHT = 220  # OFFSET_STRONG_RIGHT 조건 충족 시 사용 (255에 가깝게)
# ──────────────────────────────────────────────────────

# ───── 시리얼 초기화 ──────────────────────────────────
ser = None # 시리얼 객체 초기화
try:
    # 지정된 포트와 속도로 시리얼 통신 시작, timeout은 1초
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
    print(f"[✅] Serial connected: {SERIAL_PORT} @ {BAUDRATE}")
    time.sleep(2) # 아두이노 리셋 및 시리얼 안정화 대기 시간
except serial.SerialException as e: # 시리얼 포트 관련 예외 처리
    print(f"[❌] Serial open failed: {e}. Running without serial communication.")
except Exception as e_gen: # 그 외 모든 예외 처리
    print(f"[❌] Serial setup error: {e_gen}. Running without serial communication.")

# ───── 카메라 초기화 ──────────────────────────────────
picam2 = None # Picamera2 객체 초기화
try:
    picam2 = Picamera2()
    # 비디오 스트리밍 및 연속 캡처에 적합한 설정 생성
    config = picam2.create_video_configuration(
        main={"format": "RGB888", "size": (FRAME_W, FRAME_H)}, # 주 스트림 포맷 및 크기
        transform=Transform(hflip=1) # 카메라 이미지 수평 반전 (카메라 설치 방향에 따라)
    )
    picam2.configure(config) # 설정 적용
    picam2.start() # 카메라 시작
    print("✅ Camera started successfully.")
    time.sleep(1) # 카메라 센서 안정화 대기 시간
except Exception as e:
    print(f"[❌] Camera setup failed: {e}")
    # 카메라 초기화 실패 시 picam2는 None 상태 유지

# ───── Flask 및 상태 변수 ────────────────────────────────
app = Flask(__name__) # Flask 애플리케이션 생성
latest_frame: bytes | None = None # MJPEG 스트리밍을 위한 최신 프레임 (JPEG 바이트 데이터)
frame_lock  = threading.Lock()    # latest_frame에 대한 스레드 동기화를 위한 Lock 객체

# 마지막으로 아두이노에 전송된 제어 값 (중복 전송 방지용)
last_sent_steer_byte = STEER_BYTE_CENTER
last_sent_throttle_byte = THROTTLE_BYTE_STOP
frame_idx   = 0 # 전체 프레임 인덱스 카운터 (주기적 명령 전송 등에 사용)

# 라인 재탐색 로직을 위한 상태 변수
line_lost_counter = 0         # 라인을 연속으로 놓친 프레임 수
reverse_frame_count = 0       # 현재 후진 동작 중인 프레임 수
post_reverse_search_count = 0 # 후진 후 탐색 동작 중인 프레임 수
current_operation_mode = "FORWARD" # 현재 차량의 동작 모드 ("FORWARD", "REVERSING", "SEARCHING")
last_known_center_offset = 0  # 라인을 놓치기 직전의 center_offset 값 (후진/탐색 시 조향 참고용)
search_steer_direction = 1    # SEARCHING 모드에서 조향을 시도할 방향 (1: 오른쪽부터, -1: 왼쪽부터)

# ──────────────────────────────────────────────────────
# 라인 중심 오프셋 감지 및 시각화 함수
# 입력: BGR 컬러 프레임
# 반환: center_offset (라인 중심의 가로 오프셋, 픽셀단위), line_found (라인 발견 여부, boolean), visualized_frame (시각화 정보가 추가된 프레임)
# ──────────────────────────────────────────────────────
def detect_center_offset_and_visualize(input_bgr_frame: np.ndarray):
    visualized_frame = input_bgr_frame.copy() # 원본 프레임에 직접 그리지 않도록 복사

    # 1. 그레이스케일 변환
    gray = cv2.cvtColor(visualized_frame, cv2.COLOR_BGR2GRAY)
    # 2. 이진화 (고정 임계값 사용, 검은 선을 흰색으로 만듦)
    _, binary = cv2.threshold(gray, BINARY_THRESHOLD_VALUE, 255, cv2.THRESH_BINARY_INV)
    
    height, width = binary.shape # 프레임 전체 높이, 너비

    # 3. ROI (관심 영역) 좌표 계산
    roi_y_end = height - ROI_BOTTOM_CLEARANCE_PIXELS      # ROI의 아래쪽 y좌표
    roi_y_start = roi_y_end - ROI_ANALYSIS_STRIP_HEIGHT # ROI의 위쪽 y좌표

    # ROI 좌표가 프레임 범위를 벗어나지 않도록 보정
    roi_y_start = max(0, roi_y_start)
    roi_y_end = min(height, roi_y_end) 
    if roi_y_start >= roi_y_end: # ROI 높이가 0 또는 음수가 되는 극단적인 경우 방지
        roi_y_start = int(height * 0.45) # 안전한 기본값 (화면 중앙 부근)
        roi_y_end = int(height * 0.55)
        if roi_y_start >= roi_y_end : roi_y_start = max(0, roi_y_end - 20) # 최소 높이 보장

    # 4. 계산된 ROI 영역을 이진화 이미지에서 추출 (가로폭은 전체 사용)
    roi_binary = binary[roi_y_start:roi_y_end, 0:width]

    line_found = False  # 라인 발견 플래그 초기화
    center_offset = 0   # 라인 중심 오프셋 초기화
    
    # 5. ROI 영역이 유효한 크기인지 확인 후 모멘트 계산
    if roi_binary.shape[0] > 0 and roi_binary.shape[1] > 0: # 높이와 너비가 0보다 커야 함
        M = cv2.moments(roi_binary) # ROI 영역의 모멘트 계산
        # M["m00"]은 객체의 면적(또는 흰색 픽셀의 총합)과 유사함
        if M["m00"] > MIN_CONTOUR_AREA_FOR_LINE: # 계산된 면적이 최소 기준치보다 크면 유효한 라인으로 간주
            line_found = True
            cx = int(M["m10"] / M["m00"]) # 라인 중심의 x좌표 (ROI 내 기준, 현재는 전체 프레임 기준과 동일)
            center_offset = cx - (width // 2) # 프레임 중앙으로부터의 가로 오프셋 계산
            # 감지된 라인 중심에 녹색 원 그리기 (시각화용)
            cv2.circle(visualized_frame, (cx, (roi_y_start + roi_y_end) // 2), 5, (0, 255, 0), -1)

    # 6. 시각화 정보 추가
    cv2.rectangle(visualized_frame, (0, roi_y_start), (width - 1, roi_y_end - 1), (0, 255, 255), 1) # ROI 영역 노란색 사각형으로 표시
    status_text = "Line Found" if line_found else "Line Lost"
    cv2.putText(visualized_frame, f"Status: {status_text}", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (50,200,50), 2)
    cv2.putText(visualized_frame, f"Offset: {center_offset}", (10,55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (50,200,50), 2)
    cv2.putText(visualized_frame, f"Mode: {current_operation_mode}", (10,80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (50,200,50),2)
    cv2.putText(visualized_frame, f"LostCnt: {line_lost_counter}", (10,105), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (50,200,50),2)

    return center_offset, line_found, visualized_frame

# ──────────────────────────────────────────────────────
# 제어 바이트 계산 함수 (현재 상태, 라인 정보 등을 바탕으로 조향/스로틀 바이트 결정)
# 입력: operation_mode (현재 동작 모드), center_offset_val (라인 중심 오프셋), 
#       line_found_flag (라인 발견 여부), p_search_frame_count (탐색 모드 진행 프레임 카운트)
# 반환: steer_b (조향 바이트), throttle_b (스로틀 바이트)
# ──────────────────────────────────────────────────────
def calculate_control_bytes(operation_mode, center_offset_val, line_found_flag, p_search_frame_count):
    global last_known_center_offset, search_steer_direction # 전역 변수 사용 명시

    steer_b = STEER_BYTE_CENTER       # 기본 조향은 중앙
    throttle_b = THROTTLE_BYTE_STOP   # 기본 스로틀은 정지 (각 모드에서 덮어씀)

    if operation_mode == "FORWARD": # 일반 전진 주행 모드
        if line_found_flag: # 라인을 찾았다면
            last_known_center_offset = center_offset_val # 마지막으로 본 라인 위치 업데이트
            # center_offset 값에 따라 조향 바이트 결정
            if center_offset_val < OFFSET_STRONG_LEFT:     steer_b = STEER_BYTE_STRONG_LEFT
            elif center_offset_val < OFFSET_WEAK_LEFT:     steer_b = STEER_BYTE_WEAK_LEFT
            elif center_offset_val > OFFSET_STRONG_RIGHT:  steer_b = STEER_BYTE_STRONG_RIGHT
            elif center_offset_val > OFFSET_WEAK_RIGHT:    steer_b = STEER_BYTE_WEAK_RIGHT
            else:                                       steer_b = STEER_BYTE_CENTER
            
            # 스로틀 결정: 커브가 심하면(오프셋이 크면) 약한 전진, 아니면 일반(강한) 전진
            if abs(center_offset_val) > abs(OFFSET_WEAK_LEFT): # 예: 약한 조향 임계값을 기준으로 커브 판단
                throttle_b = THROTTLE_BYTE_FORWARD_WEAK
            else:
                throttle_b = THROTTLE_BYTE_FORWARD_STRONG
        else: # FORWARD 모드인데 선을 놓쳤다면 (아직 REVERSING으로 넘어가기 전)
            steer_b = STEER_BYTE_CENTER   # 조향은 중앙 유지 (또는 마지막 유효 조향값 사용 고려)
            throttle_b = THROTTLE_BYTE_STOP # 스로틀은 정지
    
    elif operation_mode == "REVERSING": # 후진 모드
        throttle_b = THROTTLE_BYTE_REVERSE_NORMAL # 설정된 후진 스로틀 값 사용 (후진)
        # 후진 시 조향 로직: 마지막으로 선을 본 위치의 반대 방향으로 약간 틀어주거나, 직진 후진
        if last_known_center_offset > 20: # 선이 오른쪽에 있었다면 (offset 양수)
            steer_b = STEER_BYTE_CENTER - 35 # 조향은 왼쪽으로 (값이 작아짐)
        elif last_known_center_offset < -20: # 선이 왼쪽에 있었다면 (offset 음수)
            steer_b = STEER_BYTE_CENTER + 35 # 조향은 오른쪽으로 (값이 커짐)
        else: # 중앙 근처에서 놓쳤다면 직진 후진
            steer_b = STEER_BYTE_CENTER
        # 계산된 조향값이 0-255 범위를 벗어나지 않도록 np.clip으로 제한
        steer_b = np.clip(steer_b, STEER_BYTE_MAX_LEFT, STEER_BYTE_MAX_RIGHT)

    elif operation_mode == "SEARCHING": # 라인 탐색 모드
        throttle_b = THROTTLE_BYTE_STOP # 탐색 중에는 정지
        # 탐색 중 조향: SEARCH_SWEEP_FRAMES 마다 좌우 방향을 바꿔가며 "까딱까딱"
        sweep_angle_offset = 60 # 좌우로 꺾을 조향 오프셋 (튜닝값)
        if (p_search_frame_count // SEARCH_SWEEP_FRAMES) % 2 == 0: # 짝수번째 스윕 구간
            steer_b = STEER_BYTE_CENTER + (sweep_angle_offset * search_steer_direction) # 한쪽으로 꺾음
        else: # 홀수번째 스윕 구간
            steer_b = STEER_BYTE_CENTER - (sweep_angle_offset * search_steer_direction) # 반대쪽으로 꺾음
        steer_b = np.clip(steer_b, STEER_BYTE_MAX_LEFT, STEER_BYTE_MAX_RIGHT)
    
    return steer_b, throttle_b

# ──────────────────────────────────────────────────────
# 시리얼 전송 함수 (3바이트 프로토콜)
# ──────────────────────────────────────────────────────
def send_control_command(steer_byte: int, throttle_byte: int):
    global last_sent_steer_byte, last_sent_throttle_byte, frame_idx, current_operation_mode, line_lost_counter
    if not ser: return # 시리얼 연결 안되어 있으면 함수 종료

    current_packet = (steer_byte, throttle_byte) # 현재 전송할 패킷 (조향, 스로틀)
    
    # 아두이노 기준 정지/후진 상태 확인
    is_currently_stopped = throttle_byte == 127 or throttle_byte == 128
    is_currently_reversing = throttle_byte < 127

    # 주기적 재전송이 필요한 경우인지 판단 (정지 또는 후진 중에는 불필요한 재전송 방지)
    needs_resend_check = not (is_currently_stopped or is_currently_reversing)
    
    # 명령이 변경되었거나, 주기적 재전송 조건 만족 시에만 전송
    if current_packet != (last_sent_steer_byte, last_sent_throttle_byte) or \
       (frame_idx % RESEND_FRAMES == 0 and needs_resend_check):
        try:
            # 'P' 헤더 + 조향 바이트 + 스로틀 바이트 형태로 메시지 생성
            message = bytes(['P'.encode('ascii')[0], steer_byte, throttle_byte])
            ser.write(message) # 아두이노로 전송
            last_sent_steer_byte = steer_byte       # 마지막으로 보낸 조향 값 업데이트
            last_sent_throttle_byte = throttle_byte # 마지막으로 보낸 스로틀 값 업데이트
            # 콘솔에 전송 로그 출력 (디버깅 시 주석 해제)
            print(f"[TX] → P, Steer:{steer_byte}, Throttle:{throttle_byte} (Mode: {current_operation_mode}, Lost: {line_lost_counter})")
        except Exception as e:
            print(f"[SERIAL ERR] {e}") # 시리얼 전송 중 오류 발생 시 출력

# ──────────────────────────────────────────────────────
# 카메라 프레임 처리, 라인 감지, 상태 결정, 명령 전송, MJPEG 프레임 업데이트를 수행하는 메인 루프
# ──────────────────────────────────────────────────────
def camera_loop():
    global latest_frame, frame_idx, line_lost_counter, current_operation_mode
    global reverse_frame_count, post_reverse_search_count, last_known_center_offset, search_steer_direction
    
    if not picam2: # 카메라 초기화 실패 시 camera_loop 실행 중단
        print("[CRITICAL] Camera not initialized. Camera loop cannot start.")
        # MJPEG 스트림에 에러 이미지를 계속 제공하기 위한 간단한 처리
        _blank_img = np.full((FRAME_H, FRAME_W, 3), 30, dtype=np.uint8) # 어두운 배경
        cv2.putText(_blank_img, "CAM ERROR", (FRAME_W//4, FRAME_H//2), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
        _ok_blank, _buf_blank = cv2.imencode('.jpg', _blank_img)
        if _ok_blank:
            with frame_lock: latest_frame = _buf_blank.tobytes()
        while True: # 카메라가 없으면 이 루프를 계속 돌며 에러 이미지 업데이트
            time.sleep(1) # CPU 사용 방지를 위해 긴 sleep
            if picam2: break # 혹시라도 카메라가 나중에 초기화되면 루프 탈출 시도
        return # camera_loop 함수 종료

    # 메인 루프 시작
    while True:
        try:
            # 1. 카메라에서 프레임 캡처 및 BGR 변환
            rgb_frame = picam2.capture_array() # Picamera2는 RGB 순서로 배열 반환
            bgr_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR) # OpenCV는 BGR 순서 사용
            
            # 2. 프레임에서 라인 정보(중심 오프셋, 발견 여부) 감지 및 시각화된 프레임 얻기
            center_offset, line_found, visualized_frame = detect_center_offset_and_visualize(bgr_frame)
            
            # 3. 현재 동작 모드(current_operation_mode)에 따른 상태 전이 로직
            if current_operation_mode == "FORWARD":
                if not line_found: # FORWARD 모드인데 라인을 놓쳤다면
                    line_lost_counter += 1 # 놓친 프레임 카운터 증가
                    if line_lost_counter >= LINE_LOST_TRIGGER_FRAMES: # 설정된 횟수 이상 놓치면
                        current_operation_mode = "REVERSING"          # 후진 모드로 변경
                        reverse_frame_count = 0                       # 후진 프레임 카운터 초기화
                        # print(f"[STATE] FORWARD -> REVERSING (Lost: {line_lost_counter}) LastOffset: {last_known_center_offset}")
                else: # 라인을 찾았다면
                    line_lost_counter = 0 # 놓친 프레임 카운터 리셋
            
            elif current_operation_mode == "REVERSING":
                reverse_frame_count += 1 # 후진 프레임 카운터 증가
                if reverse_frame_count >= REVERSE_DURATION_FRAMES: # 설정된 후진 시간(프레임)만큼 후진했다면
                    current_operation_mode = "SEARCHING"           # 탐색 모드로 변경
                    reverse_frame_count = 0                        # 카운터 초기화
                    post_reverse_search_count = 0                  # 탐색 프레임 카운터 초기화
                    # 탐색 시작 시 초기 조향 방향 결정 (마지막으로 본 라인 위치 기반)
                    if last_known_center_offset > 10 : search_steer_direction = 1 # 선이 오른쪽에 있었으면 오른쪽부터 탐색
                    elif last_known_center_offset < -10 : search_steer_direction = -1 # 선이 왼쪽에 있었으면 왼쪽부터 탐색
                    else: search_steer_direction = np.random.choice([-1, 1]) # 중앙에서 놓쳤으면 랜덤 방향
                    # print(f"[STATE] REVERSING -> SEARCHING (Initial search_dir: {search_steer_direction})")
            
            elif current_operation_mode == "SEARCHING":
                post_reverse_search_count += 1 # 탐색 프레임 카운터 증가
                if line_found: # 탐색 중 라인을 다시 찾았다면
                    current_operation_mode = "FORWARD" # 일반 주행 모드로 복귀
                    line_lost_counter = 0              # 놓친 프레임 카운터 리셋
                    # print("[STATE] SEARCHING -> FORWARD (Line Re-acquired!)")
                elif post_reverse_search_count >= POST_REVERSE_SEARCH_FRAMES: # 설정된 시간 동안 탐색했지만 못 찾았다면
                    current_operation_mode = "FORWARD" # 일단 일반 주행 모드로 복귀 (다시 라인을 놓칠 수 있음)
                    line_lost_counter = 0              # 놓친 프레임 카운터 리셋하고 다시 FORWARD에서 판단
                    # print("[STATE] SEARCHING -> FORWARD (Search Timeout)")
            
            # 4. 현재 상태와 라인 감지 결과를 바탕으로 실제 조향/스로틀 바이트 값 계산
            steer_b, throttle_b = calculate_control_bytes(
                current_operation_mode, 
                center_offset if line_found else last_known_center_offset, # 선을 찾았으면 현재 오프셋, 못 찾았으면 마지막 오프셋 사용
                line_found,
                post_reverse_search_count # SEARCHING 모드에서 좌우 까딱거림 제어용
            )

            # 5. SEND_GAP 프레임마다 아두이노로 제어 명령 전송
            if frame_idx % SEND_GAP == 0: send_control_command(steer_b, throttle_b)
            frame_idx += 1 # 전체 프레임 카운터 증가

            # 6. MJPEG 스트리밍을 위해 현재 시각화된 프레임을 JPEG으로 인코딩하여 latest_frame에 저장
            ret, buf = cv2.imencode('.jpg', visualized_frame, [cv2.IMWRITE_JPEG_QUALITY, 70]) # JPEG 품질 70
            if ret: # 인코딩 성공 시
                with frame_lock: # 스레드 동기화
                    latest_frame = buf.tobytes()
            # else: print("[WARN] cv2.imencode failed in camera_loop") # 인코딩 실패 시 경고 (필요시 주석 해제)

            time.sleep(0.03) # 루프 지연시간, 약 33FPS 목표 (CPU 사용량 및 반응성 조절)

        except Exception as e: # camera_loop 내에서 예외 발생 시
            print(f"[camera_loop ERR] {e}")
            import traceback; traceback.print_exc() # 자세한 오류 내용 출력
            # 오류 발생 시 안전을 위해 아두이노에 정지 명령 시도
            if ser and ser.is_open: send_control_command(STEER_BYTE_CENTER, THROTTLE_BYTE_STOP)
            time.sleep(0.5) # 오류 후 잠시 대기하여 너무 빠른 반복 방지

# ──────────────────────────────────────────────────────
# Flask 엔드포인트 (웹 페이지 및 MJPEG 스트림 제공)
# ──────────────────────────────────────────────────────
@app.route('/') # 기본 웹 페이지 URL
def index():
    # 웹 페이지에 표시될 HTML 내용
    return ("<h2>Line Tracing Cam (ROI Re-Adjusted & Commented)</h2>"
            "<img src='/video_feed' style='width:320px; height:240px; border:1px solid black;'>")

@app.route('/video_feed') # MJPEG 스트림 URL
def video_feed():
    def gen_frames(): # 프레임을 지속적으로 생성하여 스트리밍하는 제너레이터 함수
        _blank_img_bytes = None # 초기 빈 프레임 데이터 (한번만 생성하기 위한 플래그)
        while True:
            frame_bytes_to_send = None # 이번에 전송할 프레임 데이터
            with frame_lock: # latest_frame 접근 시 동기화
                if latest_frame is not None: # 카메라 루프에서 새 프레임을 만들었다면
                    frame_bytes_to_send = latest_frame
                else: # 아직 새 프레임이 없거나 카메라 오류 시
                    if _blank_img_bytes is None: # 빈 프레임 이미지를 아직 안 만들었다면 한번만 생성
                        _temp_blank_img = np.full((FRAME_H, FRAME_W, 3), 60, dtype=np.uint8) # 회색 배경
                        cv2.putText(_temp_blank_img, "Waiting...", (10, FRAME_H // 2), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,200,200) ,1)
                        _ok_blank, _buf_blank = cv2.imencode('.jpg', _temp_blank_img)
                        if _ok_blank: _blank_img_bytes = _buf_blank.tobytes()
                        else: _blank_img_bytes = b'' # 인코딩 실패 시 빈 바이트
                    frame_bytes_to_send = _blank_img_bytes
            
            if frame_bytes_to_send: # 전송할 프레임 데이터가 있다면
                # MJPEG 스트림 형식에 맞춰 데이터 전송
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes_to_send + b'\r\n')
            
            time.sleep(1/25) # 스트리밍 FPS 조절 (약 25fps)
            
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# ──────────────────────────────────────────────────────
# 메인 실행 부분
# ──────────────────────────────────────────────────────
if __name__ == '__main__':
    try:
        print("[🚀] Starting Line Tracing Application (Commented Version)")
        if picam2: # 카메라가 성공적으로 초기화되었을 경우에만 카메라 루프 스레드 시작
            cam_thread = threading.Thread(target=camera_loop, daemon=True) # 데몬 스레드로 설정 (메인 스레드 종료 시 자동 종료)
            cam_thread.start()
        else: # 카메라 초기화 실패 시
            print("[🚨] Camera not available. MJPEG stream will only show blank/error message.")

        # Flask 웹 서버 실행
        # host="0.0.0.0": 모든 네트워크 인터페이스에서 접속 허용
        # threaded=True: 여러 클라이언트 요청 동시 처리
        # debug=False, use_reloader=False: 프로덕션 또는 안정적인 실행을 위한 일반적인 설정
        app.run(host="0.0.0.0", port=5000, threaded=True, debug=False, use_reloader=False)

    except KeyboardInterrupt: # Ctrl+C 입력 시 예외 처리
        print("\n🛑 KeyboardInterrupt caught. Shutting down...")
    finally: # 프로그램 종료 시 항상 실행되는 부분 (자원 정리 등)
        print("✅ Cleaning up resources...")
        if ser and ser.is_open: # 시리얼 포트가 열려있다면
            try:
                print("   Sending STOP command to Arduino...")
                # 종료 시 아두이노에 정지 명령 전송
                stop_message = bytes(['P'.encode('ascii')[0], STEER_BYTE_CENTER, THROTTLE_BYTE_STOP])
                ser.write(stop_message)
                time.sleep(0.1) # 명령 전송을 위한 짧은 대기
                ser.close()     # 시리얼 포트 닫기
                print("   Serial port closed.")
            except Exception as e: print(f"   Error during serial cleanup: {e}")
        
        if picam2: # Picamera2 객체가 존재한다면 (초기화 성공 시)
            try: 
                picam2.stop() # 카메라 정지
                print("   Picamera2 stopped.")
            except Exception as e: print(f"   Error stopping Picamera2: {e}")
        print("✅ Application shutdown complete.")