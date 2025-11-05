#!/usr/bin/env python3
import cv2
import numpy as np
import time
from serial import Serial
import signal
import sys

# =========================
# 아두이노 통신 설정
# =========================
ser = Serial('/dev/arduino', 115200, timeout=0.5)

def create_command(steering, speed):
    STX = 0xEA
    ETX = 0x03
    Length = 0x03
    dummy1 = 0x00
    dummy2 = 0x00
    Checksum = ((~(Length + steering + speed + dummy1 + dummy2)) & 0xFF) + 1
    return bytearray([STX, Length, steering, speed, dummy1, dummy2, Checksum, ETX])

def safe_exit(signum=None, frame=None):
    print("\n[!] 안전 종료 중...")
    ser.write(create_command(90, 90))
    ser.close()
    sys.exit(0)

signal.signal(signal.SIGINT, safe_exit)

# =========================
# 카메라 설정
# =========================
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    print("카메라 연결 실패")
    safe_exit()

print("카메라 연결 완료. Ctrl+C 또는 ESC로 종료")

# =========================
# 메인 루프
# =========================
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임 캡처 실패")
            break

        # 1. 영상 전처리
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        _, binary = cv2.threshold(blur, 115, 255, cv2.THRESH_BINARY_INV)

        # 2. 특정 y좌표에서 밝기 분석
        y_line = 400
        line_pixels = binary[y_line, :]
        white_indices = np.where(line_pixels == 255)[0]

        if len(white_indices) > 0:
            lane_center = int(np.mean(white_indices))  # 흰색 영역의 중심
        else:
            lane_center = binary.shape[1] // 2  # 차선이 안 보이면 화면 중앙 사용

        midpoint = binary.shape[1] // 2
        error = lane_center - midpoint

        # 3. 조향각 계산 (우회전: 90보다 작게, 좌회전: 90보다 크게)
        steering = 90 + (error / 5.0)
        steering = int(np.clip(steering, 45, 135))

        # 4. 시각화
        vis = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
        cv2.line(vis, (midpoint, 0), (midpoint, vis.shape[0]), (255, 0, 0), 2)  # 화면 중앙
        cv2.line(vis, (lane_center, 0), (lane_center, vis.shape[0]), (0, 255, 0), 2)  # 검출된 중심
        cv2.line(vis, (0, y_line), (vis.shape[1], y_line), (0, 0, 255), 1)  # 분석한 y라인
        cv2.putText(vis, f"Steering: {steering}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.imshow("Lane View", vis)

        # 5. 조향 및 속도 명령 전송
        speed = 100
        ser.write(create_command(steering, speed))

        # ESC 키로 종료
        if cv2.waitKey(1) & 0xFF == 27:
            break

        time.sleep(0.05)

except KeyboardInterrupt:
    safe_exit()
finally:
    cap.release()
    cv2.destroyAllWindows()
    ser.close()

