import cv2 as cv
import numpy as np
import RPi.GPIO as GPIO
from collections import deque

# GPIO 설정
PWMA = 18
PWMB = 23
AIN1 = 22
AIN2 = 27
BIN1 = 25
BIN2 = 24

GPIO.setmode(GPIO.BCM)
GPIO.setup(AIN1, GPIO.OUT)
GPIO.setup(AIN2, GPIO.OUT)
GPIO.setup(BIN1, GPIO.OUT)
GPIO.setup(BIN2, GPIO.OUT)
GPIO.setup(PWMA, GPIO.OUT)
GPIO.setup(PWMB, GPIO.OUT)

pwmA = GPIO.PWM(PWMA, 100)
pwmB = GPIO.PWM(PWMB, 100)
pwmA.start(0)
pwmB.start(0)

# 기본 주행 속도와 계수 설정
base_speed = 40  # 기본 모터 속도
lm = base_speed
rm = base_speed
max_speed = 80
# PID 제어를 위한 파라미터
Kp = 0.15   # 비례 계수
Ki = 0.01   # 적분 계수
Kd = 0.1  # 미분 계수

# PID 제어 변수 초기화
previous_error = 0
integral = 0
output_integral = 0


def pid_control(error):
    global previous_error, integral

    # P, I, D 계산
    proportional = Kp * error
    integral += error  # 적분값 누적
    derivative = error - previous_error  # 미분값 계산

    # PID 출력 계산
    output = proportional + Ki * integral + Kd * derivative


    # 이전 에러 업데이트
    previous_error = error

    return output

# 딜레이 버퍼 설정 (일단 0으로 설정)
delay_frames = 0
delay_buffer = deque(maxlen=delay_frames) if delay_frames > 0 else None

# 최근 프레임 데이터 저장을 위한 deque
max_frames = 3
x_L_history = deque(maxlen=max_frames)
x_R_history = deque(maxlen=max_frames)

# 최근 감지된 좌표를 저장하는 변수 초기화
last_x_L = None
last_x_R = None

# 모터 속도 설정 함수
def set_motor(speed_a, speed_b, direction_a, direction_b):
    pwmA.ChangeDutyCycle(abs(speed_a))
    pwmB.ChangeDutyCycle(abs(speed_b))
    GPIO.output(AIN1, direction_a[0] if speed_a >= 0 else direction_a[1])
    GPIO.output(AIN2, direction_a[1] if speed_a >= 0 else direction_a[0])
    GPIO.output(BIN1, direction_b[0] if speed_b >= 0 else direction_b[1])
    GPIO.output(BIN2, direction_b[1] if speed_b >= 0 else direction_b[0])

def stop():
    set_motor(0, 0, (GPIO.LOW, GPIO.LOW), (GPIO.LOW, GPIO.LOW))

# 모터 속도 조정 함수
def adjust_motor_speed(contour_L_center, contour_R_center):
    global last_x_L, last_x_R, lm, rm

    # 현재 x 좌표 가져오기 (None인 경우에는 최근 값 유지)
    x_L = contour_L_center[0] if contour_L_center is not None else last_x_L
    x_R = contour_R_center[0] if contour_R_center is not None else last_x_R

    # 최신 값 갱신
    if contour_L_center is not None:
        last_x_L = x_L
    if contour_R_center is not None:
        last_x_R = x_R


    # 히스토리에 현재 x 좌표 추가
    if x_L is not None:
        x_L_history.append(x_L)
    if x_R is not None:
        x_R_history.append(x_R)

    # 충분한 데이터가 쌓일 때까지 기본 속도 유지
    if (contour_L_center is not None and len(x_L_history) < max_frames) or (contour_R_center is not None and len(x_R_history) < max_frames):
        return lm, rm

    # x축 미분 계산
    delta_x = 0
    if x_L is not None and x_R is not None and len(x_L_history) >= 2 and len(x_R_history) >= 2:
        delta_x_L = np.mean([x_L_history[i] - x_L_history[i - 1] for i in range(1, len(x_L_history))])
        delta_x_R = np.mean([x_R_history[i] - x_R_history[i - 1] for i in range(1, len(x_R_history))])
        delta_x = (delta_x_L + delta_x_R) / 2
    elif x_L is not None and len(x_L_history) >= 2:
        delta_x = np.mean([x_L_history[i] - x_L_history[i - 1] for i in range(1, len(x_L_history))])
    elif x_R is not None and len(x_R_history) >= 2:
        delta_x = np.mean([x_R_history[i] - x_R_history[i - 1] for i in range(1, len(x_R_history))])

    # 지연된 변화량 사용
    if delay_frames > 0:
        delay_buffer.append(delta_x)
        if len(delay_buffer) < delay_frames:
            return lm, rm
        delayed_delta_x = delay_buffer.popleft()
    else:
        delayed_delta_x = delta_x


    # PID 제어로 출력 계산
    delta_pwm = pid_control(delayed_delta_x)

    lm += delta_pwm
    rm -= delta_pwm

    # 모터 속도 조정
    lm_out = max(-max_speed, min(max_speed, lm))
    rm_out = max(-max_speed, min(max_speed, rm))

    return lm_out, rm_out

# 메인 제어 함수
def detect_lines_and_control():
    cap = cv.VideoCapture(0)
    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        return

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("프레임을 가져올 수 없습니다.")
                break

            # 프레임 전처리 및 ROI 설정
            frame = cv.rotate(frame, cv.ROTATE_180)
            height, width, _ = frame.shape
            roi = frame[height // 2:height, 0:width]
            hsv = cv.cvtColor(roi, cv.COLOR_BGR2HSV)

            # 노란색 마스크 생성
            lower_yellow = np.array([20, 70, 70])
            upper_yellow = np.array([75, 255, 255])
            mask = cv.inRange(hsv, lower_yellow, upper_yellow)
            kernel = np.ones((5, 5), np.uint8)
            mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
            mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)

            # 컨투어 검출
            contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            contours = [cnt for cnt in contours if cv.contourArea(cnt) > 100]
            contours = sorted(contours, key=cv.contourArea, reverse=True)[:2]

            contour_L, contour_R = None, None

            # 왼쪽 및 오른쪽 컨투어 중심 좌표 계산
            for contour in contours:
                M = cv.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    if cx < width // 2 and contour_L is None:
                        contour_L = (cx, cy)
                    elif cx >= width // 2 and contour_R is None:
                        contour_R = (cx, cy)

            # 모터 속도 조정
            lm, rm = adjust_motor_speed(contour_L, contour_R)

            # 모터 방향 설정
            # 여기서는 항상 전진하도록 설정. 필요에 따라 조향에 따라 방향을 변경할 수 있음
            set_motor(lm, rm, (GPIO.LOW, GPIO.HIGH), (GPIO.LOW, GPIO.HIGH))

            # 디버깅용 상태 출력
            cv.putText(roi, f"lm: {lm:.2f}, rm: {rm:.2f}", (50, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv.imshow('Frame', roi)
            cv.imshow('Mask', mask)

            if cv.waitKey(1) & 0xFF == ord('q'):
                stop()
                break

    except KeyboardInterrupt:
        stop()

    finally:
        cap.release()
        cv.destroyAllWindows()
        pwmA.stop()
        pwmB.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    detect_lines_and_control()