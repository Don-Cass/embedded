import cv2 as cv
import numpy as np
import RPi.GPIO as GPIO
from collections import deque
import threading
import time
from queue import Queue

# GPIO 설정 (기존 코드 유지)
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

# 기본 주행 속도과 계수 설정
base_speed = 30  # 기본 모터 속도
k1 = 5.0        # x축 미분 계수 (조정 필요)
k_slope = 0.5    # 기울기 변화 계수 (직선의 기울기 변화 반영)
k_rotation = 15.0  # 회전 보정 계수 (추가)

# 딜레이 버퍼 설정 (일단 0으로 설정)
delay_frames = 0
delay_buffer = deque(maxlen=delay_frames) if delay_frames > 0 else None

# 최근 프레임 데이터 저장을 위한 deque
max_frames = 5
x_L_history = deque(maxlen=max_frames)
slope_L_history = deque(maxlen=max_frames)
x_R_history = deque(maxlen=max_frames)
slope_R_history = deque(maxlen=max_frames)

# 최근 감지된 좌표를 저장하는 변수 초기화
last_x_L = None
last_x_R = None

# 모터 속도 설정 함수 (기존 코드 유지)
def set_motor(speed_a, speed_b, direction_a, direction_b):
    pwmA.ChangeDutyCycle(abs(speed_a))
    pwmB.ChangeDutyCycle(abs(speed_b))
    GPIO.output(AIN1, direction_a[0] if speed_a >= 0 else direction_a[1])
    GPIO.output(AIN2, direction_a[1] if speed_a >= 0 else direction_a[0])
    GPIO.output(BIN1, direction_b[0] if speed_b >= 0 else direction_b[1])
    GPIO.output(BIN2, direction_b[1] if speed_b >= 0 else direction_b[0])

def stop():
    set_motor(0, 0, (GPIO.LOW, GPIO.LOW), (GPIO.LOW, GPIO.LOW))

# 모터 속도 조정 함수 수정 (회전 각도 고려)
def adjust_motor_speed(contour_L_center, slope_L, contour_R_center, slope_R):
    global last_x_L, last_x_R
    
    # 현재 x 좌표 가져오기 (None인 경우에는 최근 값 유지)
    x_L = contour_L_center[0] if contour_L_center is not None else last_x_L
    x_R = contour_R_center[0] if contour_R_center is not None else last_x_R

    # 최신 값이 None이 아닌 경우에만 last_x_L, last_x_R 갱신
    if contour_L_center is not None:
        last_x_L = x_L
    if contour_R_center is not None:
        last_x_R = x_R

    # 초기 모터 속도 설정
    lm = base_speed
    rm = base_speed

    # 히스토리에 현재 x 좌표와 기울기 추가
    if x_L is not None:
        x_L_history.append(x_L)
    if slope_L is not None:
        slope_L_history.append(slope_L)
    if x_R is not None:
        x_R_history.append(x_R)
    if slope_R is not None:
        slope_R_history.append(slope_R)

    # 충분한 데이터가 쌓일 때까지 기본 속도 유지
    if (contour_L_center is not None and len(x_L_history) < max_frames) or (contour_R_center is not None and len(x_R_history) < max_frames):
        return lm, rm

    # x축 미분 계산 (둘 다 있을 경우 평균, 하나만 있을 경우 해당 값을 사용)
    delta_x = 0
    if x_L is not None and x_R is not None and len(x_L_history) >= 2 and len(x_R_history) >= 2:
        # 양쪽 히스토리의 평균 변화량 계산
        delta_x_L = np.mean([x_L_history[i] - x_L_history[i - 1] for i in range(1, len(x_L_history))])
        delta_x_R = np.mean([x_R_history[i] - x_R_history[i - 1] for i in range(1, len(x_R_history))])
        delta_x = (delta_x_L + delta_x_R) / 2
    elif x_L is not None and len(x_L_history) >= 2:
        delta_x = np.mean([x_L_history[i] - x_L_history[i - 1] for i in range(1, len(x_L_history))])
    elif x_R is not None and len(x_R_history) >= 2:
        delta_x = np.mean([x_R_history[i] - x_R_history[i - 1] for i in range(1, len(x_R_history))])

    # 기울기 변화량 계산
    delta_slope = 0
    if slope_L is not None and slope_R is not None:
        delta_slope = (slope_L + slope_R) / 2
    elif slope_L is not None:
        delta_slope = slope_L
    elif slope_R is not None:
        delta_slope = slope_R

    # 회전 각도 추정 (단순화된 예)
    rotation_angle = 0
    if slope_L is not None and slope_R is not None:
        rotation_angle = (slope_R - slope_L) / 2  # 예시: 좌우 기울기 차이를 이용한 회전 각도 추정
    elif slope_L is not None:
        rotation_angle = -slope_L  # 좌측 기울기만 있는 경우
    elif slope_R is not None:
        rotation_angle = slope_R   # 우측 기울기만 있는 경우

    # 회전 보정 적용
    lm -= k_rotation * rotation_angle
    rm += k_rotation * rotation_angle

    # 지연된 변화량 사용 (delay_frames이 0이면 즉시 반영)
    if delay_frames > 0 and delay_buffer is not None:
        delay_buffer.append((delta_x, delta_slope))
        if len(delay_buffer) < delay_frames:
            return lm, rm
        delayed_delta_x, delayed_delta_slope = delay_buffer.popleft()
    else:
        delayed_delta_x, delayed_delta_slope = delta_x, delta_slope

    # 모터 속도 조정 (기존 로직 유지)
    lm = max(0, min(80, lm + k1 * delayed_delta_x - k_slope * delayed_delta_slope))
    rm = max(0, min(80, rm - k1 * delayed_delta_x + k_slope * delayed_delta_slope))

    return lm, rm

# 큐 설정: 이미지 처리 결과를 모터 제어 스레드로 전달
control_queue = Queue()

# 이미지 처리 및 카메라 캡처 스레드 함수 (기존 코드 수정)
def camera_thread_func(stop_event):
    cap = cv.VideoCapture(0)
    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        control_queue.put(('stop', 0, 0))
        return

    try:
        while not stop_event.is_set():
            ret, frame = cap.read()
            if not ret:
                print("프레임을 가져올 수 없습니다.")
                control_queue.put(('stop', 0, 0))
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
            slope_L, slope_R = None, None

            # 왼쪽 및 오른쪽 컨투어 중심 좌표와 기울기 계산
            for contour in contours:
                M = cv.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    if cx < width // 2 and contour_L is None:
                        contour_L = (cx, cy)
                        try:
                            [vx, vy, _, _] = cv.fitLine(contour, cv.DIST_L2, 0, 0.01, 0.01)
                            if vx != 0:
                                slope_L = float(vy / vx)
                            else:
                                slope_L = 0
                        except Exception as e:
                            print(f"fitLine 오류 (왼쪽): {e}")
                            slope_L = None
                    elif cx >= width // 2 and contour_R is None:
                        contour_R = (cx, cy)
                        try:
                            [vx, vy, _, _] = cv.fitLine(contour, cv.DIST_L2, 0, 0.01, 0.01)
                            if vx != 0:
                                slope_R = float(vy / vx)
                            else:
                                slope_R = 0
                        except Exception as e:
                            print(f"fitLine 오류 (오른쪽): {e}")
                            slope_R = None

            # 모터 속도 조정
            lm, rm = adjust_motor_speed(contour_L, slope_L, contour_R, slope_R)

            # 모터 제어 큐에 속도 전달
            control_queue.put(('set_motor', lm, rm))

            # 디버깅용 상태 출력
            lm_text = f"{lm:.2f}"
            rm_text = f"{rm:.2f}"
            slope_L_text = f"{slope_L:.2f}" if slope_L is not None else "N/A"
            slope_R_text = f"{slope_R:.2f}" if slope_R is not None else "N/A"
            cv.putText(roi, f"lm: {lm_text}, rm: {rm_text}", (50, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv.putText(roi, f"slope_L: {slope_L_text}, slope_R: {slope_R_text}", (50, 100), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            cv.imshow('Frame', roi)
            cv.imshow('Mask', mask)

            # 'q' 키를 누르면 종료
            if cv.waitKey(1) & 0xFF == ord('q'):
                control_queue.put(('stop', 0, 0))
                break

    except KeyboardInterrupt:
        control_queue.put(('stop', 0, 0))

    finally:
        cap.release()
        cv.destroyAllWindows()

# 모터 제어 스레드 함수 (기존 코드 유지)
def motor_control_thread_func(stop_event):
    try:
        while not stop_event.is_set():
            if not control_queue.empty():
                command, lm, rm = control_queue.get()
                if command == 'set_motor':
                    # 회전 보정을 반영한 방향 설정
                    set_motor(lm, rm, (GPIO.LOW, GPIO.HIGH), (GPIO.LOW, GPIO.HIGH))
                elif command == 'stop':
                    stop()
                    break
            time.sleep(0.01)  # CPU 사용률을 낮추기 위해 잠시 대기
    except Exception as e:
        print(f"모터 제어 스레드 오류: {e}")
    finally:
        stop()

# 메인 함수 (기존 코드 유지)
def detect_lines_and_control():
    # 종료 신호를 전달할 이벤트 설정
    stop_event = threading.Event()

    # 스레드 생성
    camera_thread = threading.Thread(target=camera_thread_func, args=(stop_event,))
    motor_thread = threading.Thread(target=motor_control_thread_func, args=(stop_event,))

    # 스레드 시작
    camera_thread.start()
    motor_thread.start()

    try:
        # 메인 스레드는 사용자 입력을 대기
        while camera_thread.is_alive() and motor_thread.is_alive():
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("종료 신호를 받았습니다.")
        stop_event.set()
        control_queue.put(('stop', 0, 0))

    # 스레드 종료 대기
    camera_thread.join()
    motor_thread.join()

    # GPIO 정리
    pwmA.stop()
    pwmB.stop()
    GPIO.cleanup()

if __name__ == "__main__":
    detect_lines_and_control()
