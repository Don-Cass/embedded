import cv2 as cv
import numpy as np
import RPi.GPIO as GPIO
import time

PWMA = 18
PWMB = 23
AIN1 = 22
AIN2 = 27
BIN1 = 25
BIN2 = 24

# GPIO 설정
GPIO.setmode(GPIO.BCM)
GPIO.setup(AIN1, GPIO.OUT)
GPIO.setup(AIN2, GPIO.OUT)
GPIO.setup(BIN1, GPIO.OUT)
GPIO.setup(BIN2, GPIO.OUT)
GPIO.setup(PWMA, GPIO.OUT)
GPIO.setup(PWMB, GPIO.OUT)

# PWM 초기화
pwmA = GPIO.PWM(PWMA, 100)  # PWM 주파수 100Hz
pwmB = GPIO.PWM(PWMB, 100)
pwmA.start(0)  # 초기 듀티 사이클 0%
pwmB.start(0)

# 데이터 저장을 위한 리스트와 누적 시간 변수
contour_data = []
cumulative_go = 0
cumulative_right = 0
cumulative_left = 0
start_time = time.time()

def set_motor(speed_a, speed_b, direction_a, direction_b):
    pwmA.ChangeDutyCycle(speed_a)
    pwmB.ChangeDutyCycle(speed_b)
    GPIO.output(AIN1, direction_a[0])
    GPIO.output(AIN2, direction_a[1])
    GPIO.output(BIN1, direction_b[0])
    GPIO.output(BIN2, direction_b[1])

def go():
    set_motor(50, 50, (GPIO.LOW, GPIO.HIGH), (GPIO.LOW, GPIO.HIGH))

def right():
    set_motor(50, 0, (GPIO.LOW, GPIO.HIGH), (GPIO.LOW, GPIO.LOW))

def left():
    set_motor(0, 50, (GPIO.LOW, GPIO.LOW), (GPIO.LOW, GPIO.HIGH))

def stop():
    set_motor(0, 0, (GPIO.LOW, GPIO.LOW), (GPIO.LOW, GPIO.LOW))

def detect_lines_and_control():
    global cumulative_go, cumulative_right, cumulative_left, start_time
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

            # 이미지 회전 (필요 시)
            frame = cv.rotate(frame, cv.ROTATE_180)

            # 이미지 크기 가져오기
            height, width, _ = frame.shape

            # 관심 영역(ROI) 설정: 화면 하단 절반
            roi = frame[height//2:height, 0:width]

            # BGR에서 HSV 색공간으로 변환
            hsv = cv.cvtColor(roi, cv.COLOR_BGR2HSV)

            # 노란색의 HSV 범위 정의
            lower_yellow = np.array([20, 70, 70])
            upper_yellow = np.array([75, 255, 255])

            # 노란색 마스크 생성
            mask = cv.inRange(hsv, lower_yellow, upper_yellow)

            # 노이즈 제거를 위한 모폴로지 연산
            kernel = np.ones((5, 5), np.uint8)
            mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
            mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)

            # 엣지 검출
            edges = cv.Canny(mask, 50, 150, apertureSize=3)

            # 허프 변환을 이용한 선 검출
            lines = cv.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)

            # 좌우 선의 기울기 저장 리스트
            left_slopes = []
            right_slopes = []
            contour_L, contour_R = None, None
            slope_L, slope_R = None, None

            # 선 검출 및 좌우 분류
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    if y2 > y1:
                        x1, x2 = x2, x1
                        y1, y2 = y2, y1  # 좌표를 맞춰 교환
                    # 기울기 계산 (분모 0 방지)
                    if x2 - x1 == 0:
                        continue  # 수직선은 무시
                    slope = (y2 - y1) / (x2 - x1)

                    # 선의 중간 x 좌표를 기준으로 좌측 또는 우측 분류
                    if x1 < width / 2 and slope < 0:
                        left_slopes.append(slope)
                        contour_L = ((x1 + x2) // 2, (y1 + y2) // 2)
                        slope_L = slope
                        cv.line(roi, (x1, y1), (x2, y2), (0, 255, 0), 2)  # 녹색 선
                    elif x1 > width / 2 and slope > 0:
                        right_slopes.append(slope)
                        contour_R = ((x1 + x2) // 2, (y1 + y2) // 2)
                        slope_R = slope
                        cv.line(roi, (x1, y1), (x2, y2), (0, 0, 255), 2)  # 빨간색 선

            # 상태 초기화
            state = "go"

            # 우회전 감지: 왼쪽 선의 기울기가 감소하고, 오른쪽 선이 없음
            if len(left_slopes) > 0 and len(right_slopes) == 0:
                average_left_slope = np.mean(left_slopes)
                if average_left_slope < 0.5:
                    state = "right"

            # 좌회전 감지: 오른쪽 선의 기울기가 증가하고, 왼쪽 선이 없음
            elif len(right_slopes) > 0 and len(left_slopes) == 0:
                average_right_slope = np.mean(right_slopes)
                if average_right_slope > -0.5:
                    state = "left"

            # 직진 상태: 좌우 두 선이 모두 검출되고, 기울기의 변화가 일정
            elif len(left_slopes) > 0 and len(right_slopes) > 0:
                state = "go"

            # 누적 시간 갱신
            elapsed_time = time.time() - start_time
            if state == "go":
                cumulative_go += elapsed_time
            elif state == "right":
                cumulative_right += elapsed_time
            elif state == "left":
                cumulative_left += elapsed_time
            start_time = time.time()

            # 데이터 구조 저장
            data_entry = {
                "time": time.time(),
                "action": state,
                "cumulative_go": cumulative_go,
                "cumulative_right": cumulative_right,
                "cumulative_left": cumulative_left,
                "contour_L_center": contour_L if contour_L else (None, None),
                "contour_R_center": contour_R if contour_R else (None, None),
                "slope_L": slope_L,
                "slope_R": slope_R
            }
            contour_data.append(data_entry)

            # 상태 출력 및 모터 제어
            if state == "right":
                right()
            elif state == "left":
                left()
            elif state == "go":
                go()
            else:
                stop()

            # 결과 영상 보여주기
            cv.putText(roi, f"State: {state}", (50, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv.LINE_AA)
            cv.imshow('Frame', roi)
            cv.imshow('Mask', mask)

            # 'q' 키를 누르면 종료
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

        # 데이터 저장 (파일에 쓰기)
        with open("contour_data.txt", "w") as file:
            for data in contour_data:
                file.write(f"{data}\n")

if __name__ == "__main__":
    detect_lines_and_control()
