import cv2 as cv
import numpy as np

def detect_lines_and_state():
    # 비디오 캡처 객체 초기화 (기본 카메라 사용)
    cap = cv.VideoCapture(0)
    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        return

    # 윈도우 설정
    cv.namedWindow('Frame', cv.WINDOW_NORMAL)
    cv.resizeWindow('Frame', 800, 600)

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

        # 선 검출 및 좌우 분류
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                # 기울기 계산 (분모 0 방지)
                if x2 - x1 == 0:
                    continue  # 수직선은 무시
                slope = (y2 - y1) / (x2 - x1)
                if abs(slope) < 0.5:
                    continue  # 너무 평평한 선은 무시

                # 선의 중간 x 좌표를 기준으로 좌측 또는 우측 분류
                mid_x = (x1 + x2) / 2
                if mid_x < width / 2:
                    left_slopes.append(slope)
                    cv.line(roi, (x1, y1), (x2, y2), (0, 255, 0), 2)  # 녹색 선
                else:
                    right_slopes.append(slope)
                    cv.line(roi, (x1, y1), (x2, y2), (0, 0, 255), 2)  # 빨간색 선

        # 상태 초기화
        state = "직진"

        # 우회전 감지: 왼쪽 선의 기울기가 감소하고, 오른쪽 선이 없음
        if len(left_slopes) > 0 and len(right_slopes) == 0:
            average_left_slope = np.mean(left_slopes)
            if average_left_slope < 0.5:  # 임계값 조정 가능
                state = "우회전"

        # 좌회전 감지: 오른쪽 선의 기울기가 증가하고, 왼쪽 선이 없음
        elif len(right_slopes) > 0 and len(left_slopes) == 0:
            average_right_slope = np.mean(right_slopes)
            if average_right_slope > -0.5:  # 임계값 조정 가능
                state = "좌회전"

        # 직진 상태: 좌우 두 선이 모두 검출되고, 기울기의 변화가 일정
        elif len(left_slopes) > 0 and len(right_slopes) > 0:
            average_left_slope = np.mean(left_slopes)
            average_right_slope = np.mean(right_slopes)
            # 중앙선을 계산하여 오차를 기반으로 직진 상태 판단 가능
            # 여기서는 단순히 직진 상태로 설정
            state = "직진"

        # 상태 출력
        cv.putText(roi, f"State: {state}", (50, 50), cv.FONT_HERSHEY_SIMPLEX, 
                   1, (255, 255, 255), 2, cv.LINE_AA)
        print(f"현재 상태: {state}")

        # 결과 영상 보여주기
        cv.imshow('Frame', roi)
        cv.imshow('Mask', mask)

        # 'q' 키를 누르면 종료
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    # 자원 해제
    cap.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    detect_lines_and_state()
