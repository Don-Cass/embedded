import cv2
import os
import sys

def main():
    # 현재 작업 디렉토리 출력 (디버깅용)
    print("Current Working Directory:", os.getcwd())

    # Haar Cascade XML 파일의 경로 구성
    face_cascade_path = os.path.join(cv2.data.haarcascades, "haarcascade_frontalface_default.xml")
    eye_cascade_path = os.path.join(cv2.data.haarcascades, "haarcascade_eye.xml")

    # 분류기 초기화
    face_cascade = cv2.CascadeClassifier(face_cascade_path)
    eye_cascade = cv2.CascadeClassifier(eye_cascade_path)

    # 분류기가 성공적으로 로드되었는지 확인
    if face_cascade.empty():
        print(f"Error: 얼굴 분류기를 {face_cascade_path}에서 로드할 수 없습니다.")
        sys.exit(1)
    if eye_cascade.empty():
        print(f"Error: 눈 분류기를 {eye_cascade_path}에서 로드할 수 없습니다.")
        sys.exit(1)

    # 웹캠 캡처 객체 생성 (기본 카메라 사용)
    cap = cv2.VideoCapture(0)

    # 카메라가 정상적으로 열렸는지 확인
    if not cap.isOpened():
        print("Error: 카메라를 열 수 없습니다.")
        sys.exit(1)

    print("카메라가 성공적으로 열렸습니다. 'q' 키를 눌러 종료하세요.")

    while True:
        # 프레임 읽기
        ret, frame = cap.read()

        # 프레임을 성공적으로 읽었는지 확인
        if not ret:
            print("Error: 프레임을 읽을 수 없습니다.")
            break

        # 프레임을 180도 회전
        rotated_frame = cv2.rotate(frame, cv2.ROTATE_180)

        # 얼굴 검출을 위해 그레이스케일로 변환
        gray = cv2.cvtColor(rotated_frame, cv2.COLOR_BGR2GRAY)

        # 얼굴 검출
        faces = face_cascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30),
            flags=cv2.CASCADE_SCALE_IMAGE
        )

        # 검출된 얼굴 수 출력 (디버깅용)
        # print(f"Detected {len(faces)} face(s)")

        # 각 얼굴에 대해 사각형 그리기 및 눈 검출
        for (x, y, w, h) in faces:
            # 얼굴 주변에 사각형 그리기 (파란색, 두께 2)
            cv2.rectangle(rotated_frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

            # 얼굴 영역의 ROI 정의
            roi_gray = gray[y:y + h, x:x + w]
            roi_color = rotated_frame[y:y + h, x:x + w]

            # 눈 검출
            eyes = eye_cascade.detectMultiScale(
                roi_gray,
                scaleFactor=1.1,
                minNeighbors=10,
                minSize=(15, 15),
                flags=cv2.CASCADE_SCALE_IMAGE
            )

            # 각 눈에 대해 사각형 그리기 (초록색, 두께 2)
            for (ex, ey, ew, eh) in eyes:
                cv2.rectangle(roi_color, (ex, ey), (ex + ew, ey + eh), (0, 255, 0), 2)

        # 결과 프레임 표시 (회전된 컬러 이미지)
        cv2.imshow('Real-Time Face Detection (Rotated 180 Degrees)', rotated_frame)

        # 'q' 키를 누르면 루프 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("종료 키가 눌렸습니다. 프로그램을 종료합니다.")
            break

    # 리소스 해제
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
