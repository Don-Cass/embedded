
import cv2
import numpy as np
import os


print("Current working directory:", os.getcwd())

file_paths = [
    os.path.join("imgs", "1.jpg"),
    os.path.join("imgs", "2.jpg"),
    os.path.join("imgs", "3.jpg"),
    os.path.join("imgs", "4.jpg")
]



for file_path in file_paths:
    # 이미지 읽기
    img = cv2.imread(file_path)

    # 이미지를 HSV 색공간으로 변환
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # 노란색의 HSV 범위 정의
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])

    # 노란색 마스크 생성
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # 마스크를 원본 이미지에 적용
    result = cv2.bitwise_and(img, img, mask=mask)

    # 결과 이미지 저장
    output_path = os.path.join("output", os.path.basename(file_path))
    cv2.imwrite(output_path, result)


