# import RPi.GPIO as GPIO
# import time

# # 왼쪽 모터 핀 정의
# PWMA = 18
# AIN1 = 22
# AIN2 = 27

# # 오른쪽 모터 핀 정의
# PWMB = 23
# AIN3 = 24
# AIN4 = 25

# GPIO.setwarnings(False)
# GPIO.setmode(GPIO.BCM)

# # 왼쪽 모터 설정
# GPIO.setup(PWMA, GPIO.OUT)
# GPIO.setup(AIN1, GPIO.OUT)
# GPIO.setup(AIN2, GPIO.OUT)

# # 오른쪽 모터 설정
# GPIO.setup(PWMB, GPIO.OUT)
# GPIO.setup(AIN3, GPIO.OUT)
# GPIO.setup(AIN4, GPIO.OUT)

# # 왼쪽 및 오른쪽 모터의 PWM 초기화
# L_Motor = GPIO.PWM(PWMA, 500)
# R_Motor = GPIO.PWM(PWMB, 500)

# L_Motor.start(0)
# R_Motor.start(0)

# try:
#     while True:
#         # 왼쪽 모터: 정방향으로 최대 속도(100%)
#         GPIO.output(AIN1, 0)
#         GPIO.output(AIN2, 1)
#         L_Motor.ChangeDutyCycle(100)
        
#         # 오른쪽 모터: 정방향으로 50% 속도
#         GPIO.output(AIN3, 1)
#         GPIO.output(AIN4, 0)
#         R_Motor.ChangeDutyCycle(50)

#         time.sleep(1.0)

#         # 두 모터 모두 정지
#         L_Motor.ChangeDutyCycle(0)
#         R_Motor.ChangeDutyCycle(0)
#         time.sleep(1.0)

# except KeyboardInterrupt:
#     pass

# GPIO.cleanup()


import RPi.GPIO as GPIO
import time

# 모터 핀 설정
PWMA = 18  # 왼쪽 모터
AIN1 = 22
AIN2 = 27
PWMB = 23  # 오른쪽 모터
AIN3 = 24
AIN4 = 25

# 스위치 핀 설정
SW1 = 5    # 앞
SW2 = 6    # 오른쪽
SW3 = 13   # 왼쪽
SW4 = 19   # 뒤

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# 모터 핀 설정
GPIO.setup(PWMA, GPIO.OUT)
GPIO.setup(AIN1, GPIO.OUT)
GPIO.setup(AIN2, GPIO.OUT)
GPIO.setup(PWMB, GPIO.OUT)
GPIO.setup(AIN3, GPIO.OUT)
GPIO.setup(AIN4, GPIO.OUT)

# 스위치 핀 설정
GPIO.setup(SW1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(SW2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(SW3, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(SW4, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# 모터 PWM 초기화
L_Motor = GPIO.PWM(PWMA, 500)
R_Motor = GPIO.PWM(PWMB, 500)

L_Motor.start(0)
R_Motor.start(0)

try:
    while True:
        if GPIO.input(SW1) == GPIO.HIGH:  # 앞쪽 이동
            print("SW1 눌림: 앞으로 이동")
            GPIO.output(AIN1, 0)
            GPIO.output(AIN2, 1)
            GPIO.output(AIN3, 1)
            GPIO.output(AIN4, 0)
            L_Motor.ChangeDutyCycle(100)
            R_Motor.ChangeDutyCycle(100)

        elif GPIO.input(SW2) == GPIO.HIGH:  # 오른쪽 이동
            print("SW2 눌림: 오른쪽으로 이동")
            GPIO.output(AIN1, 0)
            GPIO.output(AIN2, 1)
            GPIO.output(AIN3, 0)
            GPIO.output(AIN4, 1)
            L_Motor.ChangeDutyCycle(50)
            R_Motor.ChangeDutyCycle(50)

        elif GPIO.input(SW3) == GPIO.HIGH:  # 왼쪽 이동
            print("SW3 눌림: 왼쪽으로 이동")
            GPIO.output(AIN1, 1)
            GPIO.output(AIN2, 0)
            GPIO.output(AIN3, 1)
            GPIO.output(AIN4, 0)
            L_Motor.ChangeDutyCycle(50)
            R_Motor.ChangeDutyCycle(50)

        elif GPIO.input(SW4) == GPIO.HIGH:  # 뒤로 이동
            print("SW4 눌림: 뒤로 이동")
            GPIO.output(AIN1, 1)
            GPIO.output(AIN2, 0)
            GPIO.output(AIN3, 0)
            GPIO.output(AIN4, 1)
            L_Motor.ChangeDutyCycle(100)
            R_Motor.ChangeDutyCycle(100)

        else:  # 스위치가 눌리지 않았을 때 정지
            L_Motor.ChangeDutyCycle(0)
            R_Motor.ChangeDutyCycle(0)

        time.sleep(0.1)  # 짧은 지연 시간

except KeyboardInterrupt:
    pass

GPIO.cleanup()
