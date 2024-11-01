import threading
import serial
import time
import RPi.GPIO as GPIO

# GPIO 핀 설정
PWMA = 18
PWMB = 23
AIN1 = 22
AIN2 = 27
BIN1 = 25
BIN2 = 24

# 시리얼 통신 설정
bleSerial = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=1.0)

# 글로벌 변수
gData = ""

# GPIO 모드 설정 및 핀 초기화
GPIO.setmode(GPIO.BCM)
GPIO.setup(PWMA, GPIO.OUT)
GPIO.setup(PWMB, GPIO.OUT)
GPIO.setup(AIN1, GPIO.OUT)
GPIO.setup(AIN2, GPIO.OUT)
GPIO.setup(BIN1, GPIO.OUT)
GPIO.setup(BIN2, GPIO.OUT)

# PWM 설정
pwmA = GPIO.PWM(PWMA, 100)  # PWM 주파수 100Hz
pwmB = GPIO.PWM(PWMB, 100)  # PWM 주파수 100Hz
pwmA.start(0)  # 초기 속도 0
pwmB.start(0)  # 초기 속도 0

def serial_thread():
    global gData
    while True:
        data = bleSerial.readline()
        if data:
            gData = data.decode().strip()
        
def set_motor(speed_a, speed_b, direction_a, direction_b):
    pwmA.ChangeDutyCycle(speed_a)
    pwmB.ChangeDutyCycle(speed_b)
    
    GPIO.output(AIN1, direction_a[0])
    GPIO.output(AIN2, direction_a[1])
    GPIO.output(BIN1, direction_b[0])
    GPIO.output(BIN2, direction_b[1])

def go():
    # 앞으로 이동 (양쪽 모터 전진)
    set_motor(80, 80, (GPIO.LOW, GPIO.HIGH), (GPIO.LOW, GPIO.HIGH))

def back():
    # 뒤로 이동 (양쪽 모터 후진)
    set_motor(80, 80, (GPIO.HIGH, GPIO.LOW), (GPIO.HIGH, GPIO.LOW))

def left():
    # 좌회전 (왼쪽 멈추고 오른쪽 모터 전진)
    set_motor(0, 80, (GPIO.LOW, GPIO.LOW), (GPIO.LOW, GPIO.HIGH))

def right():
    # 우회전 (오른쪽 멈추고 왼쪽 모터 전진)
    set_motor(80, 0, (GPIO.LOW, GPIO.HIGH), (GPIO.LOW, GPIO.LOW))

def stop():
    # 모터 정지
    set_motor(0, 0, (GPIO.LOW, GPIO.LOW), (GPIO.LOW, GPIO.LOW))

def main():
    global gData
    try:
        while True:
            if gData == "go":
                go()
                print("Moving forward")
                gData = ""
            elif gData == "back":
                back()
                print("Moving backward")
                gData = ""
            elif gData == "left":
                left()
                print("Turning left")
                gData = ""
            elif gData == "right":
                right()
                print("Turning right")
                gData = ""
            elif gData == "stop":
                stop()
                print("Stopping")
                gData = ""
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        stop()
        pwmA.stop()
        pwmB.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    task1 = threading.Thread(target=serial_thread)
    task1.start()
    main()
    bleSerial.close()
