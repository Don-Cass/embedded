import RPi.GPIO as GPIO
import time

BUZZER = 12
SWITCH = 5

# 각 음계의 주파수 (Hz)
scale = [261, 293, 329, 349, 392, 440, 493, 523]  # 도, 레, 미, 파, 솔, 라, 시, 도

horn = [
    (523, 0.2),  # 도
    (0, 0.1),    # 휴식 (무음)
    (523, 0.2),  # 도
    (0, 0.1),
    (659, 0.5),  # 미
]

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUZZER, GPIO.OUT)
GPIO.setup(SWITCH, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


p = GPIO.PWM(BUZZER, 100)  # 초기 주파수는 임의로 설정
p.start(50)



# # 1. "도레미파솔라시도" 음계를 출력
# try:
#     for freq in scale:
#         p.ChangeFrequency(freq)
#         time.sleep(0.5)  # 각 음을 0.5초씩 재생
#     p.stop()
    
# except KeyboardInterrupt:
#     pass

# p.stop()
# GPIO.cleanup()


# # 2. 나만의 경적 소리 구현
# try:
#     for freq, duration in horn:
#         if freq == 0:
#             p.ChangeDutyCycle(0)  # 무음 처리
#         else:
#             p.ChangeFrequency(freq)
#             p.ChangeDutyCycle(50)
#         time.sleep(duration)
#     p.stop()

# except KeyboardInterrupt:
#     pass

# p.stop()
# GPIO.cleanup()

# 3. 스위치를 한 번 누르면 경적 소리가 나도록 구현

# p.stop()

# prev_input = 0

# try:
#     while True:
#         input = GPIO.input(SWITCH)
#         if prev_input == 0 and input == 1:
#             # 스위치가 눌렸을 때 경적 소리 재생
#             p.start(50)
#             for freq, duration in horn:
#                 if freq == 0:
#                     p.ChangeDutyCycle(0)
#                 else:
#                     p.ChangeFrequency(freq)
#                     p.ChangeDutyCycle(50)
#                 time.sleep(duration)
#             p.stop()
#         prev_input = input
#         time.sleep(0.05)

# except KeyboardInterrupt:
#     pass

# p.stop()
# GPIO.cleanup()

# 4. 스위치 4개를 사용하여 나만의 음악을 연주

SW_PINS = [5, 6, 13, 19]
notes = [261, 293, 329, 349]  # 도, 레, 미, 파
for pin in SW_PINS:
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    

p.stop()  # 초기에는 소리를 내지 않음

prev_inputs = [0, 0, 0, 0]

try:
    while True:
        for idx, pin in enumerate(SW_PINS):
            input = GPIO.input(pin)
            if prev_inputs[idx] == 0 and input == 1:
                # 스위치가 눌렸을 때 해당 음계 재생
                freq = notes[idx]
                p.start(50)
                p.ChangeFrequency(freq)
                time.sleep(0.5)
                p.stop()
            prev_inputs[idx] = input
        time.sleep(0.05)

except KeyboardInterrupt:
    pass

p.stop()
GPIO.cleanup()