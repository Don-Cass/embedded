import RPi.GPIO as GPIO
import time

# 스위치와 GPIO 핀 매핑
SW1_PIN = 5    # GPIO05
SW2_PIN = 6    # GPIO06
SW3_PIN = 13   # GPIO13
SW4_PIN = 19   # GPIO19

SW_PINS = [SW1_PIN, SW2_PIN, SW3_PIN, SW4_PIN]
click_counts = [0, 0, 0, 0]
prev_values = [0, 0, 0, 0]

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
for pin in SW_PINS:
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

try:
    while True:
        for idx, pin in enumerate(SW_PINS):
            value = GPIO.input(pin)
            if prev_values[idx] == 0 and value == 1:
                click_counts[idx] += 1
                print(f"('SW{idx+1} click', {click_counts[idx]})")
            prev_values[idx] = value
        time.sleep(0.1)

except KeyboardInterrupt:
    pass

GPIO.cleanup()


