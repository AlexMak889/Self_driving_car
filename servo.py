import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

servo_pin = 18
GPIO.setup(servo_pin, GPIO.OUT)

pwm = GPIO.PWM(servo_pin, 50)  # 50Hz frequency
pwm.start(0)
print("start")
try:
    while True:
        pwm.ChangeDutyCycle(2.5)  # Move to 0 degrees
        print("o degres")
        time.sleep(2)
        pwm.ChangeDutyCycle(7.5)  # Move to 90 degrees
        print("90 degreas")
        time.sleep(2)
        pwm.ChangeDutyCycle(12.5)  # Move to 180 degrees
        print("180 degreas")
        time.sleep(2)
except KeyboardInterrupt:
    pass

finally:
    pwm.stop()
    GPIO.cleanup()
