import time
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.OUT)

p = GPIO.PWM(23, 50)  # frequency=50Hz
p.start(0)
try:
    while 1:
        p.ChangeDutyCycle(1)
        time.sleep(0.5)
        p.ChangeDutyCycle(2)
        time.sleep(0.5)
        p.ChangeDutyCycle(3)
        time.sleep(0.5)
        p.ChangeDutyCycle(4)
        time.sleep(0.5)
        p.ChangeDutyCycle(5)
        time.sleep(0.5)
except KeyboardInterrupt:
    pass
    p.stop()
    GPIO.cleanup()