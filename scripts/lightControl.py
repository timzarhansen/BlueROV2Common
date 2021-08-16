import time
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.OUT)

p = GPIO.PWM(23, 50)  # frequency=50Hz
p.start(0)
try:
    while 1:
        print("starting Light")
        p.ChangeDutyCycle(1)
        time.sleep(1.5)
        p.ChangeDutyCycle(3)
        time.sleep(1.5)
        p.ChangeDutyCycle(6)
        time.sleep(1.5)
        p.ChangeDutyCycle(8)
        time.sleep(1.5)
        p.ChangeDutyCycle(10)
        time.sleep(1.5)
except KeyboardInterrupt:
    pass
    p.stop()
    GPIO.cleanup()