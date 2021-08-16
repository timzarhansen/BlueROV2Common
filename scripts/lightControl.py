import time
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.OUT)

p = GPIO.PWM(23, 50)  # frequency=50Hz
p.start(0)
try:
    while 1:
        print("starting Light")
        p.ChangeDutyCycle(10)
        time.sleep(0.5)
        p.ChangeDutyCycle(30)
        time.sleep(0.5)
        p.ChangeDutyCycle(60)
        time.sleep(0.5)
        p.ChangeDutyCycle(80)
        time.sleep(0.5)
        p.ChangeDutyCycle(100)
        time.sleep(0.5)
except KeyboardInterrupt:
    pass
    p.stop()
    GPIO.cleanup()