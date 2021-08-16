import time
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.OUT)

p = GPIO.PWM(23, 50)  # frequency=50Hz
p.start(0)
try:
    while 1:
        p.ChangeDutyCycle(5)
        print("5")
        time.sleep(1)
        p.ChangeDutyCycle(6)
        print("6")
        time.sleep(1)
        p.ChangeDutyCycle(7)
        print("7")
        time.sleep(1)
        p.ChangeDutyCycle(8)
        print("8")
        time.sleep(1)
        p.ChangeDutyCycle(9)
        print("9")
        time.sleep(1)
        p.ChangeDutyCycle(10)
        print("10")
        time.sleep(1)
except KeyboardInterrupt:
    pass
    p.stop()
    GPIO.cleanup()