import time
import RPi.GPIO as GPIO





GPIO.setmode(GPIO.BCM)
GPIO.setup(12, GPIO.OUT)

p = GPIO.PWM(12, 50)  # frequency=50Hz
p.start(0)


def SetAngle(angle):

    duty = angle / 32 + 4
    GPIO.output(12, True)
    p.ChangeDutyCycle(duty)

    time.sleep(2)

    GPIO.output(12, False)

    p.ChangeDutyCycle(0)

try:

    SetAngle(10)
    print(10)
    time.sleep(2)
    SetAngle(30)
    print(30)
    time.sleep(2)
    SetAngle(50)
    print(50)
    time.sleep(2)
    SetAngle(90)
    print(90)
    time.sleep(2)
    SetAngle(160)
    print(160)
    time.sleep(2)
except KeyboardInterrupt:
    pass
p.stop()
GPIO.cleanup()