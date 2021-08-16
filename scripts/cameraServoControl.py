import time
import RPi.GPIO as GPIO





GPIO.setmode(GPIO.BCM)
GPIO.setup(12, GPIO.OUT)

p = GPIO.PWM(12, 50)  # frequency=50Hz
p.start(0)


def SetAngle(angle):

    duty = angle / 18 + 2
    #GPIO.output(12, True)
    p.ChangeDutyCycle(duty)

    time.sleep(2)

    #GPIO.output(12, False)

    p.ChangeDutyCycle(0)

try:
    SetAngle(30)

except KeyboardInterrupt:
    pass
p.stop()
GPIO.cleanup()