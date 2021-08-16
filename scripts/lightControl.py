import time
import RPi.GPIO as GPIO





GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.OUT)

p = GPIO.PWM(23, 50)  # frequency=50Hz
p.start(0)


def makeStrength0To10(strength):
    GPIO.output(23, True)
    p.ChangeDutyCycle(5.0+strength/2.5)
    time.sleep(0.2)
    GPIO.output(23, False)
    p.ChangeDutyCycle(0)

try:
    while 1:
        for i in range(0,100,2):
            makeStrength0To10(i/10.0)
            print(i)
            time.sleep(2)

except KeyboardInterrupt:
    pass
    p.stop()
    GPIO.cleanup()