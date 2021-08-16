import time
import RPi.GPIO as GPIO





GPIO.setmode(GPIO.BCM)
GPIO.setup(12, GPIO.OUT)

p = GPIO.PWM(12, 50)  # frequency=50Hz
p.start(0)




try:
    while 1:
        for i in range(0,10,1):
            p.ChangeDutyCycle(i)
            print(i)
            time.sleep(2)

except KeyboardInterrupt:
    pass
    p.stop()
    GPIO.cleanup()