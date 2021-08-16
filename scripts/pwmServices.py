from bluerov2common.srv import *
import rospy
import RPi.GPIO as GPIO
import time
servoPin = 12
LEDPin = 23

def initGPIOPins():
    GPIO.setmode(GPIO.BCM)


    GPIO.setup(LEDPin, GPIO.OUT)  # light Pin
    gpioPinLight = GPIO.PWM(LEDPin, 50)  # frequency=50Hz
    gpioPinLight.start(0)

    GPIO.setup(servoPin, GPIO.OUT)  # servo Pin
    gpioPinServo = GPIO.PWM(servoPin, 50)  # frequency=50Hz
    gpioPinServo.start(0)

    return [gpioPinLight, gpioPinServo]


def handleLight(req, gpioPin):
    # start correct lightning
    gpioPin.ChangeDutyCycle(5.0 + req.intensity / 2.5)
    return lightDensity0to10Response(True)

def handleAngleServo(req,gpioPin):

    duty = req.angle / 32 + 4
    GPIO.output(servoPin, True)
    gpioPin.ChangeDutyCycle(duty)

    time.sleep(1)

    GPIO.output(servoPin, False)

    gpioPin.ChangeDutyCycle(0)
    return cameraAngleResponse(True)

def startLightLEDServer(gpioPin):
    s = rospy.Service('set_light_of_leds_0_to_10', lightDensity0to10, lambda msg: handleLight(msg, gpioPin))

def startServoCameraServer(gpioPin):
    s = rospy.Service('set_angle_of_camera_0_to_180', cameraAngle, lambda msg: handleAngleServo(msg, gpioPin))

def shutdownHook(gpioPinLight,gpioPinServo):
    gpioPinLight.stop()
    gpioPinServo.stop()
    GPIO.cleanup()



if __name__ == "__main__":

    [gpioPinLight, gpioPinServo] = initGPIOPins()
    rospy.init_node('pwmSignalServer')

    startLightLEDServer(gpioPinLight)
    startServoCameraServer(gpioPinServo)



    rospy.on_shutdown(lambda msg: shutdownHook(gpioPinLight,gpioPinServo))
    rospy.spin()

