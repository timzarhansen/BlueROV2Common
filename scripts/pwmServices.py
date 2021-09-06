#!/usr/bin/env python3

from bluerov2common.srv import *
import rospy
#import RPi.GPIO as GPIO
import pigpio
import time


class pwmClass:
    servoPin = 12
    LEDPin = 23
    localPiGPIO = None
    gpioPinServo = None
    gpioPinLight = None

    def initGPIOPins(self):
        #GPIO.setmode(GPIO.BCM)

        #GPIO.setup(self.LEDPin, GPIO.OUT)  # light Pin
        #self.gpioPinLight = GPIO.PWM(self.LEDPin, 50)  # frequency=50Hz
        #self.gpioPinLight.start(0)
        self.localPiGPIO = pigpio.pi()
        self.localPiGPIO.hardware_PWM(self.LEDPin,50,0)#10000
        #GPIO.setup(self.servoPin, GPIO.OUT)  # servo Pin
        #self.gpioPinServo = GPIO.PWM(self.servoPin, 50)  # frequency=50Hz
        #self.gpioPinServo.start(0)
        #self.localPiGPIO.hardware_PWM(self.servoPin,50,0)#10000

    def handleLight(self, req):
        # start correct lightning
        #self.gpioPinLight.ChangeDutyCycle(5.0 + req.intensity / 2.5)
        self.localPiGPIO.hardware_PWM(self.LEDPin,50,req.intensity*10000)#10000
        return lightDensity0to10Response(True)

    def handleAngleServo(self, req):
        duty = req.angle / 32 + 4
        #GPIO.output(self.servoPin, True)
        #self.gpioPinServo.ChangeDutyCycle(duty)

        time.sleep(1)

        #GPIO.output(self.servoPin, False)

        #self.gpioPinServo.ChangeDutyCycle(0)
        return cameraAngleResponse(True)

    def startLightLEDServer(self):
        s = rospy.Service('set_light_of_leds_0_to_10', lightDensity0to10, self.handleLight)

    def startServoCameraServer(self):
        s = rospy.Service('set_angle_of_camera_0_to_180', cameraAngle, self.handleAngleServo)

    def shutdownHook(self):
        self.gpioPinLight.stop()
        self.gpioPinServo.stop()
        GPIO.cleanup()


if __name__ == "__main__":
    myPwmClass = pwmClass()
    myPwmClass.initGPIOPins()
    rospy.init_node('pwmSignalServer')

    myPwmClass.startLightLEDServer()
    myPwmClass.startServoCameraServer()

    rospy.on_shutdown(myPwmClass.shutdownHook)
    rospy.spin()
