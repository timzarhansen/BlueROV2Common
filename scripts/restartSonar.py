#!/usr/bin/env python3

from commonbluerovmsg.srv import *
import rospy
#import RPi.GPIO as GPIO
import pigpio
import time


class pwmClass:
    sonarPin = 13
    #localPiGPIO = None
    gpioPinSonar = None

    def initGPIOPins(self):
        #GPIO.setmode(GPIO.BCM)

        #GPIO.setup(self.LEDPin, GPIO.OUT)  # light Pin
        #self.gpioPinLight = GPIO.PWM(self.LEDPin, 50)  # frequency=50Hz
        #self.gpioPinLight.start(0)
        self.gpioPinSonar = pigpio.pi()
        self.gpioPinSonar.set_mode(self.sonarPin, pigpio.OUTPUT)
        #self.gpioPinSonar.hardware_PWM(self.sonarPin, 50, 0)#10000

    def handleSonar(self, req):
        # start restart Sonar

        self.gpioPinSonar.write(self.sonarPin,1)

        time.sleep(req.timeUntilRestart)
        self.gpioPinSonar.write(self.sonarPin,0)


        return restartSonarServiceResponse(True)

    def resetSonarSensorServer(self):
        s = rospy.Service('reset_sonar', restartSonarService, self.handleSonar)

    def shutdownHook(self):
        #self.gpioPinLight.stop()
        print("shutdown")
        #self.gpioPinServo.stop()
        #GPIO.cleanup()


if __name__ == "__main__":
    try:
        myPwmClass = pwmClass()
        myPwmClass.initGPIOPins()
        rospy.init_node('pwmSignalServer')

        myPwmClass.resetSonarSensorServer()

        rospy.on_shutdown(myPwmClass.shutdownHook)

        rospy.spin()
    except:
        pass
