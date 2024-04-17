#!/usr/bin/env python3
import sys
import commonbluerovmsg.srv
from commonbluerovmsg.srv import *
# import rospy
# import RPi.GPIO as GPIO
import pigpio
import time
import rclpy
from rclpy.node import Node


class pwmClass(Node):
    resetPin = 18
    # localPiGPIO = None
    gpioPinSonar = None

    def __init__(self):
        super().__init__("power_conrol_bottom_tube")
        # GPIO.setmode(GPIO.BCM)

        # GPIO.setup(self.LEDPin, GPIO.OUT)  # light Pin
        # self.gpioPinLight = GPIO.PWM(self.LEDPin, 50)  # frequency=50Hz
        # self.gpioPinLight.start(0)
        self.gpioPinSonar = pigpio.pi()
        self.gpioPinSonar.set_mode(self.resetPin, pigpio.OUTPUT)
        self.gpioPinSonar.write(self.resetPin, 0)
        # self.gpioPinSonar.hardware_PWM(self.sonarPin, 50, 0)#10000
        s = self.create_service(commonbluerovmsg.srv.RestartSonarService, 'power_control_bottom_service', self.handleSonar)

    def handleSonar(self, req, res):
        # start restart Sonar
        if req.tube_state == 1:
            self.gpioPinSonar.write(self.resetPin, 1)
        else:
            self.gpioPinSonar.write(self.resetPin, 0)

        # res = commonbluerovmsg.srv.RestartSonarService.Response
        res.saved = True
        return res
    # def shutdownHook(self):
    #     #self.gpioPinLight.stop()
    #     print("shutdown")
    #     #self.gpioPinServo.stop()
    #     #GPIO.cleanup()


if __name__ == "__main__":

    rclpy.init(args=sys.argv)

    try:
        myPwmClass = pwmClass()
        # myPwmClass.initGPIOPins()
        rclpy.spin(myPwmClass)
        # myPwmClass.resetSonarSensorServer()
        rclpy.shutdown()
        # rospy.on_shutdown(myPwmClass.shutdownHook)
        #
        # rospy.spin()
    except:
        pass
