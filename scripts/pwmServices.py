#!/usr/bin/env python3
import sys

from commonbluerovmsg.srv import *
import rclpy
from rclpy.node import Node
# import RPi.GPIO as GPIO
import pigpio


class PWMSignalServer(Node):

    # GPIO allocation
    servoPin = 12
    LEDPin = 13
    gpioPinServo = None
    gpioPinLight = None
    angleDesired = 90
    angleCurrent = 90

    def __init__(self):
        super().__init__('pwm_signal_server')
        self.startLightLEDServer()
        self.startServoCameraServer()

    # GPIO pin init
    def initGPIOPins(self):

        self.gpioPinLight = pigpio.pi()
        self.gpioPinLight.set_mode(self.LEDPin, pigpio.OUTPUT)
        self.gpioPinLight.hardware_PWM(self.LEDPin, 50, 0)  # 10000
        self.gpioPinServo = pigpio.pi()
        self.gpioPinServo.set_mode(self.servoPin, pigpio.OUTPUT)
        self.gpioPinServo.hardware_PWM(self.servoPin, 50, 0)  # 10000

    def handleLight(self, req):
        # start correct lightning
        self.gpioPinLight.hardware_PWM(self.LEDPin, 50, req.intensity * 10000)  # 10000
        res = True
        return res

    def handleAngleServo(self, req):
        self.angleDesired = req.angle
        res = True
        return res

    def controllerAngleServo(self):
        if self.angleDesired == self.angleCurrent:
            return
        tmpAngleDes = 0
        if abs(self.angleDesired - self.angleCurrent) > 1:
            if self.angleDesired - self.angleCurrent > 0:
                tmpAngleDes = self.angleCurrent + 2
            else:
                tmpAngleDes = self.angleCurrent - 2
        else:
            tmpAngleDes = self.angleDesired
        duty = ((tmpAngleDes / 180) * 108 + 71) / 180 * 10
        self.gpioPinLight.hardware_PWM(self.servoPin, 50, int(duty * 10000))  # 10000
        self.angleCurrent = tmpAngleDes

    def startLightLEDServer(self):
        srv = self.create_service(LightDensity, 'light_service', self.handleLight)

    def startServoCameraServer(self):
        srv = self.create_service(CameraAngle, 'camera_angle_service', self.handleAngleServo)

    def shutdownHook(self):
         #self.gpioPinLight.stop()
         print("shutdown")
         #self.gpioPinServo.stop()
         #GPIO.cleanup()


#
# class PwmClass:
#     servoPin = 12
#     LEDPin = 13
#     # localPiGPIO = None
#     gpioPinServo = None
#     gpioPinLight = None
#     angleDesired = 90
#     angleCurrent = 90
#     def initGPIOPins(self):
#         # GPIO.setmode(GPIO.BCM)
#
#         # GPIO.setup(slightDensity0to10Responseelf.LEDPin, GPIO.OUT)  # light Pin
#         # self.gpioPinLight = GPIO.PWM(self.LEDPin, 50)  # frequency=50Hz
#         # self.gpioPinLight.start(0)
#         self.gpioPinLight = pigpio.pi()
#         self.gpioPinLight.set_mode(self.LEDPin,pigpio.OUTPUT)
#         self.gpioPinLight.hardware_PWM(self.LEDPin,50,0)#10000
#
#         self.gpioPinServo = pigpio.pi()
#         self.gpioPinServo.set_mode(self.servoPin,pigpio.OUTPUT)
#         self.gpioPinServo.hardware_PWM(self.servoPin,50,0)#10000
#
#
#     def handleLight(self, req):
#         # start correct lightning
#         # self.gpioPinLight.ChangeDutyCycle(5.0 + req.intensity / 2.5)
#         self.gpioPinLight.hardware_PWM(self.LEDPin,50,req.intensity*10000)#10000
#         return LightDensityResponse(True)
#
#
#     def handleAngleServo(self, req):
#
#         self.angleDesired = req.angle
#         return CameraAngleResponse(True)
#
#
#     def controllerAngleServo(self):
#         if self.angleDesired == self.angleCurrent:
#             return
#         tmpAngleDes = 0
#         if abs(self.angleDesired-self.angleCurrent)>1:
#             if self.angleDesired-self.angleCurrent >0:
#                 tmpAngleDes = self.angleCurrent+2
#             else:
#                 tmpAngleDes = self.angleCurrent-2
#         else:
#             tmpAngleDes = self.angleDesired
#         duty = ((tmpAngleDes / 180)*108 +71)/180*10
#         self.gpioPinLight.hardware_PWM(self.servoPin,50,int(duty*10000))#10000
#         self.angleCurrent = tmpAngleDes
#
#
#     def startLightLEDServer(self):
#         s = rospy.Service('set_light_of_leds_0_to_10', lightDensity0to10, self.handleLight)
#
#
#     def startServoCameraServer(self):
#         s = rospy.Service('set_angle_of_camera_0_to_180', cameraAngle, self.handleAngleServo)
#
#
#     def shutdownHook(self):
#         #self.gpioPinLight.stop()
#         print("shutdown")
#         #self.gpioPinServo.stop()
#         #GPIO.cleanup()


if __name__ == "__main__":

    rclpy.init(args=sys.argv)
    try:
        pwm_server = PWMSignalServer()
        pwm_server.initGPIOPins()
        rate = pwm_server.create_rate(2)

        # Not sure on shutdown hook in ROS2, found this: https://github.com/mikeferguson/ros2_cookbook/blob/main/rclpy/nodes.md

        while rclpy.ok():
            pwm_server.controllerAngleServo()
            rate.sleep()


        # myPwmClass = PwmClass()
        # myPwmClass.initGPIOPins()
        # rospy.init_node('pwmSignalServer')
        #
        # myPwmClass.startLightLEDServer()
        # myPwmClass.startServoCameraServer()
        #
        # rospy.on_shutdown(myPwmClass.shutdownHook)
        # rate = rospy.Rate(2)
        # while not rospy.is_shutdown():
        #
        #     myPwmClass.controllerAngleServo()
        #     rate.sleep()
        #     #rospy.spin()
    except:
        pass
