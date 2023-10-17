#!/usr/bin/env python3

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
        # print("test1")
        self.initGPIOPins()
        timer_period = 2
        # print("test2")
        self.timer = self.create_timer(timer_period, self.controllerAngleServo)
        # print("test3")
        self.srvLight = self.create_service(LightDensity, 'light_service', self.handleLight)
        # print("test4")
        self.srvCamera = self.create_service(CameraAngle, 'camera_angle_service', self.handleAngleServo)
        # print("test5")




# GPIO pin init
    def initGPIOPins(self):

        self.gpioPinLight = pigpio.pi()
        self.gpioPinLight.set_mode(self.LEDPin, pigpio.OUTPUT)
        self.gpioPinLight.hardware_PWM(self.LEDPin, 50, 0)  # 10000
        self.gpioPinServo = pigpio.pi()
        self.gpioPinServo.set_mode(self.servoPin, pigpio.OUTPUT)
        self.gpioPinServo.hardware_PWM(self.servoPin, 50, 0)  # 10000

    def handleLight(self, request, response):
        # start correct lightning
        print("starting handle light")
        self.gpioPinLight.hardware_PWM(self.LEDPin, 50, request.intensity * 10000)  # 10000
        print("done")
        response.worked = True
        return response

    def handleAngleServo(self, req, res):
        print("starting handle servo")
        self.angleDesired = req.angle
        print("done")
        res.worked = True
        return res

    def controllerAngleServo(self):
        # print("control loop")
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

    # def shutdownHook(self):
    #      #self.gpioPinLight.stop()
    #      print("shutdown")
    #      #self.gpioPinServo.stop()
    #      #GPIO.cleanup()








def main(args=None):
    rclpy.init(args=args)
    # try:
    pwm_server = PWMSignalServer()
    # pwm_server.initGPIOPins()
    rclpy.spin(pwm_server)
    # leak_publisher.destroy_node()
    rclpy.shutdown()
#
    # except:
    #     pass


if __name__ == '__main__':
    main()
