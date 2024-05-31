#!/usr/bin/env python3

from commonbluerovmsg.srv import *
import rclpy
from rclpy.node import Node
from commonbluerovmsg.msg import LeakageDetection
# import RPi.GPIO as GPIO
import pigpio


class PWMSignalServer(Node):

    # GPIO allocation
    servoPin = 12
    LEDPin = 13
    LeakagePin = 7
    gpioPinServo = None
    gpioPinLight = None
    gpioPinLeakage = None
    angleDesired = 90
    angleCurrent = 90

    def __init__(self):
        super().__init__('pwm_signal_server')
        # print("test1")
        self.initGPIOPins()
        timer_period = 1
        # print("test2")
        self.timerLeakage = self.create_timer(timer_period, self.readLeakage)
        # print("test3")


        self.publisher_ = self.create_publisher(LeakageDetection, 'leakage_status_sensor_tube', 1)






# GPIO pin init
    def initGPIOPins(self):

        self.gpioPinLeakage = pigpio.pi()
        self.gpioPinLeakage.set_mode(self.LeakagePin, pigpio.INPUT)




    def readLeakage(self):
        output = self.gpioPinLeakage.read(self.LeakagePin)
        # print(output)
        if output:  # Physically read the pin now
            # print(colored(255, 0, 0, '################# MAYDAY LEAKAGE DETECTED WATER IN THE BOAT #################'))
            msg = LeakageDetection()
            msg.timestamp = self.get_clock().now().nanoseconds
            msg.leakage_detected = True
            self.publisher_.publish(msg)
        else:
            msg = LeakageDetection()
            msg.timestamp = self.get_clock().now().nanoseconds
            msg.leakage_detected = False
            self.publisher_.publish(msg)


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
