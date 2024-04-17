#!/usr/bin/env python3
import sys
from commonbluerovmsg.msg import LeakageDetection
import RPi.GPIO as GPIO           # Allows us to call our GPIO pins and names it just GPIO
import rclpy
from rclpy.node import Node

INPUT_PIN = 7
def colored(r, g, b, text):
    return "\033[38;2;{};{};{}m{} \033[38;2;255;255;255m".format(r, g, b, text)


class LeakPublisher(Node):

    def __init__(self):

        super().__init__('leakage_status')
        self.publisher_ = self.create_publisher(LeakageDetection, 'leakage_status', 1)
    # executes callback every timer_period seconds (change to appropriate value)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.leak_callback)
        self.count = 0

    def leak_callback(self):
        if GPIO.input(INPUT_PIN):  # Physically read the pin now
            print(colored(255, 0, 0, '################# MAYDAY LEAKAGE DETECTED WATER IN THE BOAT #################'))
            msg = LeakageDetection()
            msg.timestamp = self.get_clock().now().nanoseconds
            msg.leakage_detected = True
            self.publisher_.publish(msg)
            self.count += 1
        else:
            msg = LeakageDetection()
            msg.timestamp = self.get_clock().now().nanoseconds
            msg.leakage_detected = False
            self.publisher_.publish(msg)
            self.count += 1


def main(args=None):
    rclpy.init(args=args)

    leak_publisher = LeakPublisher()

    # Init GPIO pins -> not affected by ROS2 migration
    try:
        GPIO.setmode(GPIO.BCM)  # Set's GPIO pins to BCM GPIO numbering
        # INPUT_PIN = 7  # Sets our input pin, in this example I'm connecting our button to pin 4. Pin 0 is the SDA pin, so I avoid using it for sensors/buttons
        GPIO.setup(INPUT_PIN, GPIO.IN)
    except:
        print("Was not able to initialize GPIO Input")
        exit(-1)
    # print("test3")
    rclpy.spin(leak_publisher)
    # leak_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

