#!/usr/bin/env python3

from time import sleep           # Allows us to call the sleep function to slow down our loop test
from commonbluerovmsg.msg import leakageDetection
import RPi.GPIO as GPIO           # Allows us to call our GPIO pins and names it just GPIO
import rospy


def colored(r, g, b, text):
    return "\033[38;2;{};{};{}m{} \033[38;2;255;255;255m".format(r, g, b, text)

def main():

    rospy.init_node('drive_scenario_01')
    rate = rospy.Rate(1)
    publisher_leakageStatus = rospy.Publisher('leakageStatus', leakageDetection, queue_size=1)


    try:
        GPIO.setmode(GPIO.BCM)           # Set's GPIO pins to BCM GPIO numbering
        INPUT_PIN = 15           # Sets our input pin, in this example I'm connecting our button to pin 4. Pin 0 is the SDA pin so I avoid using it for sensors/buttons
        GPIO.setup(INPUT_PIN, GPIO.IN)
    except:
        print("was not able to initiallize GPIO Input")
        exit(-1)

    while not rospy.is_shutdown():
        if (GPIO.input(INPUT_PIN) == True): # Physically read the pin now
            print(colored(255, 0, 0, '################# LEAKAGE DETECTED WATER IN THE BOAT #################'))
            msg = leakageDetection()
            msg.header.stamp = rospy.Time.now()
            msg.leakageDetected = True
            publisher_leakageStatus.publish(msg)
        else:
            msg = leakageDetection()
            msg.header.stamp = rospy.Time.now()
            msg.leakageDetected = False
            publisher_leakageStatus.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    main()