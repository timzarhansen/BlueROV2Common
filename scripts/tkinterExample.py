import tkinter as tk
import rospy
from bluerov2common.srv import lightDensity0to10, cameraAngle
import rosservice

class rosHandler:
    angleOfCamera = 90
    intensityOfLight = 0

    def handleButtonLightPlus(self):
        self.intensityOfLight = self.intensityOfLight+1
        rosservice.call_service("set_light_of_leds_0_to_10",self.intensityOfLight)
        print("+")

    def handleButtonLightMinus(self):
        self.intensityOfLight = self.intensityOfLight-1
        rosservice.call_service("set_light_of_leds_0_to_10",self.intensityOfLight)
        print("-")

    def handleButtonCameraMotorPlus(self):
        print("+")

    def handleButtonCameraMotorMinus(self):
        print("-")

    def init(self):
        rospy.init_node('Control_from_main_hub')


####################################################     Layout of TK     ####################################################
rosClassHandler = rosHandler()
rosClassHandler.init()

root = tk.Tk()
root.title("Control Of BlueROV2")
root.geometry("300x300")

########## light ##########
lightLabel = tk.Label(root, text="Change Light")
lightLabel.place(x=10, y=50)

lightButtonPlus = tk.Button(root, text='+', width=5, height=1, command=rosClassHandler.handleButtonLightPlus, bg='red')
lightButtonPlus.place(x=13, y=80)

lightButtonMinus = tk.Button(root, text='-', width=5, height=1, command=rosClassHandler.handleButtonLightMinus,
                             bg='green')
lightButtonMinus.place(x=13, y=110)

########## Motor ##########
cameraMotorLabel = tk.Label(root, text="Adjust Angle Camera")
cameraMotorLabel.place(x=150, y=50)

angleButtonPlus = tk.Button(root, text='+', width=5, height=1, command=rosClassHandler.handleButtonCameraMotorPlus,
                            bg='red')
angleButtonPlus.place(x=180, y=80)

angleButtonMinus = tk.Button(root, text='-', width=5, height=1, command=rosClassHandler.handleButtonCameraMotorMinus,
                             bg='green')
angleButtonMinus.place(x=180, y=110)

root.mainloop()
