import tkinter as tk
import rospy
from bluerov2common.srv import lightDensity0to10, cameraAngle
import rosservice


class rosHandler:
    angleOfCamera = 90
    intensityOfLight = 0
    serviceLight = None
    serviceCameraAngle = None
    def handleButtonLightPlus(self):
        if (self.intensityOfLight >= 10):
            self.serviceLight(self.intensityOfLight)
        else:
            self.intensityOfLight = self.intensityOfLight + 1
            self.serviceLight(self.intensityOfLight)
        #print(self.intensityOfLight)

    def handleButtonLightMinus(self):
        if (self.intensityOfLight <= 0):
            self.serviceLight(self.intensityOfLight)
        else:
            self.intensityOfLight = self.intensityOfLight - 1
            self.serviceLight(self.intensityOfLight)
        #print(self.intensityOfLight)

    def handleButtonCameraMotorPlus(self):
        if (self.angleOfCamera >= 180):
            self.serviceCameraAngle(self.angleOfCamera)
        else:
            self.angleOfCamera = self.angleOfCamera + 10
            self.serviceCameraAngle(self.angleOfCamera)
        print(self.angleOfCamera)

    def handleButtonCameraMotorMinus(self):
        if (self.angleOfCamera <= 0):
            self.serviceCameraAngle(self.angleOfCamera)
        else:
            self.angleOfCamera = self.angleOfCamera - 10
            self.serviceCameraAngle(self.angleOfCamera)
        print(self.angleOfCamera)

    def init(self):
        rospy.init_node('controlRobotFromHub')
        self.serviceLight = rospy.ServiceProxy("set_light_of_leds_0_to_10", lightDensity0to10)
        self.serviceLight(self.intensityOfLight)
        self.serviceCameraAngle = rospy.ServiceProxy("set_angle_of_camera_0_to_180", cameraAngle)
        print(self.angleOfCamera)
        self.serviceCameraAngle(self.angleOfCamera)


####################################################     Layout of TK     ####################################################
rosClassHandler = rosHandler()
rosClassHandler.init()
print("test")
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
