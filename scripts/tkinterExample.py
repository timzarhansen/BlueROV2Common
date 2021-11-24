import tkinter as tk
import rospy
from mavros_msgs.msg import Altitude
from waterlinked_dvl.msg import TransducerReportStamped,PositionReportStamped
from time import sleep
from bluerov2common.srv import lightDensity0to10, cameraAngle
import rosservice
import matplotlib

matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
import numpy as np


class rosHandler:
    angleOfCamera = 90
    intensityOfLight = 0
    serviceLight = None
    serviceCameraAngle = None
    currentDepth = None
    distanceToBottom = None
    xPositionRobot = list()
    yPositionRobot = list()
    def handleButtonLightPlus(self):
        if (self.intensityOfLight >= 10):
            try:
                self.serviceLight(self.intensityOfLight)
            except:
                print("Service not Available")
        else:
            self.intensityOfLight = self.intensityOfLight + 1
            try:
                self.serviceLight(self.intensityOfLight)
            except:
                print("Service not Available")
        # print(self.intensityOfLight)

    def handleButtonLightMinus(self):
        if (self.intensityOfLight <= 0):
            try:
                self.serviceLight(self.intensityOfLight)
            except:
                print("Service not Available")
        else:
            self.intensityOfLight = self.intensityOfLight - 1
            try:
                self.serviceLight(self.intensityOfLight)
            except:
                print("Service not Available")
        # print(self.intensityOfLight)

    def handleButtonCameraMotorPlus(self):
        if (self.angleOfCamera >= 180):
            try:
                self.serviceCameraAngle(self.angleOfCamera)
            except:
                print("Service not Available")
        else:
            self.angleOfCamera = self.angleOfCamera + 10
            try:
                self.serviceCameraAngle(self.angleOfCamera)
            except:
                print("Service not Available")
        # print(self.angleOfCamera)

    def handleButtonCameraMotorMinus(self):
        if (self.angleOfCamera <= 0):
            try:
                self.serviceCameraAngle(self.angleOfCamera)
            except:
                print("Service not Available")
        else:
            self.angleOfCamera = self.angleOfCamera - 10
            try:
                self.serviceCameraAngle(self.angleOfCamera)
            except:
                print("Service not Available")
        # print(self.angleOfCamera)


    def altitudeCallback(self,data: Altitude):
        self.currentDepth=-data.local

    def distanceToBottomCallback(self,data: TransducerReportStamped):
        self.distanceToBottom=data.report.altitude

    def xyPositionRobotCallback(self,data: PositionReportStamped):
        self.xPositionRobot.append(data.report.x)
        self.yPositionRobot.append(data.report.y)

    def init(self):
        rospy.init_node('controlRobotFromHub')
        try:
            self.serviceLight = rospy.ServiceProxy("set_light_of_leds_0_to_10", lightDensity0to10)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
        try:
            self.serviceLight(self.intensityOfLight)
        except:
            print("Service not Available")
        try:
            self.serviceCameraAngle = rospy.ServiceProxy("set_angle_of_camera_0_to_180", cameraAngle)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
        print(self.angleOfCamera)
        try:
            self.serviceCameraAngle(self.angleOfCamera)
        except:
            print("Service not Available")
        rospy.Subscriber("mavros/altitude", Altitude, self.altitudeCallback)
        rospy.Subscriber("transducer_report", TransducerReportStamped, self.distanceToBottomCallback)
        rospy.Subscriber("position_report", PositionReportStamped, self.xyPositionRobotCallback)



####################################################     Layout of TK     ####################################################
root = tk.Tk()
root.title("Control Of BlueROV2")
root.geometry("2000x900")

rosClassHandler = rosHandler()
rosClassHandler.init()

########## altitude and Depth ##########
currentDepthVar = tk.DoubleVar()
currentDepthVar.set(0)
distanceToBottomVar = tk.DoubleVar()
distanceToBottomVar.set(0)
altitudeNumber = tk.Label(root, textvariable = currentDepthVar)
altitudeNumber.place(x=180, y=250)
altitudeText = tk.Label(root, text="Altitude:")
altitudeText.place(x=10, y=250)

depthToBottomText = tk.Label(root, text="Distance to Bottom:")
depthToBottomText.place(x=10, y=350)
distanceToBottomNumber = tk.Label(root, textvariable = distanceToBottomVar)
distanceToBottomNumber.place(x=180, y=350)

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

##########  Robot Position with Graph ##########
f = Figure(figsize=(5, 5), dpi=100)
a = f.add_subplot(111)
arrayXPos = np.asarray([1, 2, 3, 4, 5, 6, 7, 8])
arrayYPos = np.asarray([5, 6, 1, 3, 8, 9, 3, 5])
a.plot(arrayXPos, arrayYPos, '.')

f.suptitle("Robot Position")
canvas = FigureCanvasTkAgg(f, master=root)
canvas.get_tk_widget().pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)
canvas._tkcanvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
canvas._tkcanvas.place(x=500, y=200)

resetButtonPositionEstimation = tk.Button(root, text='+', width=5, height=1, command=rosClassHandler.handleButtonCameraMotorPlus,
                            bg='red')
resetButtonPositionEstimation.place(x=500, y=50)

while not rospy.is_shutdown():
    a.clear()
    theta = np.random.uniform(0, 360, 10)
    r = np.random.uniform(0, 1, 10)


    a.plot(rosClassHandler.xPositionRobot, rosClassHandler.yPositionRobot, linestyle="None", marker='.',markersize=2)
    a.set_xlabel('x Position in m')
    a.set_ylabel('y Position in m')
    a.axis('equal')
    canvas.draw()
    currentDepthVar.set(round(rosClassHandler.currentDepth, 2))
    if not rosClassHandler.distanceToBottom == -1:
        distanceToBottomVar.set(round(rosClassHandler.distanceToBottom, 2))
    root.update_idletasks()
    root.update()
    sleep(0.1)  # Need this to slow the changes down
