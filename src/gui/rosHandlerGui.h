//
// Created by tim-linux on 14.12.21.
//
#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "vector"
#include <QApplication>
#include <QPushButton>
#include <QMainWindow>
#include <iostream>
#include "QLabel"
#include "QSlider"
#include "QScreen"
#include "qcustomplot.h"
#include <QPixmap>
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <QtGamepad/QGamepad>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <thread>
#include <commonbluerovmsg/desiredStateForRobot.h>
#include <commonbluerovmsg/resetekf.h>
#include <ping360_sonar/sendingSonarConfig.h>
#include <commonbluerovmsg/lightDensity0to10.h>
#include <commonbluerovmsg/cameraAngle.h>


#ifndef BLUEROV2COMMON_ROSHANDLERGUI_H
#define BLUEROV2COMMON_ROSHANDLERGUI_H


class rosHandlerGui: public QObject {
    Q_OBJECT
public:
    rosHandlerGui(ros::NodeHandle n_)  {
        subscriberPosRobot = n_.subscribe("publisherPoseEkf",10,&rosHandlerGui::positionCallback,this);
        subscriberSonarImage = n_.subscribe("sonar/image",10,&rosHandlerGui::sonarImageCallback,this);
//        subscriberCameraImage = n_.subscribe("cv_camera/image_raw",1000,&rosHandlerGui::cameraImageCallback,this);
        subscriberCameraImage = n_.subscribe("camera/image_raw/compressed",10,&rosHandlerGui::cameraImageCallback,this);
        publishingDesiredState = n_.advertise<commonbluerovmsg::desiredStateForRobot>("desiredStateOfBluerov2", 10);
        clientEKF = n_.serviceClient<commonbluerovmsg::resetekf>("resetCurrentEKF");
        clientSonar = n_.serviceClient<ping360_sonar::sendingSonarConfig>("changeParametersSonar");
        clientLight = n_.serviceClient<commonbluerovmsg::lightDensity0to10>("set_light_of_leds_0_to_10");
        clientCameraAngle = n_.serviceClient<commonbluerovmsg::cameraAngle>("set_angle_of_camera_0_to_180");
    }
    //double xPositionRobot,yPositionRobot;
public slots:

    void updateDesiredState(double desiredHeight, double desiredRoll,double desiredPitch, double desiredYaw, double desiredXMovement, double desiredYMovement,bool holdPosition);
    void resetEKFEstimator(bool resetOnlyGraph);
    void updateConfigSonar(int stepSize, int rangeSonar);
    void updateAngleCamera(int angleCamera);
    void updateLightIntensity(int intensityLight);

public:
    signals:
        void updatePlotPositionVectorROS(std::vector<double> xPositionRobot, std::vector<double> yPositionRobot, std::vector<double> yawPositionRobot);
        void updateSonarImageROS(QPixmap sonarImage);
        void updateCameraImageROS(QPixmap cameraImage);
        void updateStateOfRobotROS(double xPos, double yPos, double zPos, double roll, double pitch, double yaw, Eigen::MatrixXd covariance);//covariance is just 6 values

private:
    double angleOfCamera ,intensityOfLight,currentDepth,distanceToBottom;
//        std::vector<double> xPositionRobot,yPositionRobot,yawPositionRobot;
    std::vector<double> xPositionRobot, yPositionRobot,yawPositionRobot;
    ros::Subscriber subscriberPosRobot,subscriberSonarImage,subscriberCameraImage;
    //std::atomic<double> desiredHeight, desiredRoll,desiredPitch, desiredYaw, desiredXMovement, desiredYMovement;
    ros::Publisher publishingDesiredState;
    ros::ServiceClient clientEKF,clientSonar,clientLight,clientCameraAngle;
//    dynamic_reconfigure::Client<ping360_sonar::sonarConfig> tmpClient;

    void positionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void sonarImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void cameraImageCallback(const sensor_msgs::CompressedImagePtr & msg);

public:
    Eigen::Vector3d getRollPitchYaw(Eigen::Quaterniond quat) {
    tf2::Quaternion tmp(quat.x(), quat.y(), quat.z(), quat.w());
    tf2::Matrix3x3 m(tmp);
    double r, p, y;
    m.getRPY(r, p, y);
    Eigen::Vector3d returnVector(r, p, y);
    return returnVector;
    }
};


#endif //BLUEROV2COMMON_ROSHANDLERGUI_H
