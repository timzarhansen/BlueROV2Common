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
#include <bluerov2common/desiredStateForRobot.h>
#include <underwaterslam/resetekf.h>

#ifndef BLUEROV2COMMON_ROSHANDLERGUI_H
#define BLUEROV2COMMON_ROSHANDLERGUI_H


class rosHandlerGui: public QObject {
    Q_OBJECT
public:
    rosHandlerGui(ros::NodeHandle n_) {
        subscriberPosRobot = n_.subscribe("publisherPoseEkf",1000,&rosHandlerGui::positionCallback,this);
        subscriberSonarImage = n_.subscribe("sonar/image",1000,&rosHandlerGui::sonarImageCallback,this);
        subscriberCameraImage = n_.subscribe("cv_camera/image_raw",1000,&rosHandlerGui::cameraImageCallback,this);
        publishingDesiredState = n_.advertise<bluerov2common::desiredStateForRobot>("desiredStateOfBluerov2", 10);
        clientEKF = n_.serviceClient<underwaterslam::resetekf>("resetCurrentEKF");
    }
    //double xPositionRobot,yPositionRobot;
public slots:
    void setSonarRange(double range);
    void updateDesiredState(double desiredHeight, double desiredRoll,double desiredPitch, double desiredYaw, double desiredXMovement, double desiredYMovement,bool holdPosition);
    void resetEKFEstimator(bool resetOnlyGraph);
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
    ros::ServiceClient clientEKF;


    void positionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void sonarImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void cameraImageCallback(const sensor_msgs::ImageConstPtr& msg);
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
