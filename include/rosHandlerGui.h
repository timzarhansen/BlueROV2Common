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
#ifndef BLUEROV2COMMON_ROSHANDLERGUI_H
#define BLUEROV2COMMON_ROSHANDLERGUI_H


class rosHandlerGui: public QObject {
    Q_OBJECT
public:
    rosHandlerGui(ros::NodeHandle n_) {
        subscriberPosRobot = n_.subscribe("publisherPoseEkf",1000,&rosHandlerGui::positionCallback,this);
        subscriberSonarImage = n_.subscribe("ping360_node/sonar/images",1000,&rosHandlerGui::sonarImageCallback,this);
        subscriberCameraImage = n_.subscribe("cv_camera/image_raw",1000,&rosHandlerGui::cameraImageCallback,this);

    }
    //double xPositionRobot,yPositionRobot;
public slots:
    void setSonarRange(double range);
public:
    signals:
        void updatePositionsROS(QVector<double> xPositionRobot, QVector<double> yPositionRobot, QVector<double> yawPositionRobot);
        void updateSonarImageROS(QPixmap sonarImage);
        void updateCameraImageROS(QPixmap cameraImage);

private:
    double angleOfCamera ,intensityOfLight,currentDepth,distanceToBottom;
//        std::vector<double> xPositionRobot,yPositionRobot,yawPositionRobot;
    QVector<double> xPositionRobot, yPositionRobot,yawPositionRobot;
    ros::Subscriber subscriberPosRobot,subscriberSonarImage,subscriberCameraImage;

    void positionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void sonarImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void cameraImageCallback(const sensor_msgs::ImageConstPtr& msg);

};


#endif //BLUEROV2COMMON_ROSHANDLERGUI_H
