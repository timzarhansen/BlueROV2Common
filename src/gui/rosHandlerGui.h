//
// Created by tim-linux on 14.12.21.
//
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
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
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core.hpp>

#include <QtGamepad/QGamepad>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <thread>
#include "commonbluerovmsg/msg/desired_state_for_robot.hpp"

#include <commonbluerovmsg/srv/reset_ekf.hpp>
//#include <ping360_sonar/msg/sendingSonarConfig.h>
#include <commonbluerovmsg/srv/light_density.hpp>
#include <commonbluerovmsg/srv/camera_angle.hpp>
#include "waterlinked_a50/msg/transducer_report_stamped.hpp"

#ifndef BLUEROV2COMMON_ROSHANDLERGUI_H
#define BLUEROV2COMMON_ROSHANDLERGUI_H


class rosHandlerGui : public QObject , public rclcpp::Node{
Q_OBJECT
public:
    rosHandlerGui() : Node("guiControl") {
        rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

        this->subscriberPosRobot = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "publisherPoseEkf", qos, std::bind(&rosHandlerGui::positionCallback,
                                                   this, std::placeholders::_1));
        this->subscriberSonarImage = this->create_subscription<sensor_msgs::msg::Image>("sonar/image", qos, std::bind(
                &rosHandlerGui::sonarImageCallback, this, std::placeholders::_1));
//        subscriberCameraImage = this->create_subscription("cv_camera/image_raw",1000,&rosHandlerGui::cameraImageCallback,this);
        this->subscriberCameraImage = this->create_subscription<sensor_msgs::msg::CompressedImage>(
                "camera/image_raw/compressed", qos,
                std::bind(&rosHandlerGui::cameraImageCallback, this, std::placeholders::_1));
        this->subscriberDVL = this->create_subscription<waterlinked_a50::msg::TransducerReportStamped>(
                "dvl/transducer_report", qos, std::bind(&rosHandlerGui::DVLCallback, this, std::placeholders::_1));

//        this->publishingDesiredState =
//                this->create_publisher<commonbluerovmsg::msg::DesiredStateForRobot>("desiredStateOfBluerov2", qos);
        this->publishingDesiredState = this->create_publisher<commonbluerovmsg::msg::DesiredStateForRobot>(
                "desiredStateOfBluerov2", qos);




        this->clientEKF = this->create_client<commonbluerovmsg::srv::ResetEkf>("resetCurrentEKF");

//        clientSonar = n_.serviceClient<ping360_sonar::srv::sendingSonarConfig>("changeParametersSonar");
        this->clientLight = this->create_client<commonbluerovmsg::srv::LightDensity>("set_light_of_leds_0_to_10");
        this->clientCameraAngle = this->create_client<commonbluerovmsg::srv::CameraAngle>(
                "set_angle_of_camera_0_to_180");
    }
    //double xPositionRobot,yPositionRobot;
public slots:

    void updateDesiredState(double desiredHeight, double desiredRoll, double desiredPitch, double desiredYaw,
                            double desiredXMovement, double desiredYMovement, bool holdPosition);

//    void resetEKFEstimator(bool resetOnlyGraph);

//    void updateConfigSonar(int stepSize, int rangeSonar, int frequencyRange, int numberOfSamples);

    void updateAngleCamera(int angleCamera);

    void updateLightIntensity(int intensityLight);

public:
signals:

    void updatePlotPositionVectorROS(std::vector<double> xPositionRobot, std::vector<double> yPositionRobot,
                                     std::vector<double> yawPositionRobot);

    void updateSonarImageROS(QPixmap sonarImage);

    void updateCameraImageROS(QPixmap cameraImage);

    void updateStateOfRobotROS(double xPos, double yPos, double zPos, double roll, double pitch, double yaw,
                               Eigen::MatrixXd covariance);//covariance is just 6 values
    void updateDVLStateROS(double distance1, double distance2, double distance3, double distance4);

private:
    double angleOfCamera, intensityOfLight, currentDepth, distanceToBottom;
//        std::vector<double> xPositionRobot,yPositionRobot,yawPositionRobot;
    std::vector<double> xPositionRobot, yPositionRobot, yawPositionRobot;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscriberPosRobot;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriberSonarImage;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscriberCameraImage;
    rclcpp::Subscription<waterlinked_a50::msg::TransducerReportStamped>::SharedPtr subscriberDVL;

    //std::atomic<double> desiredHeight, desiredRoll,desiredPitch, desiredYaw, desiredXMovement, desiredYMovement;
    rclcpp::Publisher<commonbluerovmsg::msg::DesiredStateForRobot>::SharedPtr publishingDesiredState;



    rclcpp::Client<commonbluerovmsg::srv::ResetEkf>::SharedPtr clientEKF;
//    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr clientSonar;
    rclcpp::Client<commonbluerovmsg::srv::LightDensity>::SharedPtr clientLight;
    rclcpp::Client<commonbluerovmsg::srv::CameraAngle>::SharedPtr clientCameraAngle;

//    dynamic_reconfigure::Client<ping360_sonar::sonarConfig> tmpClient;

    void positionCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    void sonarImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    void cameraImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

    void DVLCallback(const waterlinked_a50::msg::TransducerReportStamped::SharedPtr msg);

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
