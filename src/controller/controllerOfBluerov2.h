//
// Created by tim-linux on 22.12.21.
//
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include "commonbluerovmsg/msg/desired_state_for_robot.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include <visualization_msgs/msg/marker.hpp>
//#include "mavros_msgs/AttitudeTarget.h"
#include "px4_msgs/msg/vehicle_attitude_setpoint.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
//#include <bluerov2common/controllerConfig.h>
//#include <dynamic_reconfigure/server.h>

#ifndef BLUEROV2COMMON_CONTROLLEROFBLUEROV2_H
#define BLUEROV2COMMON_CONTROLLEROFBLUEROV2_H



class controllerOfBluerov2 : public rclcpp::Node {
public:
    controllerOfBluerov2() : Node("controllerBlueROV") {
        rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);


        this->subscriberDesiredState = this->create_subscription<commonbluerovmsg::msg::DesiredStateForRobot>(
                "desiredStateOfBluerov2", qos,
                std::bind(&controllerOfBluerov2::desiredStateCallback, this, std::placeholders::_1));
        this->subscriberCurrentPose = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "publisherPoseEkf", qos, std::bind(&controllerOfBluerov2::currentPoseCallback,
                this,std::placeholders::_1));
        this->subscriberCurrentTwist = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
                "publisherTwistEkf", qos, std::bind(&controllerOfBluerov2::currentTwistCallback,
                this,std::placeholders::_1));

        this->publisherPX4 = this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
                "/fmu/in/vehicle_attitude_setpoint", qos);

        this->publisherVisualization = this->create_publisher<visualization_msgs::msg::Marker>(
                "controllerThrustVisualization", qos);
        std::chrono::duration<double> my_timer_duration = std::chrono::duration<double>(1.0 / 30.0);
        this->timer_ = this->create_wall_timer(
                my_timer_duration, std::bind(&controllerOfBluerov2::timer_callback, this));

        this->integratorHeight = 0;
        this->integratorX = 0;
        this->integratorY = 0;

        this->desiredXThrustBody = 0;
        this->desiredDepth = 0;
        this->desiredYaw = 0;
        this->desiredXThrustBody = 0;
        this->desiredYThrustBody = 0;
        this->desiredRoll = 0;
        this->desiredPitch = 0;

        this->height_i = 0.4;
        this->height_d = 0.0;
        this->height_p = 0.9;

        this->hold_position_p = 1.0;
        this->hold_position_i = 0.1;
        this->hold_position_d = 4.0;


        //startControlLogic with outside Thread
    }

    Eigen::Vector3d controllLogic();

    static Eigen::Vector3d getRollPitchYaw(Eigen::Quaterniond quat);

    static Eigen::Quaterniond getQuaternionFromRPY(double roll, double pitch, double yaw);

    static Eigen::Quaterniond getQuaternionForMavrosFromRPY(double roll, double pitch, double yaw);

    Eigen::Vector3d getThrustForMavros(double thrust_1, double thrust_2, double thrust_3);

    double calculateDepthThrust(double desiredDepthTMP);

    void setControllerValues(double height_i_tmp, double height_d_tmp, double height_p_tmp, double hold_position_p_tmp,
                             double hold_position_i_tmp, double hold_position_d_tmp);

//    void callbackReconfiguration(bluerov2common::controllerConfig &config, uint32_t level);

    void getPoseRobot(Eigen::Vector3d &position, Eigen::Quaterniond &rotation);

    void getPoseTarget(Eigen::Vector3d &position, Eigen::Quaterniond &rotation);

    void timer_callback();

private:
    rclcpp::Subscription<commonbluerovmsg::msg::DesiredStateForRobot>::SharedPtr subscriberDesiredState;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscriberCurrentPose;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr subscriberCurrentTwist;

    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr publisherPX4;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisherVisualization;


    rclcpp::TimerBase::SharedPtr timer_;

    //Pose
    std::atomic<double> currentXPosition, currentYPosition, currentDepth, currentRoll, currentPitch, currentYaw;
    //Twist
    std::atomic<double> currentXVel, currentYVel, currentDepthVel, currentRollVel, currentPitchVel, currentYawVel;
    //Desired
    std::atomic<double> desiredXThrustBody, desiredYThrustBody, desiredDepth, desiredRoll, desiredPitch, desiredYaw;//

    std::atomic<bool> holdPosition;

    //StateToHold:
    std::atomic<double> holdXPosition, holdYPosition, holdDepth, holdRoll, holdPitch, holdYaw;
    //for Control:
    std::atomic<double> integratorHeight, integratorX, integratorY;
    // control parameter
    std::atomic<double> height_i, height_d, height_p, hold_position_p, hold_position_i, hold_position_d;


    //functions
    void desiredStateCallback(const commonbluerovmsg::msg::DesiredStateForRobot::SharedPtr msg);

    void currentTwistCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);

    void currentPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);


};


#endif //BLUEROV2COMMON_CONTROLLEROFBLUEROV2_H
