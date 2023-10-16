//
// Created by jurobotics on 13.09.21.

#include "ekfDVL.h"
#include "rclcpp/rclcpp.hpp"

// just for tricking compiler
//#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "waterlinked_a50/msg/transducer_report_stamped.hpp"
#include "waterlinked_a50/msg/position_report_stamped.hpp"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

//#include <geometry_msgs/msg/PoseWithCovarianceStamped.hpp>
#include "sensor_msgs/msg/imu.hpp"
//#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
//#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/msg/twist_stamped.hpp"
//#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/msg/vector3_stamped.hpp"
//#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "px4_msgs/msg/sensor_baro.hpp"
#include "px4_msgs/msg/vehicle_air_data.hpp"

//#include "../slamTools/generalHelpfulTools.h"
//#include "waterlinked_dvl/TransducerReportStamped.h"
#include "commonbluerovmsg/srv/reset_ekf.hpp"
#include "commonbluerovmsg/msg/height_stamped.hpp"
//#include <chrono>
#include <thread>
//#include "bluerov2common/ekfParameterConfig.h"
//#include <dynamic_reconfigure/server.h>
#include <commonbluerovmsg/msg/state_of_blue_rov.hpp>
static constexpr double CONSTANTS_ONE_G = 9.80665;
class RosClassEKF : public rclcpp::Node {
public:
    RosClassEKF() : Node("ekfStateEstimation"), currentEkf(rclcpp::Clock(RCL_ROS_TIME).now()) {
        rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);


        // External = 0; Mavros = 1; Gazebo = 2

        this->currentInputDVL = 0;
        this->currentInputIMU = 0;
//        this->subscriberDVL.shutdown();
        this->rotationOfDVL = Eigen::AngleAxisd(2.35619449019,
                                                Eigen::Vector3d::UnitZ());//yaw rotation for correct alignment of DVL data; quick fix set to default
        this->positionIMU = Eigen::Vector3d(0, 0, 0);
        this->positionDVL = Eigen::Vector3d(0, 0, 0);


        this->subscriberIMU = this->create_subscription<sensor_msgs::msg::Imu>("imu/data_frd", qos,
                                                                               std::bind(&RosClassEKF::imuCallback,
                                                                                         this, std::placeholders::_1));
        std::cout << "test DVL:" << std::endl;
        this->subscriberDVL = this->create_subscription<waterlinked_a50::msg::TransducerReportStamped>(
                "dvl/transducer_report", qos, std::bind(&RosClassEKF::DVLCallbackDVL, this, std::placeholders::_1));


        this->subscriberDepthOwnTopic = this->create_subscription<commonbluerovmsg::msg::HeightStamped>("height_baro", qos,
                                                                                                        std::bind(
                                                                                                        &RosClassEKF::depthSensorCallback,
                                                                                                        this,
                                                                                                        std::placeholders::_1));

        this->subscriberDepthSensorBaroPX4 = this->create_subscription<px4_msgs::msg::SensorBaro>("/fmu/out/sensor_baro", qos,
                                                                                                        std::bind(
                                                                                                        &RosClassEKF::depthSensorBaroPX4,
                                                                                                        this,
                                                                                                        std::placeholders::_1));
        this->subscriberDepthSensorVehicleAirData = this->create_subscription<px4_msgs::msg::VehicleAirData>("/fmu/out/vehicle_air_data", qos,
                                                                                                  std::bind(
                                                                                                          &RosClassEKF::depthSensorVehicleAir,
                                                                                                          this,
                                                                                                          std::placeholders::_1));


        this->subscriberHeading = this->create_subscription<geometry_msgs::msg::Vector3Stamped>("magnetic_heading", qos,
                                                                                                std::bind(
                                                                                                        &RosClassEKF::headingCallback,
                                                                                                        this,
                                                                                                        std::placeholders::_1));

        this->serviceResetEkf = this->create_service<commonbluerovmsg::srv::ResetEkf>("resetCurrentEKF",
                                                                                      std::bind(&RosClassEKF::resetEKF,
                                                                                                this,
                                                                                                std::placeholders::_1,
                                                                                                std::placeholders::_2));

        this->publisherPoseEkf = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "publisherPoseEkf", qos);
        this->publisherTwistEkf = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
                "publisherTwistEkf", qos);


    }

private:
//    std::deque<sensor_msgs::Imu::ConstPtr> imuDeque;
//    std::deque<mavros_msgs::Altitude::ConstPtr> depthDeque;
//    std::deque<geometry_msgs::TwistStamped::ConstPtr> dvlDeque;
    ekfClassDVL currentEkf;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriberIMU;
    rclcpp::Subscription<commonbluerovmsg::msg::HeightStamped>::SharedPtr subscriberDepthOwnTopic;
    rclcpp::Subscription<px4_msgs::msg::SensorBaro>::SharedPtr subscriberDepthSensorBaroPX4;
    rclcpp::Subscription<px4_msgs::msg::VehicleAirData>::SharedPtr subscriberDepthSensorVehicleAirData;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr subscriberHeading;
    rclcpp::Subscription<waterlinked_a50::msg::TransducerReportStamped>::SharedPtr subscriberDVL;


    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisherPoseEkf;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr publisherTwistEkf;
    std::mutex updateSlamMutex;
    Eigen::Quaterniond rotationOfDVL;
    Eigen::Vector3d positionIMU, positionDVL;
    rclcpp::Service<commonbluerovmsg::srv::ResetEkf>::SharedPtr serviceResetEkf;
    int currentInputDVL;
    int currentInputIMU;
    double pressureWhenStarted;
    bool firstMessage;

    void imuCallbackHelper(const sensor_msgs::msg::Imu::SharedPtr msg) {
        Eigen::Quaterniond tmpRot;
        tmpRot.x() = msg.get()->orientation.x;
        tmpRot.y() = msg.get()->orientation.y;
        tmpRot.z() = msg.get()->orientation.z;
        tmpRot.w() = msg.get()->orientation.w;

//        std::cout << msg->linear_acceleration.x<< " " << msg->linear_acceleration.y<< " " << msg->linear_acceleration.z << std::endl;

        currentEkf.predictionImu(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
                                 tmpRot, this->positionIMU,
                                 msg->header.stamp);

        Eigen::Vector3d euler = generalHelpfulTools::getRollPitchYaw(tmpRot);// roll pitch yaw

        //calculate roll pitch from IMU accel data
//        std::cout << "my Roll: "<< euler.x()*180/M_PI<< std::endl;
//        std::cout << "my Pitch: "<< euler.y()*180/M_PI<< std::endl;
//        std::cout << "my Yaw: "<< euler.z()*180/M_PI<< std::endl;
        currentEkf.updateIMU(euler.x(), euler.y(), msg.get()->angular_velocity.x, msg.get()->angular_velocity.y,
                             msg.get()->angular_velocity.z, tmpRot, msg->header.stamp);
        pose currentStateEkf = currentEkf.getState();
        geometry_msgs::msg::PoseWithCovarianceStamped poseMsg;
        poseMsg.header.frame_id = "map_ned";
        poseMsg.pose.pose.position.x = currentStateEkf.position.x();
        poseMsg.pose.pose.position.y = currentStateEkf.position.y();
        poseMsg.pose.pose.position.z = currentStateEkf.position.z();
        Eigen::Quaterniond rotDiff = currentEkf.getRotationVector();
        poseMsg.pose.pose.orientation.x = rotDiff.x();
        poseMsg.pose.pose.orientation.y = rotDiff.y();
        poseMsg.pose.pose.orientation.z = rotDiff.z();
        poseMsg.pose.pose.orientation.w = rotDiff.w();
//        std::cout << msg->header.stamp << std::endl;
        poseMsg.header.stamp = msg->header.stamp;
        this->publisherPoseEkf->publish(poseMsg);
        geometry_msgs::msg::TwistWithCovarianceStamped twistMsg;
        twistMsg.header.frame_id = "map_ned";
        twistMsg.twist.twist.linear.x = currentStateEkf.velocity.x();
        twistMsg.twist.twist.linear.y = currentStateEkf.velocity.y();
        twistMsg.twist.twist.linear.z = currentStateEkf.velocity.z();
        twistMsg.twist.twist.angular.x = currentStateEkf.angleVelocity.x();
        twistMsg.twist.twist.angular.y = currentStateEkf.angleVelocity.y();
        twistMsg.twist.twist.angular.z = currentStateEkf.angleVelocity.z();
        twistMsg.header.stamp = msg->header.stamp;
        this->publisherTwistEkf->publish(twistMsg);
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        //change the orientation of the IMU message


        this->updateSlamMutex.lock();
        this->imuCallbackHelper(msg);
        this->updateSlamMutex.unlock();
    }

    void DVLCallbackDVLHelper(const waterlinked_a50::msg::TransducerReportStamped::SharedPtr msg) {
        if (!msg->report.velocity_valid || msg->report.status != 0) {
            //if we don't know anything, the ekf should just go to 0, else the IMU gives direction.
            this->currentEkf.updateDVL(0, 0, 0, this->rotationOfDVL, this->positionDVL, rclcpp::Time(msg->timestamp));
        } else {
            this->currentEkf.updateDVL(msg->report.vx, msg->report.vy, msg->report.vz, this->rotationOfDVL,
                                       this->positionDVL,
                                       rclcpp::Time(msg->timestamp));
        }
        return;
    }

    void DVLCallbackDVL(const waterlinked_a50::msg::TransducerReportStamped::SharedPtr msg) {
        this->updateSlamMutex.lock();
        this->DVLCallbackDVLHelper(msg);
        this->updateSlamMutex.unlock();
    }

    void DVLCallbackSimulationHelper(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
        this->currentEkf.updateDVL(msg->vector.x, msg->vector.y, msg->vector.z, Eigen::Quaterniond(1, 0, 0, 0),
                                   this->positionDVL,
                                   msg->header.stamp);
    }

    void DVLCallbackSimulation(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
        this->updateSlamMutex.lock();
        this->DVLCallbackSimulationHelper(msg);
        this->updateSlamMutex.unlock();
    }

    void DVLCallbackMavrosHelper(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        this->currentEkf.updateDVL(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z,
                                   Eigen::Quaterniond(1, 0, 0, 0), this->positionDVL,
                                   msg->header.stamp);
    }

    void DVLCallbackMavros(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        this->updateSlamMutex.lock();
        this->DVLCallbackMavrosHelper(msg);
        this->updateSlamMutex.unlock();
    }


    bool resetEKF(const std::shared_ptr<commonbluerovmsg::srv::ResetEkf::Request> req, std::shared_ptr<commonbluerovmsg::srv::ResetEkf::Response> res) {
        this->updateSlamMutex.lock();
        this->currentEkf.resetToPos(req->x_pos, req->y_pos, req->yaw, req->reset_covariances);
        this->updateSlamMutex.unlock();
        res->reset_done = true;
        return true;
    }

    void depthSensorCallback(const commonbluerovmsg::msg::HeightStamped::SharedPtr msg) {
        this->updateSlamMutex.lock();
        this->depthSensorHelper(msg);
        this->updateSlamMutex.unlock();
    }
    void depthSensorBaroPX4(const px4_msgs::msg::SensorBaro::SharedPtr msg) {
        if(this->firstMessage){
            this->pressureWhenStarted = msg->pressure;
            this->firstMessage = false;
            return;
        }
        commonbluerovmsg::msg::HeightStamped::SharedPtr newMsg;
        newMsg->timestamp = msg->timestamp;
        newMsg->height = ((msg->pressure-this->pressureWhenStarted)*1.0f)/(CONSTANTS_ONE_G*1000.0f);
        this->updateSlamMutex.lock();
        this->depthSensorHelper(newMsg);
        this->updateSlamMutex.unlock();
    }
    void depthSensorVehicleAir(const px4_msgs::msg::VehicleAirData::SharedPtr msg) {
        if(this->firstMessage){
            this->pressureWhenStarted = msg->baro_pressure_pa;
            this->firstMessage = false;
            return;
        }
        commonbluerovmsg::msg::HeightStamped::SharedPtr newMsg;
        newMsg->timestamp = msg->timestamp;
        newMsg->height = ((msg->baro_pressure_pa-this->pressureWhenStarted)*1.0f)/(CONSTANTS_ONE_G*1000.0f);

        this->updateSlamMutex.lock();
        this->depthSensorHelper(newMsg);
        this->updateSlamMutex.unlock();
    }

    void depthSensorHelper(const commonbluerovmsg::msg::HeightStamped::SharedPtr msg) {
        this->currentEkf.updateHeight(msg->height, rclcpp::Time(msg->timestamp));
    };

    void headingCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {

        this->updateSlamMutex.lock();
        this->headingHelper(msg);
        this->updateSlamMutex.unlock();
    }

    void headingHelper(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
        this->currentEkf.updateHeading(msg->vector.z, msg->header.stamp);
    };

public:
    pose getPoseOfEKF() {
        return this->currentEkf.getState();
    }

//    void callbackReconfiguration(underwaterslam::ekfParameterConfig &config, uint32_t level) {
//        this->updateSlamMutex.lock();
//
//        this->currentEkf.setProcessNoise(config.processNoiseX, config.processNoiseY, config.processNoiseZ,
//                                         config.processNoiseVX, config.processNoiseVY, config.processNoiseVZ,
//                                         config.processNoiseRoll, config.processNoisePitch, config.processNoiseYaw,
//                                         config.processNoiseVelRoll, config.processNoiseVelPitch,
//                                         config.processNoiseVelYaw);
//
//        this->currentEkf.setMeasurementNoiseDVL(config.measurementNoiseDVLVX, config.measurementNoiseDVLVY,
//                                                config.measurementNoiseDVLVZ);
//
//        this->currentEkf.setMeasurementNoiseDepth(config.measurementNoiseDepth);
//
//        this->currentEkf.setMeasurementNoiseIMUVel(config.measurementImuRoll,
//                                                   config.measurementImuPitch,
//                                                   config.measurementImuVelocityVelRoll,
//                                                   config.measurementImuVelocityVelPitch,
//                                                   config.measurementImuVelocityVelYaw);
//        this->rotationOfDVL = Eigen::AngleAxisd(config.yawRotationDVL,
//                                                Eigen::Vector3d::UnitZ());//yaw rotation for correct alignment of DVL data;
//        this->positionDVL.x() = config.xPositionDVL;
//        this->positionDVL.y() = config.yPositionDVL;
//        this->positionDVL.z() = config.zPositionDVL;
//
//        this->positionIMU.x() = config.xPositionIMU;
//        this->positionIMU.y() = config.yPositionIMU;
//        this->positionIMU.z() = config.zPositionIMU;
//
//        // External = 0; Mavros = 1; Gazebo = 2
//        if (this->currentInputIMU != config.IMUInput) {
//            if (config.IMUInput == 0) {
//                std::cout << "External IMU used for EKF" << std::endl;
//                this->subscriberIMU.reset();// MAYBE DOES NOT WORK.
//
//                this->subscriberIMU = this->create_subscription<sensor_msgs::msg::Imu>("imu/data_frd", qos, std::bind(
//                        &RosClassEKF::imuCallback, this));
//
//            }
//            if (config.IMUInput == 1) {
//                std::cout << "Mavros IMU used for EKF" << std::endl;
//                this->subscriberIMU.reset();// MAYBE DOES NOT WORK.
//                this->subscriberIMU = this->create_subscription<sensor_msgs::msg::Imu>("mavros/imu/data_frd", qos,
//                                                                                       std::bind(
//                                                                                               &RosClassEKF::imuCallback,
//                                                                                               this));
//
//            }
//            if (config.IMUInput == 2) {
//                std::cout << "not supported therefore kept the old input" << std::endl;
//            }
//            this->currentInputIMU = config.IMUInput;
//        }
//
//        if (this->currentInputDVL != config.DVLInput) {
//            if (config.DVLInput == 0) {
//                this->subscriberDVL.shutdown();
//                std::cout << "External DVL used for EKF" << std::endl;
//                this->subscriberDVL = this->ournH->subscribe("dvl/transducer_report", 1000,
//                                                             &RosClassEKF::DVLCallbackDVL, this);
//            }
//            if (config.DVLInput == 1) {
//                this->subscriberDVL.shutdown();
//                std::cout << "Mavros DVL used for EKF" << std::endl;
//                this->subscriberDVL = this->ournH->subscribe("mavros/local_position/velocity_body_frd", 1000,
//                                                             &RosClassEKF::DVLCallbackMavros,
//                                                             this);
//            }
//            if (config.DVLInput == 2) {
//                this->subscriberDVL.shutdown();
//                std::cout << "Gazebo DVL used for EKF" << std::endl;
//                this->subscriberDVL = this->ournH->subscribe("simulatedDVL", 1000,
//                                                             &RosClassEKF::DVLCallbackSimulation,
//                                                             this);
//            }
//            this->currentInputDVL = config.DVLInput;
//        }
//
//        this->updateSlamMutex.unlock();
//    }
};


Eigen::Quaterniond getQuaternionForMavrosFromRPY(double roll, double pitch, double yaw) {
    return generalHelpfulTools::getQuaternionFromRPY(roll, -pitch, -yaw + M_PI / 2);
}

Eigen::Vector3f getPositionForMavrosFromXYZ(Eigen::Vector3f inputPosition) {
    Eigen::AngleAxisf rotation_vector180X(180.0 / 180.0 * 3.14159, Eigen::Vector3f(1, 0, 0));
    Eigen::AngleAxisf rotation_vector90Z(90.0 / 180.0 * 3.14159, Eigen::Vector3f(0, 0, 1));
    Eigen::Vector3f positionRotatedForMavros =
            rotation_vector90Z.toRotationMatrix() * rotation_vector180X.toRotationMatrix() * inputPosition;
    return positionRotatedForMavros;

}

int main(int argc, char **argv) {
//    std::this_thread::sleep_for(std::chrono::milliseconds(5000));


    rclcpp::init(argc, argv);
//    auto node = std::make_shared<RosClassEKF>();
    rclcpp::spin(std::make_shared<RosClassEKF>());


    rclcpp::shutdown();

//    RosClassEKF rosClassEKFObject(n_);

    //rclcpp::spin();
//    std::thread t1(spinningRos);

//    dynamic_reconfigure::Server<underwaterslam::ekfParameterConfig> server;
//    dynamic_reconfigure::Server<underwaterslam::ekfParameterConfig>::CallbackType f;
//
//    f = boost::bind(&RosClassEKF::callbackReconfiguration, &rosClassEKFObject, _1, _2);
//    server.setCallback(f);
//
//
//    rclcpp::Publisher publisherPoseEkfMavros = (*n_).advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 10);
//    rclcpp::Publisher publisherEasyReadEkf = (*n_).advertise<commonbluerovmsg::stateOfBlueRov>("ekfStateRPY", 10);
//    rclcpp::Rate ourRate = rclcpp::Rate(30);
//    //sending position to Mavros mavros/vision_pose/pose
//    while (rclcpp::ok()) {
//        pose currentPoseEkf = rosClassEKFObject.getPoseOfEKF();
//
//        //calculate pose for Mavros
//        geometry_msgs::PoseStamped msg_vicon_pose;
//        msg_vicon_pose.header.stamp = currentPoseEkf.timeLastPrediction;
//        msg_vicon_pose.header.frame_id = "map_ned"; //optional. Works fine without frame_id
//
////        Eigen::AngleAxisf rotation_vector180X(180.0 / 180.0 * 3.14159, Eigen::Vector3f(1, 0, 0));
////        Eigen::AngleAxisf rotation_vector90Z(90.0 / 180.0 * 3.14159, Eigen::Vector3f(0, 0, 1));
//        Eigen::Vector3f positionRotatedForMavros = getPositionForMavrosFromXYZ(
//                currentPoseEkf.position);
//
//        msg_vicon_pose.pose.position.x = positionRotatedForMavros.x();
//        msg_vicon_pose.pose.position.y = positionRotatedForMavros.y();
//        msg_vicon_pose.pose.position.z = positionRotatedForMavros.z();
//        Eigen::Quaterniond currentRotation = getQuaternionForMavrosFromRPY(currentPoseEkf.rotation.x(),
//                                                                           currentPoseEkf.rotation.y(),
//                                                                           currentPoseEkf.rotation.z());
//
//
//        msg_vicon_pose.pose.orientation.x = currentRotation.x();
//        msg_vicon_pose.pose.orientation.y = currentRotation.y();
//        msg_vicon_pose.pose.orientation.z = currentRotation.z();
//        msg_vicon_pose.pose.orientation.w = currentRotation.w();
//        publisherPoseEkfMavros.publish(msg_vicon_pose);
//        //calculate pose and publish for easy reading
//        commonbluerovmsg::stateOfBlueRov msgForRPYState;
//        msgForRPYState.header.stamp = currentPoseEkf.timeLastPrediction;
//        msgForRPYState.header.frame_id = "map_ned";
//        msgForRPYState.x = currentPoseEkf.position.x();
//        msgForRPYState.y = currentPoseEkf.position.y();
//        msgForRPYState.z = currentPoseEkf.position.z();
//
//        msgForRPYState.Roll = currentPoseEkf.rotation.x();
//        msgForRPYState.Pitch = currentPoseEkf.rotation.y();
//        msgForRPYState.Yaw = currentPoseEkf.rotation.z();
//
//        publisherEasyReadEkf.publish(msgForRPYState);
//        ourRate.sleep();
//    }
    return (0);
}
