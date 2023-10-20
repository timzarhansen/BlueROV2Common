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
#include "sensor_msgs/msg/fluid_pressure.hpp"
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
#include "px4_msgs/msg/sensor_combined.hpp"

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
        this->firstMessage = true;
        this->currentInputDVL = 0;
        this->currentInputIMU = 0;
//        this->subscriberDVL.shutdown();
        this->rotationOfDVL = Eigen::AngleAxisd(2.35619449019,
                                                Eigen::Vector3d::UnitZ());//yaw rotation for correct alignment of DVL data; quick fix set to default
        this->positionIMU = Eigen::Vector3d(0, 0, 0);
        this->positionDVL = Eigen::Vector3d(0, 0, 0);


        this->subscriberIMU = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data", qos,
                                                                               std::bind(&RosClassEKF::imuCallback,
                                                                                         this, std::placeholders::_1));

//        this->subscriberPX4IMU = this->create_subscription<px4_msgs::msg::SensorCombined>("/fmu/out/sensor_combined", qos,
//                                                                               std::bind(&RosClassEKF::imuCallbackPX4,
//                                                                                         this, std::placeholders::_1));
        std::cout << "test DVL:" << std::endl;
        this->subscriberDVL = this->create_subscription<waterlinked_a50::msg::TransducerReportStamped>(
                "/velocity_estimate", qos, std::bind(&RosClassEKF::DVLCallbackDVL, this, std::placeholders::_1));


        this->subscriberDepthOwnTopic = this->create_subscription<commonbluerovmsg::msg::HeightStamped>("height_baro", qos,
                                                                                                        std::bind(
                                                                                                        &RosClassEKF::depthSensorCallback,
                                                                                                        this,
                                                                                                        std::placeholders::_1));

//        this->subscriberDepthSensorBaroPX4 = this->create_subscription<px4_msgs::msg::SensorBaro>("/fmu/out/sensor_baro", qos,
//                                                                                                        std::bind(
//                                                                                                        &RosClassEKF::depthSensorBaroPX4,
//                                                                                                        this,
//                                                                                                        std::placeholders::_1));
        this->subscriberDepthSensorBaroSensorTube = this->create_subscription<sensor_msgs::msg::FluidPressure>("/pressure", qos,
                                                                                                  std::bind(
                                                                                                          &RosClassEKF::depthSensorBaroSensorTubeCallback,
                                                                                                          this,
                                                                                                          std::placeholders::_1));
//        this->subscriberDepthSensorVehicleAirData = this->create_subscription<px4_msgs::msg::VehicleAirData>("/fmu/out/vehicle_air_data", qos,
//                                                                                                  std::bind(
//                                                                                                          &RosClassEKF::depthSensorVehicleAir,
//                                                                                                          this,
//                                                                                                          std::placeholders::_1));


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
    rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr subscriberPX4IMU;


    rclcpp::Subscription<commonbluerovmsg::msg::HeightStamped>::SharedPtr subscriberDepthOwnTopic;
    rclcpp::Subscription<px4_msgs::msg::SensorBaro>::SharedPtr subscriberDepthSensorBaroPX4;
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr subscriberDepthSensorBaroSensorTube;
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

//        std::cout << "takin IMU message"<< std::endl;
        Eigen::Matrix3d transformationX180DegreeRotationMatrix;
        Eigen::AngleAxisd rotation_vector2(180.0 / 180.0 * 3.14159, Eigen::Vector3d(1, 0, 0));
        transformationX180DegreeRotationMatrix = rotation_vector2.toRotationMatrix();

        Eigen::Vector3d acceleration(msg->linear_acceleration.x, msg->linear_acceleration.y,
                                     msg->linear_acceleration.z);
        acceleration = transformationX180DegreeRotationMatrix * acceleration;

        Eigen::Vector3d rotationVel(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
        rotationVel = transformationX180DegreeRotationMatrix * rotationVel;


        sensor_msgs::msg::Imu newMsg;
        newMsg.header = msg->header;
        newMsg.angular_velocity.x = rotationVel.x();
        newMsg.angular_velocity.y = rotationVel.y();
        newMsg.angular_velocity.z = rotationVel.z();

        newMsg.linear_acceleration.x = acceleration.x();
        newMsg.linear_acceleration.y = acceleration.y();
        newMsg.linear_acceleration.z = acceleration.z();
        Eigen::Quaterniond rotationRP(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

        Eigen::Vector3d rollPitchYaw = this->getRollPitchYaw(rotationRP.inverse());
        // @TODO create low pass filter
        double rollIMUACCEL = atan2(-msg->linear_acceleration.y, msg->linear_acceleration.z);
        double pitchIMUACCEL = atan2(msg->linear_acceleration.x,
                                     sqrt(msg->linear_acceleration.y * msg->linear_acceleration.y +
                                          msg->linear_acceleration.z * msg->linear_acceleration.z));

        rotationRP = getQuaternionFromRPY(-rollIMUACCEL, pitchIMUACCEL, 0);
        newMsg.orientation.x = rotationRP.x();//not sure if correct
        newMsg.orientation.y = rotationRP.y();//not sure if correct
        newMsg.orientation.z = rotationRP.z();//not sure if correct
        newMsg.orientation.w = rotationRP.w();//not sure if correct





        Eigen::Quaterniond tmpRot;
        tmpRot.x() = newMsg.orientation.x;
        tmpRot.y() = newMsg.orientation.y;
        tmpRot.z() = newMsg.orientation.z;
        tmpRot.w() = newMsg.orientation.w;

//        std::cout << msg->linear_acceleration.x<< " " << msg->linear_acceleration.y<< " " << msg->linear_acceleration.z << std::endl;

        currentEkf.predictionImu(newMsg.linear_acceleration.x, newMsg.linear_acceleration.y, newMsg.linear_acceleration.z,
                                 tmpRot, this->positionIMU,
                                 newMsg.header.stamp);

        Eigen::Vector3d euler = generalHelpfulTools::getRollPitchYaw(tmpRot);// roll pitch yaw

        //calculate roll pitch from IMU accel data
//        std::cout << "my Roll: "<< euler.x()*180/M_PI<< std::endl;
//        std::cout << "my Pitch: "<< euler.y()*180/M_PI<< std::endl;
//        std::cout << "my Yaw: "<< euler.z()*180/M_PI<< std::endl;
        currentEkf.updateIMU(euler.x(), euler.y(), newMsg.angular_velocity.x, newMsg.angular_velocity.y,
                             newMsg.angular_velocity.z, tmpRot, newMsg.header.stamp);
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
        twistMsg.header.stamp = newMsg.header.stamp;
        this->publisherTwistEkf->publish(twistMsg);
    }




    void imuCallbackPX4(const px4_msgs::msg::SensorCombined::SharedPtr msg) {
        //change the orientation of the IMU message
        sensor_msgs::msg::Imu newMsg{};
        newMsg.linear_acceleration.x = msg->accelerometer_m_s2[0];
        newMsg.linear_acceleration.y = msg->accelerometer_m_s2[1];
        newMsg.linear_acceleration.z = msg->accelerometer_m_s2[2];

        newMsg.angular_velocity.x = msg->gyro_rad[0];
        newMsg.angular_velocity.y = msg->gyro_rad[1];
        newMsg.angular_velocity.z = msg->gyro_rad[2];
        newMsg.header.stamp =  rclcpp::Clock(RCL_ROS_TIME).now();





        this->updateSlamMutex.lock();
        this->imuCallbackHelper(std::make_shared<sensor_msgs::msg::Imu>(newMsg));
        this->updateSlamMutex.unlock();
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        //change the orientation of the IMU message


        this->updateSlamMutex.lock();
        this->imuCallbackHelper(msg);
        this->updateSlamMutex.unlock();
    }

    void DVLCallbackDVLHelper(const waterlinked_a50::msg::TransducerReportStamped::SharedPtr msg) {
//        std::cout << "getting DVL message" << std::endl;
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
        commonbluerovmsg::msg::HeightStamped newMsg{};
//        std::cout << msg->timestamp << std::endl;
        auto currentTimeOfMessage = std::chrono::microseconds(msg->timestamp)*1000;
        newMsg.timestamp = uint64_t(currentTimeOfMessage.count()*1000);//currentTimeOfMessage.count();

        newMsg.height = ((msg->pressure-this->pressureWhenStarted)*10.0f)/(CONSTANTS_ONE_G*1000.0f);
//        std::cout << "start" << std::endl;
//        std::cout << this->pressureWhenStarted << std::endl;
//        std::cout << msg->pressure << std::endl;
//        std::cout <<  newMsg.height << std::endl;
        this->updateSlamMutex.lock();
        this->depthSensorHelper(std::make_shared<commonbluerovmsg::msg::HeightStamped>(newMsg));
        this->updateSlamMutex.unlock();
    }

    void depthSensorBaroSensorTubeCallback(const sensor_msgs::msg::FluidPressure::SharedPtr msg) {\

        if(this->firstMessage){
            this->pressureWhenStarted = msg->fluid_pressure;
            this->firstMessage = false;
            return;
        }
        commonbluerovmsg::msg::HeightStamped newMsg{};
//        std::cout << msg->timestamp << std::endl;
        auto currentTimeOfMessage = std::chrono::microseconds(msg->header.stamp.nanosec)*1000;
        newMsg.timestamp = uint64_t(currentTimeOfMessage.count()*1000);//currentTimeOfMessage.count();

        newMsg.height = ((msg->fluid_pressure-this->pressureWhenStarted)*0.01f)/(CONSTANTS_ONE_G*1000.0f);
//        std::cout << "start" << std::endl;
//        std::cout << this->pressureWhenStarted << std::endl;
//        std::cout << msg->fluid_pressure << std::endl;
//        std::cout <<  newMsg.height << std::endl;
        this->updateSlamMutex.lock();
        this->depthSensorHelper(std::make_shared<commonbluerovmsg::msg::HeightStamped>(newMsg));
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
    }

    void headingCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {

        this->updateSlamMutex.lock();
        this->headingHelper(msg);
        this->updateSlamMutex.unlock();
    }

    void headingHelper(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
        this->currentEkf.updateHeading(msg->vector.z, msg->header.stamp);
    }


    Eigen::Vector3d getRollPitchYaw(Eigen::Quaterniond quat) {
        tf2::Quaternion tmp(quat.x(), quat.y(), quat.z(), quat.w());
        tf2::Matrix3x3 m(tmp);
        double r, p, y;
        m.getRPY(r, p, y);
        Eigen::Vector3d returnVector(r, p, y);
        return returnVector;
    }

    Eigen::Quaterniond getQuaternionFromRPY(double roll, double pitch, double yaw) {
//        tf2::Matrix3x3 m;
//        m.setRPY(roll,pitch,yaw);
//        Eigen::Matrix3d m2;
        tf2::Quaternion qtf2;
        qtf2.setRPY(roll, pitch, yaw);
        Eigen::Quaterniond q;
        q.x() = qtf2.x();
        q.y() = qtf2.y();
        q.z() = qtf2.z();
        q.w() = qtf2.w();

//        q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
//            * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
//            * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        return q;
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
