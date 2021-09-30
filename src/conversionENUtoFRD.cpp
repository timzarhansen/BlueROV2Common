//
// Created by jurobotics on 28.09.21.
//
#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"
#include "mavros_msgs/Altitude.h"
#include "geometry_msgs/TwistStamped.h"
#include <Eigen/Geometry>
#include <Eigen/StdVector>

class rosConversionClass {
public:
    rosConversionClass(ros::NodeHandle n_){
        subscriberImuData = n_.subscribe("mavros/imu/data", 1000, &rosConversionClass::imuCallback, this);
        subscriberVelocityBody = n_.subscribe("mavros/local_position/velocity_body", 1000, &rosConversionClass::velocityBodyCallback, this);
        subscriberAltitude = n_.subscribe("mavros/altitude", 1000, &rosConversionClass::altitudeCallback, this);

        publisherImuData = n_.advertise<sensor_msgs::Imu>("mavros/imu/data_frd", 1000);
        publisherVelocityBody = n_.advertise<geometry_msgs::TwistStamped>("mavros/local_position/velocity_body_frd", 1000);
        publisherAltitude = n_.advertise<mavros_msgs::Altitude>("mavros/altitude_frd", 1000);

        Eigen::AngleAxisd rotation_vector2(180.0 / 180.0 * 3.14159, Eigen::Vector3d(1, 0, 0));
        transformationX180DegreeRotationMatrix = rotation_vector2.toRotationMatrix();
        transformationX180DegreeQuaternion = Eigen::Quaterniond(transformationX180DegreeRotationMatrix);
    }

private:
    ros::Subscriber subscriberImuData, subscriberVelocityBody, subscriberAltitude;
    ros::Publisher publisherAltitude,publisherVelocityBody,publisherImuData;
    Eigen::Matrix3d transformationX180DegreeRotationMatrix;
    Eigen::Quaterniond transformationX180DegreeQuaternion;

    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg){
        Eigen::Vector3d acceleration(msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z);
        acceleration = transformationX180DegreeRotationMatrix*acceleration;

        Eigen::Vector3d rotationVel(msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z);
        rotationVel = transformationX180DegreeRotationMatrix*rotationVel;
        sensor_msgs::Imu newMsg;
        newMsg.header = msg->header;
        newMsg.angular_velocity.x = rotationVel.x();
        newMsg.angular_velocity.y = rotationVel.y();
        newMsg.angular_velocity.z = rotationVel.z();

        newMsg.linear_acceleration.x = acceleration.x();
        newMsg.linear_acceleration.y = acceleration.y();
        newMsg.linear_acceleration.z = acceleration.z();
        newMsg.orientation = msg->orientation;//not sure if correct
        publisherImuData.publish(newMsg);

    }

    void velocityBodyCallback(const geometry_msgs::TwistStamped::ConstPtr &msg){

        Eigen::Vector3d velocityBody(msg->twist.linear.x,msg->twist.linear.y,msg->twist.linear.z);
        velocityBody = transformationX180DegreeRotationMatrix*velocityBody;
        Eigen::Vector3d angularVelocityBody(msg->twist.angular.x,msg->twist.angular.y,msg->twist.angular.z);
        angularVelocityBody = transformationX180DegreeRotationMatrix*angularVelocityBody;

        geometry_msgs::TwistStamped newMsg;
        newMsg.header = msg->header;
        newMsg.twist.angular.x = angularVelocityBody.x();
        newMsg.twist.angular.y = angularVelocityBody.y();
        newMsg.twist.angular.z = angularVelocityBody.z();

        newMsg.twist.linear.x = velocityBody.x();
        newMsg.twist.linear.y = velocityBody.y();
        newMsg.twist.linear.z = velocityBody.z();

        publisherVelocityBody.publish(newMsg);
    }

    void altitudeCallback(const mavros_msgs::Altitude::ConstPtr &msg){

        mavros_msgs::Altitude newMsg;
        newMsg.header = msg->header;
        newMsg.local=-msg->local;
        newMsg.relative=-msg->relative;
        newMsg.amsl=-msg->amsl;
        publisherAltitude.publish(newMsg);
    }
};



int main(int argc, char **argv) {
    ros::init(argc, argv, "conversionenutofrd");
    ros::start();
    ros::NodeHandle n_;
    rosConversionClass tmpClass(n_);


    ros::spin();


    return (0);
}