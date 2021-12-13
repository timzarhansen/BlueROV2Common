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
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

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
        Eigen::Quaterniond rotationRP(msg->orientation.w,msg->orientation.x,msg->orientation.y,msg->orientation.z);

        Eigen::Vector3d rollPitchYaw = getRollPitchYaw(rotationRP.inverse());
        // @TODO create low pass filter
        double rollIMUACCEL = atan2(-msg->linear_acceleration.y,msg->linear_acceleration.z);
        double pitchIMUACCEL = atan2(msg->linear_acceleration.x,sqrt(msg->linear_acceleration.y*msg->linear_acceleration.y+msg->linear_acceleration.z*msg->linear_acceleration.z));
//        std::cout << "my Roll: "<< rollIMUACCEL*180/M_PI<< std::endl;
//        std::cout << "my Pitch: "<< pitchIMUACCEL*180/M_PI<< std::endl;

        //std::cout <<"r: " <<rollPitchYaw(0)*180/M_PI <<" p: " << rollPitchYaw (1)*180/M_PI<< " y: " <<rollPitchYaw(2)*180/M_PI << std::endl;
        //rollPitchYaw(1)=-rollPitchYaw(1);//only invert pitch
//        if(abs(rollPitchYaw(0))<M_PI/2){
//            rollPitchYaw(1)=-rollPitchYaw(1);
//        }
        rotationRP = getQuaternionFromRPY(-rollIMUACCEL,pitchIMUACCEL,0);
        newMsg.orientation.x = rotationRP.x();//not sure if correct
        newMsg.orientation.y = rotationRP.y();//not sure if correct
        newMsg.orientation.z = rotationRP.z();//not sure if correct
        newMsg.orientation.w = rotationRP.w();//not sure if correct

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

    Eigen::Vector3d getRollPitchYaw(Eigen::Quaterniond quat){
        tf2::Quaternion tmp(quat.x(),quat.y(),quat.z(),quat.w());
        tf2::Matrix3x3 m(tmp);
        double r, p, y;
        m.getRPY(r, p, y);
        Eigen::Vector3d returnVector(r,p,y);
        return returnVector;
    }

    Eigen::Quaterniond getQuaternionFromRPY(double roll, double pitch, double yaw){
//        tf2::Matrix3x3 m;
//        m.setRPY(roll,pitch,yaw);
//        Eigen::Matrix3d m2;
        tf2::Quaternion qtf2;
        qtf2.setRPY(roll,pitch,yaw);
        Eigen::Quaterniond q;
        q.x()=qtf2.x();
        q.y()=qtf2.y();
        q.z()=qtf2.z();
        q.w()=qtf2.w();

//        q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
//            * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
//            * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        return q;
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