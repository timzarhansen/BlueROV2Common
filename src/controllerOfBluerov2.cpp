//
// Created by tim-linux on 22.12.21.
//

#include "controllerOfBluerov2.h"
double controllerOfBluerov2::calculateDepthThrust(double desiredDepthTMP){

    double errorInZ = desiredDepthTMP - this->currentDepth;
    //make sure the integrational part is not to high
    if(std::abs(this->integratorHeight+0.1*errorInZ)<0.2){
        this->integratorHeight = this->integratorHeight+0.1*errorInZ;
    }

    double thrustHeight = 0.5*errorInZ - 0.1*this->currentDepthVel + 0.1*this->integratorHeight;//PID values missing
    return thrustHeight;

}
void controllerOfBluerov2::controllLogic() {
//    double roll = 0.0;
//    double pitch = 0.0;
//    double yaw = 0.0;
//    double thrust_1 = -0.5;
//    double thrust_2 = 0.0;
//    double thrust_3 = 0.1;

    if(this->holdPosition){
        double thrustHeight = this->calculateDepthThrust(this->holdDepth);

        double thrust1 = this->holdXPosition -this->currentXPosition;
        double thrust2 = this->holdYPosition -this->currentYPosition;
        Eigen::Vector2d thrust12{thrust1,thrust2};

        Eigen::Matrix2d rotationYaw;
        rotationYaw(0,0) = cos(this->currentYaw);
        rotationYaw(0,1) = -sin(this->currentYaw);
        rotationYaw(1,1) = cos(this->currentYaw);
        rotationYaw(1,0) = sin(this->currentYaw);
        thrust12 = rotationYaw*thrust12;



        Eigen::Vector3d thrustVec = getThrustForMavros(thrust12(0),thrust12(1), thrustHeight);

        Eigen::Quaterniond rotationSend = controllerOfBluerov2::getQuaternionForMavrosFromRPY(this->holdRoll, this->holdPitch, this->holdYaw);


        mavros_msgs::AttitudeTarget msg;
        msg.header.stamp = ros::Time::now();
        msg.orientation.w = rotationSend.w();
        msg.orientation.x = rotationSend.x();
        msg.orientation.y = rotationSend.y();
        msg.orientation.z = rotationSend.z();
        msg.body_rate.x = thrustVec(0);
        msg.body_rate.y = thrustVec(1);
        msg.body_rate.z = thrustVec(2);
        msg.thrust = 0.4;
        //changing the coordinate system
        msg.body_rate.y = -msg.body_rate.y;
        msg.body_rate.z = -msg.body_rate.z;
        this->publisherMavros.publish(msg);


    }else{//not holding position

        double thrustHeight = this->calculateDepthThrust(this->desiredDepth);
        Eigen::Vector3d thrustVec = getThrustForMavros(this->desiredXThrustBody, this->desiredYThrustBody, thrustHeight);

        Eigen::Quaterniond rotationSend = controllerOfBluerov2::getQuaternionForMavrosFromRPY(this->desiredRoll, this->desiredPitch, this->desiredYaw);


        mavros_msgs::AttitudeTarget msg;
        msg.header.stamp = ros::Time::now();
        msg.orientation.w = rotationSend.w();
        msg.orientation.x = rotationSend.x();
        msg.orientation.y = rotationSend.y();
        msg.orientation.z = rotationSend.z();
        msg.body_rate.x = thrustVec(0);
        msg.body_rate.y = thrustVec(1);
        msg.body_rate.z = thrustVec(2);
        msg.thrust = 0.4;
        //changing the coordinate system
        msg.body_rate.y = -msg.body_rate.y;
        msg.body_rate.z = -msg.body_rate.z;
        this->publisherMavros.publish(msg);
    }
}

Eigen::Vector3d controllerOfBluerov2::getRollPitchYaw(Eigen::Quaterniond quat) {
    tf2::Quaternion tmp(quat.x(), quat.y(), quat.z(), quat.w());
    tf2::Matrix3x3 m(tmp);
    double r, p, y;
    m.getRPY(r, p, y);
    Eigen::Vector3d returnVector(r, p, y);
    return returnVector;
}

Eigen::Quaterniond controllerOfBluerov2::getQuaternionFromRPY(double roll, double pitch, double yaw) {
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
}

Eigen::Quaterniond controllerOfBluerov2::getQuaternionForMavrosFromRPY(double roll, double pitch, double yaw) {
    return controllerOfBluerov2::getQuaternionFromRPY(roll, -pitch, -yaw + M_PI / 2);
}

Eigen::Vector3d controllerOfBluerov2::getThrustForMavros(double thrust_1, double thrust_2, double thrust_3) {
    return Eigen::Vector3d(thrust_1, thrust_2, thrust_3);
}

void controllerOfBluerov2::desiredStateCallback(const commonbluerovmsg::desiredStateForRobot::ConstPtr &msg){
    this->desiredDepth = msg->desiredHeight;
    this->desiredYaw = msg->desiredYaw;
    this->desiredXThrustBody = msg->desiredXThrust;
    this->desiredYThrustBody = msg->desiredYThrust;
    this->desiredRoll = msg->desiredRoll;
    this->desiredPitch = msg->desiredPitch;
    bool tmpBool = this->holdPosition;
    this->holdPosition = msg->holdPosition;
    //if hold positions toggled
    if(this->holdPosition!=tmpBool && this->holdPosition){
        //save current position
        double tmpDouble;
        tmpDouble = this->currentXPosition;
        this->holdXPosition = tmpDouble;

        tmpDouble = this->currentYPosition;
        this->holdYPosition = tmpDouble;

        tmpDouble = this->currentDepth;
        this->holdDepth = tmpDouble;

        tmpDouble = this->currentRoll;
        this->holdRoll = tmpDouble;

        tmpDouble = this->currentPitch;
        this->holdPitch =tmpDouble;
        std::cout << "hold position:" << std::endl;

        std::cout << this->currentYaw << std::endl;
        tmpDouble = this->currentYaw;
        this->holdYaw = tmpDouble;
    }
}

void controllerOfBluerov2::currentTwistCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &msg){
    this->currentXVel = msg->twist.twist.linear.x;
    this->currentYVel = msg->twist.twist.linear.y;
    this->currentDepthVel = msg->twist.twist.linear.z;
    this->currentRollVel = msg->twist.twist.angular.x;
    this->currentPitchVel = msg->twist.twist.angular.y;
    this->currentYawVel = msg->twist.twist.angular.z;
}

void controllerOfBluerov2::currentPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg){
    this->currentXPosition = msg->pose.pose.position.x;
    this->currentYPosition = msg->pose.pose.position.y;
    this->currentDepth = msg->pose.pose.position.z;
    Eigen::Quaterniond currentRotation(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z);
    Eigen::Vector3d rollPitchYaw = getRollPitchYaw(currentRotation);
    this->currentRoll =rollPitchYaw(0);
    this->currentPitch= rollPitchYaw(1);
    this->currentYaw = rollPitchYaw(2);
}
