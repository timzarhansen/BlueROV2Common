//
// Created by tim-linux on 22.12.21.
//

#include "controller/controllerOfBluerov2.h"

//void controllerOfBluerov2::callbackReconfiguration(bluerov2common::msg::controllerConfig &config, uint32_t level) {
//    this->setControllerValues(config.height_i, config.height_d, config.height_p, config.hold_position_p,
//                              config.hold_position_i, config.hold_position_d);
//}


double controllerOfBluerov2::calculateDepthThrust(double desiredDepthTMP) {

    double errorInZ = desiredDepthTMP - this->currentDepth;
    //make sure the integrational part is not to high
    if (std::abs(this->integratorHeight + 0.01 * errorInZ) < 0.2) {
        this->integratorHeight = this->integratorHeight + 0.01 * errorInZ;
    }

    double thrustHeight =
            this->height_p * errorInZ - this->height_d * this->currentDepthVel + this->height_i * this->integratorHeight;//PID values
    return thrustHeight;

}

void controllerOfBluerov2::setControllerValues(double height_i_tmp, double height_d_tmp, double height_p_tmp,
                                               double hold_position_p_tmp, double hold_position_i_tmp,
                                               double hold_position_d_tmp) {
    this->height_p = height_p_tmp;
    this->height_i = height_i_tmp;
    this->height_d = height_d_tmp;

    this->hold_position_p = hold_position_p_tmp;
    this->hold_position_i = hold_position_i_tmp;
    this->hold_position_d = hold_position_d_tmp;
}

Eigen::Vector3d controllerOfBluerov2::controllLogic() {
//    double roll = 0.0;
//    double pitch = 0.0;
//    double yaw = 0.0;
//    double thrust_1 = -0.5;
//    double thrust_2 = 0.0;
//    double thrust_3 = 0.1;
    Eigen::Vector3d returnThrust;
    if (this->holdPosition) {
        double thrustHeight = this->calculateDepthThrust(this->holdDepth);
        double errorXPosition = this->holdXPosition - this->currentXPosition;
        double errorYPosition = this->holdYPosition - this->currentYPosition;



        //make sure the integrational part is not to high
        if (std::abs(this->integratorX + 0.01 * errorXPosition) < 0.2) {
            this->integratorX = this->integratorX + 0.01 * errorXPosition;
        }
        if (std::abs(this->integratorY + 0.01 * errorYPosition) < 0.2) {
            this->integratorY = this->integratorY + 0.01 * errorYPosition;
        }

        double thrust1 = this->hold_position_p*(errorXPosition)+this->hold_position_i*this->integratorX-this->hold_position_d*this->currentXVel;
        double thrust2 = this->hold_position_p*(errorYPosition)+this->hold_position_i*this->integratorY-this->hold_position_d*this->currentYVel;
        Eigen::Vector2d thrust12{thrust1, thrust2};
        returnThrust[0] = thrust1;
        returnThrust[1] = thrust2;
        returnThrust[2] = thrustHeight;
        //conversion from global to body
        Eigen::Matrix2d rotationYaw;
        rotationYaw(0, 0) = cos(this->currentYaw);
        rotationYaw(0, 1) = sin(this->currentYaw);
        rotationYaw(1, 1) = cos(this->currentYaw);
        rotationYaw(1, 0) = -sin(this->currentYaw);
        thrust12 = rotationYaw * thrust12;


        Eigen::Vector3d thrustVec = getThrustForMavros(thrust12(0), thrust12(1), thrustHeight);


        Eigen::Quaterniond rotationSend = controllerOfBluerov2::getQuaternionForMavrosFromRPY(this->holdRoll,
                                                                                              this->holdPitch,
                                                                                              this->holdYaw);


        px4_msgs::msg::VehicleAttitudeSetpoint msg;
        msg.timestamp = rclcpp::Clock(RCL_ROS_TIME).now().nanoseconds();
        msg.thrust_body[0] = thrustVec(0);
        msg.thrust_body[1] = thrustVec(1);
        msg.thrust_body[2] = thrustVec(2);
        msg.roll_body = this->holdRoll;
        msg.pitch_body = this->holdPitch;
        msg.yaw_body = this->holdYaw;

//        msg.orientation.w = rotationSend.w();
//        msg.orientation.x = rotationSend.x();
//        msg.orientation.y = rotationSend.y();
//        msg.orientation.z = rotationSend.z();
//        msg.body_rate.x = thrustVec(0);
//        msg.body_rate.y = thrustVec(1);
//        msg.body_rate.z = thrustVec(2);
//        msg.thrust = 0.1;//this is unused but should be non zero
//        //changing the coordinate system
//        msg.body_rate.y = -msg.body_rate.y;
//        msg.body_rate.z = -msg.body_rate.z;
        this->publisherPX4->publish(msg);


    } else {//not holding position

        double thrustHeight = this->calculateDepthThrust(this->desiredDepth);
        Eigen::Quaterniond currentRotation = controllerOfBluerov2::getQuaternionFromRPY(this->currentRoll,this->currentPitch,this->currentYaw);
        returnThrust[0] = this->desiredXThrustBody;
        returnThrust[1] = this->desiredYThrustBody;
        returnThrust[2] = thrustHeight;
        returnThrust = currentRotation*returnThrust;
        Eigen::Vector3d thrustVec = getThrustForMavros(this->desiredXThrustBody, this->desiredYThrustBody,
                                                       thrustHeight);

        Eigen::Quaterniond rotationSend = controllerOfBluerov2::getQuaternionForMavrosFromRPY(this->desiredRoll,
                                                                                              this->desiredPitch,
                                                                                              this->desiredYaw);

        px4_msgs::msg::VehicleAttitudeSetpoint msg;
        msg.timestamp = rclcpp::Clock(RCL_ROS_TIME).now().nanoseconds();
        msg.thrust_body[0] = thrustVec(0);
        msg.thrust_body[1] = thrustVec(1);
        msg.thrust_body[2] = thrustVec(2);
        msg.roll_body = this->desiredRoll;
        msg.pitch_body = this->desiredPitch;
        msg.yaw_body = this->desiredYaw;

//        mavros_msgs::AttitudeTarget msg;
//        msg.header.stamp = rclcpp::Time::now();
//        msg.orientation.w = rotationSend.w();
//        msg.orientation.x = rotationSend.x();
//        msg.orientation.y = rotationSend.y();
//        msg.orientation.z = rotationSend.z();
//        msg.body_rate.x = thrustVec(0);
//        msg.body_rate.y = thrustVec(1);
//        msg.body_rate.z = thrustVec(2);
//        msg.thrust = 0.1;//this is unused but should be non zero
//        //changing the coordinate system
//        msg.body_rate.y = -msg.body_rate.y;
//        msg.body_rate.z = -msg.body_rate.z;
        this->publisherPX4->publish(msg);
    }

    return returnThrust;
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
    Eigen::Vector3d tmpVec(thrust_1, thrust_2, thrust_3);
    if(tmpVec.norm()>1){
        tmpVec.normalize();
    }
    return tmpVec;
}

void controllerOfBluerov2::desiredStateCallback(const commonbluerovmsg::msg::DesiredStateForRobot::SharedPtr msg) {
    this->desiredDepth = msg->desired_height;
    this->desiredYaw = msg->desired_yaw;
    this->desiredXThrustBody = msg->desired_x_thrust;
    this->desiredYThrustBody = msg->desired_y_thrust;
    this->desiredRoll = msg->desired_roll;
    this->desiredPitch = msg->desired_pitch;
    bool tmpBool = this->holdPosition;
    this->holdPosition = msg->hold_position;
    //if hold positions toggled
    if (this->holdPosition != tmpBool && this->holdPosition) {
        this->integratorX = 0;
        this->integratorY = 0;

        //save current position
        double tmpDouble;
        tmpDouble = this->currentXPosition;
        this->holdXPosition = tmpDouble;

        tmpDouble = this->currentYPosition;
        this->holdYPosition = tmpDouble;

        tmpDouble = this->desiredDepth;
        this->holdDepth = tmpDouble;

        tmpDouble = this->currentRoll;
        this->holdRoll = 0;// sould be 0 for now tmpDouble;

        tmpDouble = this->currentPitch;
        this->holdPitch = 0;// sould be 0 for now tmpDouble;
        //std::cout << "hold position:" << std::endl;

        //std::cout << this->currentYaw << std::endl;
        tmpDouble = this->currentYaw;
         this->holdYaw = tmpDouble;

    }
}

void controllerOfBluerov2::currentTwistCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
    this->currentXVel = msg->twist.twist.linear.x;
    this->currentYVel = msg->twist.twist.linear.y;
    this->currentDepthVel = msg->twist.twist.linear.z;
    this->currentRollVel = msg->twist.twist.angular.x;
    this->currentPitchVel = msg->twist.twist.angular.y;
    this->currentYawVel = msg->twist.twist.angular.z;
}

void controllerOfBluerov2::currentPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    this->currentXPosition = msg->pose.pose.position.x;
    this->currentYPosition = msg->pose.pose.position.y;
    this->currentDepth = msg->pose.pose.position.z;
    Eigen::Quaterniond currentRotation(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                       msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    Eigen::Vector3d rollPitchYaw = getRollPitchYaw(currentRotation);
    this->currentRoll = rollPitchYaw(0);
    this->currentPitch = rollPitchYaw(1);
    this->currentYaw = rollPitchYaw(2);
}

void controllerOfBluerov2::getPoseRobot( Eigen::Vector3d &position, Eigen::Quaterniond &rotation){
    position.x() = this->currentXPosition;
    position.y() = this->currentYPosition;
    position.z() = this->currentDepth;
    rotation = getQuaternionFromRPY(this->currentRoll,this->currentPitch,this->currentYaw);
}

void controllerOfBluerov2::getPoseTarget(Eigen::Vector3d &position, Eigen::Quaterniond &rotation){
    position.x() = this->holdXPosition;
    position.y() = this->holdYPosition;
    position.z() = this->holdDepth;
    rotation = getQuaternionFromRPY(this->holdRoll,this->holdPitch,this->holdYaw);
}

void controllerOfBluerov2::timer_callback(){
    Eigen::Vector3d thrustVector = this->controllLogic();

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map_ned";
    marker.header.stamp = rclcpp::Time();
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    Eigen::Vector3d position;
    Eigen::Quaterniond rotation;
    this->getPoseRobot(position,rotation);
    geometry_msgs::msg::Point p;
    p.x=position.x();
    p.y=position.y();
    p.z=position.z();
    marker.points.push_back(p);
    p.x=position.x()+thrustVector.x();
    p.y=position.y()+thrustVector.y();
    p.z=position.z()+thrustVector.z();
    marker.points.push_back(p);


    marker.scale.x = 0.06;
    marker.scale.y = 0.06;
    marker.scale.z = 0.06;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    this->publisherVisualization->publish(marker);

    this->getPoseTarget(position,rotation);
    marker.header.frame_id = "map_ned";
    marker.header.stamp = rclcpp::Time();
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;


    marker.pose.position.x=position.x();
    marker.pose.position.y=position.y();
    marker.pose.position.z=position.z();
    marker.pose.orientation.w =1;


    marker.scale.x = 0.06;
    marker.scale.y = 0.06;
    marker.scale.z = 0.06;
    marker.color.a = 0.8; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    this->publisherVisualization->publish(marker);

}



