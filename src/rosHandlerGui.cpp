//
// Created by tim-linux on 14.12.21.
//

#include "rosHandlerGui.h"


void rosHandlerGui::positionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
//    std::cout << "test1332"<< std::endl;
    this->xPositionRobot.push_back(msg->pose.pose.position.x);
    this->yPositionRobot.push_back(msg->pose.pose.position.y);
    Eigen::Quaterniond rotation;
    rotation.x() = msg->pose.pose.orientation.x;
    rotation.y() = msg->pose.pose.orientation.y;
    rotation.z() = msg->pose.pose.orientation.z;
    rotation.w() = msg->pose.pose.orientation.w;
    Eigen::MatrixXd covariance;
    covariance = Eigen::MatrixXd::Zero(6, 6);
    for (int i = 0; i < 6; i++) {
        covariance(i, i) = msg->pose.covariance.at(i);
    }

    Eigen::Vector3d rollPitchYaw = this->getRollPitchYaw(rotation);

//    std::cout << "test132"<< std::endl;
    emit this->updatePlotPositionVectorROS(this->xPositionRobot, this->yPositionRobot, this->yawPositionRobot);
    emit this->updateStateOfRobotROS(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z,
                                     rollPitchYaw[0], rollPitchYaw[1], rollPitchYaw[2], covariance);
}

void rosHandlerGui::setSonarRange(double range) {

}

void rosHandlerGui::sonarImageCallback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_ptr = cv_bridge::cvtColor(cv_ptr,"rgb8");

    QImage imgIn = QImage((uchar *) cv_ptr->image.data, cv_ptr->image.cols, cv_ptr->image.rows, cv_ptr->image.step,
                          QImage::Format_RGB888);
//    QImage imgIn = QImage(&msg->data[0], msg->width, msg->height, QImage::Format_RGB888);
    imgIn = imgIn.rgbSwapped();
    QPixmap myPixMap = QPixmap::fromImage(imgIn);
    emit updateSonarImageROS(myPixMap);
}

void rosHandlerGui::cameraImageCallback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_ptr = cv_bridge::cvtColor(cv_ptr,"rgb8");

    QImage imgIn = QImage((uchar *) cv_ptr->image.data, cv_ptr->image.cols, cv_ptr->image.rows, cv_ptr->image.step,
                          QImage::Format_RGB888);

    //QImage::Format_BGR30  Format_RGB888
    QPixmap myPixMap = QPixmap::fromImage(imgIn);

    emit updateCameraImageROS(myPixMap);
}

void rosHandlerGui::updateDesiredState(double desiredHeight, double desiredRoll, double desiredPitch, double desiredYaw,
                                       double desiredXMovement, double desiredYMovement, bool holdPosition) {

    //this just sends the data to ros, the frame rate is made by mainwindow
    bluerov2common::desiredStateForRobot msg;
    msg.header.stamp = ros::Time::now();
    msg.desiredHeight = desiredHeight;
    msg.desiredRoll = desiredRoll;
    msg.desiredPitch = desiredPitch;
    msg.desiredXThrust = desiredXMovement;
    msg.desiredYaw = desiredYaw;
    msg.desiredYThrust = desiredYMovement;
    msg.holdPosition = holdPosition;
    publishingDesiredState.publish(msg);

}

