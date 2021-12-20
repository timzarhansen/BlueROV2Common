//
// Created by tim-linux on 14.12.21.
//

#include "rosHandlerGui.h"


void rosHandlerGui::positionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
//    std::cout << "test1332"<< std::endl;
    this->xPositionRobot.append(msg->pose.pose.position.x);
    this->yPositionRobot.append(msg->pose.pose.position.y);
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
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    QImage imgIn = QImage((uchar *) cv_ptr->image.data, cv_ptr->image.cols, cv_ptr->image.rows, cv_ptr->image.step,
                          QImage::Format_RGB888);
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
    QImage imgIn = QImage((uchar *) cv_ptr->image.data, cv_ptr->image.cols, cv_ptr->image.rows, cv_ptr->image.step,
                          QImage::Format_RGB888);
    QPixmap myPixMap = QPixmap::fromImage(imgIn);
    emit updateCameraImageROS(myPixMap);
}

void rosHandlerGui::updateDesiredState(double desiredHeight, double desiredRoll, double desiredPitch, double desiredYaw,
                                       double desiredXMovement, double desiredYMovement, bool holdPosition) {
    this->desiredHeight = desiredHeight;
    this->desiredRoll = desiredRoll;
    this->desiredPitch = desiredPitch;
    this->desiredXMovement = desiredXMovement;
    this->desiredYaw = desiredYaw;
    this->desiredYMovement = desiredYMovement;
}

