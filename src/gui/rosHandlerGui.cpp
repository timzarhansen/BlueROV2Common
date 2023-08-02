//
// Created by tim-linux on 14.12.21.
//

#include "rosHandlerGui.h"

void rosHandlerGui::updateConfigSonar(int stepSize, int rangeSonar,int frequencyRange,int numberOfSamples) {
    ping360_sonar::sendingSonarConfig srv;
    srv.request.range = rangeSonar;
    srv.request.stepSize = stepSize;
    srv.request.frequencyRange = frequencyRange;
    srv.request.numberOfSamples = numberOfSamples;

    clientSonar.call(srv);
//    std::cout << "changed sonar Config" << std::endl;
}


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
    this->yawPositionRobot.push_back(rollPitchYaw[2]);
//    std::cout << "test132"<< std::endl;
    emit this->updatePlotPositionVectorROS(this->xPositionRobot, this->yPositionRobot, this->yawPositionRobot);
    emit this->updateStateOfRobotROS(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z,
                                     rollPitchYaw[0], rollPitchYaw[1], rollPitchYaw[2], covariance);
}


void rosHandlerGui::DVLCallback(const waterlinked_dvl::TransducerReportStamped::ConstPtr &msg) {

    emit updateDVLStateROS(msg->report.transducers[0].distance, msg->report.transducers[1].distance,
                           msg->report.transducers[2].distance, msg->report.transducers[3].distance);
}


void rosHandlerGui::updateAngleCamera(int angleCamera) {
    commonbluerovmsg::cameraAngle srv;
    srv.request.angle = angleCamera;
    this->clientCameraAngle.call(srv);
}

void rosHandlerGui::updateLightIntensity(int intensityLight) {
    commonbluerovmsg::lightDensity0to10 srv;
    srv.request.intensity = intensityLight;
    this->clientLight.call(srv);
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

    cv_ptr = cv_bridge::cvtColor(cv_ptr, "rgb8");

    cv::applyColorMap(cv_ptr->image, cv_ptr->image, cv::COLORMAP_JET);
    QImage imgIn = QImage((uchar *) cv_ptr->image.data, cv_ptr->image.cols, cv_ptr->image.rows, cv_ptr->image.step,
                          QImage::Format_RGB888);
//    QImage imgIn = QImage(&msg->data[0], msg->width, msg->height, QImage::Format_RGB888);
    imgIn = imgIn.rgbSwapped();
    QPixmap myPixMap = QPixmap::fromImage(imgIn);
    emit updateSonarImageROS(myPixMap);


}

void rosHandlerGui::cameraImageCallback(const sensor_msgs::CompressedImagePtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_ptr = cv_bridge::cvtColor(cv_ptr, "rgb8");

    QImage imgIn = QImage((uchar *) cv_ptr->image.data, cv_ptr->image.cols, cv_ptr->image.rows, cv_ptr->image.step,
                          QImage::Format_RGB888);

    //QImage::Format_BGR30  Format_RGB888
    QPixmap myPixMap = QPixmap::fromImage(imgIn);

    emit updateCameraImageROS(myPixMap);
}

void rosHandlerGui::updateDesiredState(double desiredHeight, double desiredRoll, double desiredPitch, double desiredYaw,
                                       double desiredXMovement, double desiredYMovement, bool holdPosition) {

    //this just sends the data to ros, the frame rate is made by mainwindow
    commonbluerovmsg::desiredStateForRobot msg;
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

void rosHandlerGui::resetEKFEstimator(bool resetOnlyGraph) {
    if (not resetOnlyGraph) {
        commonbluerovmsg::resetekf srv;
        srv.request.xPos = 0;
        srv.request.yPos = 0;
        srv.request.yaw = 0;
        srv.request.resetCovariances = true;
        this->clientEKF.call(srv);
    }

    this->xPositionRobot.clear();
    this->yPositionRobot.clear();
    this->yawPositionRobot.clear();
}
