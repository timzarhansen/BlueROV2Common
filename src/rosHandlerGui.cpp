//
// Created by tim-linux on 14.12.21.
//

#include "rosHandlerGui.h"


void rosHandlerGui::positionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg){
//    std::cout << "test1332"<< std::endl;
    this->xPositionRobot.append(msg->pose.pose.position.x);
    this->yPositionRobot.append(msg->pose.pose.position.y);

//    std::cout << "test132"<< std::endl;
    emit this->updatePositionsROS(this->xPositionRobot ,this->yPositionRobot ,this->yawPositionRobot);
}

void  rosHandlerGui::setSonarRange(double range){

}

void rosHandlerGui::sonarImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    QImage imgIn= QImage((uchar*) cv_ptr->image.data, cv_ptr->image.cols, cv_ptr->image.rows, cv_ptr->image.step, QImage::Format_RGB888);
    QPixmap myPixMap = QPixmap::fromImage( imgIn );
    emit updateSonarImageROS(myPixMap);
}
void rosHandlerGui::cameraImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    QImage imgIn= QImage((uchar*) cv_ptr->image.data, cv_ptr->image.cols, cv_ptr->image.rows, cv_ptr->image.step, QImage::Format_RGB888);
    QPixmap myPixMap = QPixmap::fromImage( imgIn );
    emit updateCameraImageROS(myPixMap);
}
