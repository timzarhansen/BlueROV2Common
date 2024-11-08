//
// Created by jurobotics on 13.09.21.
//

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/Imu.h"
#include "mavros_msgs/Altitude.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/PoseStamped.h"
//#include "../slamTools/generalHelpfulTools.h"
#include "waterlinked_dvl/TransducerReportStamped.h"
#include "commonbluerovmsg/resetekf.h"
#include "commonbluerovmsg/heightStamped.h"
//#include <chrono>
#include <thread>
#include "bluerov2common/ekfnoiseConfig.h"
#include <dynamic_reconfigure/server.h>
#include <commonbluerovmsg/stateOfBlueRov.h>
#include <Eigen/Geometry>
#include "generalTools/generalHelpfulTools.h"
#include "math.h"


class rosClassEKF {
public:
    rosClassEKF(ros::NodeHandle n_){


        this->randomPublisher = n_.advertise<commonbluerovmsg::stateOfBlueRov>("testTopic", 10);
        this->subscriberVelocitySimulation = n_.subscribe("mavros/vision_pose/pose", 1000, &rosClassEKF::callback,this);
    }

    void callback(geometry_msgs::PoseStamped msg){
        Eigen::Quaterniond tmpQuat(msg.pose.orientation.w,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z);
        Eigen::Vector3d resultRPY = generalHelpfulTools::getRollPitchYaw(tmpQuat);

        commonbluerovmsg::stateOfBlueRov resultMessage;
        resultMessage.header = msg.header;
        resultMessage.Yaw = std::fmod((double)(-resultRPY[2]+M_PI_2+M_PI),M_PI*2)-M_PI;

        randomPublisher.publish(resultMessage);
    }

    ros::Publisher randomPublisher;
    ros::Subscriber subscriberVelocitySimulation;

};

int main(int argc, char **argv) {
//    std::this_thread::sleep_for(std::chrono::milliseconds(5000));


    ros::init(argc, argv, "testFile");
    ros::start();
    ros::NodeHandle n_;
    rosClassEKF ourClass(n_);



    ros::spin();



    return (0);
}
