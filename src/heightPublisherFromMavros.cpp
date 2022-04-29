//
// Created by tim-linux on 21.04.22.
//

//out.baro_alt_meter =


//
// Created by jurobotics on 28.09.21.
//
#include <ros/ros.h>
//#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/FluidPressure.h"
#include "commonbluerovmsg/heightStamped.h"
//#include "mavros_msgs/Altitude.h"
//#include "geometry_msgs/TwistStamped.h"
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

static constexpr double CONSTANTS_ONE_G = 9.80665;

class rosConversionClass {
public:
    rosConversionClass(ros::NodeHandle n_, std::string heightMode){
        this->firstMessage = true;
        subscriberPx4Pressure = n_.subscribe("mavros/imu/static_pressure", 1000, &rosConversionClass::heightCallback, this);
        publisherPx4Height = n_.advertise<commonbluerovmsg::heightStamped>("height_baro", 1000);

        if (heightMode=="simulation"){
            simulationCalculation = true;
        }else{
            simulationCalculation = false;
        }
    }

private:
    ros::Subscriber subscriberPx4Pressure;
    ros::Publisher publisherPx4Height;
    double pressureWhenStarted;
    bool firstMessage;
    bool simulationCalculation;

    void heightCallback(const sensor_msgs::FluidPressure::ConstPtr &msg){
        if(this->firstMessage){
            this->pressureWhenStarted = msg->fluid_pressure;
            this->firstMessage = false;
            return;
        }
        commonbluerovmsg::heightStamped newMsg;
        newMsg.header.stamp = msg->header.stamp;
        if(simulationCalculation){
           // std::cout << "calculating based on simulation" << std::endl;
            newMsg.height = ((msg->fluid_pressure-this->pressureWhenStarted)*1000.0f)/(CONSTANTS_ONE_G*1000.0f);//not sure if correct in real/ simulation / im simulation 1000.0f instead
        }else{
            newMsg.height = ((msg->fluid_pressure-this->pressureWhenStarted)*1.0f)/(CONSTANTS_ONE_G*1000.0f);//not sure if correct in real/ simulation / im simulation 1000.0f instead
        }
        publisherPx4Height.publish(newMsg);
    }
};



int main(int argc, char **argv) {
    ros::init(argc, argv, "baro_to_height");
    ros::start();
    ros::NodeHandle n_;



    std::string heightMode;

    if (n_.getParam("/mavrosHeightCalculation/height_mode", heightMode))
    {
        ROS_INFO("Height Mode used is: %s", heightMode.c_str());
    }
    else
    {
        std::vector<std::string> keys;
        n_.getParamNames(keys);

        for(int i = 0;i<keys.size();i++){
            std::cout << keys[i]<< std::endl;
        }
        ROS_ERROR("Failed to get heightMode parameter, which to use");
    }

    if(heightMode != "simulation" && heightMode != "real"){
        ROS_ERROR("You have to use simulation or real as parameter for heightMode");
        exit(-1);
    }



    rosConversionClass tmpClass(n_,heightMode);//heightMode);


    ros::spin();


    return (0);
}
