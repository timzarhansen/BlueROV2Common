//
// Created by tim-linux on 22.12.21.
//
#include "controllerOfBluerov2.h"



int main(int argc, char **argv) {
    ros::init(argc, argv, "controllerofbluerov");
    ros::start();
    ros::NodeHandle n_;
    controllerOfBluerov2 classOfController(n_);
    ros::Rate rate(30);
//    ros::spin();
    while(ros::ok()){
        classOfController.controllLogic();

        rate.sleep();
        ros::spinOnce();
    }

    return (0);
}


