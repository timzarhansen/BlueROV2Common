//
// Created by tim-linux on 22.12.21.
//
#include "controllerOfBluerov2.h"


//class rosNode {
//public:
//    rosNode(ros::NodeHandle n_){
//
//    }
//
//
//
//};
int main(int argc, char **argv) {
    ros::init(argc, argv, "controllerofbluerov");

    ros::start();
    ros::NodeHandle n_;
    controllerOfBluerov2 objectOfController(n_);

    dynamic_reconfigure::Server<bluerov2common::controllerConfig> server;
    dynamic_reconfigure::Server<bluerov2common::controllerConfig>::CallbackType f;

    f = boost::bind(&controllerOfBluerov2::callbackReconfiguration,&objectOfController, _1, _2);

    server.setCallback(f);
    ros::Rate rate(30);
//    ros::spin();
    while(ros::ok()){
        objectOfController.controllLogic();

        rate.sleep();
        ros::spinOnce();
    }

    return (0);
}


