//
// Created by tim-linux on 22.12.21.
//
#include "controllerOfBluerov2.h"



int main(int argc, char **argv) {
    ros::init(argc, argv, "ekffordvlwithros");
    ros::start();
    ros::NodeHandle n_;
    controllerOfBluerov2 classOfController(n_);

    ros::spin();


    return (0);
}


