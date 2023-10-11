//
// Created by tim-linux on 22.12.21.
//
#include "controllerOfBluerov2.h"


//#include "mavros_msgs/CommandBool.h"
//class rosNode {
//public:
//    rosNode(rclcpp::NodeHandle n_){
//
//    }
//
//
//
//};
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<controllerOfBluerov2>());
    rclcpp::shutdown();

//    rclcpp::ServiceClient clientArming;
//    clientArming = n_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
//    dynamic_reconfigure::Server<bluerov2common::controllerConfig> server;
//    dynamic_reconfigure::Server<bluerov2common::controllerConfig>::CallbackType f;
//
//    f = boost::bind(&controllerOfBluerov2::callbackReconfiguration,&objectOfController, _1, _2);
//    rclcpp::Publisher vis_pub = n_.



//    server.setCallback(f);
//    rclcpp::Rate rate(30);
////    rclcpp::spin();
//    while(rclcpp::ok()){
//        Eigen::Vector3d thrustVector = objectOfController.controllLogic();
//
//        visualization_msgs::Marker marker;
//        marker.header.frame_id = "map_ned";
//        marker.header.stamp = rclcpp::Time();
//        marker.id = 0;
//        marker.type = visualization_msgs::Marker::ARROW;
//        marker.action = visualization_msgs::Marker::ADD;
//
//        Eigen::Vector3d position;
//        Eigen::Quaterniond rotation;
//        objectOfController.getPoseRobot(position,rotation);
//        geometry_msgs::Point p;
//        p.x=position.x();
//        p.y=position.y();
//        p.z=position.z();
//        marker.points.push_back(p);
//        p.x=position.x()+thrustVector.x();
//        p.y=position.y()+thrustVector.y();
//        p.z=position.z()+thrustVector.z();
//        marker.points.push_back(p);
//
//
//        marker.scale.x = 0.06;
//        marker.scale.y = 0.06;
//        marker.scale.z = 0.06;
//        marker.color.a = 1.0; // Don't forget to set the alpha!
//        marker.color.r = 0.0;
//        marker.color.g = 1.0;
//        marker.color.b = 0.0;
//        vis_pub.publish(marker);
//
//        objectOfController.getPoseTarget(position,rotation);
//        marker.header.frame_id = "map_ned";
//        marker.header.stamp = rclcpp::Time();
//        marker.id = 1;
//        marker.type = visualization_msgs::Marker::SPHERE;
//        marker.action = visualization_msgs::Marker::ADD;
//
//
//        marker.pose.position.x=position.x();
//        marker.pose.position.y=position.y();
//        marker.pose.position.z=position.z();
//        marker.pose.orientation.w =1;
//
//
//        marker.scale.x = 0.06;
//        marker.scale.y = 0.06;
//        marker.scale.z = 0.06;
//        marker.color.a = 0.8; // Don't forget to set the alpha!
//        marker.color.r = 1.0;
//        marker.color.g = 0.0;
//        marker.color.b = 0.0;
//        vis_pub.publish(marker);
//
//
//
//        rate.sleep();
//        rclcpp::spinOnce();
//    }

//    mavros_msgs::CommandBool tmpMSG;
//    tmpMSG.request.value = false;
//    clientArming.call(tmpMSG);

    return (0);
}


