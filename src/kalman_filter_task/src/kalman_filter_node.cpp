#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <stdio.h>

void receive_odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
    ROS_INFO("Seq: [%d]", msg->header.seq);
}

int main(int argc, char **argv){

    ros::init(argc, argv, "kalman_filter_node");
    ros::NodeHandle my_node; 
    ros::Subscriber pose_subscriber = my_node.subscribe("/odom",10, receive_odom_callback);
    ros::spin();
    return 0;
}