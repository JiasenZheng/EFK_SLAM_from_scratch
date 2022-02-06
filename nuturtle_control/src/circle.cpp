/**
 * \file circle.cpp
 * \author Jiasen Zheng (jiasenzheng2020@u.northwestern)
 * \brief 
 * \version 0.1
 * \date 2022-02-06
 * 
 * PUBLISHERS:
 *      cmd_pub: (geometry_msgs::Twist -> /cmd_vel): cause the turtlebot to drive in a circle
 * SERVICES:
 *      control: control the velocity of the turtlebot
 *      reverse: reverse the turtlebot
 *      stop: stop the turtlebot
**/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

static int frequency = 100;
static ros::Publisher cmd_pub;
static ros::ServiceServer control;
static ros::ServiceServer reverse;
static ros::ServiceServer stop;
static geometry_msgs::Twist twist;



int main(int argc, char** argv)
{
    ros::init(argc,argv,"circle");
    ros::NodeHandle nh;

    // Initialize publisher and services
    cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",100);
    control = nh.advertiseService("/control",control_callback);
    reverse = nh.advertiseService("/reserve",reverse_callback);
    stop = nh.advertiseService("/stop",stop_callback);

    ros::Rate loop_rate(frequency);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}