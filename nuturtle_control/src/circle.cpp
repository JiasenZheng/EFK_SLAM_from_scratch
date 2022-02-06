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


}