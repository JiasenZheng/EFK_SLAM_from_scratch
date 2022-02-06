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

static ros::Publisher cmd_pub;
