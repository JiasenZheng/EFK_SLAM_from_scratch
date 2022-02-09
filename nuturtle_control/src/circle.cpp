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
#include <std_srvs/Empty.h>
#include <std_srvs/EmptyRequest.h>
#include <std_srvs/EmptyResponse.h>
#include <nuturtle_control/control.h>
#include <turtlelib/rigid2d.hpp>

static int frequency = 100;
static ros::Publisher cmd_pub;
static ros::ServiceServer control;
static ros::ServiceServer reverse;
static ros::ServiceServer stop;
static geometry_msgs::Twist twist;

bool control_callback(nuturtle_control::control::Request &req, nuturtle_control::control::Response &res)
{
    twist.linear.x = req.velocity;
    if (turtlelib::almost_equal(req.radius,0.0))
    {
        throw std::logic_error("radius should not be zero.");
    }
    twist.angular.z = req.velocity/req.radius;
    return true;
}

bool reverse_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    twist.linear.x = -twist.linear.x;
    twist.angular.z = -twist.angular.z;
    return true;
}

bool stop_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"circle");
    ros::NodeHandle nh;

    // Initialize publisher and services
    cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",100);
    control = nh.advertiseService("/control",control_callback);
    reverse = nh.advertiseService("/reverse",reverse_callback);
    stop = nh.advertiseService("/stop",stop_callback);

    // Initialize twist
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;

    ros::Rate loop_rate(frequency);
    while(ros::ok())
    {
        cmd_pub.publish(twist);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}