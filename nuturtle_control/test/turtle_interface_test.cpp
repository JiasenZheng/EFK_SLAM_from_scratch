/**
 * \file turtle_interface_test.cpp
 * \author Jiasen Zheng (jiasenzheng2020@u.northwestern)
 * \brief ROS API test for turtle_interface
 * \version 0.1
 * \date 2022-02-08
 * 
 * 
**/
#include <catch_ros/catch.hpp>
#include <ros/ros.h>
#include <turtlelib/rigid2d.hpp>
#include <turtlelib/diff_drive.hpp>
#include <nuturtlebot_msgs/WheelCommands.h>
#include <nuturtlebot_msgs/SensorData.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

using turtlelib::almost_equal;
using turtlelib::PI;

static int rate = 200;
static int left_velocity;
static int right_velocity;
static double js_left;
static double js_right;

void cmd_callback(const nuturtlebot_msgs::WheelCommands::ConstPtr &msg)
{
    left_velocity = msg->left_velocity;
    right_velocity = msg->right_velocity;
}


TEST_CASE("pure translation","[nuturtle control]")
{
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/wheel_cmd",100,cmd_callback);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10,true);

    
    geometry_msgs::Twist twist;
    twist.linear.x = 0.12;
    pub.publish(twist);
    ros::Rate loop_rate(rate);
    for(int i = 0; i< rate; i++)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    CHECK(left_velocity == 139);
    CHECK(right_velocity == 139);
}

TEST_CASE("pure rotation","[nuturtle control]")
{
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/wheel_cmd",100,cmd_callback);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",100,true);

    
    geometry_msgs::Twist twist;
    twist.angular.z = 0.50;
    pub.publish(twist);
    ros::Rate loop_rate(rate);
    for(int i = 0; i< rate; i++)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    CHECK(left_velocity == -46);
    CHECK(right_velocity == 46);
}

void js_callback(const sensor_msgs::JointState::ConstPtr &msg)
{
    js_left = msg->position[0];
    js_right = msg->position[1];
}

TEST_CASE("encoder data to joint state","[nuturtle control]")
{
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/joint_states",100,js_callback);
    ros::Publisher pub = nh.advertise<nuturtlebot_msgs::SensorData>("/sensor_data",100,true);

    nuturtlebot_msgs::SensorData sd;
    sd.left_encoder = 0;
    sd.right_encoder = 4096;
    pub.publish(sd);
    ros::Rate loop_rate(rate);
    for(int i = 0; i< rate; i++)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    CHECK(js_left == 0.0);
    CHECK(almost_equal(js_right,2*PI,0.01));
}

