/**
 * \file turtle_odom_test.cpp
 * \author Jiasen Zheng (jiasenzheng2020@u.northwestern)
 * \brief ROS API test node for odometry
 * \version 0.1
 * \date 2022-02-08
 * 
 * 
**/
#include <catch_ros/catch.hpp>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <turtlelib/rigid2d.hpp>
#include <turtlelib/diff_drive.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <nuturtle_control/set_pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>

using turtlelib::almost_equal;
using turtlelib::PI;

static int rate = 200;

TEST_CASE("transform from odom to base_footprint","[nuturtle control]")
{
    ros::NodeHandle nh;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate loop_rate(rate);
    for (int i = 0; i<rate ; i++)
    {
        geometry_msgs::TransformStamped transformStamped;
        try
        {
            transformStamped = tfBuffer.lookupTransform("blue-base_footprint", "odom", ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        ros::spinOnce;
        loop_rate.sleep();
    }

}