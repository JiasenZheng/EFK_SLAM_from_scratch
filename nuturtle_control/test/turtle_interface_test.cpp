#include <catch_ros/catch.hpp>
#include <ros/ros.h>
#include <turtlelib/rigid2d.hpp>
#include <turtlelib/diff_drive.hpp>
#include <nuturtlebot_msgs/WheelCommands.h>
#include <nuturtlebot_msgs/SensorData.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

static int rate = 200;
static int left_velocity;
static int right_velocity;

void cmd_callback(const nuturtlebot_msgs::WheelCommands msg)
{
    CHECK(msg.left_velocity == 19);
    CHECK(msg.right_velocity == 139);
}


TEST_CASE("pure translation","[nuturtle control]")
 {
    //setup
    ros::NodeHandle nh;
    const ros::Subscriber sub = nh.subscribe("/wheel_cmd",10,cmd_callback);
    const ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10,true);

    //publish
    geometry_msgs::Twist twis;
    twis.linear.x = 0.12;
    pub.publish(twis);

    //spin
    ros::Rate loop_rate(rate);
    for(int i = 0; ros::ok() && i!=200; i++)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

}