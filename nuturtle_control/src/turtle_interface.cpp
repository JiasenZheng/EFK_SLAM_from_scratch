/// \file turtle_interface.cpp
/// \author Jiasen Zheng (jiasenzheng2020@u.northwestern.edu)
/// \brief Enables control of the robot turtlebot using twist command
/// \version 0.1
/// \date 2022-02-04
///  
/// PARAMETERS:
///     wr (double): radius of the wheel
///     wt (double): distance between the two wheels
/// PUBLISHERS:
///     wc_pub (nuturtlebot_msgs::WheelCommands-> /wheel_cmd): wheel command that cause the turtlebot to follow the specific twist
///     js_pub (sensor_msgs::JointState-> /joint_states): joint states to provide the angle (rad) and velocity (rad/s)
/// SUBSCRIBERS:
///     cmd_sub (geometry_msgs::Twist-> /cmd_vel): get the twist command
///     sensor_sub (nuturtlebot_msgs::SensorData-> /sensor_data): get the sensor data

#include <ros/ros.h>
#include <turtlelib/rigid2d.hpp>
#include <turtlelib/diff_drive.hpp>
#include <nuturtlebot_msgs/WheelCommands.h>
#include <nuturtlebot_msgs/SensorData.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

static ros::Publisher wc_pub;
static ros::Publisher js_pub;
static ros::Subscriber cmd_sub;
static ros::Subscriber sensor_sub;

static turtlelib::DiffDrive dd;
static turtlelib::Transform2D tf = turtlelib::Transform2D();
static double wr;
static double wt;



/// \brief callback function for cmd subscriber
/// \param twist input twist command
void cmd_callback(const geometry_msgs::TwistConstPtr &twist)
{
    // store twist info
    turtlelib::Twist2D t;
    t.omega = twist->angular.z;
    t.x_dot = twist->linear.x;
    t.y_dot = twist->linear.y;
    // convert twist to velocity using inverse kinematics
    turtlelib::Velocity vel;
    vel = dd.inverse_kinematics(t);
    // convert and store velocity in nuturtlebot::WheelCommands
    nuturtlebot_msgs::WheelCommands wc;
    wc.left_velocity = 256*vel.left*wr/2.2;
    wc.right_velocity = 256*vel.right*wr/2.2;
    // publishe wheel command 
    wc_pub.publish(wc);
}

void sensor_callback(const nuturtlebot_msgs::SensorData &sensor_data)
{

}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"turtle_interface");
    ros::NodeHandle nh;
    //define parameters
    nh.getParam("/wheel_radius",wr);
    nh.getParam("/track_width",wt);

    //initialize publishers and subscribers
    wc_pub = nh.advertise<nuturtlebot_msgs::WheelCommands>("/wheel_cmd",100);
    js_pub = nh.advertise<sensor_msgs::JointState>("/joint_states",100);
    cmd_sub = nh.subscribe("/cmd_vel",100,cmd_callback);
    sensor_sub = nh.subscribe("/red/sensor_data",100,sensor_callback);

    //DiffDrive 
    dd = turtlelib::DiffDrive(wr,wt,tf);
}