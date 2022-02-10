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
#include <algorithm>

static const int rate = 500;
static ros::Publisher wc_pub;
static ros::Publisher js_pub;
static ros::Subscriber cmd_sub;
static ros::Subscriber sensor_sub;

static turtlelib::DiffDrive dd;
static double wr = 0.0;
static double wt = 0.0;
static double et_to_rad;
static sensor_msgs::JointState js;
static bool first = true;
static nuturtlebot_msgs::SensorData first_data;
static nuturtlebot_msgs::SensorData previous_data;
static nuturtlebot_msgs::SensorData current_data;



/// \brief callback function for cmd subscriber
/// \param twist input twist command
void cmd_callback(const geometry_msgs::TwistConstPtr &twist)
{
    // store twist info
    turtlelib::Twist2D t;
    t.omega = twist->angular.z;
    t.x_dot = twist->linear.x;
    t.y_dot = twist->linear.y;
    // ROS_INFO("x_dot: %f",t.x_dot);
    // ROS_INFO("y_dot: %f",t.y_dot);
    // ROS_INFO("omega: %f",t.omega);

    // convert twist to velocity using inverse kinematics
    turtlelib::Velocity vel;
    vel = dd.inverse_kinematics(t);
    // convert to ticks and store velocity in nuturtlebot::WheelCommands
    nuturtlebot_msgs::WheelCommands wc;
    wc.left_velocity =  (vel.left/0.024);
    wc.right_velocity =  (vel.right/0.024);
    // ROS_INFO("Left vel in rad/sec: %f",vel.left);
    // ROS_INFO("Left vel in tick: %i",wc.left_velocity);
    // ROS_INFO("Right vel in rad/sec: %f",vel.right);
    // ROS_INFO("Right vel in tick: %i",wc.right_velocity);
    // publishe wheel command 
    wc_pub.publish(wc);
}


void sensor_callback(const nuturtlebot_msgs::SensorDataConstPtr &sensor_data)
{
    js.header.stamp = ros::Time::now();
    if (first)
    {
        first_data = *sensor_data;
        previous_data = *sensor_data;
        first = false;
    }
    else
    {
        current_data = *sensor_data;
        // compare to the first data to get positions
        js.position[0] = current_data.left_encoder*et_to_rad - first_data.left_encoder*et_to_rad;
        js.position[1] = current_data.right_encoder*et_to_rad - first_data.right_encoder*et_to_rad;
        // compare to the previous data to compute the velocities
        js.velocity[0] = (current_data.left_encoder*et_to_rad - previous_data.left_encoder*et_to_rad);
        js.velocity[1] = (current_data.right_encoder*et_to_rad - previous_data.right_encoder*et_to_rad);
        previous_data = current_data;
        if (js.velocity[0] > 3)
        {
            js.velocity[0] -= 2*turtlelib::PI;
        }
        else if (js.velocity[0] < -3)
        {
            js.velocity[0] += 2*turtlelib::PI;
        }

        if (js.velocity[1] > 3)
        {
            js.velocity[1] -= 2*turtlelib::PI;
        }
        else if (js.velocity[1] < -3)
        {
            js.velocity[1] += 2*turtlelib::PI;
        }
        
    }
    // ROS_INFO("Sensor Data: %d  %d",sensor_data->left_encoder,sensor_data->right_encoder);
    // ROS_INFO("JS_Position: %f  %f ",js.position[0],js.position[1]);
    // ROS_INFO("JS_Velocity: %f  %f ",js.velocity[0],js.velocity[1]);
    js_pub.publish(js);
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"turtle_interface");
    ros::NodeHandle nh;
    //define parameters
    nh.getParam("/wheel_radius",wr);
    nh.getParam("/track_width",wt);
    nh.getParam("/encoder_ticks_to_rad",et_to_rad);

    //initialize publishers and subscribers
    wc_pub = nh.advertise<nuturtlebot_msgs::WheelCommands>("/wheel_cmd",100);
    js_pub = nh.advertise<sensor_msgs::JointState>("/joint_states",100);
    cmd_sub = nh.subscribe("/cmd_vel",100,cmd_callback);
    sensor_sub = nh.subscribe("/sensor_data",100,sensor_callback);

    //DiffDrive 
    turtlelib::Transform2D tf = turtlelib::Transform2D();
    dd = turtlelib::DiffDrive(wr,wt,tf);

    // Initialize joint states
    js.name.push_back("red-wheel_left_joint");
    js.name.push_back("red-wheel_right_joint");
    js.position.push_back(0);
    js.position.push_back(0);
    js.velocity.push_back(0);
    js.velocity.push_back(0);


    ros::Rate loop_rate(rate);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}