/**
 * \file odometry.cpp
 * \author Jiasen Zheng (jiasenzheng2020@u.northwestern)
 * \brief 
 * \version 0.1
 * \date 2022-02-05
 * 
 * PARAMETERS:
 *      body_id (string): the name of the body frame of the robot
 *      odom_id (string): the name of the odometry frame
 *      wheel_left (string): the name of the left wheel joint
 *      wheel_right (string): the name of the right wheel joint
 * PUBLISHERS:
 *      odom_pub (nav_msgs::Odometry-> /odom) odometry messages
 * SUBSCRIBERS:
 *      js_sub (sensor_msgs::JointState-> /joint_states) joint states
 * SERVICES:
 *      set_pose: set a pose for the turtlebot
 * BROADCASTER:
 *      odom_br: transform between odom_id and body_id
 * 
**/

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

static const int rate = 500;
static ros::Publisher odom_pub;
static ros::Subscriber js_sub;
static ros::ServiceServer set_pose;
static std::string body_id;
static std::string odom_id;
static std::string wheel_left;
static std::string wheel_right;
static double wt;
static double wr;
static turtlelib::Transform2D tf = turtlelib::Transform2D();
static turtlelib::Position p = turtlelib::Position();
static turtlelib::DiffDrive dd;
static sensor_msgs::JointState js;
static nav_msgs::Odometry odom;
static turtlelib::Twist2D twist;
static geometry_msgs::Quaternion rot;
static tf2::Quaternion q;
static geometry_msgs::TransformStamped trans;


void js_callback(const sensor_msgs::JointStateConstPtr &js)
{
    turtlelib::Velocity vel;
    vel.left = js->velocity[0];
    vel.right = js->velocity[1];
    // update twists and configurations
    twist = dd.forward_kinematics(vel);
    dd.update_config(vel);
}

bool set_pose_callback(nuturtle_control::set_pose::Request &req, nuturtle_control::set_pose::Response &res)
{
    turtlelib::Transform2D tf;
    tf = turtlelib::Transform2D(req.x,req.y,req.theta);
    // update DiffDrive
    dd = turtlelib::DiffDrive(wr,wt,tf,p);
    return true;
}

void publish_odom()
{
    odom.header.stamp = ros::Time::now();
    odom.twist.twist.angular.z = twist.omega;
    odom.twist.twist.linear.x = twist.x_dot;
    odom.twist.twist.linear.y = twist.y_dot;
    odom.pose.pose.position.x = dd.get_trans().get_x();
    odom.pose.pose.position.y = dd.get_trans().get_y();
    q.setRPY( 0, 0, dd.get_trans().get_theta() );
    rot = tf2::toMsg(q);
    odom.pose.pose.orientation = rot;
}

void broadcast_odom()
{
    static tf2_ros::TransformBroadcaster br;
    // define trans
    trans.header.stamp = ros::Time::now();
    trans.transform.translation.x = dd.get_trans().get_x();
    trans.transform.translation.y = dd.get_trans().get_y();
    trans.transform.translation.z = 0.0;
    trans.transform.rotation = rot;

    // broadcast
    br.sendTransform(trans);

}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"odometry");
    ros::NodeHandle nh;

    // get parameters
    nh.getParam("/wheel_radius",wr);
    nh.getParam("/track_width",wt);
    odom_id = "odom";                   //
    body_id = "base_footprint";         // 

    // initialize publishers, subscribers, services and br
    js_sub = nh.subscribe("/joint_states",100,js_callback);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom",100);
    set_pose = nh.advertiseService("set_pose",set_pose_callback);
    // static tf2_ros::TransformBroadcaster br;

    // DiffDrive
    dd = turtlelib::DiffDrive(wr,wt,tf,p);

    // set up odom
    odom.header.frame_id = odom_id;
    odom.child_frame_id = body_id;

    // set up trans
    trans.header.frame_id = odom_id;
    trans.child_frame_id = body_id;

    // Initialize joint states
    js.name.push_back("blue-wheel_left_joint");
    js.name.push_back("blue-wheel_right_joint");
    js.position.push_back(0);
    js.position.push_back(0);
    js.velocity.push_back(0);
    js.velocity.push_back(0);
    
    ros::Rate loop_rate(rate);
    while(ros::ok())
    {
        publish_odom();
        broadcast_odom();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
