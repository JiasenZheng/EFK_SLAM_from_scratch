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
 *      path_pub (nav_msgs::Path -> /blue_path) publish the path of odometry turtlebot
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
#include <nav_msgs/Path.h>

static const int rate = 500;
static ros::Publisher odom_pub;
static ros::Publisher path_pub;
static ros::Subscriber js_sub;
static ros::ServiceServer set_pose;
static std::string body_id;
static std::string odom_id;
static std::string wheel_left;
static std::string wheel_right;
static double wt;
static double wr;
static turtlelib::Transform2D tf = turtlelib::Transform2D();
static turtlelib::DiffDrive dd;
static sensor_msgs::JointState js;
static nav_msgs::Odometry odom;
static nav_msgs::Path path;
static turtlelib::Twist2D twist;
static geometry_msgs::Quaternion rot;
static tf2::Quaternion q;
static geometry_msgs::TransformStamped trans;

/**
 * \brief the callback function fot joint state subscriber
 * 
 * \param js joint states
**/
void js_callback(const sensor_msgs::JointStateConstPtr &js)
{
    turtlelib::Velocity vel;
    vel.left = js->velocity[0];
    vel.right = js->velocity[1];
    // ROS_INFO("velocity: %f  %f", vel.left, vel.right);
    // update twists and configurations
    twist = dd.calculate_twist(vel);
    dd.update_config(vel);
}

/**
 * \brief the callback function for pose service
 * 
 * \param req set the x, y, and theta for the turtlebot
 * \param res empty
 * \return true success
 * \return false fail
**/
bool set_pose_callback(nuturtle_control::set_pose::Request &req, nuturtle_control::set_pose::Response &res)
{
    turtlelib::Transform2D tf;
    tf = turtlelib::Transform2D(req.x,req.y,req.theta);
    // update DiffDrive
    dd = turtlelib::DiffDrive(wr,wt,tf);
    return true;
}

/**
 * \brief Publish odometry
 * 
**/
void publish_odom()
{
    odom.header.stamp = ros::Time::now();
    odom.twist.twist.angular.z = twist.omega;
    odom.twist.twist.linear.x = twist.x_dot;
    odom.twist.twist.linear.y = twist.y_dot;
    odom.pose.pose.position.x = dd.get_trans().get_x();
    odom.pose.pose.position.y = dd.get_trans().get_y();
    q.setRPY( 0, 0, dd.get_trans().rotation() );
    rot = tf2::toMsg(q);
    odom.pose.pose.orientation = rot;
    odom_pub.publish(odom);
}

/**
 * \brief broadcast odometry
 * 
**/
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


/// \brief publish path for blue robot
void publish_path()
{
    std_msgs::Header h;
    geometry_msgs::PoseStamped p;
    tf2::Quaternion rot;

    h.stamp = ros::Time::now();
    h.frame_id = "world";
    p.header = h;
    path.header = h;
    p.pose.position.x = dd.get_trans().get_x();
    p.pose.position.y = dd.get_trans().get_y();
    p.pose.position.z = 0;

    rot.setRPY(0,0,dd.get_trans().rotation());
    p.pose.orientation = tf2::toMsg(rot);

    path.poses.push_back(p);
    path_pub.publish(path);
}
  

int main(int argc, char** argv)
{
    ros::init(argc,argv,"odometry");
    ros::NodeHandle nh;

    // get parameters

    if (nh.getParam("/wheel_radius", wr))
    {
      ROS_INFO_STREAM("Got param:" << " wheel_radius");
    }
    else
    {
      ROS_ERROR_STREAM("Failed to get param 'wheel_radius'");
    }

    if (nh.getParam("/track_width", wt))
    {
      ROS_INFO_STREAM("Got param:" << " track_width");
    }
    else
    {
      ROS_ERROR_STREAM("Failed to get param 'track_width'");
    }

    if (nh.getParam("/odom", odom_id))
    {
      ROS_INFO_STREAM("Got param:" << " odom");
    }
    else
    {
      ROS_ERROR_STREAM("Failed to get param 'odom'");
      ros::shutdown();
    }

    if (nh.getParam("/body", body_id))
    {
      ROS_INFO_STREAM("Got param:" << " body");
    }
    else
    {
      ROS_ERROR_STREAM("Failed to get param 'body'");
      ros::shutdown();
    }

    if (nh.getParam("/left_wheel", wheel_left))
    {
      ROS_INFO_STREAM("Got param:" << " left_wheel");
    }
    else
    {
      ROS_ERROR_STREAM("Failed to get param 'left_wheel'");
      ros::shutdown();
    }

    if (nh.getParam("/right_wheel", wheel_right))
    {
      ROS_INFO_STREAM("Got param:" << " right_wheel");
    }
    else
    {
      ROS_ERROR_STREAM("Failed to get param 'right_wheel'");
      ros::shutdown();
    }

    // initialize publishers, subscribers, services and br
    js_sub = nh.subscribe("/joint_states",100,js_callback);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom",100);
    path_pub = nh.advertise<nav_msgs::Path>("/blue_path",100);
    set_pose = nh.advertiseService("set_pose",set_pose_callback);
    // static tf2_ros::TransformBroadcaster br;

    // DiffDrive
    dd = turtlelib::DiffDrive(wr,wt,tf);

    // set up odom
    odom.header.frame_id = odom_id;
    odom.child_frame_id = body_id;

    // set up trans
    trans.header.frame_id = odom_id;
    trans.child_frame_id = body_id;

    // Initialize joint states
    js.name.push_back(wheel_left);
    js.name.push_back(wheel_right);
    js.position.push_back(0);
    js.position.push_back(0);
    js.velocity.push_back(0);
    js.velocity.push_back(0);
    
    ros::Rate loop_rate(rate);
    while(ros::ok())
    {
        publish_odom();
        publish_path();
        broadcast_odom();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
