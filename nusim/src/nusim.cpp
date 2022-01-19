#include <ros/ros.h>
#include <std_msgs/UInt64.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/console.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/**
 * @brief to simulate and visualize the turtlebot in Rviz
 * 
 * PUBLISHERS:
 *      time_pub (std_msgs::UInt64; timestep): current timestep of the simulation
 *      js_pub (sensor_msgs::JointState; red/joint_states): simulated joint states
 *      array_pub (visualization_msgs::MarkerArray; obstacles): add cylindrical obstacles to the environment
 *      
 *      
 * BROADCASTERS:
 *      tf: broadcast a transform between the world frame and red::base_footprint
 *      
 * SERVICES:
 *      reset_service (reset): restores the initial state of the simulation
 *      telep_service (telep): enable moving the robot to a desired (x,y,theta) pose
 * 
 */

static const int rate = 500; 
static std_msgs::UInt64 timestep;
static ros::Publisher time_pub;
static ros::Publisher js_pub;
static long count;
static ros::ServiceServer reset_service;
static sensor_msgs::JointState js;
static tf2_ros::TransformBroadcaster br;
// static std::string js_left;
// static std::string js_right;



bool reset_callback(std_srvs::Trigger::Request &req,
                    std_srvs::Trigger::Response &res)
{
    count = 0;
    // timestep.data = 0;
    ROS_INFO("Timestep reset!");
    res.success = 1;
    res.message = "Timestep reset!";
    return true;
}

// void poseCallback()
// {
//     static tf2_ros::TransformBroadcaster br;
    
// }


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "nusim");
    ros::NodeHandle nh;
    time_pub = nh.advertise<std_msgs::UInt64>("timestep",100);
    js_pub = nh.advertise<sensor_msgs::JointState>("red/joint_states",100);
    reset_service = nh.advertiseService("reset",reset_callback);
    ros::Rate loop_rate(rate);
    count = 0;
    // js.header.stamp = ros::Time::now();
    js.name.push_back("red-wheel_left_joint");
    js.name.push_back("red-wheel_right_joint");
    js.position.push_back(0);
    js.position.push_back(0);
    while (ros::ok())
    {
        count++;
        js.header.stamp = ros::Time::now();
        timestep.data = count/rate;
        time_pub.publish(timestep);
        js_pub.publish(js);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}