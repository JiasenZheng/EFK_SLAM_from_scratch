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
#include <nusim/Telep.h>

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
static ros::ServiceServer telep_service;
static sensor_msgs::JointState js;
static double x_0;
static double y_0;
static double theta_0;
static double x;
static double y;
static double theta;





bool reset_callback(std_srvs::Trigger::Request &req,
                    std_srvs::Trigger::Response &res)
{
    count = 0;
    x = x_0;
    y = y_0;
    theta = theta_0;
    res.success = 1;
    res.message = "Reset successfully!";
    return true;
}

bool telep_callback(nusim::Telep::Request &req,
                    nusim::Telep::Response &res)
{
    ROS_INFO("X_coordinate: %f  Y_coordinate: %f  Angle_radians: %f", req.x_coord, req.y_coord, req.radians);

    x = req.x_coord;
    y = req.y_coord;
    theta = req.radians;

    return true;
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "nusim");
    ros::NodeHandle nh;
    static tf2_ros::TransformBroadcaster br;
    time_pub = nh.advertise<std_msgs::UInt64>("timestep",100);
    js_pub = nh.advertise<sensor_msgs::JointState>("red/joint_states",100);
    reset_service = nh.advertiseService("reset",reset_callback);
    telep_service = nh.advertiseService("telep",telep_callback);
    ros::Rate loop_rate(rate);
    count = 0;
    js.name.push_back("red-wheel_left_joint");
    js.name.push_back("red-wheel_right_joint");
    js.position.push_back(0);
    js.position.push_back(0);

    nh.param("x0",x,0.0);
    nh.param("y0",y,0.0);
    nh.param("theta0",theta,0.0);

    x_0 = x;
    y_0 = y;
    theta_0 = theta;


    while (ros::ok())
    {
        // construct a transform
        geometry_msgs::TransformStamped trans;
        
        trans.header.stamp = ros::Time::now();
        trans.child_frame_id = "red-base_footprint";
        trans.header.frame_id = "world";

        trans.transform.translation.x = x;
        trans.transform.translation.y = y;
        trans.transform.translation.z = 0;

        tf2::Quaternion q;
        q.setRPY(0,0,theta);
        trans.transform.rotation.x = q.x();
        trans.transform.rotation.y = q.y();
        trans.transform.rotation.z = q.z();
        trans.transform.rotation.w = q.w();
        br.sendTransform(trans); 

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