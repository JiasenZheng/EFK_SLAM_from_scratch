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
#include <nuturtlebot_msgs/WheelCommands.h>
#include <nuturtlebot_msgs/SensorData.h>
#include "nusim/Teleport.h"
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"


/// @brief to simulate and visualize the turtlebot in Rviz
/// 
/// PUBLISHERS:
///      time_pub (std_msgs::UInt64; timestep): current timestep of the simulation
///      js_pub (sensor_msgs::JointState; red/joint_states): simulated joint states
///      obs_pub (visualization_msgs::MarkerArray; obstacles): add cylindrical obstacles to the environment
///      walls_pub (visualization_msgs::MarkerArray; walls): add walls to the environment
///      wp_pub (nuturtlebot_msgs::SeneorData; red/sensor_data): update the wheel positions
///
/// SUBSCRIBERS:
///      wc_sub (nuturtlebot_msgs::WheelCommands; red/wheel_cmd): to receive motion commands for the turtlebot
///      
/// BROADCASTERS:
///      tf: broadcast a transform between the world frame and red::base_footprint
///      
/// SERVICES:
///      reset_service (reset): restores the initial state of the simulation
///      teleport_service (teleport): enable moving the robot to a desired (x,y,theta) pose


static const int rate = 500; 
static std_msgs::UInt64 timestep;
static ros::Publisher time_pub;
static ros::Publisher js_pub;
static ros::Publisher obs_pub;
static ros::Publisher wp_pub;
static ros::Publisher walls_pub;
static ros::Subscriber wc_sub;
static long count;
static ros::ServiceServer reset_service;
static ros::ServiceServer teleport_service;
static sensor_msgs::JointState js;
static double x_0;
static double y_0;
static double theta_0;
static double x;
static double y;
static double theta;
static double x_length = 2.0;
static double y_length = 3.0;




/// @brief The call back function for reset service to rest the timestep and the position of the robot
/// 
/// @param req An empty request
/// @param res An response contains a message and tells if succeed
/// @return true if the call back implemented successfully

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

/// \brief 
/// 
/// \param wc 
void wc_sub_callback(const nuturtlebot_msgs::WheelCommands wc)
{
    ROS_INFO("WC_SUB_Callback");
}

/// @brief The call back function for teleport to move the robot to a user-specified position
/// 
/// @param req A request contains the position information (x,y coordinates, and rotational angle)
/// @param res An empty response
/// @return true if the callback implemented successfully
bool teleport_callback(nusim::Teleport::Request &req,
                    nusim::Teleport::Response &res)
{
    ROS_INFO("X_coordinate: %f  Y_coordinate: %f  Angle_radians: %f", req.x_coord, req.y_coord, req.radians);

    x = req.x_coord;
    y = req.y_coord;
    theta = req.radians;

    return true;
}


/// @brief Set the obstacles markers in Rviz 
/// 
/// @param nh node handle
void set_obs(ros::NodeHandle nh)
{
    visualization_msgs::MarkerArray obs;
    std::vector<double> v_x;
    std::vector<double> v_y;
    double r;
    double h;

    geometry_msgs::Quaternion rotation;
    rotation.x = 0;
    rotation.y = 0;
    rotation.z = 0;
    rotation.w = 1;

    std_msgs::ColorRGBA colour;
    colour.r = 1;
    colour.g = 0;
    colour.b = 0;
    colour.a = 1;

    nh.getParam("cylinder_xs",v_x);
    nh.getParam("cylinder_ys",v_y);
    nh.getParam("cylinder_r",r);
    nh.getParam("cylinder_h",h);
    
    for (int i=0; i<v_x.size();i++)
    {
        visualization_msgs::Marker marker;
        geometry_msgs::Point position;

        marker.type = marker.CYLINDER;
        marker.color = colour;
        marker.scale.x = r;
        marker.scale.y = r;
        marker.scale.z = h;

        position.x = v_x[i];
        position.y = v_y[i];
        position.z = h/2;


        marker.pose.position = position;
        marker.pose.orientation = rotation;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.id = i;

        obs.markers.push_back(marker);
    }
    obs_pub.publish(obs);

}

void set_walls(ros::NodeHandle nh, double x_len, double y_len)
{
    visualization_msgs::MarkerArray walls;
    double wall_thickness = 0.1;
    std::vector<double> scale_x{x_len,x_len,wall_thickness,wall_thickness};
    std::vector<double> scale_y{wall_thickness,wall_thickness,y_len,y_len};
    std::vector<double> position_x{0.0,0.0,x_len/2+wall_thickness/2,-x_len/2-wall_thickness/2};
    std::vector<double> position_y{y_len/2 + wall_thickness/2,- y_len/2 - wall_thickness/2,0.0,0.0};


    geometry_msgs::Quaternion rotation;
    rotation.x = 0;
    rotation.y = 0;
    rotation.z = 0;
    rotation.w = 1;

    std_msgs::ColorRGBA colour;
    colour.r = 1;
    colour.g = 1;
    colour.b = 1;
    colour.a = 1;

    
    for (int i = 0; i<scale_x.size(); i++)
    {
        visualization_msgs::Marker marker;
        geometry_msgs::Point position;

        marker.type = marker.CUBE;
        marker.color = colour;
        marker.scale.z = 0.25;
        position.z = 0.25/2;

        marker.scale.x = scale_x[i];
        marker.scale.y = scale_y[i];
        position.x = position_x[i];
        position.y = position_y[i];


        marker.pose.position = position;
        marker.pose.orientation = rotation;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.id = i;

        walls.markers.push_back(marker);
    }
    walls_pub.publish(walls);

}

void send_transform()
{

}
int main(int argc, char * argv[])
{
    ros::init(argc, argv, "nusim");
    ros::NodeHandle nh;
    static tf2_ros::TransformBroadcaster br;
    time_pub = nh.advertise<std_msgs::UInt64>("timestep",100);
    js_pub = nh.advertise<sensor_msgs::JointState>("red/joint_states",100);
    obs_pub = nh.advertise<visualization_msgs::MarkerArray>("nusim/obstacles",100);
    walls_pub = nh.advertise<visualization_msgs::MarkerArray>("nusim/walls",100);
    wp_pub = nh.advertise<nuturtlebot_msgs::SensorData>("/sensor_data",100);
    wc_sub = nh.subscribe("red/wheel_cmd",100,wc_sub_callback);
    reset_service = nh.advertiseService("nusim/reset",reset_callback);
    teleport_service = nh.advertiseService("nusim/teleport",teleport_callback);
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
        set_obs(nh);
        set_walls(nh,x_length,y_length);
        count++;
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

        js.header.stamp = ros::Time::now();
        timestep.data = count/rate;
        time_pub.publish(timestep);
        js_pub.publish(js);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}