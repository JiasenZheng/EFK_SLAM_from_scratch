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
 *      odom_path_pub (nav_msgs::Path -> /blue_path) publish the path of odometry turtlebot
 *      slam_path_pub (nav_msgs::Path -> /green_path) publish the path of slam turtlebot
 * SUBSCRIBERS:
 *      js_sub (sensor_msgs::JointState-> /joint_states) joint states
 * SERVICES:
 *      set_pose: set a pose for the turtlebot
 * BROADCASTER:
 *      odom_br: transform from odom_id to body_id
 *      map_br: transform from map to odom
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
#include <armadillo>
#include <nuslam/nuslam.hpp>
#include <visualization_msgs/MarkerArray.h>
#include <unordered_map>

static const int rate = 500;
static ros::Publisher odom_pub;
static ros::Publisher odom_path_pub;
static ros::Publisher green_obs_pub;
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
static turtlelib::Twist2D twist;
static geometry_msgs::Quaternion rot;
static tf2::Quaternion q;
static geometry_msgs::TransformStamped trans;
static geometry_msgs::TransformStamped trans2;
static geometry_msgs::TransformStamped trans3;
static double lidar_range = 3.5;
static double r,h; 

static ros::Subscriber fake_lidar_sub;
static ros::Publisher slam_path_pub;
static std::vector<double> v_x, v_y;
static turtlelib::Transform2D Tm_tt = turtlelib::Transform2D();
static nuslam::EKF ekf = nuslam::EKF(3);

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
void broadcast_world2blue()
{
    static tf2_ros::TransformBroadcaster br;
    // define trans
    trans.header.stamp = ros::Time::now();
    trans.transform.translation.x = dd.get_trans().get_x();
    trans.transform.translation.y = dd.get_trans().get_y();
    trans.transform.translation.z = 0.0;
    q.setRPY( 0, 0, dd.get_trans().rotation() );
    rot = tf2::toMsg(q);
    trans.transform.rotation = rot;

    // broadcast
    br.sendTransform(trans);
}

/**
 * \brief broadcast transform from map to odom
 * 
**/
void broadcast_map2odom()
{
  turtlelib::Transform2D To_tt = dd.get_trans();
  turtlelib::Transform2D Tm_o = Tm_tt*To_tt.inv();

  static tf2_ros::TransformBroadcaster br2;
  // define trans
  trans2.header.stamp = ros::Time::now();
  trans2.transform.translation.x = Tm_o.get_x();
  trans2.transform.translation.y = Tm_o.get_y();
  trans2.transform.translation.z = 0.0;
  q.setRPY( 0, 0, Tm_o.rotation() );
  rot = tf2::toMsg(q);
  trans2.transform.rotation = rot;

  // broadcast
  br2.sendTransform(trans2);
}

void broadcast_odom2green()
{
  turtlelib::Transform2D To_tt = dd.get_trans();

  static tf2_ros::TransformBroadcaster br3;
  // define trans
  trans3.header.stamp = ros::Time::now();
  trans3.transform.translation.x = To_tt.get_x();
  trans3.transform.translation.y = To_tt.get_y();
  trans3.transform.translation.z = 0.0;
  q.setRPY( 0, 0, To_tt.rotation() );
  rot = tf2::toMsg(q);
  trans3.transform.rotation = rot;

  // broadcast
  br3.sendTransform(trans3);
}


/// \brief publish path for blue robot
void pub_odom_path()
{
    static nav_msgs::Path odom_path;
    std_msgs::Header h;
    geometry_msgs::PoseStamped p;
    tf2::Quaternion rot;

    h.stamp = ros::Time::now();
    h.frame_id = "world";
    p.header = h;
    odom_path.header = h;
    p.pose.position.x = dd.get_trans().get_x();
    p.pose.position.y = dd.get_trans().get_y();
    p.pose.position.z = 0;

    rot.setRPY(0,0,dd.get_trans().rotation());
    p.pose.orientation = tf2::toMsg(rot);

    odom_path.poses.push_back(p);
    odom_path_pub.publish(odom_path);
}

/// \brief publish path for green robot
void pub_slam_path()
{
    static nav_msgs::Path slam_path;
    std_msgs::Header h;
    geometry_msgs::PoseStamped p;
    tf2::Quaternion rot;

    h.stamp = ros::Time::now();
    h.frame_id = "world";
    p.header = h;
    slam_path.header = h;
    p.pose.position.x = Tm_tt.get_x();
    p.pose.position.y = Tm_tt.get_y();
    p.pose.position.z = 0;

    rot.setRPY(0,0,Tm_tt.rotation());
    p.pose.orientation = tf2::toMsg(rot);

    slam_path.poses.push_back(p);
    slam_path_pub.publish(slam_path);
}

/// \brief Set the obstacles markers relative to slam turtlebot in Rviz 
/// 
void set_green_obs()
{
    visualization_msgs::MarkerArray obs;

    geometry_msgs::Quaternion rotation;
    rotation.x = 0;
    rotation.y = 0;
    rotation.z = 0;
    rotation.w = 1;

    std_msgs::ColorRGBA colour;
    colour.r = 0;
    colour.g = 1;
    colour.b = 0;
    colour.a = 1;

    for (int i=0; i<v_x.size();i++)
    {
        turtlelib::Vector2D Vm_obs;
        Vm_obs.x = v_x[i];
        Vm_obs.y = v_y[i];
        // turtlelib::Transform2D Tw_tt = real_dd.get_trans();
        

        turtlelib::Transform2D Ttt_m = Tm_tt.inv();
        turtlelib::Vector2D Vtt_obs;
        Vtt_obs = Ttt_m(Vm_obs);
        double l = sqrt(pow(Vtt_obs.x,2)+pow(Vtt_obs.y,2));
        // double phi = atan2(Vtt_obs.y,Vtt_obs.x);
        if (l>lidar_range)
        {
            continue;
        }

        visualization_msgs::Marker marker;
        geometry_msgs::Point position;

        marker.type = marker.CYLINDER;
        marker.color = colour;
        marker.scale.x = 2*r;
        marker.scale.y = 2*r;
        marker.scale.z = h;
        marker.lifetime = ros::Duration(0.2);

        position.x = Vtt_obs.x;
        position.y = Vtt_obs.y;
        position.z = h/2;


        marker.pose.position = position;
        marker.pose.orientation = rotation;
        marker.header.frame_id = "green-base_footprint";
        marker.header.stamp = ros::Time::now();
        marker.id = i;

        obs.markers.push_back(marker);
    }
    green_obs_pub.publish(obs);

}

void fake_sensor_callback(const visualization_msgs::MarkerArrayPtr &data)
{
    static std::unordered_map<int,bool> map;   
    
    //predict
    ekf.predict(twist,dd.get_trans());
    // ROS_INFO("x_dot: %f, y_dot: %f, omega: %f \n\r",twist.x_dot,twist.y_dot,twist.omega);
    // ROS_INFO("x: %f, y: %f, theta: %f \n\r",dd.get_trans().get_x(),dd.get_trans().get_y(),dd.get_trans().rotation());

  
    int len = data->markers.size();
    // ROS_INFO("size: %i\r",len);
    for(int i = 0; i < len; i++)
    {
        visualization_msgs::Marker measurement = data->markers[i];

        // convert measurement to polar
        turtlelib::Vector2D location;
        double x = data->markers[i].pose.position.x;
        double y = data->markers[i].pose.position.y;
        location.x = x;
        location.y = y;
        // ROS_INFO("x: %f, y: %f \r",x,y);
        // ROS_INFO("%d tt2Obs: x: %f, y: %f\r",i,x,y);
        double r = sqrt(pow(x,2)+pow(y,2));
        double phi = atan2(y,x);
        arma::mat z = arma::mat(2,1,arma::fill::zeros);
        z(0,0) = r;
        z(1,0) = phi;

        // get id
        int j = data->markers[i].id+1;

        //initialize landmark
        if (map.find(j) == map.end())
        {
            map[j] = true;
            turtlelib::Vector2D landmark_loc = Tm_tt(location);
            ekf.add_landmark(j,landmark_loc);
        }

        //update
        ekf.update(j,z);
    }

    // find transform from map to robot
    auto state = ekf.get_state();
    Tm_tt = turtlelib::Transform2D(state(1,0),state(2,0),state(0,0));
    // publish slam/odom path
    pub_slam_path();
    pub_odom_path();
    set_green_obs();
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

    if (nh.getParam("/cylinder_xs", v_x))
    {
      ROS_INFO_STREAM("Got param:" << " cylinder_xs");
    }
    else
    {
      ROS_ERROR_STREAM("Failed to get param 'cylinder_xs'");
      ros::shutdown();
    }

    if (nh.getParam("/cylinder_ys", v_y))
    {
      ROS_INFO_STREAM("Got param:" << " cylinder_ys");
    }
    else
    {
      ROS_ERROR_STREAM("Failed to get param 'cylinder_ys'");
      ros::shutdown();
    }
    nh.getParam("cylinder_r",r);
    nh.getParam("cylinder_h",h);


    // initialize publishers, subscribers, services and br
    js_sub = nh.subscribe("/joint_states",100,js_callback);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom",100);
    odom_path_pub = nh.advertise<nav_msgs::Path>("/blue_path",100);
    slam_path_pub = nh.advertise<nav_msgs::Path>("/green_path",100);
    green_obs_pub = nh.advertise<visualization_msgs::MarkerArray>("/slam_sensor",100);
    fake_lidar_sub = nh.subscribe("nusim/fake_sensor",100,fake_sensor_callback);
    set_pose = nh.advertiseService("set_pose",set_pose_callback);

    // DiffDrive
    dd = turtlelib::DiffDrive(wr,wt,tf);

    // set up odom
    odom.header.frame_id = "world";
    odom.child_frame_id = body_id;

    // set up trans
    trans.header.frame_id = "world";
    trans.child_frame_id = body_id;
    trans2.header.frame_id = "map";
    trans2.child_frame_id = "odom";
    trans3.header.frame_id = "odom";
    trans3.child_frame_id = "green-base_footprint";
    

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
        // publish_odom();
        broadcast_world2blue();
        broadcast_map2odom();
        broadcast_odom2green();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
