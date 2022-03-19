#include <ros/ros.h>
#include <std_msgs/UInt64.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
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
#include <random>
#include <algorithm>
#include <nav_msgs/Path.h>


/// \brief to simulate and visualize the turtlebot in Rviz
/// 
/// PUBLISHERS:
///      time_pub (std_msgs::UInt64; timestep): current timestep of the simulation
///      obs_pub (visualization_msgs::MarkerArray; obstacles): add cylindrical obstacles to the environment
///      walls_pub (visualization_msgs::MarkerArray; walls): add walls to the environment
///      wp_pub (nuturtlebot_msgs::SeneorData; red/sensor_data): update the wheel positions
///      path_pub (nav_msgs::Path; red_path): path of the real turtlebot
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

/// settable parameters
static std::uniform_real_distribution<double> wheel_position(-10,10);
// static std::uniform_real_distribution<double> wheel_position(0,0);
static std::normal_distribution<> fake_obs(0.0,0.01);
static std::normal_distribution<> wheel_control(0.0,0.5);
// static std::normal_distribution<> wheel_control(0.0,0.0);
static std::normal_distribution<> lidar_noise(0.0,0.00);
static double lidar_range = 3.5;
static double x_length = 3.0;
static double y_length = 3.0;
///
static const int rate = 500; 
static long count = 0;
static std_msgs::UInt64 timestep;
static ros::Publisher time_pub;
static ros::Publisher obs_pub;
static ros::Publisher fake_obs_pub;
static ros::Publisher wp_pub;
static ros::Publisher lidar_pub;
static ros::Publisher walls_pub;
static ros::Publisher path_pub;
static ros::Subscriber wc_sub;
static ros::ServiceServer reset_service;
static ros::ServiceServer teleport_service;
static turtlelib::Velocity wheel_vel_cmd = turtlelib::Velocity();
static turtlelib::DiffDrive real_dd = turtlelib::DiffDrive();
static double cmd_to_radsec;
static double et_to_rad;
static double var_wheel_velocity;
static geometry_msgs::TransformStamped trans;
static turtlelib::Position wheel_position_tick = turtlelib::Position();
static nuturtlebot_msgs::SensorData sd;
static double slip_min;
static double slip_max;
static bool collide = false;
static double angle_min = 0.0;
static double angle_max = 6.28319;
static double range_min = 0.120;
static double range_max = 3.500;
static int sample_num = 360;
static std::string lidar_frame_id = "red-base_footprint";
static double scan_time = 0.2;
static std::vector<double> v_x;
static std::vector<double> v_y;
static std::vector<turtlelib::Vector2D> obs_world;
static nav_msgs::Path path;
static ros::Timer obs_timer;
static ros::Timer lidar_timer;
static double r;
static double h;
static double x_0;
static double y_0;
static double theta_0;
static double x;
static double y;
static double theta;
static double previous_x;
static double previous_y;


/// \brief Generate random variables
///
std::mt19937 & get_random()
{
    // static variables inside a function are created once and persist for the remainder of the program
    static std::random_device rd{}; 
    static std::mt19937 mt{rd()};
    // we return a reference to the pseudo-random number genrator object. This is always the
    // same object every time get_random is called
    return mt;
}

/// \brief The call back function for reset service to rest the timestep and the position of the robot
/// 
/// \param req An empty request
/// \param res An response contains a message and tells if succeed
/// \return true if the call back implemented successfully

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

/// \brief the callback function for wheel command subscriber
/// 
/// \param wc wheel commands
void wc_callback(const nuturtlebot_msgs::WheelCommandsConstPtr &wc)
{
    if (wc->left_velocity == 0 && wc->right_velocity == 0)
    {
        wheel_vel_cmd.left = 0.0;
        wheel_vel_cmd.right = 0.0;   
    }
    else
    {
        wheel_vel_cmd.left = wc->left_velocity + wheel_control(get_random());
        wheel_vel_cmd.right = wc->right_velocity + wheel_control(get_random());
    }

    
    // ROS_INFO("left wheel vel cmd: %f", wheel_vel_cmd.left);
    // ROS_INFO("right wheel vel cmd: %f", wheel_vel_cmd.right);
}

/// \brief The call back function for teleport to move the robot to a user-specified position
/// 
/// \param req A request contains the position information (x,y coordinates, and rotational angle)
/// \param res An empty response
/// \return true if the callback implemented successfully
bool teleport_callback(nusim::Teleport::Request &req,
                    nusim::Teleport::Response &res)
{
    ROS_INFO("X_coordinate: %f  Y_coordinate: %f  Angle_radians: %f", req.x_coord, req.y_coord, req.radians);

    x = req.x_coord;
    y = req.y_coord;
    theta = req.radians;

    return true;
}


/// \brief Set the obstacles markers in Rviz 
/// 
/// \param nh node handle
void set_obs(ros::NodeHandle nh)
{
    visualization_msgs::MarkerArray obs;

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

    for (int i=0; i<v_x.size();i++)
    {
        visualization_msgs::Marker marker;
        geometry_msgs::Point position;

        marker.type = marker.CYLINDER;
        marker.color = colour;
        marker.scale.x = 2*r;
        marker.scale.y = 2*r;
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

/// \brief Set fake obstacles markers in Rviz 
/// 
void obs_callback(const ros::TimerEvent& event)
{
    visualization_msgs::MarkerArray obs;

    geometry_msgs::Quaternion rotation;
    rotation.x = 0;
    rotation.y = 0;
    rotation.z = 0;
    rotation.w = 1;

    std_msgs::ColorRGBA colour;
    colour.r = 1;
    colour.g = 1;
    colour.b = 0;
    colour.a = 1;

    for (int i=0; i<v_x.size();i++)
    {
        turtlelib::Vector2D Vw_obs;
        Vw_obs.x = v_x[i];
        Vw_obs.y = v_y[i];
        turtlelib::Transform2D Tw_tt = real_dd.get_trans();
        // ROS_INFO("W2tt: x: %f, y: %f, theta: %f \n\r",real_dd.get_trans().get_x(),real_dd.get_trans().get_y(),real_dd.get_trans().rotation());

        turtlelib::Transform2D Ttt_w = Tw_tt.inv();
        turtlelib::Vector2D Vtt_obs;
        Vtt_obs = Ttt_w(Vw_obs);
        double l = sqrt(pow(Vtt_obs.x,2)+pow(Vtt_obs.y,2));
        double phi = atan2(Vtt_obs.y,Vtt_obs.x);
        if (l>lidar_range)
        {
            continue;
        }
        l+= fake_obs(get_random());
        phi += fake_obs(get_random());
        double x = l*cos(phi);
        double y = l*sin(phi);
        
        visualization_msgs::Marker marker;
        geometry_msgs::Point position;

        marker.type = marker.CYLINDER;
        marker.color = colour;
        marker.scale.x = 2*r;
        marker.scale.y = 2*r;
        marker.scale.z = h;
        marker.lifetime = ros::Duration(0.2);

        position.x = x;
        position.y = y;
        position.z = h/2;


        marker.pose.position = position;
        marker.pose.orientation = rotation;
        marker.header.frame_id = "red-base_footprint";
        marker.header.stamp = ros::Time::now();
        marker.id = i;

        obs.markers.push_back(marker);
    }
    fake_obs_pub.publish(obs);

}

/**
 * \brief Set the wall markers
 * 
 * \param nh node handles
 * \param x_len the x length of the space
 * \param y_len the y length of the space
**/
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
    colour.g = 0;
    colour.b = 0;
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


/**
 * \brief send tranform between world frame and body frame
 * 
 * \param br broadcaster
**/
void send_transform(ros::NodeHandle nh, tf2_ros::TransformBroadcaster &br)
{
    double cr;
    nh.getParam("collision_radius",cr);
    double x = real_dd.get_trans().get_x();
    double y = real_dd.get_trans().get_y();
    trans.header.stamp = ros::Time::now();
    
    trans.transform.translation.z = 0;

    tf2::Quaternion q;
    q.setRPY(0,0,real_dd.get_trans().rotation());
    trans.transform.rotation.x = q.x();
    trans.transform.rotation.y = q.y();
    trans.transform.rotation.z = q.z();
    trans.transform.rotation.w = q.w();

    for (int i = 0; i < v_x.size(); i++)
    {
        double dis = sqrt(pow(v_x[i]-x,2)+pow(v_y[i]-y,2));
        if (dis<=(r+cr))
        {
            collide = true;
            break;
        }
    }
    trans.transform.translation.x = x;
    trans.transform.translation.y = y;


    br.sendTransform(trans);
     
}


/**
 * \brief publish wheel position in encoder ticks
 * 
**/
void publish_wheel_position()
{
    // compute wheel rotation in tick from wheel velocity
    turtlelib::Velocity wheel_rotation_tick;
    wheel_rotation_tick.left = (wheel_vel_cmd.left*cmd_to_radsec/rate)/et_to_rad;
    // ROS_INFO("left wheel_rotation_tick: %f", wheel_rotation_tick.left);
    wheel_rotation_tick.right=(wheel_vel_cmd.right*cmd_to_radsec/rate)/et_to_rad;
    // ROS_INFO("right wheel_rotation_ticwheel_position_tickk: %f", wheel_rotation_tick.right);
    // update the wheel position in tick 
    wheel_position_tick.left =  remainder((wheel_position_tick.left + wheel_rotation_tick.left + 
    wheel_position(get_random())*wheel_vel_cmd.left/rate),4096);
    wheel_position_tick.right =  remainder((wheel_position_tick.right + wheel_rotation_tick.right + 
    wheel_position(get_random())*wheel_vel_cmd.left/rate),4096);
    // publish as SensorData
    sd.left_encoder = wheel_position_tick.left;
    sd.right_encoder = wheel_position_tick.right;
    wp_pub.publish(sd);
}

/**
 * \brief update the configuration of the turtlebot
 * 
**/
void update_pose()
{
    turtlelib::Velocity vel;
    vel.left = (wheel_vel_cmd.left*cmd_to_radsec)/rate;
    vel.right = (wheel_vel_cmd.right*cmd_to_radsec)/rate;

    if (collide == false)
    {
        real_dd.update_config(vel);
    }
    else
    {
        turtlelib::Velocity real_vel;
        real_vel.left = 0.0;
        real_vel.right = 0.0;
        real_dd.update_config(real_vel);
    }
}

/**
 * \brief helper function to calculate intersections
 * 
 * \param x 
 * \return int 
**/
int sgn(double x)
{
    if (x<0)
    {
        return -1;
    }
    else
    {
        return 1;
    }
}

/**
 * \brief simulate laser data
**/
void lidar_callback(const ros::TimerEvent& event)
{
    sensor_msgs::LaserScan lidar_data;
    lidar_data.header.stamp = ros::Time::now();
    lidar_data.header.frame_id = lidar_frame_id;
    lidar_data.angle_min = angle_min;
    lidar_data.angle_max = angle_max;
    lidar_data.range_min = range_min;
    lidar_data.range_max = range_max;
    lidar_data.angle_increment = angle_max / sample_num;
    lidar_data.scan_time = scan_time;
    // lidar_data.time_increment = scan_time / sample_num;
    lidar_data.ranges.resize(sample_num);


    // obstacles in turtle frame
    std::vector<turtlelib::Vector2D> obs_tt;
    for (int i = 0; i < obs_world.size(); i++)
    {
        obs_tt.push_back((real_dd.get_trans().inv())(obs_world[i]));
        // ROS_INFO("x: %f, y: %f",obs_tt[i].x, obs_tt[i].y);
    }

    // wall corners in turtle frame
    turtlelib::Vector2D UR_w, UL_w, BR_w, BL_w;
    turtlelib::Vector2D UR_tt, UL_tt, BR_tt, BL_tt;
    // angles of each corner relative to turtle frame
    double UR, UL, BR, BL;
    // upper-right corner
    UR_w.x = x_length/2;
    UR_w.y = y_length/2;
    UR_tt = (real_dd.get_trans().inv())(UR_w);
    UR = atan2(UR_tt.y,UR_tt.x);
    UR = turtlelib::normalize_angle(UR);
    // upper-left corner
    UL_w.x = -x_length/2;
    UL_w.y = y_length/2;
    UL_tt = (real_dd.get_trans().inv())(UL_w);
    UL = atan2(UL_tt.y,UL_tt.x);
    UL = turtlelib::normalize_angle(UL);
    // bottom-right corner
    BR_w.x = x_length/2;
    BR_w.y = -y_length/2;
    BR_tt = (real_dd.get_trans().inv())(BR_w);
    BR = atan2(BR_tt.y,BR_tt.x);
    BR = turtlelib::normalize_angle(BR);
    // bottom-left corner
    BL_w.x = -x_length/2;
    BL_w.y = -y_length/2;
    BL_tt = (real_dd.get_trans().inv())(BL_w); 
    BL = atan2(BL_tt.y,BL_tt.x);
    BL = turtlelib::normalize_angle(BL);

    


    for (int i = 0; i < sample_num; i++)
    {
        double range = range_max;
        turtlelib::Vector2D p_min,p_max;
        double theta = turtlelib::deg2rad(i);
        theta = turtlelib::normalize_angle(theta);

        p_min.x = range_min*cos(theta);
        p_min.y = range_min*sin(theta);
        p_max.x = range_max*cos(theta);
        p_max.y = range_max*sin(theta);
        double line_slope = (p_max.y-p_min.y)/(p_max.x-p_min.x);
        double line_intercept = p_min.y-line_slope*p_min.x;
        turtlelib::Vector2D p_max_world = (real_dd.get_trans())(p_max);
        // ROS_INFO("UR: %f, Theta: %f, BR: %f",UR,theta,BR);



        for(int j=0; j<obs_tt.size(); j++)
        {
            double x1 = p_min.x - obs_tt[j].x;
            double x2 = p_max.x - obs_tt[j].x;            
            double y1 = p_min.y - obs_tt[j].y;
            double y2 = p_max.y - obs_tt[j].y;
            double dx = x2 - x1;
            double dy = y2 - y1;
            double dr = sqrt(pow(dx,2)+pow(dy,2));
            double D = x1*y2 - x2*y1;
            double delta = pow(r,2)*pow(dr,2)-pow(D,2);
            // std::vector<turtlelib::Vector2D> intersections;

            if (delta >0)
            {
                // calculate 2 intersections
                turtlelib::Vector2D intersection1, intersection2, intersection;
                intersection1.x = (D*dy+sgn(dy)*dx*sqrt(delta))/pow(dr,2);
                intersection2.x = (D*dy-sgn(dy)*dx*sqrt(delta))/pow(dr,2);
                intersection1.y = (-D*dx+fabs(dy)*sqrt(delta))/pow(dr,2);
                intersection2.y = (-D*dx-fabs(dy)*sqrt(delta))/pow(dr,2);

                // find the intersection closer to the turtlebot
                double dis;
                double dis1 = sqrt(pow((intersection1.x + obs_tt[j].x),2)+pow((intersection1.y + obs_tt[j].y),2));
                double dis2 = sqrt(pow((intersection2.x + obs_tt[j].x),2)+pow((intersection2.y + obs_tt[j].y),2));


                if (dis1 < dis2)
                {
                    dis = sqrt(pow((intersection1.x - x1),2)+pow((intersection1.y - y1),2));
                    if (dis < dis1)
                    {
                        range = dis1;
                        
                    }
                }
                else
                {
                    dis = sqrt(pow((intersection2.x - x1),2)+pow((intersection2.y - y1),2));
                    if (dis < dis2)
                    {
                        range = dis2;
                        
                    }
                }
      
            }
        }
        if (range == range_max)
        {
            // ROS_INFO("UR: %f, BR: %f", UR, BR);
            bool right_wall = (UR > BR) ? (theta < UR && theta >= BR) : (theta < UR or theta >= BR);
            bool upper_wall = (UL > UR) ? (theta < UL && theta >= UR) : (theta < UL or theta >= UR);
            bool left_wall = (BR > BL) ? (theta < BR && theta >= BL) : (theta < BR or theta >= BL);
            if (right_wall) 
            {
                double x = x_length/2 - real_dd.get_trans().get_x();
                double theta2 = theta + real_dd.get_trans().rotation();
                theta2 = turtlelib::normalize_angle(theta2);
                range = x/cos(theta2);
            
            }
            else if (upper_wall)
            {
                double y = y_length/2 - real_dd.get_trans().get_y();
                double theta2 = theta + real_dd.get_trans().rotation();
                theta2 = turtlelib::normalize_angle(theta2);
                range = y/sin(theta2);
            }
            else if (left_wall)
            {
                double y = -y_length/2 - real_dd.get_trans().get_y();
                double theta2 =  theta + real_dd.get_trans().rotation();
                theta2 = turtlelib::normalize_angle(theta2);
                range = y/sin(theta2);
            }
            else
            {
                double x = -x_length/2 - real_dd.get_trans().get_x();
                double theta2 = theta + real_dd.get_trans().rotation();
                theta2 = turtlelib::normalize_angle(theta2);
                range = x/cos(theta2);
            }
        }
        lidar_data.ranges[i] = range + lidar_noise(get_random());
    }  
    lidar_pub.publish(lidar_data);
}

/// \brief publish path for red robot
void pub_path()
{
    std_msgs::Header h;
    geometry_msgs::PoseStamped p;
    tf2::Quaternion rot;

    h.stamp = ros::Time::now();
    h.frame_id = "world";
    p.header = h;
    path.header = h;
    p.pose.position.x = real_dd.get_trans().get_x();
    p.pose.position.y = real_dd.get_trans().get_y();
    p.pose.position.z = 0;

    rot.setRPY(0,0,real_dd.get_trans().rotation());
    p.pose.orientation = tf2::toMsg(rot);

    path.poses.push_back(p);
    path_pub.publish(path);
}
    

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "nusim");
    ros::NodeHandle nh;
    static tf2_ros::TransformBroadcaster br;
    time_pub = nh.advertise<std_msgs::UInt64>("timestep",100);
    obs_pub = nh.advertise<visualization_msgs::MarkerArray>("nusim/obstacles",100);
    fake_obs_pub = nh.advertise<visualization_msgs::MarkerArray>("nusim/fake_sensor",100);
    walls_pub = nh.advertise<visualization_msgs::MarkerArray>("nusim/walls",100);
    wp_pub = nh.advertise<nuturtlebot_msgs::SensorData>("/red/sensor_data",100);
    lidar_pub = nh.advertise<sensor_msgs::LaserScan>("/laser",100);
    path_pub = nh.advertise<nav_msgs::Path>("/red_path",100);
    wc_sub = nh.subscribe("red/wheel_cmd",100,wc_callback);
    reset_service = nh.advertiseService("nusim/reset",reset_callback);
    teleport_service = nh.advertiseService("nusim/teleport",teleport_callback);
    obs_timer = nh.createTimer(ros::Duration(0.2), obs_callback);
    lidar_timer = nh.createTimer(ros::Duration(0.2), lidar_callback);
    ros::Rate loop_rate(rate);


    // set up tf
    trans.child_frame_id = "red-base_footprint";
    trans.header.frame_id = "world";
    // get parameters
    nh.getParam("motor_cmd_to_radsec",cmd_to_radsec);
    nh.getParam("encoder_ticks_to_rad",et_to_rad);
    // nh.getParam("var_wheel_vel", var_wheel_velocity);
    nh.param("x0",x,0.0);
    nh.param("y0",y,0.0);
    nh.param("theta0",theta,0.0);
    nh.getParam("cylinder_xs",v_x);
    nh.getParam("cylinder_ys",v_y);
    nh.getParam("cylinder_r",r);
    nh.getParam("cylinder_h",h);
    x_0 = x;
    y_0 = y;
    theta_0 = theta;

    // initialize obstacle in world frame
    for (int i = 0; i<v_x.size();i++)
    {
        turtlelib::Vector2D ob;
        ob.x= v_x[i];
        ob.y = v_y[i];
        obs_world.push_back(ob);
    }


    while (ros::ok())
    {
        count++;
        set_obs(nh);
        set_walls(nh,x_length,y_length);
        send_transform(nh,br);
        publish_wheel_position();
        update_pose();
        pub_path();

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}