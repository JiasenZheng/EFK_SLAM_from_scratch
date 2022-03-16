/**
 * \file landmarks.cpp
 * \author Jiasen Zheng (jiasenzheng2020@u.northwestern)
 * \brief 
 * \version 0.1
 * \date 2022-03-15
 * 
 * 
 * 
**/

#include <ros/ros.h>
#include <armadillo>
#include <turtlelib/rigid2d.hpp>
#include <nuslam/nuslam.hpp>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath>


static int frequency = 100;
static double d_thresh = 0.1;
static ros::Subscriber laser_sub;
static ros::Publisher lm_pub;

void remove_non_circles(std::vector<std::vector<turtlelib::Vector2D>> & clusters) 
{
    for (int i=0; i<clusters.size(); i++) {
        std::vector<turtlelib::Vector2D> cluster = clusters[i];
        turtlelib::Vector2D point_start = cluster[0];                 
        turtlelib::Vector2D point_end = cluster.back();    
        std::vector<double> angles; 
        double sum = 0.0;                                                          

        for (int j=1; j<cluster.size()-1; j++) {

            // compute inscribed angle
            turtlelib::Vector2D point_in = cluster[j];
            turtlelib::Vector2D v_start = point_start-point_in;
            turtlelib::Vector2D v_end = point_end-point_in;
            double angle_in = turtlelib::rad2deg(turtlelib::angle(v_start,v_end));
            sum+=angle_in;
            // push back to angles
            angles.push_back(angle_in);
        }

        // compute mean
        double mean = sum/angles.size();
        ROS_INFO("MEAN: %f",mean);

        // remove cluster with inapproate angle
        if (mean < 90.0 or mean > 135.0)
        {
            clusters.erase(clusters.begin()+i);
            i-=1;
            continue;
        }

        // compute std dev
        double accum = 0.0;
        std::for_each (std::begin(angles), std::end(angles), [&](const double d) {
            accum += (d - mean) * (d - mean);
        });
        double stdev = sqrt(accum / (angles.size()-1));
        ROS_INFO("STDEV: %f",stdev);


        //remove clusters if standard deviation is too large
        if (stdev > 0.15)
        {
            clusters.erase(clusters.begin()+i);
            i-=1;
        }
    }
}

void laser_cb(const sensor_msgs::LaserScanConstPtr &laser)
{
    // Create clusters containers
    std::vector<std::vector<turtlelib::Vector2D>> clusters;
    std::vector<turtlelib::Vector2D> cluster;

    int num = laser->ranges.size();
    for (int i = 1; i< num; i++)
    {
        double range_new = laser->ranges[i];
        double range_old = laser->ranges[i-1];
        
        // convert to cartesians
        turtlelib::Vector2D point_new = nuslam::toXY(range_new, i);
        turtlelib::Vector2D point_old = nuslam::toXY(range_old, i-1);

        // first point
        if (cluster.size()==0)
        {
            cluster.push_back(point_old);
        }

        // computer distance between new and old points
        double dis = sqrt(pow(point_new.x-point_old.x,2)+pow(point_new.y-point_old.y,2));

        if (dis<d_thresh)
        {
            // add new point
            cluster.push_back(point_new);
        }
        else
        {
            // push back the cluster
            clusters.push_back(cluster);
            // clean cluster
            cluster.clear();
        }
        if (i == num-1)
        {
            clusters.push_back(cluster);
        }
    }
    // ROS_INFO("THE NUMBER OF CLUSTERS: %ld",clusters.size());
    // decide whether need to join the last and the first cluster
    double range_first = laser->ranges[0];
    double range_last = laser->ranges[num-1];
    turtlelib::Vector2D point_first = nuslam::toXY(range_first,0.0);
    turtlelib::Vector2D point_last = nuslam::toXY(range_last,num-1);
    double dis = sqrt(pow(point_last.x-point_first.x,2)+pow(point_last.y-point_first.y,2));
    if (dis < d_thresh)
    {
        // join two clusters
        clusters[0].insert(clusters[0].begin(),clusters.back().begin(),clusters.back().end());
        // pop the last cluster
        clusters.pop_back();
    }
    // remove the cluster that smaller than 3
    for (int i = 0; i < clusters.size(); i++)
    {
        if (clusters[i].size() < 3)
        {
            clusters.erase(clusters.begin()+i);
        }
    }
    ROS_INFO("THE NUMBER OF CLUSTERS: %ld",clusters.size());
    remove_non_circles(clusters);
    ROS_INFO("THE NUMBER OF CIRCLE CLUSTERS: %ld",clusters.size());
}


int main(int argc, char** argv)
{
    // Initialize node
    ros::init(argc, argv, "landmarks");
    ros::NodeHandle nh;

    // Create pub and sub
    laser_sub = nh.subscribe("/laser",10,laser_cb);
    lm_pub = nh.advertise<visualization_msgs::MarkerArray>("/measured_landmarks",10);


    ros::Rate loop_rate(frequency);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}