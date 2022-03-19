/**
 * \file landmarks.cpp
 * \author Jiasen Zheng (jiasenzheng2020@u.northwestern)
 * \brief Detect and locate landmarks based on laser scan
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
static ros::Publisher lms_pub;
static double r,h;

/**
 * \brief classify the clusters of points into circle and non-circle. Then, remove the non-circle ones
 * 
 * \param clusters a vector of clusters
**/
void remove_non_circles(std::vector<std::vector<turtlelib::Vector2D>> & clusters) 
{
    for (int i=0; i<clusters.size(); i++) {
        std::vector<turtlelib::Vector2D> cluster = clusters[i];
        turtlelib::Vector2D point_start = cluster.front();                 
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
        // ROS_INFO("MEAN: %f",mean);

        // remove cluster with inapproate angle
        if (mean < 90.0 or mean > 155.0)
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
        // ROS_INFO("STDEV: %f",stdev);

        //remove clusters if standard deviation is too large
        if (stdev > 8.6)
        {
            clusters.erase(clusters.begin()+i);
            i-=1;
        }
        cluster.clear();
    }
}

/**
 * \brief implement a circle fitting algorithm and find center coordinates of the circle
 * 
 * \param clusters a vector of cluster 
 * \return std::vector<turtlelib::Vector2D> a vector of coordinates of circle wrt turtlebot frame
**/
std::vector<turtlelib::Vector2D> circle_fitting(std::vector<std::vector<turtlelib::Vector2D>> & clusters) 
{
    // Circle vector container
    std::vector<turtlelib::Vector2D> circles = {};

    for (int i=0; i<clusters.size(); i++) {
        std::vector<turtlelib::Vector2D> cluster = clusters[i];
        int n = cluster.size();

        // compute x y means
        double x_sum = 0.0;
        double y_sum = 0.0;

        for (int j=0; j<n; j++)
        {
            x_sum+=cluster[j].x;
            y_sum+=cluster[j].y;
        }

        double x_mean = x_sum/n;
        double y_mean = y_sum/n;

        // Shift the coordinates and create 3 lists
        std::vector<double> x_list, y_list, z_list;
        double z_sum = 0;
        for (int j=0; j<n; j++)
        {
            double x_val = cluster[j].x - x_mean;
            double y_val = cluster[j].y - y_mean;
            double z_val = pow(x_val,2)+pow(y_val,2);
            x_list.push_back(x_val);
            y_list.push_back(y_val);
            z_list.push_back(z_val);
            z_sum+=z_val;
        }
        double z_mean = z_sum/n;

        // construct Z matrix
        arma::mat z1,z2,z3,z4;
        z1 = arma::colvec(z_list);
        z2 = arma::colvec(x_list);
        z3 = arma::colvec(y_list);
        z4 = arma::mat(n,1,arma::fill::ones);
        arma::mat Z = arma::join_horiz(z1,z2,z3,z4);
        // ROS_INFO("%i:",i);
        // Z.print("Z: ");

        // construct momemt matrix
        arma::mat M = (1/n)*arma::trans(Z)*Z;

        // construct constraint matrix
        arma::mat H= {  {8*z_mean, 0, 0, 2},
                        {0, 1, 0, 0},
                        {0, 0, 1, 0},
                        {2, 0, 0, 0} };
        arma::mat H_inv={{0, 0, 0, 0.5},
                         {0, 1, 0, 0},
                         {0, 0, 1, 0},
                         {0.5, 0, 0, -2*z_mean}  };


        // construct singular value decomposition
        arma::mat U,V;
        arma::vec s;
        arma::svd(U,s,V,Z);

        // solve for A
        arma::vec A;
        // U.print("U: ");
        // s.print("s: ");
        // V.print("V: ");
        // Z.print("Z: ");
        if (s.size()<4)
        {
            continue;
        }
        if (s(3) < 1e-12)
        {
            A = V.col(3);
        }
        else
        {
            try
            {
                arma::mat Y = V*arma::diagmat(s)*arma::trans(V);
                arma::mat Q = Y*H_inv*Y;
                // find eigenvalues and vectors of Q
                arma::vec eigen_vals;
                arma::mat eigen_vecs;
                arma::eig_sym(eigen_vals,eigen_vecs,Q);

                // find the smallest positive eigenvalue of Q
                int idx = 0;
                double eigen_val = 999.0;
                for (int j = 0; j<eigen_vals.size(); j++)
                {
                    if (eigen_vals[j]>0 and eigen_vals[j]<eigen_val)
                    {
                        eigen_val = eigen_vals[j];
                        idx = j;
                    }
                }
                arma::vec A_star = eigen_vecs.col(idx);
                A = arma::solve(Y,A_star);
            }
            catch(const std::exception& e)
            {
                ROS_INFO("%s",e.what());
            }
            
        }

        // Compute circle center and radius
        turtlelib::Vector2D circle;
        circle.x = -A(1)/(2*A(0))+x_mean;
        // ROS_INFO("X: %f", circle.x);
        circle.y = -A(2)/(2*A(0))+y_mean;
        // ROS_INFO("Y: %f", circle.y);
        circles.push_back(circle);

        double R = sqrt((pow(A(1),2) + pow(A(2),2) -4*A(0)*A(3)) / (4*pow(A(0),2)));
    }
    return circles;
}

/**
 * \brief publish measured landmark
 * 
 * \param circles a vector of circle coordinates
**/
void publish_measured_lm(const std::vector<turtlelib::Vector2D> &circles)
{
    visualization_msgs::MarkerArray lms;

    geometry_msgs::Quaternion rotation;
    rotation.x = 0;
    rotation.y = 0;
    rotation.z = 0;
    rotation.w = 1;

    std_msgs::ColorRGBA colour;
    colour.r = 0;
    colour.g = 1;
    colour.b = 1;
    colour.a = 1;

    for (int i=0; i<circles.size();i++)
    {
        visualization_msgs::Marker marker;
        geometry_msgs::Point position;

        marker.type = marker.CYLINDER;
        marker.color = colour;
        marker.scale.x = 2*r;
        marker.scale.y = 2*r;
        marker.scale.z = h;

        position.x = circles[i].x;
        position.y = circles[i].y;
        position.z = h/2;


        marker.pose.position = position;
        marker.pose.orientation = rotation;
        marker.header.frame_id = "red-base_footprint";
        marker.header.stamp = ros::Time::now();
        marker.id = i;

        lms.markers.push_back(marker);
    }
    lms_pub.publish(lms);
}

/**
 * \brief callback function for laser subscriber
 * 
 * \param laser laser scan data
**/
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
        
        // push back the last cluster
        if (i == num-1)
        {
            clusters.push_back(cluster);
            cluster.clear();
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
        if (clusters[i].size() <= 3)
        {
            clusters.erase(clusters.begin()+i);
        }
    }
    remove_non_circles(clusters);
    // ROS_DEBUG("# of circles: %li\r\n",clusters.size());
    std::vector<turtlelib::Vector2D> circles;
    circles = circle_fitting(clusters);
    publish_measured_lm(circles);
}



int main(int argc, char** argv)
{
    // Initialize node
    ros::init(argc, argv, "landmarks");
    ros::NodeHandle nh;

    // Create pub and sub
    laser_sub = nh.subscribe("/laser",10,laser_cb);
    // laser_sub = nh.subscribe("/scan",10,laser_cb);
    lms_pub = nh.advertise<visualization_msgs::MarkerArray>("/measured_landmarks",10);

    // Get parameters
    ros::param::get("/cylinder_r",r);
    ros::param::get("/cylinder_h",h);


    ros::Rate loop_rate(frequency);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}