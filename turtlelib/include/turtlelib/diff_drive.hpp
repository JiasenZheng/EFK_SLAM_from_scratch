#ifndef DIFFDRIVE_INCLUDE_GUARD_HPP
#define DIFFDRIVE_INCLUDE_GUARD_HPP

/**
 * \file diff_drive.hpp
 * \author Jiasen Zheng (jiasenzheng2020@u.northwestern)
 * \brief Kinematics of the turtlebot
 * \version 0.1
 * \date 2022-02-8
 * 
 * 
**/

#include "turtlelib/rigid2d.hpp"


namespace turtlelib
{   
    /**
     * \brief 2D velocity of the turtlebot in both cmd ticks and rad/sec
     * 
    **/
    struct Velocity
    {
        double left;
        double right;
        /**
         * \brief Construct a new Velocity object with 0 velocities
         * 
        **/
        Velocity();
        /**
         * \brief Construct a new Velocity object with specified velocities
         * 
         * \param l Left velocity
         * \param r Right velocity
        **/
        Velocity(const double &l, const double &r);
    };

    /**
     * \brief 2D position of the turtlebot in both encoder ticks and rad
     * 
    **/
    struct Position
    {
        double left;
        double right;
        /**
         * \brief Construct a new Position object with 0 positions
         * 
        **/
        Position();
        /**
         * \brief Construct a new Position object with specified positions
         * 
         * \param l Left postion
         * \param r Right position
        **/
        Position(const double &l, const double &r);
    };

    /**
     * \brief A DiffDrive class to calculate the forward and inverse kinematic of the turtlebot
     * 
    **/
    class DiffDrive
    {
        public:
            /**
             * \brief Construct a new Diff Drive object with all default values
             * 
            **/
            DiffDrive();
            /**
             * \brief Construct a new Diff Drive object with specified values
             * 
             * \param wr Wheel radius
             * \param wt Wheel track
             * \param tf Tranformation from world to the body of the turtlebot
            **/
            DiffDrive( const double &wr,  const double &wt, const Transform2D &tf);

            /**
             * \brief Calulate wheel velocities based on body twist
             * 
             * \param t Body twist
             * \return Velocity- velocities of both wheels
            **/
            Velocity calculate_velocity(const Twist2D &t);
            /**
             * \brief Calculate body twist based on wheel velocities
             * 
             * \param v Wheel velocities
             * \return Twist2D- Body twist
            **/
            Twist2D calculate_twist(const Velocity &v);
            /**
             * \brief Update world-body transform based on wheel velocities
             * 
             * \param vel Wheel velocities
            **/
            void update_config(const Velocity &vel);
            /**
             * \brief Get the transform
             * 
             * \return Transform2D World-body transform
            **/
            Transform2D get_trans();




        private:
            Transform2D Twb;
            double wheel_radius;
            double wheel_track;
        
    };

}

#endif