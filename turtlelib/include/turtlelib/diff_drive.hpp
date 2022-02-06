#ifndef DIFFDRIVE_INCLUDE_GUARD_HPP
#define DIFFDRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief kinematics of twheeled mobile robots

#include "turtlelib/rigid2d.hpp"

namespace turtlelib
{   
    struct Velocity
    {
        double left;
        double right;
        Velocity();
        Velocity(const double &l, const double &r);
    };

    struct Position
    {
        double left;
        double right;
        Position();
        Position(const double &l, const double &r);
    };

    class DiffDrive
    {
        public:
            
            DiffDrive();
            
            DiffDrive( const double &wr,  const double &wt, const Transform2D &tf, const Position &wp);
 
            Velocity inverse_kinematics(const Twist2D &t);

            Twist2D forward_kinematics(const Velocity &v);
            
            void update_position_tick(const double &left_tick, const double &right_tick);

            void update_config(const Velocity &vel);
            
            Position get_wheel_position();

            Transform2D get_trans();




        private:
            Position wheel_position;
            Transform2D Twb;
            double wheel_radius;
            double wheel_track;
        
    };

}

#endif