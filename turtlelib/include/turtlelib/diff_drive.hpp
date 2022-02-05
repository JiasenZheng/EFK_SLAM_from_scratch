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
            
            DiffDrive( const double &wr,  const double &wt, const Transform2D &tf);
 
            Velocity inverse_kinematics(const Twist2D &t);

            Twist2D forward_kinematics(const Velocity &v);

            void update_config(const Position &p);


        private:
            Transform2D Twb;
            double wheel_radius;
            double wheel_track;
        
    };

}

#endif