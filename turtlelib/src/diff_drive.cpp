#include "turtlelib/diff_drive.hpp"
#include "turtlelib/rigid2d.hpp"
#include <iostream>


namespace turtlelib
{
    Velocity::Velocity()
    {
        left = 0.0;
        right = 0.0;
    }
    Velocity::Velocity(const double &l, const double &r)
    {
        left = l;
        right = r;
    }
    Position::Position()
    {
        left = 0.0;
        right = 0.0;
    }
    Position::Position(const double &l, const double &r)
    {
        left = l;
        right = r;
    }

    DiffDrive::DiffDrive()
    {
        wheel_radius = 0.033;
        wheel_track = 0.16;
        Twb = Transform2D();
        wheel_position = Position();
    }

    DiffDrive::DiffDrive( const double &wr, const double &wt, const Transform2D &tf, const Position &wp)
    {
        wheel_radius = wr;
        wheel_track = wt;
        Twb = tf;
        wheel_position = wp;
    }


    Velocity DiffDrive::inverse_kinematics(const Twist2D &t)
    {
        if (!almost_equal(t.y_dot,0.0))
        {
            throw std::logic_error("Y_velocity should be zero.");
        }
        Velocity vel;
        vel.left = (-(wheel_track/2)*t.omega+t.x_dot)/wheel_radius;
        vel.right = ((wheel_track/2)*t.omega+t.x_dot)/wheel_radius;
        return vel;
    }

    Twist2D DiffDrive::forward_kinematics(const Velocity &vel)
    {
        Twist2D t;

        t.y_dot = 0.0;
        t.x_dot = wheel_radius*(vel.right+vel.left)/2;
        t.omega = wheel_radius*(vel.right-vel.left)/wheel_track;
        return t;
    }



    void DiffDrive::update_position_tick(const double &left_tick, const double &right_tick)
    {
        wheel_position.left+=left_tick;
        wheel_position.right+=right_tick;
        while(wheel_position.left >= 4096)
        {
            wheel_position.left-=4096;
        }
        while(wheel_position.left < 0)
        {
            wheel_position.left+=4096;
        }
        while(wheel_position.right >= 4096)
        {
            wheel_position.right-=4096;
        }
        while(wheel_position.right < 0)
        {
            wheel_position.right+=4096;
        }
    }
    void DiffDrive::update_config(const Velocity &vel)
    {
        Twist2D t = forward_kinematics(vel);
        Transform2D Tbb1 = integrate_twist(t);
        // update configuration in private member
        Twb*=Tbb1;
    }

    Position DiffDrive::get_wheel_position()
    {
        return wheel_position;
    }

    Transform2D DiffDrive::get_trans()
    {
        return Twb;
    }
}