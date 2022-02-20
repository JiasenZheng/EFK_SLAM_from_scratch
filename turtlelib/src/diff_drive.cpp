#include "turtlelib/diff_drive.hpp"
#include "turtlelib/rigid2d.hpp"
#include <iostream>


namespace turtlelib
{
    Velocity::Velocity() // # use constructor initializer lists
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
    }

    DiffDrive::DiffDrive( const double &wr, const double &wt, const Transform2D &tf)
    {
        wheel_radius = wr;
        wheel_track = wt;
        Twb = tf;
    }


    Velocity DiffDrive::calculate_velocity(const Twist2D &t)
    {
        if (!almost_equal(t.y_dot,0.0))
        {
            throw std::logic_error("Y_velocity should be zero.");
        }
        Velocity vel;
        vel.left = (-(wheel_track/2)*t.omega+t.x_dot)/wheel_radius; //# divide by 2.0
        vel.right = ((wheel_track/2)*t.omega+t.x_dot)/wheel_radius;
        return vel;
    }

    Twist2D DiffDrive::calculate_twist(const Velocity &vel)
    {
        Twist2D t;

        t.y_dot = 0.0;
        t.x_dot = wheel_radius*(vel.right+vel.left)/2;
        t.omega = wheel_radius*(vel.right-vel.left)/wheel_track;
        return t;
    }




    void DiffDrive::update_config(const Velocity &vel)
    {
        Twist2D t = calculate_twist(vel);
        Transform2D Tbb1 = integrate_twist(t);
        // define dq 
        Twist2D dq;
        dq.x_dot = Tbb1.translation().x;
        dq.y_dot = Tbb1.translation().y;
        dq.omega = Twb.rotation()+Tbb1.rotation();
        // convert from body frame to world frame
        Vector2D v;
        v.x = dq.x_dot*Twb.get_cos()-dq.y_dot*Twb.get_sin()+Twb.translation().x;
        v.y = dq.x_dot*Twb.get_sin()+dq.y_dot*Twb.get_cos()+Twb.translation().y;
        // update Twb
        Twb = Transform2D(v,dq.omega);
    }

    Transform2D DiffDrive::get_trans()
    {
        return Twb;
    }
}
