#include"../include/rigid2d.hpp"
#include<iostream>

namespace turtlelib
{
    Vector2D Vector2D::normalize()
    {
        Vector2D v_nor;
        double mag;

        mag = sqrt(pow(x,2)+pow(y,2));
        v_nor.x = x/mag;
        v_nor.y = y/mag;
        return v_nor;
    }

    std::ostream & operator<<(std::ostream & os, const Vector2D & v)
    {
        os << "[" << v.x << " " << v.y << "]";
        return os;
    }

    std::istream & operator>>(std::istream & is, Vector2D & v)
    {
        is >> v.x;
        is >> v.y;
        return is;
    }

    std::ostream & operator<<(std::ostream & os, const Twist2D & t)
    {
        os << "[" << t.omega << " " << t.x_dot << " " << t.y_dot << "]";
        return os;
    }

    std::istream & operator>>(std::istream & is, Twist2D & t)
    {
        is >> t.omega;
        is >> t.x_dot;
        is >> t.y_dot;
        return is;
    }

    Transform2D::Transform2D()
    {
        x = 0.0;
        y = 0.0;
        theta = 0.0;
        sintheta = 0.0;
        costheta = 1.0;

    }

    Transform2D::Transform2D(Vector2D trans)
    {
        x = trans.x;
        y = trans.y;
        theta = 0.0;
        sintheta = 0.0;
        costheta = 1.0;

    }

    Transform2D::Transform2D(double radians)
    {
        x = 0.0;
        y = 0.0;
        theta = radians;
        sintheta = sin(theta);
        costheta = cos(theta);
    }

    Transform2D::Transform2D(Vector2D trans, double radians)
    {
        x = trans.x;
        y = trans.y;
        theta = radians;
        sintheta = sin(theta);
        costheta = cos(theta);
    }

    Vector2D Transform2D::operator()(Vector2D v) const
    {
        Vector2D v_trans;
        v_trans.x = costheta*v.x - sintheta*v.y + x;
        v_trans.y = sintheta*v.x + costheta*v.y + y;
        return v_trans;
    }

    Twist2D Transform2D::operator()(Twist2D t) const
    {
        Twist2D t_trans;
        t_trans.omega = t.omega;
        t_trans.x_dot = t.omega*y + costheta*t.x_dot - sintheta*t.y_dot;
        t_trans.y_dot = -t.omega*x + sintheta*t.x_dot + costheta*t.y_dot;
        return t_trans;
    }

    Transform2D Transform2D::inv() const
    {
        Transform2D tf_inv(-theta);

        tf_inv.x = -x*costheta-y*sintheta;
        tf_inv.y = -y*costheta+x*sintheta;
        return tf_inv;
    }

    Transform2D & Transform2D::operator*=(const Transform2D & rhs)
    {

        x = costheta*rhs.x - sintheta*rhs.y + x;
        y = sintheta*rhs.x + costheta*rhs.y + y;
        theta += rhs.theta;
        costheta = cos(theta);
        sintheta = sin(theta);

        return *this;
    }

    Vector2D Transform2D::translation() const
    {
        Vector2D v;
        v.x = x;
        v.y = y;
        return v;
    }

    double Transform2D::rotation() const
    {
        return theta;
    }

    std::ostream & operator<<(std::ostream & os, const Transform2D & tf)
    {
        double degree;
        degree = rad2deg(tf.theta);
        os<<"deg: "<< degree <<" "<<"x: "<<tf.x<<" "<<"y: "<<tf.y;
        return os;
    }

    std::istream & operator>>(std::istream & is, Transform2D & tf)
    {
        Vector2D v;
        double degree;
        std::string input;

        is >> input;
        is >> degree;
        is >> input;
        is >> v.x;
        is >> input;
        is >> v.y;

        tf = Transform2D(v,deg2rad(degree));
        return is;

    }

    Transform2D operator*(Transform2D lhs, const Transform2D & rhs)
    {
        Transform2D tf;
        tf = lhs;
        tf *= rhs;
        return tf;
    }

}