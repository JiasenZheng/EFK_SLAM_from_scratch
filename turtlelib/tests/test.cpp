// #define CATCH_CONFIG_MAIN
#include<catch_ros/catch.hpp>
#include"turtlelib/rigid2d.hpp"
#include"turtlelib/diff_drive.hpp"
#include<sstream>


/// \brief set up namespaces
using turtlelib::Transform2D;
using turtlelib::DiffDrive;
using turtlelib::Twist2D;
using turtlelib::Vector2D;
using turtlelib::Velocity;
using turtlelib::almost_equal;
using turtlelib::normalize_angle;
using turtlelib::angle;
using turtlelib::PI;
using std::stringstream;
using std::string;

TEST_CASE("Default constructor","[transform]")
{
    Transform2D tf = Transform2D();
    
    CHECK(almost_equal(tf.get_x(),0));
    CHECK(almost_equal(tf.get_y(),0));
    CHECK(almost_equal(tf.rotation(),0));
    CHECK(almost_equal(tf.get_sin(),0));
    CHECK(almost_equal(tf.get_cos(),1));
}

TEST_CASE("Constructor with an angle input","[transform]")
{
    Transform2D tf = Transform2D(PI);
    
    CHECK(almost_equal(tf.get_x(),0));
    CHECK(almost_equal(tf.get_y(),0));
    CHECK(almost_equal(tf.rotation(),PI));
    CHECK(almost_equal(tf.get_sin(),sin(PI)));
    CHECK(almost_equal(tf.get_cos(),cos(PI)));
}

TEST_CASE("Constructor with a vector input","[transform]")
{
    Vector2D v;
    v.x = 0;
    v.y = 1;
    Transform2D tf = Transform2D(v);
    
    CHECK(almost_equal(tf.get_x(),0));
    CHECK(almost_equal(tf.get_y(),1));
    CHECK(almost_equal(tf.rotation(),0));
    CHECK(almost_equal(tf.get_sin(),0));
    CHECK(almost_equal(tf.get_cos(),1));
}

TEST_CASE("Constructor with a vector input and an angle input","[transform]")
{
    Vector2D v;
    v.x = 0;
    v.y = 1;
    Transform2D tf = Transform2D(v,PI);
    
    CHECK(almost_equal(tf.get_x(),0));
    CHECK(almost_equal(tf.get_y(),1));
    CHECK(almost_equal(tf.rotation(),PI));
    CHECK(almost_equal(tf.get_sin(),sin(PI)));
    CHECK(almost_equal(tf.get_cos(),cos(PI)));
}

TEST_CASE("The '()' operator to transform vectors", "[transform]")
{
    Vector2D v_a, v_b;
    v_b.x = 1;
    v_b.y = 0;
    Transform2D tf_ab(PI/2);
    v_a = tf_ab(v_b);

    CHECK(almost_equal(v_a.x, 0));
    CHECK(almost_equal(v_a.y, 1));
}

TEST_CASE("The '()' operator to transform twists", "[transform]")
{
    Twist2D t_a, t_b;
    Vector2D v;
    t_b.omega = 1;
    t_b.x_dot = 1;
    t_b.y_dot = 1;
    v.x = 0;
    v.y = 1;
    Transform2D tf_ab(v,PI/2);
    t_a = tf_ab(t_b);

    CHECK(almost_equal(t_a.omega, 1));
    CHECK(almost_equal(t_a.x_dot, 0));
    CHECK(almost_equal(t_a.y_dot, 1));
}

TEST_CASE("Inverse method of a transform","[transform]")
{
    Vector2D v;
    v.x = 0;
    v.y = 1;
    Transform2D tf_ab(v,PI/2),tf_ba;
    tf_ba= tf_ab.inv();

    CHECK(almost_equal(tf_ba.get_x(),-1));
    CHECK(almost_equal(tf_ba.get_y(), 0));
    CHECK(almost_equal(tf_ba.rotation(),-PI/2));

}

TEST_CASE("The '*=' operator of the transform", "[transform]")
{
    Vector2D v1,v2;
    v1.x = 0;
    v1.y = 1;
    v2.x = 1;
    v2.y = 0;
    Transform2D tf_ab(v1,PI/2),tf_bc(v2,PI/2);
    tf_ab*=tf_bc;
    
    CHECK(almost_equal(tf_ab.get_x(), 0));
    CHECK(almost_equal(tf_ab.get_y(), 2));
    CHECK(almost_equal(tf_ab.rotation(), PI));
}

TEST_CASE("Getter function for translation part of the transform","[transform]")
{
    Vector2D v1,v2;
    v1.x = 0;
    v1.y = 1;
    Transform2D tf(v1);
    v2 = tf.translation();

    CHECK(almost_equal(v2.x,0));
    CHECK(almost_equal(v2.y,1));
}

TEST_CASE("Getter function for rotation part of the transform","[transform]")
{
    double r;
    Transform2D tf(PI);
    r = tf.rotation();

    CHECK(almost_equal(r,PI));
}

TEST_CASE("Getter functions of the private variables in the transform","[transform]")
{
    Vector2D v;
    v.x = 1;
    v.y = 0;
    Transform2D tf(v,PI/2);

    CHECK(almost_equal(tf.get_x(),1));
    CHECK(almost_equal(tf.get_y(),0));
    CHECK(almost_equal(tf.rotation(),PI/2));
    CHECK(almost_equal(tf.get_sin(),sin(PI/2)));
    CHECK(almost_equal(tf.get_cos(),cos(PI/2)));
}

TEST_CASE("The '<<' operator of the transform", "[transform]")
{
    Vector2D v;
    v.x = 0;
    v.y = 1;
    Transform2D tf(v,PI/2);
    stringstream ss;
    string str;

    ss << tf;
    ss >> str;   
    CHECK(str == "deg:");
    ss >> str;
    CHECK(str == "90");
    ss >> str;
    CHECK(str == "x:");
    ss >> str;
    CHECK(str == "0");
    ss >> str;
    CHECK(str == "y:");
    ss >> str;
    CHECK(str == "1");
}

TEST_CASE("The '>>' operator of the transform", "[transform]")
{
    string str;
    stringstream ss;
    Transform2D tf;

    str = "deg: 90 x: 0 y: 1";
    ss << str;
    ss >> tf;

    CHECK(almost_equal(tf.get_x(),0));
    CHECK(almost_equal(tf.get_y(),1));
    CHECK(almost_equal(tf.rotation(),PI/2));
}

TEST_CASE("The '*' operator of the transform", "[transform]")
{
    Vector2D v1,v2;
    v1.x = 0;
    v1.y = 1;
    v2.x = 1;
    v2.y = 0;
    Transform2D tf_ab(v1,PI/2),tf_bc(v2,PI/2),tf_ac;
    tf_ac=tf_ab*tf_bc;
    
    CHECK(almost_equal(tf_ac.get_x(), 0));
    CHECK(almost_equal(tf_ac.get_y(), 2));
    CHECK(almost_equal(tf_ac.rotation(), PI));
}

TEST_CASE("Normalize angle","[transform]")
{
    double r1=PI, r2=-PI, r3=0,r4=-PI/4, r5=3*PI/2, r6=-5*PI/2;

    CHECK(almost_equal(normalize_angle(r1),PI));
    CHECK(almost_equal(normalize_angle(r2),PI));
    CHECK(almost_equal(normalize_angle(r3),0));
    CHECK(almost_equal(normalize_angle(r4),-PI/4));
    CHECK(almost_equal(normalize_angle(r5),-PI/2));
    CHECK(almost_equal(normalize_angle(r6),-PI/2));
}

TEST_CASE("Operations in Vector2D","[transform]")
{
    Vector2D v1, v2, v3, v4;
    double i = 2.0;
    v1.x = 1.0;
    v1.y = 1.0;
    v2.x = 2.0;
    v2.y = 2.0;
    v4.x = -1.0;
    v4.y = 1.0;

    v1+=v2;
    CHECK(almost_equal(v1.x,3.0));
    CHECK(almost_equal(v1.y,3.0));
    v1-=v2;
    CHECK(almost_equal(v1.x,1.0));
    CHECK(almost_equal(v1.y,1.0));
    v1*=i;
    CHECK(almost_equal(v1.x,2.0));
    CHECK(almost_equal(v1.y,2.0));
    v3 = v1+v2;
    CHECK(almost_equal(v3.x,4.0));
    CHECK(almost_equal(v3.y,4.0));
    v3 = v1-v2;
    CHECK(almost_equal(v3.x,0.0));
    CHECK(almost_equal(v3.y,0.0));
    v3 = i*v2;
    CHECK(almost_equal(v3.x,4.0));
    CHECK(almost_equal(v3.y,4.0));
    v3 = v2*i;
    CHECK(almost_equal(v3.x,4.0));
    CHECK(almost_equal(v3.y,4.0));
    i = v1.dot(v2);
    CHECK(almost_equal(i,8.0));
    i = v2.mag();
    CHECK(almost_equal(i,sqrt(8.0)));
    i = angle(v2,v4);
    CHECK(almost_equal(i,PI/2));
}

TEST_CASE("Integrate Twist","[transform]")
{
    Twist2D t;
    t.omega = 1.5;
    t.x_dot = 0.0;
    t.y_dot = 0.0;
    Transform2D tf1 = integrate_twist(t);
    CHECK(almost_equal(tf1.rotation(),1.5));
    CHECK(almost_equal(tf1.get_x(),0.0));
    CHECK(almost_equal(tf1.get_y(),0.0));
    t.omega = 0.0;
    t.x_dot = 1.0;
    t.y_dot = 2.0;
    Transform2D tf2 = integrate_twist(t);
    CHECK(almost_equal(tf2.rotation(),0.0));
    CHECK(almost_equal(tf2.get_x(),1.0));
    CHECK(tf2.get_y()==Approx(2.0));

    // CHECK(almost_equal(tf2.get_y(),1.0));
    t.omega = PI/2;
    t.x_dot = 2.0;
    t.y_dot = 1.0;
    Transform2D tf3 = integrate_twist(t);
    CHECK(almost_equal(tf3.rotation(),PI/2));
    CHECK(tf3.get_x()==Approx(0.6366197724));
    CHECK(tf3.get_y()==Approx(1.9098593171));
}

TEST_CASE("Forward kinematics","[diff-drive]")
{
    Velocity v;
    Twist2D t;
    DiffDrive dd = DiffDrive();
    v.left = 1.5;
    v.right = 1.5;
    t = dd.forward_kinematics(v);
    CHECK(t.x_dot==Approx(0.0495));
    v.left = -1.2;
    v.right = 1.2;
    t = dd.forward_kinematics(v);
    CHECK(t.omega==Approx(0.495));
    v.left = -1.0606060606;
    v.right = 1.3636363636;
    t = dd.forward_kinematics(v);
    CHECK(t.x_dot==Approx(0.005));
    CHECK(t.omega==Approx(0.5));
}

TEST_CASE("Inverse kinematics","[diff-drive]")
{   
    Transform2D tf=Transform2D();
    DiffDrive dd = DiffDrive();
    Twist2D t;
    Velocity v;
    t.y_dot = 0.0;
    t.x_dot = 0.05;
    t.omega = 0.0;
    v = dd.inverse_kinematics(t);
    CHECK(v.left==Approx(1.5151515152));
    CHECK(v.right==Approx(1.5151515152));
    t.x_dot = -0.05;
    v = dd.inverse_kinematics(t);
    CHECK(v.left==Approx(-1.5151515152));
    CHECK(v.right==Approx(-1.5151515152));
    t.x_dot = 0.0;
    t.omega = 0.5;
    v = dd.inverse_kinematics(t);
    CHECK(v.left==Approx(-1.2121212121));
    CHECK(v.right==Approx(1.2121212121));
    t.omega = -0.5;
    v = dd.inverse_kinematics(t);
    CHECK(v.left==Approx(1.2121212121));
    CHECK(v.right==Approx(-1.2121212121));
    t.x_dot = 0.005;
    t.omega = 0.5;
    v = dd.inverse_kinematics(t);
    CHECK(v.left==Approx(-1.0606060606));
    CHECK(v.right==Approx(1.3636363636));
    t.omega = -0.5;
    v = dd.inverse_kinematics(t);
    CHECK(v.left==Approx(1.3636363636));
    CHECK(v.right==Approx(-1.0606060606));
    t.y_dot = 0.005;
    CHECK_THROWS(v = dd.inverse_kinematics(t));
}
