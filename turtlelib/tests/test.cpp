// #define CATCH_CONFIG_MAIN
#include<catch_ros/catch.hpp>
// #include"../include/turtlelib/rigid2d.hpp"
#include<turtlelib/rigid2d.hpp>
#include<sstream>


/// \brief set up namespaces
using turtlelib::Transform2D;
using turtlelib::Twist2D;
using turtlelib::Vector2D;
using turtlelib::almost_equal;
using turtlelib::PI;
using std::stringstream;
using std::string;

TEST_CASE("Default constructor","[transform]")
{
    Transform2D tf = Transform2D();
    
    REQUIRE(almost_equal(tf.get_x(),0));
    REQUIRE(almost_equal(tf.get_y(),0));
    REQUIRE(almost_equal(tf.get_theta(),0));
    REQUIRE(almost_equal(tf.get_sin(),0));
    REQUIRE(almost_equal(tf.get_cos(),1));
}

TEST_CASE("Constructor with an angle input","[transform]")
{
    Transform2D tf = Transform2D(PI);
    
    REQUIRE(almost_equal(tf.get_x(),0));
    REQUIRE(almost_equal(tf.get_y(),0));
    REQUIRE(almost_equal(tf.get_theta(),PI));
    REQUIRE(almost_equal(tf.get_sin(),sin(PI)));
    REQUIRE(almost_equal(tf.get_cos(),cos(PI)));
}

TEST_CASE("Constructor with a vector input","[transform]")
{
    Vector2D v;
    v.x = 0;
    v.y = 1;
    Transform2D tf = Transform2D(v);
    
    REQUIRE(almost_equal(tf.get_x(),0));
    REQUIRE(almost_equal(tf.get_y(),1));
    REQUIRE(almost_equal(tf.get_theta(),0));
    REQUIRE(almost_equal(tf.get_sin(),0));
    REQUIRE(almost_equal(tf.get_cos(),1));
}

TEST_CASE("Constructor with a vector input and an angle input","[transform]")
{
    Vector2D v;
    v.x = 0;
    v.y = 1;
    Transform2D tf = Transform2D(v,PI);
    
    REQUIRE(almost_equal(tf.get_x(),0));
    REQUIRE(almost_equal(tf.get_y(),1));
    REQUIRE(almost_equal(tf.get_theta(),PI));
    REQUIRE(almost_equal(tf.get_sin(),sin(PI)));
    REQUIRE(almost_equal(tf.get_cos(),cos(PI)));
}

TEST_CASE("The '()' operator to transform vectors", "[transform]")
{
    Vector2D v_a, v_b;
    v_b.x = 1;
    v_b.y = 0;
    Transform2D tf_ab(PI/2);
    v_a = tf_ab(v_b);

    REQUIRE(almost_equal(v_a.x, 0));
    REQUIRE(almost_equal(v_a.y, 1));
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

    REQUIRE(almost_equal(t_a.omega, 1));
    REQUIRE(almost_equal(t_a.x_dot, 0));
    REQUIRE(almost_equal(t_a.y_dot, 1));
}

TEST_CASE("Inverse method of a transform","[transform]")
{
    Vector2D v;
    v.x = 0;
    v.y = 1;
    Transform2D tf_ab(v,PI/2),tf_ba;
    tf_ba= tf_ab.inv();

    REQUIRE(almost_equal(tf_ba.get_x(),-1));
    REQUIRE(almost_equal(tf_ba.get_y(), 0));
    REQUIRE(almost_equal(tf_ba.get_theta(),-PI/2));

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
    
    REQUIRE(almost_equal(tf_ab.get_x(), 0));
    REQUIRE(almost_equal(tf_ab.get_y(), 2));
    REQUIRE(almost_equal(tf_ab.get_theta(), PI));
}

TEST_CASE("Getter function for translation part of the transform","[transform]")
{
    Vector2D v1,v2;
    v1.x = 0;
    v1.y = 1;
    Transform2D tf(v1);
    v2 = tf.translation();

    REQUIRE(almost_equal(v2.x,0));
    REQUIRE(almost_equal(v2.y,1));
}

TEST_CASE("Getter function for rotation part of the transform","[transform]")
{
    double r;
    Transform2D tf(PI);
    r = tf.rotation();

    REQUIRE(almost_equal(r,PI));
}

TEST_CASE("Getter functions of the private variables in the transform","[transform]")
{
    Vector2D v;
    v.x = 1;
    v.y = 0;
    Transform2D tf(v,PI/2);

    REQUIRE(almost_equal(tf.get_x(),1));
    REQUIRE(almost_equal(tf.get_y(),0));
    REQUIRE(almost_equal(tf.get_theta(),PI/2));
    REQUIRE(almost_equal(tf.get_sin(),sin(PI/2)));
    REQUIRE(almost_equal(tf.get_cos(),cos(PI/2)));
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
    REQUIRE(str == "deg:");
    ss >> str;
    REQUIRE(str == "90");
    ss >> str;
    REQUIRE(str == "x:");
    ss >> str;
    REQUIRE(str == "0");
    ss >> str;
    REQUIRE(str == "y:");
    ss >> str;
    REQUIRE(str == "1");
}

TEST_CASE("The '>>' operator of the transform", "[transform]")
{
    string str;
    stringstream ss;
    Transform2D tf;

    str = "deg: 90 x: 0 y: 1";
    ss << str;
    ss >> tf;

    REQUIRE(almost_equal(tf.get_x(),0));
    REQUIRE(almost_equal(tf.get_y(),1));
    REQUIRE(almost_equal(tf.get_theta(),PI/2));
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
    
    REQUIRE(almost_equal(tf_ac.get_x(), 0));
    REQUIRE(almost_equal(tf_ac.get_y(), 2));
    REQUIRE(almost_equal(tf_ac.get_theta(), PI));
}

