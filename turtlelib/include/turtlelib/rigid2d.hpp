#ifndef RIGID2D_INCLUDE_GUARD_HPP
#define RIGID2D_INCLUDE_GUARD_HPP
/// \file
/// \brief Two-dimensional rigid body transformations.


#include<iosfwd> // contains forward definitions for iostream objects
#include<cmath>  // compute common mathematical operations
#include<string> // C++ string library

namespace turtlelib
{
    /// \brief PI.  Not in C++ standard until C++20.
    constexpr double PI=3.14159265358979323846;

    /// \brief approximately compare two floating-point numbers using
    ///        an absolute comparison
    /// \param d1 - a number to compare
    /// \param d2 - a second number to compare
    /// \param epsilon - absolute threshold required for equality
    /// \return true if abs(d1 - d2) < epsilon
    /// NOTE: implement this in the header file
    /// constexpr means that the function can be computed at compile time
    /// if given a compile-time constant as input
    constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-12)
    {
        if (fabs(d1-d2)< epsilon)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    /// \brief convert degrees to radians
    /// \param deg - angle in degrees
    /// \returns radians
    constexpr double deg2rad(double deg)
    {
        return (deg*PI)/180;
    }

    /// \brief convert radians to degrees
    /// \param rad - angle in radians
    /// \returns the angle in degrees
    constexpr double rad2deg(double rad)
    {
        return (rad*180)/PI;
    }

    /// static_assertions test compile time assumptions.
    /// You should write at least one more test for each function
    /// You should also purposely (and temporarily) make one of these tests fail
    /// just to see what happens
    static_assert(almost_equal(0, 0), "is_zero failed");
    // static_assert(almost_equal(1, 1.5, 0.5), "almost_euqal failed");
    static_assert(almost_equal(1.0, 1.1, 0.5), "almost_euqal failed");
    // static_assert(false, "almost_euqal failed");

    static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");
    static_assert(almost_equal(deg2rad(180.0), PI), "deg2rad failed");
    // static_assert(1 == 2, "deg2rad failed");


    static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg failed");
    static_assert(almost_equal(rad2deg(PI), 180), "rad2deg failed");

    static_assert(almost_equal(deg2rad(rad2deg(2.1)), 2.1), "deg2rad failed");
    static_assert(almost_equal(deg2rad(rad2deg(3.14)), 3.14), "deg2rad failed");

    /// \brief A 2-Dimensional Vector
    struct Vector2D
    {
        /// \brief the x coordinate
        
        double x = 0.0;

        /// \brief the y coordinate
        double y = 0.0;

    

        /// \brief normalize a 2D vector
        /// \param v - an input 2D vector
        /// \return Vector2D - the output normalized vector
        Vector2D normalize();  

        /// \brief += operator for Vector2D
        /// \param rhs - an input 2D vector on the righ hand side
        /// \return Vector2D - the output 2D calculated vector
        Vector2D & operator+=(const Vector2D & rhs);

        /// \brief -= operator for Vector2D
        /// \param rhs - an input 2D vector on the righ hand side
        /// \return Vector2D - the output 2D calculated vector
        Vector2D & operator-=(const Vector2D & rhs);

        /// \brief *= operator for Vector2D
        /// \param rhs - an input 2D vector on the righ hand side
        /// \return Vector2D - the output 2D calculated vector
        Vector2D & operator*=(const double & rhs);

        /// \brief dot product of two vector
        /// \param
        /// \return 
        double dot(const Vector2D & rhs);

        /// \brief magnitude of the vector
        /// \return calculated magnitude
        double mag();
    };

    /// \brief output a 2 dimensional vector as [xcomponent ycomponent]
    /// os - stream to output to
    /// v - the vector to print
    std::ostream & operator<<(std::ostream & os, const Vector2D & v);

    /// \brief + operator for Vectpr2D
    /// 
    /// \param lhs - an input 2D vector 
    /// \param rhs - an input 2D vector
    /// \return Vector2D - the output 2D calculated vector
    Vector2D operator+(const Vector2D &lhs, const Vector2D &rhs);

    /// \brief - operator for Vectpr2D
    /// 
    /// \param lhs - an input 2D vector 
    /// \param rhs - an input 2D vector
    /// \return Vector2D - the output 2D calculated vector
    Vector2D operator-(const Vector2D &lhs, const Vector2D &rhs);

    /// \brief * operator for Vectpr2D
    /// 
    /// \param lhs - a scalar
    /// \param rhs - an input 2D vector
    /// \return Vector2D - the output 2D calculated vector
    Vector2D operator*(const double &lhs, const Vector2D &rhs);

    /// \brief * operator for Vectpr2D
    /// 
    /// \param lhs - an input 2D vector
    /// \param rhs - a scalar
    /// \return Vector2D - the output 2D calculated vector
    Vector2D operator*(const Vector2D &lhs, const double &rhs);

    double angle(Vector2D &lhs, Vector2D &rhs);

    /// \brief input a 2 dimensional vector
    ///   You should be able to read vectors entered as follows:
    ///   [x y] or x y
    /// is - stream from which to read
    /// v [out] - output vector
    /// Hint: The following may be useful:
    /// https://en.cppreference.com/w/cpp/io/basic_istream/peek
    /// https://en.cppreference.com/w/cpp/io/basic_istream/get
    ///
    /// The way input works is (more or less): what the user types is stored in a buffer until the user enters
    /// a newline (by pressing enter).  The iostream methods then process the data in this buffer character by character.
    /// Typically, each character is examined and then removed from the buffer automatically.
    /// If the characters don't match what is expected (e.g., we are expecting an int but the letter 'q' is encountered)
    /// an error flag is set on the stream object (e.g., std::cin).
    ///
    /// We have lower level control however. For example:
    /// peek looks at the next unprocessed character in the buffer without removing it
    /// get removes the next unprocessed character from the buffer.
    std::istream & operator>>(std::istream & is, Vector2D & v);


    /// \brief A 2-Dimensional Twist
    struct Twist2D
    {
        /// \brief the angular velocity
        double omega = 0.0;  

        /// \brief the x velocity
        double x_dot = 0.0;

        /// \brief the y velocity
        double y_dot = 0.0;
    };

    /// \brief output a 2 dimensional Twist as [omega x_dot y_dot]
    /// os - stream to output to
    /// t - the vector to print
    std::ostream & operator<<(std::ostream & os, const Twist2D & t);

    /// \brief input a 2 dimensional Twist as [omega x_dot y_dot]
    /// is - stream to input to
    /// t - the twist to input
    std::istream & operator>>(std::istream & is, Twist2D & t);

    /// \brief a rigid body transformation in 2 dimensions
    class Transform2D
    {
        public:
            /// \brief Create an identity transformation
            Transform2D();

            /// \brief create a transformation that is a pure translation
            /// \param trans - the vector by which to translate
            explicit Transform2D(Vector2D trans);

            /// \brief create a pure rotation
            /// \param radians - angle of the rotation, in radians
            explicit Transform2D(double radians);

            /// \brief Create a transformation with a translational and rotational
            /// component
            /// \param trans - the translation
            /// \param radians - the rotation, in radians
            Transform2D(Vector2D trans, double radians);

            /// \brief Create a transformation with x,y-coordinates and rotational
            /// component
            /// \param x_coord - x-coordinate
            /// \param y_coord - y-coordinate
            /// \param radians the rotation, in radians
            Transform2D(double x_coord, double y_coord, double radians);

            /// \brief apply a transformation to a Vector2D
            /// \param v - the vector to transform
            /// \return a vector in the new coordinate system
            Vector2D operator()(Vector2D v) const;

            /// \brief apply a transformation to a Twist2D
            /// \param t - the twist to transform
            /// \return a twist in the new coordinate system
            Twist2D operator()(Twist2D t) const;


            /// \brief invert the transformation
            /// \return the inverse transformation. 
            Transform2D inv() const;

            /// \brief compose this transform with another and store the result 
            /// in this object
            /// \param rhs - the first transform to apply
            /// \return a reference to the newly transformed operator
            Transform2D & operator*=(const Transform2D & rhs);

            /// \brief the translational component of the transform
            /// \return the x,y translation
            Vector2D translation() const;

            /// \brief get the angular displacement of the transform
            /// \return the angular displacement, in radians
            double rotation() const;

            /// \brief \see operator<<(...) (declared outside this class)
            /// for a description
            friend std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

            /// \brief getter function for x
            double get_x() const;

            /// \brief getter function for y
            double get_y() const;



            /// \brief getter function for sintheta
            double get_sin() const;

            /// \brief getter function for costheta
            double get_cos() const;


        private:
            /// \brief x-coordinate
            double x = 0.0;
            /// \brief y-coordinate
            double y = 0.0;
            /// \brief 2D rotational angle theta
            double theta = 0.0;
            /// \brief sin(theta)
            double sintheta = 0.0;
            /// \brief cos(theta)
            double costheta = 1.0;

    };


    /// \brief should print a human readable version of the transform:
    /// An example output:
    /// deg: 90 x: 3 y: 5
    /// \param os - an output stream
    /// \param tf - the transform to print
    std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

    /// \brief Read a transformation from stdin
    /// Should be able to read input either as output by operator<< or
    /// as 3 numbers (degrees, dx, dy) separated by spaces or newlines
    std::istream & operator>>(std::istream & is, Transform2D & tf);

    /// \brief multiply two transforms together, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the composition of the two transforms
    /// HINT: This function should be implemented in terms of *=
    Transform2D operator*(const Transform2D & lhs, const Transform2D & rhs);

    /// 
    /// \brief convert angle to equivalent angle angle in the interval (-pi,pi]
    /// 
    /// \param rad input angle in radians
    /// \return output angle in the interval 
    double normalize_angle(double & rad);

    Transform2D integrate_twist( const Twist2D &t);
}

#endif
