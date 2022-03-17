#ifndef NUSLAM_HPP
#define NUSLAM_HPP

#include <armadillo>
#include <turtlelib/rigid2d.hpp>

namespace nuslam
{
    class EKF
    {
        public:

            /**
             * \brief Construct a new EKF object (Unknown Data Association)
             * 
            **/
            EKF();

            /**
             * \brief Construct a new EKF object (Known Data Association)
             * 
             * \param num 
            **/
            EKF(int num);

            /**
             * \brief Calculate At matrix
             * 
             * \param t twist
             * \return arma::mat At
            **/
            arma::mat compute_At(turtlelib::Twist2D t);

            /**
             * \brief predict state and uncertainty
             * 
             * \param t twist
             * \param tf odometry
            **/
            void predict(turtlelib::Twist2D t, turtlelib::Transform2D tf);

            /**
             * \brief Compute the theoretical measurement, given the current state estimation
             * 
             * \param j idex of the landmark
             * \return arma::mat zt: theoretical measurement
            **/
            arma::mat compute_z(int j);

            /**
             * \brief Compute H
             * 
             * \param j index of the landmark
             * \return arma::mat H: derivative of the measurement wrt the state
            **/
            arma::mat compute_H(int j);
            
            /**
             * \brief Compute H with temp matrix
             * 
             * \param j index of the landmark
             * \param temp temp matrix
             * \return arma::mat H: derivative of the measurement wrt the state
            **/
            arma::mat compute_H(int j,const arma::mat &temp);

            /**
             * \brief update state and uncertainty
             * 
             * \param j index of landmark
             * \param z measurements
            **/
            void update(int j, arma::mat z);

            /**
             * \brief add a landmark to the state
             * 
             * \param j index of landmark
             * \param lm landmark position
            **/
            void add_landmark(int j, turtlelib::Vector2D lm);

            /**
             * \brief Get the state object
             * 
             * \return arma::mat state matrix
            **/
            arma::mat get_state();


            /**
             * \brief associate measurements with landmark number
             * 
             * \param z measurement
             * \return int landmark number
            **/
            int assoc_data(arma::mat z);


            int N = 0;  // The number of landmarks already seen

        private:
            int n;  // The number of landmarks with data associated
            arma::mat sigma;
            arma::mat state;
            arma::mat Q;
            arma::mat R;
   
    };

    /**
     * \brief convert from polar to cartesian
     * 
     * \param range range
     * \param deg bearing
     * \return turtlelib::Vector2D cartesian coordinates
    **/
    turtlelib::Vector2D toXY(double range, double deg);
}

#endif