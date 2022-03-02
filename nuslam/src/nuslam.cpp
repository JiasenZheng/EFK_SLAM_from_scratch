#include <nuslam/nuslam.hpp>

namespace nuslam
{
    
    EKF::EKF(int num)
    {
        n = num;
        arma::mat q(3,3,arma::fill::zeros);
        arma::mat ur(3,2*n,arma::fill::zeros);
        arma::mat bl(2*n,3,arma::fill::zeros);
        arma::mat m = arma::mat(2*n,2*n,arma::fill::eye)*999999.0;
        arma::mat left = arma::join_cols(q,bl);
        arma::mat right = arma::join_cols(ur,m);
        sigma = arma::join_rows(left,right);

        state = arma::mat(2*n+3,1,arma::fill::zeros);

        Q = arma::mat(3,3,arma::fill::eye)*1.0;
        R = arma::mat(2,2,arma::fill::eye)*0.00001;
    }

    arma::mat EKF::compute_At(turtlelib::Twist2D t)
    {
        arma::mat At,ul;
        // arma::mat I = arma::mat(2*n+3,2*n+3,arma::fill::eye);
        arma::mat ur = arma::mat(3,2*n,arma::fill::zeros);
        arma::mat bl = arma::mat(2*n,3,arma::fill::zeros);
        arma::mat br = arma::mat(2*n,2*n,arma::fill::zeros);
        double theta = state(0,0);
        if (turtlelib::almost_equal(t.omega,0))
        {
            ul = 
            {
                {0,0,0},
                {-t.x_dot*sin(theta),0,0},
                {t.x_dot*cos(theta),0,0}
            };
        }
        else
        {
            ul = 
            {
                {0,0,0},
                {-(t.x_dot/t.omega)*cos(theta)+(t.x_dot/t.omega)*cos(theta+t.omega),0,0},
                {-(t.x_dot/t.omega)*sin(theta)+(t.x_dot/t.omega)*sin(theta+t.omega),0,0}
            };
        }
        arma::mat left = arma::join_cols(ul,bl);
        arma::mat right = arma::join_cols(ur,br);
        At = arma::join_rows(left,right);
        return At;
    }
    void EKF::predict(turtlelib::Twist2D t, turtlelib::Transform2D tf)
    {
        // update the estimate
        state(0,0) = tf.rotation();
        state(1,0) = tf.get_x();
        state(2,0) = tf.get_y();

        // compute Q_bar
        arma::mat ur = arma::mat(3,2*n,arma::fill::zeros);
        arma::mat bl = arma::mat(2*n,3,arma::fill::zeros);
        arma::mat br = arma::mat(2*n,2*n,arma::fill::zeros);
        arma::mat left = arma::join_cols(Q,bl);
        arma::mat right = arma::join_cols(ur,br);
        arma::mat Q_bar = arma::join_rows(left,right);

        // propagate the uncertainty
        arma::mat At = this->compute_At(t);
        arma::mat sigma_m = At*sigma*At.t()+Q_bar;
        sigma = sigma_m;
    }

    arma::mat EKF::compute_z(int j)
    {
        arma::mat z = arma::mat(2,1,arma::fill::zeros);
        // landmark coordinates
        double xj = state(1+2*j,0);
        double yj = state(2+2*j,0);
        // robot coordinates
        double xt = state(1,0);
        double yt = state(2.0);
        double thetat = state(0,0);
        // compute range and bearing
        double r = sqrt(pow(xj-xt,2) + pow(yj-yt,2));
        double phi = atan2(yj-yt,xj-xt)-thetat;
        phi = turtlelib::normalize_angle(phi);
        z(0,0) = r;
        z(1,0) = phi;

        return z;
    }

    arma::mat EKF::compute_H(int j)
    {
        // landmark coordinates
        double xj = state(1+2*j,0);
        double yj = state(2+2*j,0);
        // robot coordinates
        double xt = state(1,0);
        double yt = state(2.0);
        double thetat = state(0,0);
        // relative coordinates
        double dx = xj - xt;
        double dy = yj - yt;
        double d = pow(dx,2) + pow(dy,2);
        // H matrix
        arma::mat H,m1,m2,m3,m4;
        m1 = 
        {
            {0, -dx/sqrt(d),-dy/sqrt(d)},
            {-1, dy/d, -dx/d}
        };
        m2 = arma::mat(2,2*(j-1),arma::fill::zeros);
        m3 = 
        {
            {dx/sqrt(d),dy/sqrt(d)},
            {-dy/d,dx/d}
        };
        m4 = arma::mat(2,2*n-2*j,arma::fill::zeros);
        H = arma::join_rows(m1,m2,m3,m4);
        return H;
    }

    void EKF::update(int j, arma::mat z)
    {
        // calculate kalman gain
        arma::mat z_h = compute_z(j);
        arma::mat H = compute_H(j);
        arma::mat Ki = sigma*H.t()*(H*sigma*H.t()+R).i();
        // compute the posterior state update
        arma::mat delta = z-z_h;
        delta(1,0) = turtlelib::normalize_angle(delta(1,0));
        state = state + Ki*delta;
        // compute the posterior covariance
        arma::mat I = arma::mat(2*n+3,2*n+3,arma::fill::eye);
        sigma = (I-Ki*H)*sigma;
    }

    void EKF::add_landmark(int j, turtlelib::Vector2D lm)
    {
        state(2*j+1,0) = lm.x;
        state(2*j+2,0) = lm.y;
    }

    arma::mat EKF::get_state()
    {
        return state;
    }


}