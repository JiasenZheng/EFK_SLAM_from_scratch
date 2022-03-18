#include <armadillo>
#include <catch_ros/catch.hpp>
#include <turtlelib/rigid2d.hpp>
#include <nuslam/nuslam.hpp>

TEST_CASE("Circle fitting algorithm","[landmark]")
{
    turtlelib::Vector2D p1,p2,p3,p4,p5,p6;
    turtlelib::Vector2D pa,pb,pc,pd;
    p1.x = 1;
    p1.y = 7;
    p2.x = 2;
    p2.y = 6;
    p3.x = 5;
    p3.y = 8;
    p4.x = 7;
    p4.y = 7;
    p5.x = 9;
    p5.y = 5;
    p6.x = 3;
    p6.y = 7;
    std::vector<turtlelib::Vector2D> c1{p1,p2,p3,p4,p5,p6};
    pa.x = -1;
    pa.y = 0;
    pb.x = -0.3;
    pb.y = -0.06;
    pc.x = 0.3;
    pc.y = 0.1;
    pd.x = 1;
    pd.y = 0;
    std::vector<turtlelib::Vector2D> c2{pa,pb,pc,pd};
    std::vector<std::vector<turtlelib::Vector2D>> clusters{c1,c2};

    // Circle vector container
    std::vector<turtlelib::Vector2D> circles;
    std::vector<double> Rs;

    for (int i=0; i<clusters.size(); i++) 
    {
        std::vector<turtlelib::Vector2D> cluster = clusters[i];
        int n = cluster.size();

        // compute x y means
        double x_sum = 0.0;
        double y_sum = 0.0;

        for (int j=0; j<n; j++)
        {
            x_sum+=cluster[j].x;
            y_sum+=cluster[j].y;
        }

        double x_mean = x_sum/n;
        double y_mean = y_sum/n;

        // Shift the coordinates and create 3 lists
        std::vector<double> x_list, y_list, z_list;
        double z_sum = 0;
        for (int j=0; j<n; j++)
        {
            double x_val = cluster[j].x - x_mean;
            double y_val = cluster[j].y - y_mean;
            double z_val = pow(x_val,2)+pow(y_val,2);
            x_list.push_back(x_val);
            y_list.push_back(y_val);
            z_list.push_back(z_val);
            z_sum+=z_val;
        }
        double z_mean = z_sum/n;

        // construct Z matrix
        arma::mat z1,z2,z3,z4;
        z1 = arma::colvec(z_list);
        z2 = arma::colvec(x_list);
        z3 = arma::colvec(y_list);
        z4 = arma::mat(n,1,arma::fill::ones);
        arma::mat Z = arma::join_horiz(z1,z2,z3,z4);
        // ROS_INFO("%i:",i);
        // Z.print("Z: ");

        // construct momemt matrix
        arma::mat M = (1/n)*arma::trans(Z)*Z;

        // construct constraint matrix
        arma::mat H= {  {8*z_mean, 0, 0, 2},
                        {0, 1, 0, 0},
                        {0, 0, 1, 0},
                        {2, 0, 0, 0} };
        arma::mat H_inv={{0, 0, 0, 0.5},
                         {0, 1, 0, 0},
                         {0, 0, 1, 0},
                         {0.5, 0, 0, -2*z_mean}  };


        // construct singular value decomposition
        arma::mat U,V;
        arma::vec s;
        arma::svd(U,s,V,Z);

        // solve for A
        arma::vec A;
        // U.print("U: ");
        // s.print("s: ");
        // V.print("V: ");
        // Z.print("Z: ");
        if (s.size()<4)
        {
            continue;
        }
        if (s(3) < 1e-12)
        {
            A = V.col(3);
        }
        else
        {
            arma::mat Y = V*arma::diagmat(s)*arma::trans(V);
            arma::mat Q = Y*H_inv*Y;
            // find eigenvalues and vectors of Q
            arma::vec eigen_vals;
            arma::mat eigen_vecs;
            arma::eig_sym(eigen_vals,eigen_vecs,Q);

            // find the smallest positive eigenvalue of Q
            int idx = 0;
            double eigen_val = 999.0;
            for (int j = 0; j<eigen_vals.size(); j++)
            {
                if (eigen_vals[j]>0 and eigen_vals[j]<eigen_val)
                {
                    eigen_val = eigen_vals[j];
                    idx = j;
                }
            }
            arma::vec A_star = eigen_vecs.col(idx);
            A = arma::solve(Y,A_star);
            
        }

        // Compute circle center and radius
        turtlelib::Vector2D circle;
        circle.x = -A(1)/(2*A(0))+x_mean;
        // ROS_INFO("X: %f", circle.x);
        circle.y = -A(2)/(2*A(0))+y_mean;
        // ROS_INFO("Y: %f", circle.y);
        circles.push_back(circle);

        double R = sqrt((pow(A(1),2) + pow(A(2),2) -4*A(0)*A(3)) / (4*pow(A(0),2)));
        Rs.push_back(R);
    }
    CHECK(circles[0].x == Approx(4.615482));
    CHECK(circles[0].y == Approx(2.807354));
    CHECK(circles[1].x == Approx(0.4908357));
    CHECK(circles[1].y == Approx(-22.15212));
    CHECK(Rs[0] == Approx(4.82757517));
    CHECK(Rs[1] == Approx(22.17979));
}

