#include"../include/rigid2d.hpp"
#include<iostream>

/// \brief set up namespaces
using turtlelib::Transform2D;
using turtlelib::Twist2D;
using turtlelib::Vector2D;
using std::cin;
using std::cout;
using std::endl;
using std::string;


int main()
{
    /// \brief initialize parameters
    Transform2D Tab, Tba, Tbc, Tcb, Tac, Tca;
    Vector2D v_a, v_b, v_c, v_bhat;
    Twist2D t_a, t_b, t_c;

    /// \brief input transformations Tab and Tbc
    cout << "Enter transform T_{a,b}: " << endl;
    cin >> Tab;
    cout << "Enter transform T_{b,c}: " << endl;
    cin >> Tbc;

    /// \brief calculate other transformations
    Tba = Tab.inv();
    Tcb = Tbc.inv();
    Tac = Tab*Tbc;
    Tca = Tac.inv();

    /// \brief output all transformations
    cout << "T_{a,b}: " << Tab << endl;
    cout << "T_{b,a}: " << Tba << endl;
    cout << "T_{b,c}: " << Tbc << endl;
    cout << "T_{c,b}: " << Tcb << endl;
    cout << "T_{a,c}: " << Tac << endl;
    cout << "T_{c,a}: " << Tca << endl;

    /// \brief input vector b
    cout << "Enter vector v_b: " << endl;
    cin >> v_b;

    /// \brief calculate other vectors
    v_bhat = v_b.normalize();
    v_a = Tab(v_b);
    v_c = Tcb(v_b);

    /// \brief output all vectors
    cout << "v_bhat: " << v_bhat << endl;
    cout << "v_a: " << v_a << endl;
    cout << "v_b: " << v_b << endl;
    cout << "v_c: " << v_c << endl;

    /// \brief input twist b
    cout << "Enter twist V_b: " << endl;
    cin >> t_b;

    /// \brief calculate other twists
    t_a = Tab(t_b);
    t_c = Tcb(t_b);

    /// \brief output all twists
    cout << "V_a: " << t_a << endl;
    cout << "V_b: " << t_b << endl;
    cout << "V_c: " << t_c << endl;

    return 0;
}