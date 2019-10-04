// STATES:
// px = The current location in the x-axis of an arbitrary global map coordinate system
// py = The current location in the y-axis of an arbitrary global map coordinate system
// pz = The current location in the z-axis of an arbitrary global map coordinate system

// psi = The current orientation / heading of the vehicle
// v_x = The current velocity/speed of the vehicle in x direction.
// v_y = The current velocity/speed of the vehicle in y direction.
// v_z = The current velocity/speed of the vehicle in z direction.

// cte_track = This is the cross track error which is the difference between our desired position and actual position.
//        We can use our fitted polynomial at point px = 0 to get the position where we should currently be.
// cte_track = f(0)

// cte_goal = This is the cross track error which is the difference between our desired goal position and actual position.
// cte_goal = f(0)


// epsi = This is the orientation error which is the difference between our desired heading and actual heading. Our desired orientation is the heading tangent to our road curve.
//        This can be computed using an arctan to the derivative of the fitted polynomial function at point px = 0 to get the angle to which we should be heading.
// epsi = arctan(f`(0)) where f` is the derivative of f


// ACTUATIONS:
// Steering, forward/backward, right/left AND THROTTLE (up/down).

// delta = is steering (rotating around yaw), where 1 is counter clockwise and -1 is clockwise.
// a_x = acceleration forward/backward, where 1 is forward and -1 is backwards
// a_y = acceleration right/left, where 1 is left and -1 is right
// a_z = acceleration up/down, where 1 is up and -1 is down (meter/s)

#ifndef MPC_H
#define MPC_H

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <stddef.h>
//
//#include <ifopt/ipopt_solver.h>
//#include <ifopt/solver.h>
//#include <ifopt/problem.h>
//#include <ifopt/bounds.h>
//#include <ifopt/composite.h>
//#include <ifopt/constraint_set.h>
//#include <ifopt/cost_term.h>
//#include <ifopt/test_vars_constr_cost.h>
//#include <ifopt/variable_set.h>



#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <martin_msg_lib/Segment.h>
#include <path_navigator/line.h>



typedef CPPAD_TESTVECTOR(double) Dvector;

const int N = 15; // how many states we "lookahead" in the future
const double dt = 0.07; // how much time we expect environment changes

const double Lf = 2.67; // this is the length from front of vehicle to Center-of-Gravity
const double VELOCITY_MAX = 1; // this is what we ideally want our speed to always be

const int NUMBER_OF_STATES = 14; // px, py,pz, psi, v_x, v_y, v_z, cte_track, cte_goal, epsi
const int NUMBER_OF_ACTUATIONS = 4; // steering angle, acceleration

const int NX = N * NUMBER_OF_STATES + (N) * NUMBER_OF_ACTUATIONS; // number of state + actuation variables
const int NG = N * NUMBER_OF_STATES; // number of constraints




// where the first element of each state variable is stored in the vector to be feeded the optimization algorithm
const int ID_FIRST_px = 0;
const int ID_FIRST_py = ID_FIRST_px + N;
const int ID_FIRST_pz = ID_FIRST_py + N;
const int ID_FIRST_psi = ID_FIRST_pz + N;
const int ID_FIRST_v_x = ID_FIRST_psi + N;
const int ID_FIRST_v_y = ID_FIRST_v_x + N;
const int ID_FIRST_v_z = ID_FIRST_v_y + N;
const int ID_FIRST_cte_track_x = ID_FIRST_v_z + N;
const int ID_FIRST_cte_track_y = ID_FIRST_cte_track_x + N;
const int ID_FIRST_cte_track_z = ID_FIRST_cte_track_y + N;
const int ID_FIRST_cte_goal_x = ID_FIRST_cte_track_z + N;
const int ID_FIRST_cte_goal_y = ID_FIRST_cte_goal_x + N;
const int ID_FIRST_cte_goal_z = ID_FIRST_cte_goal_y + N;
const int ID_FIRST_epsi = ID_FIRST_cte_goal_z + N;

// actuators
const int ID_FIRST_delta = ID_FIRST_epsi + N;
const int ID_FIRST_a_x = ID_FIRST_delta + N;
const int ID_FIRST_a_y = ID_FIRST_a_x + N;
const int ID_FIRST_a_z = ID_FIRST_a_y + N - 1;


// weights for cost computations
const double W_cte_track_x = 1500.0;
const double W_cte_track_y = 1500.0;
const double W_cte_track_z = 1000.0;
const double W_cte_goal_x = 1000.0;
const double W_cte_goal_y = 1000.0;
const double W_cte_goal_z = 500.0;
const double W_epsi = 1000;
const double W_v = 1;
const double W_delta = 10;
const double W_a = 1;
const double W_ddelta = 150; // weight cost for high difference between consecutive steering actuations
const double W_da = 1; // weight cost for high difference between consecutive acceleration actuations

class MPC {

public:

    double yaw_rotation;
    double a_x; //acceleration forward/backward, where 1 is forward and -1 is backwards
    double a_y; //acceleration right/left, where 1 is left and -1 is right
    double a_z; //acceleration up/down, where 1 is up and -1 is down (meter/s)

    Dvector x; // where all the state and actuation variables will be stored
    Dvector x_lowerbound; //lower limit for each corresponding variable in x
    Dvector x_upperbound; //upper limit for each corresponding variable in x
    Dvector g_lowerbound; // value constraint for each corresponding constraint expression
    Dvector g_upperbound; // value constraint for each corresponding constraint expression

    std::vector<double> future_xs;
    std::vector<double> future_ys;
    std::vector<double> future_zs;


    MPC();

    virtual ~MPC();

    // this function solves the model given the current state and road curve coefficients.
    void solve(Eigen::VectorXd state, Eigen::VectorXd K_x, Eigen::VectorXd K_y, Eigen::VectorXd K_z,
               martin_msg_lib::Segment currentSegment, double lf);
};

#endif /* MPC_H */
