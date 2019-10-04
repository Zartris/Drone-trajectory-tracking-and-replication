//#include <utility>

#include "path_navigator/MPC.h"

using CppAD::AD;
using namespace std;

class FG_eval {

public:

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    Eigen::VectorXd K_x; // Fitted road curve polynomial coefficients
    Eigen::VectorXd K_y; // Fitted road curve polynomial coefficients
    Eigen::VectorXd K_z; // Fitted road curve polynomial coefficients
    path_nav_math::Line line1;
    path_nav_math::Line line2;
    double Lf;


    FG_eval(Eigen::VectorXd Kin_x, Eigen::VectorXd Kin_y, Eigen::VectorXd Kin_z, martin_msg_lib::Segment segment,
            double lf)
            : K_x(std::move(Kin_x)), K_y(std::move(Kin_y)), K_z(std::move(Kin_z)) {
        Lf = lf;
        geometry_msgs::Point p1;
        p1.x = segment.p1.position.x;
        p1.y = segment.p1.position.y;
        p1.z = segment.p1.position.z;
        geometry_msgs::Point p2;
        p2.x = segment.p2.position.x;
        p2.y = segment.p2.position.y;
        p2.z = segment.p2.position.z;
        geometry_msgs::Point p3;
        p3.x = segment.p3.position.x;
        p3.y = segment.p3.position.y;
        p3.z = segment.p3.position.z;

        line1 = path_nav_math::Line(p1, p2);
        line2 = path_nav_math::Line(p2, p3);
    }

    void operator()(ADvector &fg, const ADvector &vars) {
        // fg a vector containing the cost and all constraints
        // vars is a vector containing all states and actuations for N "lookahead" states and actuations.

        //*********************************************************
        //* COST DEFINED HERE
        //*********************************************************
        fg[0] = 0.0;

        // The part of the cost based on the reference state.
        for (int i = 0; i < N; ++i) {

            const auto cte_track_x = vars[ID_FIRST_cte_track_x + i];
            const auto cte_track_y = vars[ID_FIRST_cte_track_y + i];
            const auto cte_track_z = vars[ID_FIRST_cte_track_z + i];
            const auto cte_goal_x = vars[ID_FIRST_cte_goal_x + i];
            const auto cte_goal_y = vars[ID_FIRST_cte_goal_y + i];
            const auto cte_goal_z = vars[ID_FIRST_cte_goal_z + i];
            const auto epsi = vars[ID_FIRST_epsi + i];
            const auto v_x = vars[ID_FIRST_v_x + i] - VELOCITY_MAX;
            const auto v_y = vars[ID_FIRST_v_y + i] - VELOCITY_MAX;
            const auto v_z = vars[ID_FIRST_v_z + i] - VELOCITY_MAX;

            fg[0] += W_cte_track_x * CppAD::pow(cte_track_x, 2);
            fg[0] += W_cte_track_y * CppAD::pow(cte_track_y, 2);
            fg[0] += W_cte_track_z * CppAD::pow(cte_track_z, 2);
            fg[0] += W_cte_goal_x * CppAD::pow(cte_goal_x, 2);
            fg[0] += W_cte_goal_y * CppAD::pow(cte_goal_y, 2);
            fg[0] += W_cte_goal_z * CppAD::pow(cte_goal_z, 2);
            fg[0] += W_epsi * CppAD::pow(epsi, 2);
            fg[0] += W_v * CppAD::pow(v_x, 2);
            fg[0] += W_v * CppAD::pow(v_y, 2);
            fg[0] += W_v * CppAD::pow(v_z, 2);
        }

        // Minimize the use of actuators.
        for (int i = 0; i < N; ++i) {
            const auto delta = vars[ID_FIRST_delta + i];
            const auto a_x = vars[ID_FIRST_a_x + i];
            const auto a_y = vars[ID_FIRST_a_y + i];
            const auto a_z = vars[ID_FIRST_a_z + i];

            fg[0] += W_delta * CppAD::pow(delta, 2);
            fg[0] += W_a * CppAD::pow(a_x, 2);
            fg[0] += W_a * CppAD::pow(a_y, 2);
            fg[0] += W_a * CppAD::pow(a_z, 2);

        }

        // Minimize the value gap between sequential actuations.
        for (int i = 0; i < N - 2; ++i) {

            const auto ddelta = vars[ID_FIRST_delta + i + 1] - vars[ID_FIRST_delta + i];
            const auto da_x = vars[ID_FIRST_a_x + i + 1] - vars[ID_FIRST_a_x + i];
            const auto da_y = vars[ID_FIRST_a_y + i + 1] - vars[ID_FIRST_a_y + i];
            const auto da_z = vars[ID_FIRST_a_z + i + 1] - vars[ID_FIRST_a_z + i];

            fg[0] += W_ddelta * CppAD::pow(ddelta, 2);
            fg[0] += W_da * CppAD::pow(da_x, 2);
            fg[0] += W_da * CppAD::pow(da_y, 2);
            fg[0] += W_da * CppAD::pow(da_z, 2);
        }

        //*********************************************************
        //* CONSTRAINTS DEFINED HERE
        //*********************************************************
        // Setup model constraints

        // Initial constraints
        // We add 1 to each of the starting indices due to cost being located at
        // index 0 of `fg`. This bumps up the position of all the other values.
        // given state does not vary
        fg[ID_FIRST_px + 1] = vars[ID_FIRST_px];
        fg[ID_FIRST_py + 1] = vars[ID_FIRST_py];
        fg[ID_FIRST_pz + 1] = vars[ID_FIRST_pz];
        fg[ID_FIRST_psi + 1] = vars[ID_FIRST_psi];

        fg[ID_FIRST_v_x + 1] = vars[ID_FIRST_v_x];
        fg[ID_FIRST_v_y + 1] = vars[ID_FIRST_v_y];
        fg[ID_FIRST_v_z + 1] = vars[ID_FIRST_v_z];

        fg[ID_FIRST_cte_track_x + 1] = vars[ID_FIRST_cte_track_x];
        fg[ID_FIRST_cte_track_y + 1] = vars[ID_FIRST_cte_track_y];
        fg[ID_FIRST_cte_track_z + 1] = vars[ID_FIRST_cte_track_z];
        fg[ID_FIRST_cte_goal_x + 1] = vars[ID_FIRST_cte_goal_x];
        fg[ID_FIRST_cte_goal_y + 1] = vars[ID_FIRST_cte_goal_y];
        fg[ID_FIRST_cte_goal_z + 1] = vars[ID_FIRST_cte_goal_z];
        fg[ID_FIRST_epsi + 1] = vars[ID_FIRST_epsi];
        bool switchLine = false;


        // constraints based on our kinematic model
        // The rest of the constraints
        for (int t = 0; t < N - 1; ++t) {

            // where the current state variables of interest are stored
            // stored for readability
            const int ID_CURRENT_px = ID_FIRST_px + t;
            const int ID_CURRENT_py = ID_FIRST_py + t;
            const int ID_CURRENT_pz = ID_FIRST_pz + t;
            const int ID_CURRENT_psi = ID_FIRST_psi + t;
            const int ID_CURRENT_v_x = ID_FIRST_v_x + t;
            const int ID_CURRENT_v_y = ID_FIRST_v_y + t;
            const int ID_CURRENT_v_z = ID_FIRST_v_z + t;
            const int ID_CURRENT_cte_track_x = ID_FIRST_cte_track_x + t;
            const int ID_CURRENT_cte_track_y = ID_FIRST_cte_track_y + t;
            const int ID_CURRENT_cte_track_z = ID_FIRST_cte_track_z + t;
            const int ID_CURRENT_cte_goal_x = ID_FIRST_cte_goal_x + t;
            const int ID_CURRENT_cte_goal_y = ID_FIRST_cte_goal_y + t;
            const int ID_CURRENT_cte_goal_z = ID_FIRST_cte_goal_z + t;
            const int ID_CURRENT_epsi = ID_FIRST_epsi + t;
            const int ID_CURRENT_delta = ID_FIRST_delta + t;
            const int ID_CURRENT_a_x = ID_FIRST_a_x + t;
            const int ID_CURRENT_a_y = ID_FIRST_a_y + t;
            const int ID_CURRENT_a_z = ID_FIRST_a_z + t;

            // The state at time t+1 .
            const auto px1 = vars[ID_CURRENT_px + 1];
            const auto py1 = vars[ID_CURRENT_py + 1];
            const auto pz1 = vars[ID_CURRENT_pz + 1];
            const auto psi1 = vars[ID_CURRENT_psi + 1];
            const auto v1_x = vars[ID_CURRENT_v_x + 1];
            const auto v1_y = vars[ID_CURRENT_v_y + 1];
            const auto v1_z = vars[ID_CURRENT_v_z + 1];
            const auto cte1_track_x = vars[ID_CURRENT_cte_track_x + 1];
            const auto cte1_track_y = vars[ID_CURRENT_cte_track_y + 1];
            const auto cte1_track_z = vars[ID_CURRENT_cte_track_z + 1];
            const auto cte1_goal_x = vars[ID_CURRENT_cte_goal_x + 1];
            const auto cte1_goal_y = vars[ID_CURRENT_cte_goal_y + 1];
            const auto cte1_goal_z = vars[ID_CURRENT_cte_goal_z + 1];
            const auto epsi1 = vars[ID_CURRENT_epsi + 1];



            //Current state
            //The state at time t.

            const auto psi0 = vars[ID_CURRENT_psi];
            const auto v0_x = vars[ID_CURRENT_v_x];
            const auto v0_y = vars[ID_CURRENT_v_y];
            const auto v0_z = vars[ID_CURRENT_v_z];
            const auto cte0_track_x = vars[ID_CURRENT_cte_track_x];
            const auto cte0_track_y = vars[ID_CURRENT_cte_track_y];
            const auto cte0_track_z = vars[ID_CURRENT_cte_track_z];
            const auto cte0_goal_x = vars[ID_CURRENT_cte_goal_x];
            const auto cte0_goal_y = vars[ID_CURRENT_cte_goal_y];
            const auto cte0_goal_z = vars[ID_CURRENT_cte_goal_z];
            const auto epsi0 = vars[ID_CURRENT_epsi];
            const AD<double> px0 = vars[ID_CURRENT_px];
            const AD<double> py0 = vars[ID_CURRENT_py];
            const AD<double> pz0 = vars[ID_CURRENT_pz];
            // FML... okay, i cannot get access to the value of these so i have to convert to string and convert back to double. >.<
            std::stringstream buffer;
            buffer << px0;
            std::string px0_s = buffer.str();
            buffer.str("");
            buffer.clear(); // Clear state flags.
            buffer << py0;
            std::string py0_s = buffer.str();
            buffer.str("");
            buffer.clear(); // Clear state flags.
            buffer << pz0;
            std::string pz0_s = buffer.str();
            buffer.str("");
            buffer.clear(); // Clear state flags.

            std::cout << "p0 " << t << " (" << px0_s << "," << py0_s << "," << pz0_s << ")\n";

            // Only consider the actuation at time t.
            const auto delta0 = vars[ID_CURRENT_delta];
            const auto a0_x = vars[ID_CURRENT_a_x];
            const auto a0_y = vars[ID_CURRENT_a_y];
            const auto a0_z = vars[ID_CURRENT_a_z];


            // TODO:: Change this to real kinematic model
            // desired py and psi
            // find t_cur

            geometry_msgs::Point position = geometry_msgs::Point();
            position.x = std::stod(px0_s);
            position.y = std::stod(py0_s);
            position.z = std::stod(pz0_s);
            // TODO:: Tune this parameter!
            double t_cur;
            double t_desired;
            if (!switchLine) {
                line1.findShortestVectorFromPointToLine(position);
                t_cur = line1.getTCurUncapped() > 0 ? line1.getTCurUncapped() : 0;
                if (abs(1 - t_cur) < 0.05) {
                    switchLine = true;
                }
                t_desired = 1;
            } else {
                line2.findShortestVectorFromPointToLine(position);
                t_cur = line2.getTCurUncapped() > 0 ? line2.getTCurUncapped() : 0;
                t_desired = 2;
            }


            const auto px_track_desired =
                    K_x[2] * t_cur * t_cur + K_x[1] * t_cur + K_x[0]; // K[3] pxo³ + K[2] pxo² +K[1] pxo + K[0]
            const auto py_track_desired =
                    K_y[2] * t_cur * t_cur + K_y[1] * t_cur + K_y[0]; // K[3] pxo³ + K[2] pxo² +K[1] pxo + K[0]
            const auto pz_track_desired =
                    K_z[2] * t_cur * t_cur + K_z[1] * t_cur + K_z[0]; // K[3] pxo³ + K[2] pxo² +K[1] pxo + K[0]

            const auto px_goal_desired = K_x[2] * t_desired * t_desired + K_x[1] * t_desired +
                                         K_x[0]; // K[3] pxo³ + K[2] pxo² +K[1] pxo + K[0]
            const auto py_goal_desired = K_y[2] * t_desired * t_desired + K_y[1] * t_desired +
                                         K_y[0]; // K[3] pxo³ + K[2] pxo² +K[1] pxo + K[0]
            const auto pz_goal_desired = K_z[2] * t_desired * t_desired + K_z[1] * t_desired +
                                         K_z[0]; // K[3] pxo³ + K[2] pxo² +K[1] pxo + K[0]


            AD<double> psides0 = 0.0;
            for (int j = 1; j < K_y.size(); j++) {
                psides0 += j * K_y[j] * CppAD::pow(t_cur, j - 1); // f'(x0)
            }
            psides0 = CppAD::atan(psides0);


            // relationship of current state + actuations and next state
            // based on our kinematic model
//            std::cout << "psi0: "<< psi0 << "\n";
            const auto px1_f = px0 + v0_x * dt * CppAD::cos(psi0);
            const auto py1_f = py0 + v0_y * dt * CppAD::sin(psi0);
            const auto pz1_f = pz0 + v0_z * dt;
            const auto psi1_f = psi0 + (CppAD::sqrt(CppAD::pow(v0_x, 2) + CppAD::pow(v0_y, 2))) * (delta0) / Lf * dt;

            const auto v1_f_x = v0_x + a0_x * dt;
            const auto v1_f_y = v0_y + a0_y * dt;
            const auto v1_f_z = v0_z + a0_z * dt;

            const auto cte1_f_track_x = px_track_desired - px0 + v0_x * CppAD::cos(epsi0) * dt;
            const auto cte1_f_track_y = py_track_desired - py0 + v0_y * CppAD::sin(epsi0) * dt;
//            const auto cte1_f_track_x = px_track_desired - px0 + v0_x * dt;
//            const auto cte1_f_track_y = py_track_desired - py0 + v0_y * dt;
//
            const auto cte1_f_track_z = pz_track_desired - pz0 + v0_z * dt;

            const auto cte1_f_goal_x = px_goal_desired - px0 + v0_x * dt;
            const auto cte1_f_goal_y = py_goal_desired - py0 + v0_y * dt;
            const auto cte1_f_goal_z = pz_goal_desired - pz0 + v0_z * dt;

            const auto epsi1_f =
                    (psi0 - psides0) + (CppAD::sqrt(CppAD::pow(v0_x, 2) + CppAD::pow(v0_y, 2))) * (delta0) / Lf * dt;

            // store the constraint expression of two consecutive states
            fg[ID_CURRENT_px + 2] = px1 - px1_f;
            fg[ID_CURRENT_py + 2] = py1 - py1_f;
            fg[ID_CURRENT_pz + 2] = pz1 - pz1_f;
            fg[ID_CURRENT_psi + 2] = psi1 - psi1_f;
            fg[ID_CURRENT_v_x + 2] = v1_x - v1_f_x;
            fg[ID_CURRENT_v_y + 2] = v1_y - v1_f_y;
            fg[ID_CURRENT_v_z + 2] = v1_z - v1_f_z;
            fg[ID_CURRENT_cte_track_x + 2] = cte1_track_x - cte1_f_track_x;
            fg[ID_CURRENT_cte_track_y + 2] = cte1_track_y - cte1_f_track_y;
            fg[ID_CURRENT_cte_track_z + 2] = cte1_track_z - cte1_f_track_z;
            fg[ID_CURRENT_cte_goal_x + 2] = cte1_goal_x - cte1_f_goal_x;
            fg[ID_CURRENT_cte_goal_y + 2] = cte1_goal_y - cte1_f_goal_y;
            fg[ID_CURRENT_cte_goal_z + 2] = cte1_goal_z - cte1_f_goal_z;
            fg[ID_CURRENT_epsi + 2] = epsi1 - epsi1_f;
        }
    }
};

MPC::MPC() {

    //**************************************************************
    //* SET INITIAL VALUES OF VARIABLES
    //**************************************************************
    this->x.resize(NX);

    // all states except the ID_FIRST are set to zero
    // the aformentioned states will be initialized when solve() is called

    for (int i = 0; i < NX; ++i) {
        this->x[i] = 0.0;
    }

    //**************************************************************
    //* SET UPPER AND LOWER LIMITS OF VARIABLES
    //**************************************************************

    this->x_lowerbound.resize(NX);
    this->x_upperbound.resize(NX);

    // all other values large values the computer can handle
    for (int i = 0; i < ID_FIRST_delta; ++i) {
        this->x_lowerbound[i] = -1.0e10;
        this->x_upperbound[i] = 1.0e10;
    }

    // all actuation inputs (steering, acceleration) should have values between [-1, 1]
    for (int i = ID_FIRST_delta; i < ID_FIRST_a_x; ++i) {
        this->x_lowerbound[i] = -0.1;
        this->x_upperbound[i] = 0.1;
    }

    // This is the speed limit of the drone:
    for (int i = ID_FIRST_a_x; i < NX; ++i) {
        this->x_lowerbound[i] = -0.1;
        this->x_upperbound[i] = 0.1;
    }

    //**************************************************************
    //* SET UPPER AND LOWER LIMITS OF CONSTRAINTS
    //**************************************************************
    this->g_lowerbound.resize(NG);
    this->g_upperbound.resize(NG);

    // the first constraint for each state veriable
    // refer to the initial state conditions
    // this will be initialized when solve() is called
    // the succeeding constraints refer to the relationship
    // between succeeding states based on our kinematic model of the system

    for (int i = 0; i < NG; ++i) {
        this->g_lowerbound[i] = 0.0;
        this->g_upperbound[i] = 0.0;
    }
}

MPC::~MPC() {}

void MPC::solve(Eigen::VectorXd state, Eigen::VectorXd K_x, Eigen::VectorXd K_y, Eigen::VectorXd K_z,
                martin_msg_lib::Segment currentSegment, double lf) {

    const double px = state[0];
    const double py = state[1];
    const double pz = state[2];
    const double psi = state[3];
    const double v_x = state[4];
    const double v_y = state[5];
    const double v_z = state[6];
    const double cte_track_x = state[7];
    const double cte_track_y = state[8];
    const double cte_track_z = state[9];
    const double cte_goal_x = state[10];
    const double cte_goal_y = state[11];
    const double cte_goal_z = state[12];
    const double epsi = state[13];

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
    const int ID_FIRST_delta = ID_FIRST_epsi + N;
    const int ID_FIRST_a_x = ID_FIRST_delta + N;
    const int ID_FIRST_a_y = ID_FIRST_a_x + N;
    const int ID_FIRST_a_z = ID_FIRST_a_y + N - 1;


    this->x[ID_FIRST_px] = px;
    this->x[ID_FIRST_py] = py;
    this->x[ID_FIRST_pz] = pz;
    this->x[ID_FIRST_psi] = psi;
    this->x[ID_FIRST_v_x] = v_x;
    this->x[ID_FIRST_v_y] = v_y;
    this->x[ID_FIRST_v_z] = v_z;
    this->x[ID_FIRST_cte_track_x] = cte_track_x;
    this->x[ID_FIRST_cte_track_y] = cte_track_y;
    this->x[ID_FIRST_cte_track_z] = cte_track_z;
    this->x[ID_FIRST_cte_goal_x] = cte_goal_x;
    this->x[ID_FIRST_cte_goal_y] = cte_goal_y;
    this->x[ID_FIRST_cte_goal_z] = cte_goal_z;
    this->x[ID_FIRST_epsi] = epsi;

    this->g_lowerbound[ID_FIRST_px] = px;
    this->g_lowerbound[ID_FIRST_py] = py;
    this->g_lowerbound[ID_FIRST_pz] = pz;
    this->g_lowerbound[ID_FIRST_psi] = psi;
    this->g_lowerbound[ID_FIRST_v_x] = v_x;
    this->g_lowerbound[ID_FIRST_v_y] = v_y;
    this->g_lowerbound[ID_FIRST_v_z] = v_z;
    this->g_lowerbound[ID_FIRST_cte_track_x] = cte_track_x;
    this->g_lowerbound[ID_FIRST_cte_track_y] = cte_track_y;
    this->g_lowerbound[ID_FIRST_cte_track_z] = cte_track_z;
    this->g_lowerbound[ID_FIRST_cte_goal_x] = cte_goal_x;
    this->g_lowerbound[ID_FIRST_cte_goal_y] = cte_goal_y;
    this->g_lowerbound[ID_FIRST_cte_goal_z] = cte_goal_z;
    this->g_lowerbound[ID_FIRST_epsi] = epsi;

    this->g_upperbound[ID_FIRST_px] = px;
    this->g_upperbound[ID_FIRST_py] = py;
    this->g_upperbound[ID_FIRST_pz] = pz;
    this->g_upperbound[ID_FIRST_psi] = psi;
    this->g_upperbound[ID_FIRST_v_x] = v_x;
    this->g_upperbound[ID_FIRST_v_y] = v_y;
    this->g_upperbound[ID_FIRST_v_z] = v_z;
    this->g_upperbound[ID_FIRST_cte_track_x] = cte_track_x;
    this->g_upperbound[ID_FIRST_cte_track_y] = cte_track_y;
    this->g_upperbound[ID_FIRST_cte_track_z] = cte_track_z;
    this->g_upperbound[ID_FIRST_cte_goal_x] = cte_goal_x;
    this->g_upperbound[ID_FIRST_cte_goal_y] = cte_goal_y;
    this->g_upperbound[ID_FIRST_cte_goal_z] = cte_goal_z;
    this->g_upperbound[ID_FIRST_epsi] = epsi;


//    //**************************************************************
//    //* SOLVE
//    //**************************************************************
//

    // object that computes objective and constraints
    FG_eval fg_eval(K_x, K_y, K_z, currentSegment, lf);

    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    options += "Numeric max_cpu_time          1\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
            options,
            x,
            x_lowerbound,
            x_upperbound,
            g_lowerbound,
            g_upperbound,
            fg_eval,
            solution);

    std::cout<< " HEY\n";
    // comment out the lines below to debug!

    bool ok = true;
    auto cost = solution.obj_value;

    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    if (ok) {
        std::cout << "OK! Cost:" << cost << std::endl;
    } else {
        std::cout << "SOMETHING IS WRONG!" << cost << std::endl;
    }


    //**************************************************************
    //* STORE RELEVANT INFORMATION FROM SOLUTION
    //**************************************************************

    this->yaw_rotation = solution.x[ID_FIRST_delta];
    this->a_x = solution.x[ID_FIRST_a_x];
    this->a_y = solution.x[ID_FIRST_a_y];
    this->a_z = solution.x[ID_FIRST_a_z];

    this->future_xs = {};
    this->future_ys = {};
    this->future_zs = {};

    for (int i = 0; i < N; ++i) {
        const double px = solution.x[ID_FIRST_px + i];
        const double py = solution.x[ID_FIRST_py + i];
        const double pz = solution.x[ID_FIRST_pz + i];

        this->future_xs.emplace_back(px);
        this->future_ys.emplace_back(py);
        this->future_zs.emplace_back(pz);

    }
}
