#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// Set the timestep length and duration
size_t N = 40;
double dt = 0.05;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

double ref_v = 32; // in m/s

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;


class FG_eval
{
public:
    // Fitted polynomial coefficients
    Eigen::VectorXd coeffs;
    FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    void operator()(ADvector& fg, const ADvector& vars) {
        // fg a vector of constraints, x is a vector of constraints.
        fg[0] = 0;

        // Reference State Cost
        for (int i = 0; i < N; ++i)
        {
            fg[0] += 1 * CppAD::pow(vars[cte_start + i], 2);
            fg[0] += 1 * CppAD::pow(vars[epsi_start + i], 2);
            fg[0] += 1e-1 * CppAD::pow(vars[v_start+ i] - ref_v, 2);
        }

        // Minimize the actuator values
        for (int i = 0; i < N - 1; ++i)
        {
            fg[0] += 100 * CppAD::pow(vars[delta_start + i], 2);
            fg[0] += 5 * CppAD::pow(vars[a_start], 2);
        }

        // Minimize the sudden change
        for (int i = 0; i < N - 2; ++i)
        {
            fg[0] += 5000000 * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
            fg[0] +=  0 * CppAD::pow(vars[a_start + i +1] - vars[a_start], 2);
        }

        //
        // Setup Constraints
        //
        // NOTE: In this section you'll setup the model constraints.

        // Initial constraints
        //
        // We add 1 to each of the starting indices due to cost being located at
        // index 0 of `fg`.
        // This bumps up the position of all the other values.
        fg[1 + x_start] = vars[x_start];
        fg[1 + y_start] = vars[y_start];
        fg[1 + psi_start] = vars[psi_start];
        fg[1 + v_start] = vars[v_start];
        fg[1 + cte_start] = vars[cte_start];
        fg[1 + epsi_start] = vars[epsi_start];

        // The rest of the constraints
        for (int i = 0; i < N - 1; i++)
        {
            // The state at time t.
            AD<double> x1 = vars[x_start + i + 1];
            AD<double> y1 = vars[y_start + i + 1];
            AD<double> psi1 = vars[psi_start + i + 1];
            AD<double> v1 = vars[v_start + i + 1];
            AD<double> cte1 = vars[cte_start + i + 1];
            AD<double> epsi1 = vars[epsi_start + i + 1];

            // The state at time t.
            AD<double> x0 = vars[x_start + i];
            AD<double> y0 = vars[y_start + i];
            AD<double> psi0 = vars[psi_start + i];
            AD<double> v0 = vars[v_start + i];
            AD<double> cte0 = vars[cte_start + i];
            AD<double> epsi0 = vars[epsi_start + i];

            // Only consider the actuation at time t.
            AD<double> delta0 = vars[delta_start + i];
            AD<double> a0 = vars[a_start + i];

            AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0 + coeffs[3] * x0 * x0 * x0;
            AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * x0 * x0);

            // Here's `x` to get you started.
            // The idea here is to constraint this value to be 0.
            //
            // Recall the equations for the model:
            // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
            // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
            // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
            // v_[t+1] = v[t] + a[t] * dt
            // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
            // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
            // NOTE: The use of `AD<double>` and use of `CppAD`!
            // This is also CppAD can compute derivatives and pass
            // these to the solver.

            // TODO: Setup the rest of the model constraints
            fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
            fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
            fg[2 + psi_start + i] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
            fg[2 + v_start + i] = v1 - (v0 + a0 * dt);
            fg[2 + cte_start + i] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
            fg[2 + epsi_start + i] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
        }
    }
};

//
// MPC class definition implementation.
//
MPC::MPC() {
    const auto latency = 0.1; // in seconds
    latency_position = static_cast<std::size_t>(latency / dt);
    latency_offset = latency / dt - latency_position;
}
MPC::~MPC() {}

std::tuple<double, double, std::vector<double>, std::vector<double>, double, double>
MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, double minx, double maxx)
{
    bool ok = true;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    double x = state[0];
    double y = state[1];
    double psi = state[2];
    double v = state[3];
    double cte = state[4];
    double epsi = state[5];

    // Set the number of model variables (includes both states and inputs).
    size_t n_vars = N * 6 + (N - 1) * 2;

    // Set the number of constraints
    size_t n_constraints = N * 6;

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (std::size_t i = 0; i < n_vars; ++i)
    {
        vars[i] = 0;
    }

    // Set the initial variable values
    vars[x_start] = x;
    vars[y_start] = y;
    vars[psi_start] = psi;
    vars[v_start] = v;
    vars[cte_start] = cte;
    vars[epsi_start] = epsi;

    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    // Set lower and upper limits for variables.
    for (std::size_t i = 0; i < delta_start; i++)
    {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = +1.0e19;
    }

    // The upper and lower limits of delta are set to -25 and 25
    // degrees (values in radians).
    // NOTE: Feel free to change this to something else.
    for (std::size_t i = delta_start; i < delta_start + N - 1; ++i)
    {
        vars_lowerbound[i] = -deg2rad(25);
        vars_upperbound[i] = +deg2rad(25);
    }

    // Acceleration/deceleration upper and lower limits.
    auto poly = [&coeffs](double x) { return coeffs[0] + coeffs[1] * x + coeffs[2] * x * x + coeffs[3] * x * x * x; };
    std:size_t ncross = 0, nsample = 1000;
    double prevy = poly(0);
    for (std::size_t i = 1; i < nsample; ++i)
    {
        double y = maxx * i / nsample;
        ncross += (prevy * y <= 0.);
        prevy = y;
    }

    for (std::size_t i = a_start; i < a_start + N - 1; ++i)
    {
        vars_lowerbound[i] = -1.0;
        vars_upperbound[i] = +1.0; // / (3 * ncross + 1);
    }

    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (std::size_t i = 0; i < n_constraints; ++i)
    {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }

    constraints_lowerbound[x_start] = x;
    constraints_lowerbound[y_start] = y;
    constraints_lowerbound[psi_start] = psi;
    constraints_lowerbound[v_start] = v;
    constraints_lowerbound[cte_start] = cte;
    constraints_lowerbound[epsi_start] = epsi;

    constraints_upperbound[x_start] = x;
    constraints_upperbound[y_start] = y;
    constraints_upperbound[psi_start] = psi;
    constraints_upperbound[v_start] = v;
    constraints_upperbound[cte_start] = cte;
    constraints_upperbound[epsi_start] = epsi;

    auto curv = [&coeffs](double x) { return (2. * coeffs[2] + 6. * coeffs[3] * x) / pow(pow(coeffs[1] + 2. * coeffs[2] * x + 3. * coeffs[3] * x * x, 2.) + 1., 1.5); };
    double curv2 = 0.;
    for (double x = minx, dx = (maxx - minx) / 1000; x <= maxx; x += dx)
    {
        curv2 += pow(curv(x), 2.) * dx;
    }

    curv2 /= (maxx - minx);
    ref_v = 50. - 30. / (1. + exp(-.5e5*(curv2-1.2e-4)));

    // object that computes objective and constraints
    FG_eval fg_eval(coeffs);

    //
    // NOTE: You don't have to worry about these options
    //
    // options for IPOPT solver
    std::string options;

    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.5\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
                                          constraints_upperbound, fg_eval, solution);

    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
    if (!ok)
        return {0., 0., std::vector<double>(), std::vector<double>(), std::numeric_limits<double>::quiet_NaN(), ref_v};

    auto curv_xy = [&solution](int i) {
           auto dx = (solution.x[x_start + i + 1] - solution.x[x_start + i - 1]) / (2. * dt);
           auto dy = (solution.x[y_start + i + 1] - solution.x[y_start + i - 1]) / (2. * dt);
           auto d2x = (solution.x[x_start + i + 1] - 2. * solution.x[x_start + i] +  solution.x[x_start + i - 1]) / (dt * dt);
           auto d2y = (solution.x[y_start + i + 1] - 2. * solution.x[y_start + i] +  solution.x[y_start + i - 1]) / (dt * dt);
           return (dx * d2y - dy * d2x) / pow(dx * dx + dy * dy, 1.5);
    };
    double prev_curv = pow(curv_xy(1), 2.);
    double curv2_xy = prev_curv * dt;
    for (int i = 2; i < N - 1; ++i)
    {
        double curr_curv = pow(curv_xy(i), 2.);
        curv2_xy += (curr_curv  + prev_curv) * dt / 2.;
        prev_curv = curr_curv;
    }
    curv2_xy += prev_curv * dt;

    curv2_xy /= (N * dt);
    // ref_v = 54. - 28. / (1. + exp(-1e5*(curv2_xy-1.2e-4)));


    // Cost
    auto cost = solution.obj_value;
    std::cerr << "Cost " << cost << " " << solution.x[delta_start] << " " << solution.x[a_start]
              << " " << maxx << " " << ncross << " " << "curvature " << curv2 << " vs " << curv2_xy << "\n";

    std::vector<double> mpc_x_vals, mpc_y_vals;
    for (std::size_t i = 0; i < N; ++i)
    {
        mpc_x_vals.push_back(solution.x[x_start + i]);
        mpc_y_vals.push_back(solution.x[y_start + i]);
    }

    // Return the first actuator values.
    auto delta = solution.x[delta_start + latency_position] +
        (solution.x[delta_start + latency_position + 1] - solution.x[delta_start + latency_position]) * latency_offset;
    auto acceleration = solution.x[a_start + latency_position] +
        (solution.x[a_start + latency_position + 1] - solution.x[a_start + latency_position]) * latency_offset;
    return {delta, acceleration, mpc_x_vals, mpc_y_vals, cost, ref_v};
}
