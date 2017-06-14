#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

// For converting back and forth between radians and degrees.
static inline constexpr double pi() { return M_PI; }
static inline double deg2rad(double x) { return x * pi() / 180; }
static inline double rad2deg(double x) { return x * 180 / pi(); }

class MPC
{
public:
    MPC();

    virtual ~MPC();

    // Solve the model given an initial state and polynomial coefficients.
    // Return the first actuatotions.
    std::tuple<double, double, std::vector<double>, std::vector<double>, double, double>
    Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, double minx, double maxx);

    std::size_t latency_position;
    double latency_offset;

};

#endif /* MPC_H */
