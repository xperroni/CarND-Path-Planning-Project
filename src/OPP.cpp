#include "OPP.h"

#include "settings.h"

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

using CppAD::AD;

#include <Eigen/Dense>

using Eigen::VectorXd;

static const size_t X = 0;
static const size_t Y = 1;
static const size_t A = 1;
static const size_t D = 2;
static const size_t SIZEOF_POINT = 2;
static const size_t SIZEOF_CONSTRAINT = 3;
static const size_t SIZEOF_OBSTACLE = 2;

struct Cost {
    /** @brief Basic scalar value type. */
    typedef AD<double> Scalar;

    /** @brief Differentiable variable vector type. */
    typedef CPPAD_TESTVECTOR(Scalar) ADvector;

    /** @brief Previous longitudinal speed. */
    Scalar v_0;

    /** @brief Reference longitudinal speed. */
    Scalar v_r;

    /** @brief Coefficients of the polynomial describing the reference route. */
    VectorXd route;

    /** @brief State of other vehicles around the car. */
//     Vehicles vehicles;

    /** @brief Number of `(x, y)` points in the plan. */
    size_t n_plan;

    /**
     * @brief Create a new optimization task with given initial speed and reference route.
     */
    Cost(const State &state, size_t n_plan, const VectorXd &route) {
        this->v_0 = state.v;
        this->v_r = V_PLAN;
        this->route = route;
//         this->vehicles = state.vehicles;
        this->n_plan = n_plan;
    }

    /**
     * @brief Compute the cost function for the OPP.
     */
    void operator () (ADvector &fg, const ADvector &vars) {
//         static Scalar ZERO = 0.0;
//         static Scalar ONE = 1.0;
//
//         size_t a_obstacles = 1 + SIZEOF_CONSTRAINT * (n_plan - 1);
//         size_t n_obstacles = vehicles.size();

        Scalar w_0 = 0;

        for (size_t i = 0, m = n_plan - 1; i < m; ++i) {
            auto &x_0 = vars[X + SIZEOF_POINT * i];
            auto &y_0 = vars[Y + SIZEOF_POINT * i];
            auto &x_1 = vars[X + SIZEOF_POINT * (i + 1)];
            auto &y_1 = vars[Y + SIZEOF_POINT * (i + 1)];

            auto x_d = x_1 - x_0;
            auto y_d = y_1 - y_0;

            // Longitudinal and lateral speeds.
            auto v_1 = x_d / T_PLAN;
            auto w_1 = y_d / T_PLAN;

            // Reference state.
            auto y_r = reference(x_1);

            // Contribution to the cost function.
            fg[0] += CppAD::pow(y_r - y_1, 2);
            fg[0] += CppAD::pow(v_r - v_1, 2);

            // Constraint functions values.
            fg[1 + X + SIZEOF_CONSTRAINT * i] = x_d;
            fg[1 + A + SIZEOF_CONSTRAINT * i] = (v_1 - v_0) / T_PLAN;
            fg[1 + D + SIZEOF_CONSTRAINT * i] = (w_1 - w_0) / T_PLAN;

//             for (size_t j = 0; j < n_obstacles; ++j) {
//                 double &x_p = vehicles.positions.x[j];
//                 double &y_p = vehicles.positions.y[j];
//
//                 x_p += vehicles.speeds.x[j] * T_PLAN;
//                 y_p += vehicles.speeds.y[j] * T_PLAN;
//
//                 auto x_e = x_p - x_0;
//                 auto y_e = y_p - y_0;
//
//                 // See https://math.stackexchange.com/a/330329/467980
//                 auto t_d = (x_d * x_e + y_d * y_e) / (CppAD::pow(x_d, 2) + CppAD::pow(y_d, 2));
//                 auto t_s = CppAD::CondExpLt(ZERO, t_d, CppAD::CondExpLt(t_d, ONE, t_d, ONE), ZERO);
//
//                 auto x_s = x_0 + t_s * x_d;
//                 auto y_s = y_0 + t_s * y_d;
//
//                 auto d = CppAD::pow(x_s - x_p, 2) + CppAD::pow(y_s - y_p, 2);
//
//                 fg[0] += 1.0 / (d + 10e-4);
//
//                 size_t offset = a_obstacles + (i * n_obstacles + j) * SIZEOF_OBSTACLE;
//                 fg[offset + X] = d;
//                 fg[offset + Y] = 1.0;
//             }

            v_0 = v_1;
            w_0 = w_1;
        }
    }

private:
  /**
   * @brief Compute the `y` coordinate for the reference route.
   */
  Scalar reference(const Scalar &x) const {
    Scalar y = route(0);
    for (size_t i = 1, n = route.rows(); i < n; ++i) {
      y += route(i) * CppAD::pow(x, i);
    }

    return y;
  }
};

Waypoints OPP(const State &state, const Lane &lane) {
    // Differentiable value vector type.
    typedef CPPAD_TESTVECTOR(double) Vector;

    size_t n_plan = N_PLAN - state.route.size();

    // Initialize independent variable and bounds vectors.
    Vector vars(n_plan * SIZEOF_POINT);
    Vector vars_lowerbound(n_plan * SIZEOF_POINT);
    Vector vars_upperbound(n_plan * SIZEOF_POINT);
    for (size_t i = 0; i < n_plan; i++) {
        size_t i_x = X + i * SIZEOF_POINT;
        size_t i_y = Y + i * SIZEOF_POINT;

        vars[i_x] = 0;
        vars[i_y] = 0;

        vars_lowerbound[i_x] = 0;
        vars_upperbound[i_x] = 2 * V_PLAN * T_PLAN * n_plan;

        vars_lowerbound[i_y] = -2 * lane.width;
        vars_upperbound[i_y] = 2 * lane.width;
    }

    // Lock the first point in place, as it represents the car's current position.
    vars_lowerbound[X] = 0;
    vars_upperbound[X] = 0;
    vars_lowerbound[Y] = 0;
    vars_upperbound[Y] = 0;

    // Initialize constraint vectors.
    size_t n_constraints = n_plan - 1;
//     size_t n_obstacles = state.vehicles.size();
    Vector constraints_lowerbound(n_constraints * SIZEOF_CONSTRAINT); // (SIZEOF_CONSTRAINT + n_obstacles * SIZEOF_OBSTACLE));
    Vector constraints_upperbound(n_constraints * SIZEOF_CONSTRAINT); // (SIZEOF_CONSTRAINT + n_obstacles * SIZEOF_OBSTACLE));
    for (size_t i = 0; i < n_constraints; ++i) {
        // Ensure x(t) is a strictly increasing function.
        constraints_lowerbound[X + SIZEOF_CONSTRAINT * i] = 0.01;
        constraints_upperbound[X + SIZEOF_CONSTRAINT * i] = 1.1 * V_PLAN * T_PLAN;

        // Ensure accelerations stay within reasonable limits.
        constraints_lowerbound[A + SIZEOF_CONSTRAINT * i] = -5.0;
        constraints_upperbound[A + SIZEOF_CONSTRAINT * i] = 5.0;
        constraints_lowerbound[D + SIZEOF_CONSTRAINT * i] = -5.0;
        constraints_upperbound[D + SIZEOF_CONSTRAINT * i] = 5.0;

//         for (size_t j = 0; j < n_obstacles; ++j) {
//             size_t offset = n_constraints * SIZEOF_CONSTRAINT + (i * n_obstacles + j) * SIZEOF_OBSTACLE;
//             constraints_lowerbound[offset + X] = 0.1;
//             constraints_upperbound[offset + X] = std::numeric_limits<double>::max();
//             constraints_lowerbound[offset + Y] = 0.1;
//             constraints_upperbound[offset + Y] = std::numeric_limits<double>::max();
//         }
    }

    // Fit a polynomial to waypoints sampled from the lane.
    Waypoints samples = lane.sample(state);
    state.toLocalFrame(samples);
    auto route = samples.fit();

    // Define the cost function.
    Cost cost(state, n_plan, route);

    // Options for IPOPT solver.
    std::string options =
        "Integer print_level 0\n"
        "Sparse true forward\n"
        "Sparse true reverse\n"
        "Numeric max_cpu_time 0.5\n";

    // Solution to the cost optimization problem.
    CppAD::ipopt::solve_result<Vector> solution;

    // Call the solver on the cost function and given parameters.
    CppAD::ipopt::solve<Vector, Cost>(
        options,
        vars,
        vars_lowerbound,
        vars_upperbound,
        constraints_lowerbound,
        constraints_upperbound,
        cost,
        solution
    );

    auto &control = solution.x;
//     auto value = solution.obj_value;
//     auto status = (solution.status == CppAD::ipopt::solve_result<Vector>::success ? "succeeded" : "failed");
//     std::cout << "Solver " << status << ", final cost value = " << value << std::endl;

    std::vector<double> x;
    std::vector<double> y;

    // Discard first waypoint, which is the same as the last waypoint in the
    // current route.
    for (size_t i = 1; i < n_plan; ++i) {
        double x_i = control[X + SIZEOF_POINT * i];
        double y_i = control[Y + SIZEOF_POINT * i];
        x.push_back(x_i);
        y.push_back(y_i);
    }

    Waypoints latest = {x, y};
    state.toGlobalFrame(latest);

    Waypoints waypoints = state.route;
    waypoints.x.insert(waypoints.x.end(), latest.x.begin(), latest.x.end());
    waypoints.y.insert(waypoints.y.end(), latest.y.begin(), latest.y.end());

    return waypoints;
}
