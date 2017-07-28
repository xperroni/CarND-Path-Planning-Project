#include "MPP.h"

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

struct Cost {
    /** @brief Basic scalar value type. */
    typedef AD<double> Scalar;

    /** @brief Differentiable variable vector type. */
    typedef CPPAD_TESTVECTOR(Scalar) ADvector;

    /** @brief Previous longitudinal speed. */
    Scalar v_0;

    /** @brief Reference longitudinal speed. */
    Scalar v_r;

    /** @brief Previous lateral speed. */
    Scalar w_0;

    /** @brief Coefficients of the polynomial describing the reference route. */
    VectorXd route;

    /** @brief Number of `(x, y)` points in the plan. */
    size_t n_plan;

    /**
     * @brief Create a new optimization task with given initial speed and reference route.
     */
    Cost(double v_0, size_t n_plan, const VectorXd &route) {
        this->v_0 = v_0;
        this->v_r = V_PLAN;
        this->w_0 = 0;
        this->route = route;
        this->n_plan = n_plan;
    }

    /**
     * @brief Compute the cost function for the MPP.
     */
    void operator () (ADvector &fg, const ADvector &vars) {
        for (size_t i = 0, n = n_plan - 1; i < n; ++i) {
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

Waypoints MPP(State state, const Lane &lane, const Waypoints &previous) {
    // Differentiable value vector type.
    typedef CPPAD_TESTVECTOR(double) Vector;

    size_t n_plan = N_PLAN - previous.size();
    if (previous.size() > 1) {
        state = previous.stateLast();
    }

    // Independent variables and bounds.
    Vector vars(n_plan * SIZEOF_POINT);
    Vector vars_lowerbound(n_plan * SIZEOF_POINT);
    Vector vars_upperbound(n_plan * SIZEOF_POINT);

    // Constraint bounds.
    Vector constraints_lowerbound((n_plan - 1) * SIZEOF_CONSTRAINT);
    Vector constraints_upperbound((n_plan - 1) * SIZEOF_CONSTRAINT);

    // Initialize independent variable and bounds vectors.
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
    for (size_t i = 0, n = n_plan - 1; i < n; i++) {
        // Ensure x(t) is a strictly increasing function.
        constraints_lowerbound[X + SIZEOF_CONSTRAINT * i] = 0.01;
        constraints_upperbound[X + SIZEOF_CONSTRAINT * i] = 1.1 * V_PLAN * T_PLAN;

        // Ensure accelerations stay within reasonable limits.
        constraints_lowerbound[A + SIZEOF_CONSTRAINT * i] = -5.0;
        constraints_upperbound[A + SIZEOF_CONSTRAINT * i] = 5.0;
        constraints_lowerbound[D + SIZEOF_CONSTRAINT * i] = -5.0;
        constraints_upperbound[D + SIZEOF_CONSTRAINT * i] = 5.0;
    }

    // Fit a polynomial to waypoints sampled from the lane.
    Waypoints samples = lane.sample(state);
    samples.toLocalFrame(state);
    auto route = samples.fit();

    // Define the cost function.
    Cost cost(state.v, n_plan, route);

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
    std::vector<double> x;
    std::vector<double> y;

    // Discard first waypoint, which is the same as the last waypoint in the
    // `previous` vector.
    for (size_t i = 1; i < n_plan; ++i) {
        double x_i = control[X + SIZEOF_POINT * i];
        double y_i = control[Y + SIZEOF_POINT * i];
        x.push_back(x_i);
        y.push_back(y_i);
    }

    Waypoints latest = {x, y};
    latest.toGlobalFrame(state);

    Waypoints waypoints = previous;
    waypoints.x.insert(waypoints.x.end(), latest.x.begin(), latest.x.end());
    waypoints.y.insert(waypoints.y.end(), latest.y.begin(), latest.y.end());

    return waypoints;
}
