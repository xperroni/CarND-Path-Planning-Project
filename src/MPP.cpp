#include "MPP.h"

#include "settings.h"

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

using CppAD::AD;

#include <Eigen/Dense>

using Eigen::VectorXd;

static const int A = 0;
static const int D = 1;
static const int SIZEOF_ACTUATION = 2;

struct Cost {
    /** @brief Basic scalar value. */
    typedef AD<double> Scalar;

    /** @brief Differentiable variable vector type. */
    typedef CPPAD_TESTVECTOR(Scalar) ADvector;

    /** @brief Size of the time step. */
    Scalar dt;

    /** @brief Initial speed at the beginning of the time window. */
    Scalar v0;

    /** @brief Reference speed. */
    Scalar vr;

    /** @brief Coefficients of the polynomial describing the reference route. */
    VectorXd route;

    /**
     * @brief Create a new optimization task with given initial speed and reference route.
     */
    Cost(double v0, const VectorXd &route) {
        this->dt = T_PLAN;
        this->v0 = v0;
        this->vr = V_PLAN;
        this->route = route;
    }

    /**
     * @brief Compute the cost function for the MPP.
     */
    void operator () (ADvector &fg, const ADvector &vars) {
        // Route is given relative to the car's current pose, so
        // planning always start from the origin at (0, 0, 0).
        Scalar xt = 0;
        Scalar yt = 0;
        Scalar ht = 0;
        Scalar vt = v0;

        for (int i = 0; i < N_PLAN; ++i) {
            // Compute the controller-proposed state at time (i * dt).
            auto &a = vars[A + SIZEOF_ACTUATION * i];
            auto &d = vars[D + SIZEOF_ACTUATION * i];
            ht += vt * Lf * d * dt;
            vt += a * dt;
            xt += CppAD::cos(ht) * vt * dt;
            yt += CppAD::sin(ht) * vt * dt;

            // Compute the reference state at time (i * dt).
            auto yr = reference(xt);
            auto hr = CppAD::atan2(yr, xt);

            // Compute the contribution at time i * dt to the cost function.
            fg[0] += CppAD::pow(yt - yr, 2);
            fg[0] += CppAD::pow(ht - hr, 2);
            fg[0] += CppAD::pow(vt - vr, 2);

            // Minimize actuator use.
            fg[0] += CppAD::pow(a, 2);
            fg[0] += CppAD::pow(d, 2);
        }

        // Minimize the value gap between sequential actuations.
        for (int i = 1; i < N_PLAN; ++i) {
            auto &a1 = vars[A + SIZEOF_ACTUATION * (i - 1)];
            auto &d1 = vars[D + SIZEOF_ACTUATION * (i - 1)];

            auto &a2 = vars[A + SIZEOF_ACTUATION * i];
            auto &d2 = vars[D + SIZEOF_ACTUATION * i];

            fg[0] += CppAD::pow(a1 - a2, 2);
            fg[0] += 10 * CppAD::pow(d1 - d2, 2); // Be more strict about direction changes.
        }
    }

private:
  /**
   * @brief Compute the `y` coordinate for the reference route.
   */
  Scalar reference(const Scalar &x) const {
    Scalar y = route(0);
    for (int i = 1, n = route.rows(); i < n; ++i) {
      y += route(i) * CppAD::pow(x, i);
    }

    return y;
  }
};

Waypoints MPP(const State &state, const Lane &lane) {
    // Differentiable value vector type.
    typedef CPPAD_TESTVECTOR(double) Vector;

    // Independent variables and bounds.
    Vector vars(N_PLAN * SIZEOF_ACTUATION);
    Vector vars_lowerbound(N_PLAN * SIZEOF_ACTUATION);
    Vector vars_upperbound(N_PLAN * SIZEOF_ACTUATION);

    // Constraint bounds, set to size 0 as the cost function includes no constraints.
    Vector constraints_lowerbound(0);
    Vector constraints_upperbound(0);

    // Initialize independent variable and bounds vectors.
    for (int i = 0; i < N_PLAN; i++) {
        int i_a = A + i * SIZEOF_ACTUATION;
        int i_d = D + i * SIZEOF_ACTUATION;

        vars[i_a] = 0;
        vars[i_d] = 0;

        vars_lowerbound[i_a] = -1.0;
        vars_upperbound[i_a] = 1.0;

        vars_lowerbound[i_d] = -0.523598;
        vars_upperbound[i_d] = 0.523598;
    }

    // Fit a polynomial to waypoints sampled from the lane.
    Waypoints samples = lane.sample(state);
    samples.toLocalFrame(state);
    auto route = samples.fit();

    // Define the cost function.
    Cost cost(state.v, route);

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

    // Report solution results.
    auto value = solution.obj_value;
    auto status = (solution.status == CppAD::ipopt::solve_result<Vector>::success ? "succeeded" : "failed");
//    std::cout << "Solver " << status << ", final cost value = " << value << std::endl;
    auto &control = solution.x;

    std::vector<double> x; // = {state.x};
    std::vector<double> y; // = {state.y};

    double x_i = state.x;
    double y_i = state.y;
    double o_i = state.o;
    double v_i = state.v;
    for (int i = 0; i < N_PLAN; ++i) {
        double a = control[A + SIZEOF_ACTUATION * i];
        double d = control[D + SIZEOF_ACTUATION * i];

        o_i += v_i * Lf * d * T_PLAN;
        v_i += a * T_PLAN;
        x_i += std::cos(o_i) * v_i * T_PLAN;
        y_i += std::sin(o_i) * v_i * T_PLAN;

        x.push_back(x_i);
        y.push_back(y_i);
    }

    return {x, y};
}
