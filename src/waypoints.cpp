#include "waypoints.h"

#include "settings.h"

#include <Eigen/QR>

using Eigen::ArrayXd;
using Eigen::Map;
using Eigen::MatrixXd;
using Eigen::VectorXd;

#include <cmath>

Waypoints::Waypoints() {
    // Nothing to so.
}

Waypoints::Waypoints(const std::vector<double> &x, const std::vector<double> &y) {
    this->x = x;
    this->y = y;
}

State Waypoints::stateLast() const {
    size_t l = size() - 1;
    double x_a = x[l - 1];
    double y_a = y[l - 1];
    double x_b = x[l];
    double y_b = y[l];
    double x_d = x_b - x_a;
    double y_d = y_b - y_a;

    State state;
    state.x = x_b;
    state.y = y_b;
    state.o = std::atan2(y_d, x_d);
    state.v = std::sqrt(x_d * x_d + y_d * y_d) / T_PLAN;

    return state;
}

VectorXd Waypoints::fit() const {
    int rows = size();
    Map<const ArrayXd> x(this->x.data(), rows);
    Map<const VectorXd> y(this->y.data(), rows);

    MatrixXd A(rows, N_FIT + 1);
    A.block(0, 0, rows, 1).fill(1.0);
    for (int j = 0; j < N_FIT; j++) {
        auto Aj = A.block(0, j, rows, 1).array();
        A.block(0, j + 1, rows, 1) = Aj * x;
    }

    auto Q = A.householderQr();
    auto result = Q.solve(y);
    return result;
}

void Waypoints::toGlobalFrame(const State &state) {
    double x_0 = state.x;
    double y_0 = state.y;
    double o_0 = state.o;

    double cos_o = std::cos(o_0);
    double sin_o = std::sin(o_0);

    for (int i = 0, n = x.size(); i < n; ++i) {
        double x_l = x[i];
        double y_l = y[i];

        double x_r = x_l * cos_o - y_l * sin_o;
        double y_r = y_l * cos_o + x_l * sin_o;

        x[i] = x_0 + x_r;
        y[i] = y_0 + y_r;
    }
}

void Waypoints::toLocalFrame(const State &state) {
    double x_0 = state.x;
    double y_0 = state.y;
    double o_0 = state.o;

    double cos_o = std::cos(o_0);
    double sin_o = std::sin(o_0);

    for (int i = 0, n = x.size(); i < n; ++i) {
        double x_s = x[i] - x_0;
        double y_s = y[i] - y_0;

        x[i] = x_s * cos_o + y_s * sin_o;
        y[i] = y_s * cos_o - x_s * sin_o;
    }
}

size_t Waypoints::size() const {
    return x.size();
}

double eval(const VectorXd &a, double x) {
    double y = a(0);
    for (int i = 1, n = a.rows(); i < n; ++i) {
        y += a(i) * std::pow(x, i);
    }

    return y;
}
