#ifndef WAYPOINTS_H
#define WAYPOINTS_H

#include "state.h"

#include <Eigen/Dense>

#include <cstddef>
#include <vector>

struct Waypoints {
    /** @brief Waypoint horizontal coordinates. */
    std::vector<double> x;

    /** @brief Waypoint vertical coordinates. */
    std::vector<double> y;

    /**
     * @brief Default constructor.
     */
    Waypoints();

    /**
     * @brief Create a new waypoint with given coordinate vectors.
     */
    Waypoints(const std::vector<double> &x, const std::vector<double> &y);

    /**
     * @brief Return the predicted state at the last waypoint.
     */
    State stateLast() const;

    /**
     * @brief Computes polynomial coefficients for these waypoints.
     */
    Eigen::VectorXd fit() const;

    /**
     * @brief Convert these waypoints to the global frame using the given state as reference.
     */
    void toGlobalFrame(const State &state);

    /**
     * @brief Convert these waypoints to a local frame relative to the given state.
     */
    void toLocalFrame(const State &state);

    /**
     * @brief Return the number of waypoints.
     */
    size_t size() const;
};

/**
 * @brief Evaluate the given polynomial at the given position.
 */
double eval(const Eigen::VectorXd &a, double x);

#endif
