#ifndef WAYPOINTS_H
#define WAYPOINTS_H

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
     * @brief Computes polynomial coefficients for these waypoints.
     */
    Eigen::VectorXd fit() const;

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
