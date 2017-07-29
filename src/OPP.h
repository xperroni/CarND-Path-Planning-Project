#ifndef OPP_H
#define OPP_H

#include "lane.h"
#include "state.h"
#include "waypoints.h"

#include <Eigen/Dense>

/**
 * @brief The Optimizing Path Planner (OPP) computes a sequence of waypoints to approach the given route from the origin at `(0, 0)`.
 */
Waypoints OPP(size_t n_plan, double v_0, double v_r, const Eigen::VectorXd &route);

#endif
