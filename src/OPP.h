#ifndef OPP_H
#define OPP_H

#include "lane.h"
#include "state.h"
#include "waypoints.h"

/**
 * @brief The Optimizing Path Planner (OPP) computes a sequence of waypoints to approach the given lane from the given state.
 */
Waypoints OPP(const State &state, const Lane &lane);

#endif
