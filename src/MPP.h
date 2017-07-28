#ifndef MPP_H
#define MPP_H

#include "lane.h"
#include "state.h"
#include "waypoints.h"

/**
 * @brief Compute a sequence of waypoints to approach the given lane from the given state.
 */
Waypoints MPP(const State &state, const Lane &lane);

#endif
