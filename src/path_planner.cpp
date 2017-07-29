#include "path_planner.h"

#include "OPP.h"
#include "settings.h"

PathPlanner::PathPlanner(int lanes, double width):
    highway(lanes, width)
{
    // Nothing to do.
}

Waypoints PathPlanner::operator () (const State &state) {
    const Lane &lane = highway.lanes[1];
    return OPP(state, lane);
}
