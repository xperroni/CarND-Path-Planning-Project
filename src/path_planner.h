#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include "highway_map.h"
#include "state.h"
#include "waypoints.h"

#include <tuple>
#include <vector>

struct PathPlanner {
    /** @brief Map of the highway for which paths are going to be planned. */
    HighwayMap highway;

    /**
     * @brief Create a path planner for a highway with given settings.
     */
    PathPlanner(int lanes, double width);

    /**
     * @brief Generate a path from a given car state.
     */
    Waypoints operator () (State state, const Waypoints &previous);
};

#endif
