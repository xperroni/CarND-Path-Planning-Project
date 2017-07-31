#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include "behavior_planner.h"
#include "waypoints.h"


struct PathPlanner {
    /** @brief Current route plan. */
    Waypoints plan;

    /**
     * @brief Plan a path for the given behavior.
     */
    const Waypoints &operator () (const BehaviorPlanner &behavior);
};

#endif
