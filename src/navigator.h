#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include "behavior_planner.h"
#include "waypoints.h"

struct Navigator {
    /** @brief Behavior planner. */
    BehaviorPlanner behavior;

    /** @brief Current route plan. */
    Waypoints route;

    /**
     * @brief Update the navigator with sensor information, returning the updated route.
     */
    const Waypoints &operator () (const nlohmann::json &json);
};

#endif
