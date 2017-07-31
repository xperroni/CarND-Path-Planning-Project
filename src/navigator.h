#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include "behavior_planner.h"
#include "path_planner.h"
#include "waypoints.h"

struct Navigator {
    /** @brief Behavior planner. */
    BehaviorPlanner behavior;

    /** @brief Path planner. */
    PathPlanner path;

    /**
     * @brief Update the navigator with sensor information, returning the updated route.
     */
    const Waypoints &operator () (const nlohmann::json &json);
};

#endif
