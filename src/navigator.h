#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include "highway_map.h"
#include "obstacles.h"
#include "state.h"
#include "waypoints.h"

struct Navigator {
    /** @brief Map of the highway on which the navigator will drive. */
    HighwayMap highway;

    /** @brief Current car position and speed. */
    State state;

    /** @brief Current route plan. */
    Waypoints route;

    /** @brief Surrounding obstacles. */
    Obstacles obstacles;

    /**
     * @brief Create a navigator for a highway with given settings.
     */
    Navigator(int lanes, double width);

    /**
     * @brief Update the navigator with sensor information, returning the updated route.
     */
    const Waypoints &operator () (const nlohmann::json &json);
};

#endif
