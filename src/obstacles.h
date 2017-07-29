#ifndef OBSTACLES_H
#define OBSTACLES_H

#include "state.h"
#include "waypoints.h"

#include "json.hpp"

struct Obstacles {
    /** @brief Position of each obstacle. */
    Waypoints positions;

    /** @brief Speed of each obstacle. */
    Waypoints speeds;

    /**
     * @brief Update this object with data from the give state and JSON node.
     */
    void update(double t, const State &state, const nlohmann::json &json);

    /**
     * @brief Return the number of obstacles.
     */
    size_t size() const;
};

#endif
