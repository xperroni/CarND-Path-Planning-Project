#ifndef STATE_H
#define STATE_H

#include "waypoints.h"

#include "json.hpp"

// Forward declaration.
struct HighwayMap;

struct State {
    /** @brief Current position in Cartesian coordinates. */
    double x, y;

    /** @brief Current orientation in radians. */
    double o;

    /** @brief Current position in Frenet coordinates. */
    double s, d;

    /** @brief Current linear speed. */
    double v;

    /** @brief Current lane. */
    size_t lane;

    /**
     * @brief Default constructor.
     */
    State();

    /**
     * @brief Update this state with data from the given route and JSON node.
     */
    void update(const HighwayMap &highway, const Waypoints &route, const nlohmann::json &json);

    /**
     * @brief Convert the given waypoints to a local frame relative to this state.
     */
    void toLocalFrame(Waypoints &waypoints) const;

    /**
     * @brief Convert the given waypoints to the global frame using this state as reference.
     */
    void toGlobalFrame(Waypoints &waypoints) const;
};

#endif
