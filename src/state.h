#ifndef STATE_H
#define STATE_H

#include "waypoints.h"

#include "json.hpp"

struct State {
    /** @brief Current position in the horizontal axis. */
    double x;

    /** @brief Current position in the vertical axis. */
    double y;

    /** @brief Current orientation in radians. */
    double o;

    /** @brief Current linear speed. */
    double v;

    /**
     * @brief Default constructor.
     */
    State();

    /**
     * @brief Update this state with data from the given JSON node.
     */
    void update(const nlohmann::json &json);

    /**
     * @brief Update this state with data from the given route.
     */
    void update(const Waypoints &route);

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
