#ifndef STATE_H
#define STATE_H

#include "vehicles.h"
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

    /** @brief Planned route. */
    Waypoints route;

    /** @brief Information on other vehicles. */
    Vehicles vehicles;

    /**
     * @brief Default constructor.
     */
    State();

    /**
     * @brief Create a new state from the given JSON node.
     */
    State(const nlohmann::json &json);

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
