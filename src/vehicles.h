#ifndef VEHICLES_H
#define VEHICLES_H

#include "waypoints.h"

#include "json.hpp"

struct Vehicles {
    /** @brief Position of each vehicle. */
    Waypoints positions;

    /** @brief Speeds of each vehicle. */
    Waypoints speeds;

    /**
     * @brief Default constructor.
     */
    Vehicles();

    /**
     * @brief Create a new object from the given JSON node.
     */
    Vehicles(const nlohmann::json &json);

    /**
     * @brief Return the number of vehicles.
     */
    size_t size() const;
};

#endif
