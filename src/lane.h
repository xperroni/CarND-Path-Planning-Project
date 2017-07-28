#ifndef LANE_H
#define LANE_H

#include "waypoints.h"

#include <cstddef>
#include <tuple>
#include <vector>

struct Lane: Waypoints {
    double width;

    /**
     * @brief Default constructor.
     */
    Lane();

    /**
     * @brief Create a new Lane of given width.
     */
    Lane(double width);

    /**
     * @brief Return the index of the lane waypoint closest to the given point.
     */
    size_t closestIndex(double x, double y) const;

    /**
     * @brief Return the index of the lane waypoint closest to the given point, and the square distance to it..
     */
    std::tuple<size_t, double> closestIndexD2(double x, double y) const;

    /**
     * @brief Return the index of the closest waypoint in front of the given pose.
     */
    size_t nextIndex(const State &state) const;

    /**
     * @brief Return a list of sample waypoints from the state's location.
     */
    Waypoints sample(const State &state) const;
};

#endif
