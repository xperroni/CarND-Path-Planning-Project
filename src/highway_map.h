#ifndef HIGHWAY_MAP_H
#define HIGHWAY_MAP_H

#include "lane.h"

#include <iostream>
#include <vector>

struct HighwayMap {
    std::vector<Lane> lanes;

    /**
     * @brief Create a highway map with the given number of lanes, each of given
     * width in meters.
     */
    HighwayMap(size_t lanes, double width);

    /**
     * @brief Return the index of the lane closest to the given point.
     */
    size_t closestIndex(double x, double y) const;

    /**
     * @brief Return a reference to the lane closest to the given point.
     */
    const Lane &closestLane(double x, double y) const;

    /**
     * @brief Return the number of lanes in the highway.
     */
    size_t size() const;
};

std::istream &operator >> (std::istream &in, HighwayMap &highway);

#endif
