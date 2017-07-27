#ifndef HIGHWAY_MAP_H
#define HIGHWAY_MAP_H

#include <iostream>
#include <vector>

struct Lane {
    double width;

    std::vector<double> x;

    std::vector<double> y;

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
     * @brief Return the index of the closest waypoint in front of the given pose.
     */
    size_t nextIndex(double x, double y, double o) const;

    /**
     * @brief Return the number of waypoints in the lane.
     */
    size_t size() const;
};

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
