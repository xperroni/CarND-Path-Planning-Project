#include "path_planner.h"

PathPlanner::PathPlanner(int lanes, double width):
    highway(lanes, width)
{
    // Nothing to do.
}

Waypoints PathPlanner::operator () (const State &state) {
    const Lane &lane = highway.closestLane(state.x, state.y);
    size_t k = lane.nextIndex(state.x, state.y, state.o);
    size_t n = lane.size();

    std::vector<double> x;
    std::vector<double> y;
    for(size_t i = 0; i < 10; ++i, ++k) {
        x.push_back(lane.x[k % n]);
        y.push_back(lane.y[k % n]);
    }

    return std::make_tuple(x, y);
}
