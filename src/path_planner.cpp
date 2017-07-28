#include "path_planner.h"

#include "MPP.h"
#include "settings.h"

PathPlanner::PathPlanner(int lanes, double width):
    highway(lanes, width)
{
    // Nothing to do.
}

Waypoints PathPlanner::operator () (State state, const Waypoints &previous) {
    const Lane &lane = highway.lanes[1];
    Waypoints waypoints = MPP(state, lane, previous);

//     if (previous.size() > 1) {
//         state = previous.stateLast();
//     }
//
//     Waypoints samples = lane.sample(state);
//     samples.toLocalFrame(state);
//     auto route = samples.fit();
//
//     std::vector<double> x;
//     std::vector<double> y;
//
//     for (size_t i = 0, n = N_PLAN - previous.size(); i < n; ++i) {
//         double x_i = i * T_PLAN * V_PLAN;
//         double y_i = eval(route, x_i);
//         x.push_back(x_i);
//         y.push_back(y_i);
//     }
//
//     Waypoints latest = {x, y};
//     latest.toGlobalFrame(state);
//
//     Waypoints waypoints = previous;
//     waypoints.x.insert(waypoints.x.end(), latest.x.begin(), latest.x.end());
//     waypoints.y.insert(waypoints.y.end(), latest.y.begin(), latest.y.end());

    return waypoints;
}
