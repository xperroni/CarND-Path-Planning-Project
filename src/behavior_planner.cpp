#include "behavior_planner.h"

using Eigen::VectorXd;

#include "settings.h"

#include <limits>

typedef BehaviorPlanner::Behavior Behavior;

static Behavior START(BehaviorPlanner &planner);

static Behavior CRUISING(BehaviorPlanner &planner);

static std::tuple<size_t, double> closestFrontObstacle(BehaviorPlanner &planner) {
    const Obstacles &obstacles = planner.obstacles;
    double d_closest = std::numeric_limits<double>::max();
    size_t i_closest = 0;

    size_t l = planner.state.lane;
    double s = planner.state.s;

    for (size_t i = 0, n = obstacles.size(); i < n; ++i) {
//         std::cout << "    Obstacle (i, lane) = " << i << ", " << obstacles.lanes[i] << std::endl;
        if (l != obstacles.lanes[i]) {
            continue;
        }

        double s_i = obstacles.frenet.x[i];
        double d = (s < s_i ? s_i - s : s_i + S_MAX - s);
        if (d < d_closest) {
            d_closest = d;
            i_closest = i;
        }
    }

//     std::cout << "Lane, s, d_closest = " << l << ", " << s << ", " << d_closest << std::endl;

    return std::make_tuple(i_closest, d_closest);
}

static Behavior START(BehaviorPlanner &planner) {
    size_t lane = planner.highway.closestIndex(planner.state);
    planner.origin_lane = lane;
    planner.target_lane = lane;
    planner.v = V_PLAN;

    return CRUISING;
}

static Behavior CRUISING(BehaviorPlanner &planner) {
    size_t i;
    double d;
    std::tie(i, d) = closestFrontObstacle(planner);

    if (d >= V_PLAN) {
        planner.v = V_PLAN;
    }
    else {
        planner.v = planner.obstacles.speeds.s[i];
    }

    return CRUISING;
}

BehaviorPlanner::BehaviorPlanner():
    behavior(START)
{
    // Nothing to do.
}

void BehaviorPlanner::update(const Waypoints &plan, const nlohmann::json &json) {
    state.update(highway, plan, json);
    obstacles.update(plan.size() * T_PLAN, highway, json["sensor_fusion"]);

    // Update the current state of the behavior state machine.
    behavior = behavior(*this);

    // Fit a polynomial to waypoints sampled from the current lane.
    const Lane &lane = highway.lanes[target_lane];
    auto samples = lane.sample(state);
    state.toLocalFrame(samples);
    route = samples.fit();
}
