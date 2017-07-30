#include "behavior_planner.h"

using Eigen::VectorXd;

#include "settings.h"

#include <limits>

typedef BehaviorPlanner::Behavior Behavior;

static Behavior START(BehaviorPlanner &planner);

static Behavior CRUISING(BehaviorPlanner &planner);

static Behavior TAILING(BehaviorPlanner &planner);

static Behavior CHANGING_LANES(BehaviorPlanner &planner);

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
    std::tie(i, d) = planner.obstacles.closestAhead(planner.state.lane, planner.state.s);

    if (d >= V_PLAN) {
        planner.v = V_PLAN;
        return CRUISING;
    }
    else {
        planner.v = planner.obstacles.speeds.s[i];
        return TAILING;
    }
}

static Behavior TAILING(BehaviorPlanner &planner) {
    const State &state = planner.state;
    const Obstacles &obstacles = planner.obstacles;
    const HighwayMap &highway = planner.highway;
    double s = state.s;

    size_t i_front;
    double d_front;
    std::tie(i_front, d_front) = obstacles.closestAhead(state.lane, state.s);
    if (d_front >= 2 * V_PLAN) {
        planner.v = V_PLAN;
        return CRUISING;
    }

    planner.v = obstacles.speeds.s[i_front];

    for (size_t lane: highway.adjacentLanes(state.lane)) {
        size_t i_behind = 0;
        double d_behind = 0;
        std::tie(i_behind, d_behind) = obstacles.closestAhead(lane, s);
        if (
            (d_behind < 5) ||
            (d_behind - 3 * (obstacles.speeds.s[i_behind] - planner.v) < 5)
        ) {
            continue;
        }

        size_t i_ahead = 0;
        double d_ahead = 0;
        std::tie(i_ahead, d_ahead) = obstacles.closestAhead(lane, s);
        if (
            (d_ahead - d_front > 5) &&
            (d_ahead + 3 * obstacles.speeds.s[i_ahead] - (d_front + 3 * obstacles.speeds.s[i_front]) > 5)
        ) {
            planner.target_lane = lane;
            return CHANGING_LANES;
        }
    }

    return TAILING;
}

static Behavior CHANGING_LANES(BehaviorPlanner &planner) {
    const State &state = planner.state;
    const Obstacles &obstacles = planner.obstacles;
    const HighwayMap &highway = planner.highway;
    double s = state.s;

    size_t i_origin = 0;
    double d_origin = 0;
    std::tie(i_origin, d_origin) = obstacles.closestAhead(planner.origin_lane, s);

    size_t i_target = 0;
    double d_target = 0;
    std::tie(i_target, d_target) = obstacles.closestAhead(planner.origin_lane, s);

    if (highway.closestIndex(state.d) == planner.target_lane) {
        planner.origin_lane = planner.target_lane;
        return CRUISING;
    }

    planner.v = std::min(obstacles.speeds.s[i_origin], obstacles.speeds.s[i_target]);

    return CHANGING_LANES;
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
