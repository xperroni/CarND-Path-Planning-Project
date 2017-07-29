#include "navigator.h"

#include "OPP.h"
#include "settings.h"

Navigator::Navigator(int lanes, double width):
    highway(lanes, width)
{
    // Nothing to do.
}

const Waypoints &Navigator::operator () (const nlohmann::json &json) {
    route.update(json);
    state.update(json);
    obstacles.update(route.size() * T_PLAN, state, json["sensor_fusion"]);
    state.update(route);

    // Compute the number of waypoints to generate.
    size_t n_plan = N_PLAN - route.size();

    // Fit a polynomial to waypoints sampled from the current lane.
    const Lane &lane = highway.lanes[1];
    auto samples = lane.sample(state);
    state.toLocalFrame(samples);
    auto a = samples.fit();

    Waypoints planned = OPP(n_plan, state.v, V_PLAN, a);
    state.toGlobalFrame(planned);
    route.x.insert(route.x.end(), planned.x.begin(), planned.x.end());
    route.y.insert(route.y.end(), planned.y.begin(), planned.y.end());

    return route;
}

