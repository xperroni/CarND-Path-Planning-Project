#include "navigator.h"

#include "OPP.h"
#include "settings.h"

const Waypoints &Navigator::operator () (const nlohmann::json &json) {
    route.update(json);
    behavior.update(route, json);

    // Compute the number of waypoints to generate.
    size_t n_plan = N_PLAN - route.size();

    Waypoints planned = OPP(n_plan, behavior.state.v, behavior.v, behavior.route);
    behavior.state.toGlobalFrame(planned);
    route.x.insert(route.x.end(), planned.x.begin(), planned.x.end());
    route.y.insert(route.y.end(), planned.y.begin(), planned.y.end());

    return route;
}

