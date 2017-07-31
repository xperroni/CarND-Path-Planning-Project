#include "navigator.h"

const Waypoints &Navigator::operator () (const nlohmann::json &json) {
    path.plan.update(json);
    behavior.update(path.plan, json);
    return path(behavior);
}
