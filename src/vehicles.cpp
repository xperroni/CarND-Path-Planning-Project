#include "vehicles.h"

Vehicles::Vehicles() {
    // Nothing to do.
}

Vehicles::Vehicles(const nlohmann::json &json) {
    for (int i = 0, n = json.size(); i < n; ++i) {
        positions.x.push_back(json[i][1]);
        positions.y.push_back(json[i][2]);

        speeds.x.push_back(json[i][3]);
        speeds.y.push_back(json[i][4]);
    }
}

size_t Vehicles::size() const {
    return positions.x.size();
}
