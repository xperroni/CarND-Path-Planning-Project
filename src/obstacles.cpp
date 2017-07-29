#include "obstacles.h"

void Obstacles::update(double t, const State &state, const nlohmann::json &json) {
    for (int i = 0, n = json.size(); i < n; ++i) {
        positions.x.push_back(json[i][1]);
        positions.y.push_back(json[i][2]);

        speeds.x.push_back(json[i][3]);
        speeds.y.push_back(json[i][4]);
    }

    state.toLocalFrame(positions);

    double cos_o = std::cos(state.o);
    double sin_o = std::sin(state.o);
    for (size_t i = 0, n = size(); i < n; ++i) {
        double v_x = speeds.x[i];
        double v_y = speeds.y[i];

        speeds.x[i] = v_x * cos_o + v_y * sin_o;
        speeds.y[i] = v_y * cos_o - v_x * sin_o;

        positions.x[i] += speeds.x[i] * t;
        positions.y[i] += speeds.y[i] * t;

//         std::cout << positions.x[i] << ", " << positions.y[i] << std::endl;
    }
}

size_t Obstacles::size() const {
    return positions.x.size();
}
