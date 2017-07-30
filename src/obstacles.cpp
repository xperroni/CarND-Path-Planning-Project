#include "obstacles.h"

// void Obstacles::update(double t, const State &state, const nlohmann::json &json) {
void Obstacles::update(double t, const HighwayMap &highway, const nlohmann::json &json) {
    cartesian.x.clear();
    cartesian.y.clear();
    frenet.x.clear();
    frenet.y.clear();
    lanes.clear();
    speeds.x.clear();
    speeds.y.clear();
    speeds.s.clear();
    speeds.d.clear();

    // Select an arbitrary lane for determining longitudinal position of obstacles.
    const Lane &lane = highway.lanes[0];

    // Read obstacles from the JSON node.
    for (int i = 0, n = json.size(); i < n; ++i) {
        double x_i = json[i][1];
        double y_i = json[i][2];
        double v_x = json[i][3];
        double v_y = json[i][4];
        double s_i = json[i][5];
        double d_i = json[i][6];

        // Convert cartesian speeds to frenet speeds,
        // using the road's local frame as reference.
        size_t l = lane.closestIndex(s_i);
        double v = std::sqrt(v_x * v_x + v_y * v_y);
        double o = std::atan2(v_y, v_x) - lane.o[l];
        double v_s = v * std::cos(o);
        double v_d = v * std::sin(o);

//         std::cout << x_i << ", " << y_i << ", " << s_i << ", " << d_i << std::endl;

        cartesian.x.push_back(x_i);
        cartesian.y.push_back(y_i);
        frenet.x.push_back(s_i);
        frenet.y.push_back(d_i);

        speeds.x.push_back(v_x);
        speeds.y.push_back(v_y);
        speeds.s.push_back(v_s);
        speeds.d.push_back(v_d);
    }

//         std::cout << std::endl;

    // Update obstacle positions to the given time step.
    for (size_t i = 0, n = size(); i < n; ++i) {
        cartesian.x[i] += speeds.x[i] * t;
        cartesian.y[i] += speeds.y[i] * t;
        frenet.x[i] += speeds.s[i] * t;
        frenet.y[i] += speeds.d[i] * t;

        // Record estimated lane
        lanes.push_back(highway.closestIndex(frenet.y[i]));
    }

//     // Convert obstacle positions and speeds to the current state's local frame.
//     state.toLocalFrame(positions);
//
//     double cos_o = std::cos(state.o);
//     double sin_o = std::sin(state.o);
//     for (size_t i = 0, n = size(); i < n; ++i) {
//         double v_x = speeds.x[i];
//         double v_y = speeds.y[i];
//
//         speeds.x[i] = v_x * cos_o + v_y * sin_o;
//         speeds.y[i] = v_y * cos_o - v_x * sin_o;
// //         std::cout << positions.x[i] << ", " << positions.y[i] << std::endl;
//     }
}

size_t Obstacles::size() const {
    return cartesian.x.size();
}
