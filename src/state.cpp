#include "state.h"

#include "settings.h"

#include <cmath>

State::State():
    x(0),
    y(0),
    o(0),
    v(0)
{
    // Nothing to do.
}

static void updateVehicles(double t, Vehicles &vehicles) {
    for (int i = 0, n = vehicles.size(); i < n; i++) {
        double v_x = vehicles.speeds.x[i];
        double v_y = vehicles.speeds.y[i];
        vehicles.positions.x[i] += v_x * t;
        vehicles.positions.y[i] += v_y * t;

//         std::cout << vehicles.positions.x[i] << ", " << vehicles.positions.y[i] << std::endl;
    }

//     std::cout << std::endl;
}

State::State(const nlohmann::json &json):
    x(json["x"]),
    y(json["y"]),
    o(((double) json["yaw"]) * M_PI / 180.0), // Convert from degrees to radians
    v(((double) json["speed"]) * 0.447), // Convert from MPH to m/s
    route(json["previous_path_x"], json["previous_path_y"]),
    vehicles(json["sensor_fusion"])
{
    toLocalFrame(vehicles.positions);

    double cos_o = std::cos(o);
    double sin_o = std::sin(o);
    for (size_t i = 0, n = vehicles.size(); i < n; ++i) {
        double v_x = vehicles.speeds.x[i];
        double v_y = vehicles.speeds.y[i];

        vehicles.speeds.x[i] = v_x * cos_o + v_y * sin_o;
        vehicles.speeds.y[i] = v_y * cos_o - v_x * sin_o;
    }

    size_t n = route.size();
    if (n == 0) {
        return;
    }

    if (n > 10) {
        n = 10;
        route.x.erase(route.x.begin() + 10, route.x.end());
        route.y.erase(route.y.begin() + 10, route.y.end());
    }

    // If the route is not empty, use the last waypoint
    // to update the state's position.
    size_t l = route.size() - 1;
    double x_b = route.x[l];
    double y_b = route.y[l];

    x = x_b;
    y = y_b;

    if (l == 0) {
        updateVehicles(T_PLAN, vehicles);
        return;
    }

    // If the route has more than one waypoint, use the last two waypoints to
    // update the state's orientation and speed.
    double x_a = route.x[l - 1];
    double y_a = route.y[l - 1];
    double x_d = x_b - x_a;
    double y_d = y_b - y_a;

    o = std::atan2(y_d, x_d);
    v = std::sqrt(x_d * x_d + y_d * y_d) / T_PLAN;

    updateVehicles(n * T_PLAN, vehicles);
}

void State::toLocalFrame(Waypoints &waypoints) const {
    double cos_o = std::cos(o);
    double sin_o = std::sin(o);

    for (int i = 0, n = waypoints.size(); i < n; ++i) {
        double &x_i = waypoints.x[i];
        double &y_i = waypoints.y[i];

        double x_s = x_i - x;
        double y_s = y_i - y;

        x_i = x_s * cos_o + y_s * sin_o;
        y_i = y_s * cos_o - x_s * sin_o;
    }
}

void State::toGlobalFrame(Waypoints &waypoints) const {
    double cos_o = std::cos(o);
    double sin_o = std::sin(o);

    for (int i = 0, n = waypoints.size(); i < n; ++i) {
        double &x_i = waypoints.x[i];
        double &y_i = waypoints.y[i];

        double x_r = x_i * cos_o - y_i * sin_o;
        double y_r = y_i * cos_o + x_i * sin_o;

        x_i = x + x_r;
        y_i = y + y_r;
    }
}
