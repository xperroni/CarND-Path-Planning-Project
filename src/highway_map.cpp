#include "highway_map.h"

#include <cmath>
#include <limits>
#include <tuple>

HighwayMap::HighwayMap(size_t n, double width):
    lanes(n, Lane(width))
{
    // Nothing to do.
}

size_t HighwayMap::closestIndex(double x, double y) const {
    size_t i_closest = 0;
    double d2_closest = std::numeric_limits<double>::max();

    for(size_t i = 0, n = size(); i < n; i++) {
        size_t _;
        double d2;
        std::tie(_, d2) = lanes[i].closestIndexD2(x, y);
        if (d2 < d2_closest) {
            d2_closest = d2;
            i_closest = i;
        }
    }

    return i_closest;
}

const Lane &HighwayMap::closestLane(double x, double y) const {
    return lanes[closestIndex(x, y)];
}

size_t HighwayMap::size() const {
    return lanes.size();
}

std::istream &operator >> (std::istream &in, HighwayMap &highway) {
    // Read waypoints located over the line separating highway sides.
    std::vector<double> line_x;
    std::vector<double> line_y;
    for (;;) {
        double x, y, s, d_x, d_y;
        in >> x >> y >> s >> d_x >> d_y;
        if (in.eof()) {
            break;
        }

        line_x.push_back(x);
        line_y.push_back(y);
    }

    // Compute waypoints for each highway lane.
    for (size_t i = 0, m = line_x.size(); i < m; ++i) {
        // Fetch the "current" and "next" waypoints.
        double x_0 = line_x[i];
        double y_0 = line_y[i];
        double x_1 = line_x[(i + 1) % m];
        double y_1 = line_y[(i + 1) % m];

        // Compute a normalized vector perpendicular to the
        // direction from (x_0, y_0) to (x_1, y_1)
        // and pointing rightwards.
        double x_d = x_1 - x_0;
        double y_d = y_1 - y_0;
        double l_d = std::sqrt(x_d * x_d + y_d * y_d);
        double x_p = y_d / l_d;
        double y_p = -x_d / l_d;

        // Compute the wapoints for each lane in the highway map.
        std::vector<Lane> &lanes = highway.lanes;
        for (size_t j = 0, n = lanes.size(); j < n; ++j) {
            Lane &lane = lanes[j];
            double d = (j + 0.5) * lane.width;
            lane.x.push_back(x_0 + x_p * d);
            lane.y.push_back(y_0 + y_p * d);
        }
    }

    return in;
}
