#ifndef STATE_H
#define STATE_H

#include <cmath>

struct State {
    /** @brief Current position in the horizontal axis. */
    double x;

    /** @brief Current position in the vertical axis. */
    double y;

    /** @brief Current orientation in radians. */
    double o;

    /** @brief Current linear speed. */
    double v;

    /**
     * @brief Default constructor.
     */
    State():
        x(0),
        y(0),
        o(0),
        v(0)
    {
        // Nothing to do.
    }

    /**
     * @brief Create a new state from the given map.
     */
    template<class T> State(const T &map);
};

template<class T> State::State(const T &map) {
    x = map["x"];
    y = map["y"];
    o = ((double) map["yaw"]) * M_PI / 180.0; // Convert from degrees to radians
    v = ((double) map["speed"]) * 0.447; // Convert from MPH to m/s
}

#endif
