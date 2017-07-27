#ifndef STATE_H
#define STATE_H

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
    o = map["yaw"];
    v = map["speed"];
}

#endif
