#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H

#include "highway_map.h"
#include "obstacles.h"
#include "state.h"

#include "json.hpp"

#include <Eigen/Dense>

#include <functional>

struct BehaviorPlanner {
    /**
     * @brief A state in the Behavior Planner's state machine.
     */
    struct Behavior: std::function<Behavior(BehaviorPlanner&)> {
        /** @brief A convenient alias for the base type. */
        typedef std::function<Behavior(BehaviorPlanner&)> functor;

        /**
         * @brief Default constructor.
         */
        Behavior():
            functor()
        {
            // Nothing to do.
        }

        /**
         * @brief Wraps a custom function into a Behavior object.
         */
        template<class F> Behavior(F f):
            functor(f)
        {
            // Nothing to do.
        }
    };

    /** @brief Currently selected behavior. */
    Behavior behavior;

    /** @brief Map of the highway on which the navigator will drive. */
    HighwayMap highway;

    /** @brief Current lane, or departing lane if the car is in the process of changing lanes. */
    size_t origin_lane;

    /** @brief Current lane, or approaching lane if the car is in the process of changing lanes. */
    size_t target_lane;

    /** @brief Current car position and speed. */
    State state;

    /** @brief Surrounding obstacles. */
    Obstacles obstacles;

    /** @brief Target speed. */
    double v;

    /** @brief Target route. */
    Eigen::VectorXd route;

    /**
     * @brief Default constructor.
     */
    BehaviorPlanner();

    /**
     * @brief Update the behavior planner with data from the currently planned route and given JSON node.
     */
    void update(const Waypoints &plan, const nlohmann::json &json);
};

#endif
