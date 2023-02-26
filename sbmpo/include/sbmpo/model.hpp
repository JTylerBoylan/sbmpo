#ifndef SBMPO_MODEL_HPP
#define SBMPO_MODEL_HPP

#include <sbmpo/types/types.hpp>

namespace sbmpo {

class Model {

    public:

    // Get the beginning state
    virtual State initial_state() = 0;

    // Evaluate a node with a control
    virtual State next_state(const State& state, const Control& control, const float time_span) = 0;

    // Get the cost of a control
    virtual float cost(const State& state, const Control& control, const float time_span) = 0;

    // Get the heuristic of a state
    virtual float heuristic(const State& state) = 0;

    // Determine if state is goal
    virtual bool is_goal(const State& state) = 0;

    // Determine if state is valid
    virtual bool is_valid(const State& state) = 0;

};

}

#endif