#ifndef SBMPO_MODEL_HPP
#define SBMPO_MODEL_HPP

#include <sbmpo/types/node.hpp>

#include <cmath>

namespace sbmpo {

class Model {

    public:

    /// @brief Get the initial state of the Model
    /// @return State object
    virtual State initial_state() = 0;

    /// @brief Evaluate a state with a control
    /// @param state Current state
    /// @param control Current control
    /// @param time_span Control time span
    /// @return New state from control
    virtual State next_state(const State& state, const Control& control, const float time_span) = 0;

    /// @brief Get the cost of a control
    /// @param state New state
    /// @param control Control used
    /// @param time_span Control time span
    /// @return Cost of the state and control
    virtual float cost(const State& state, const Control& control, const float time_span) = 0;

    /// @brief Get the heuristic of a state
    /// @param state State to get heuristic of
    /// @return Value of heuristic
    virtual float heuristic(const State& state) = 0;

    /// @brief Determine if a state is the goal
    /// @param state State to be determined
    /// @return True if the state is the goal
    virtual bool is_goal(const State& state) = 0;

    /// @brief Determine if a state is valid
    /// @param state State to be determined
    /// @return True if the state is valid
    virtual bool is_valid(const State& state) = 0;

};

}

#endif