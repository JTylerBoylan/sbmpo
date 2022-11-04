#ifndef MODEL_HPP
#define MODEL_HPP

#include <sbmpo/types.hpp>

namespace sbmpo {

    class Model {

        public:

            // Evaluate a node with a control
            virtual bool next_state(State& state2, const State& state1, const Control& control, const float time_span) = 0;

            // Get the cost of a control
            virtual float cost(const State& state2, const State& state1, const Control& control, const float time_span) = 0;

            // Get the heuristic of a state
            virtual float heuristic(const State& state, const State& goal) = 0;

            // Determine if node is valid
            virtual bool is_valid(const State& state) = 0;

            // Determine if state is goal
            virtual bool is_goal(const State& state, const State& goal, const float goal_threshold) = 0;

    };

}

#endif