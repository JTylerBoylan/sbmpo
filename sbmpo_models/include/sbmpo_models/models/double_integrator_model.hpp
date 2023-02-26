#ifndef SBMPO_DOUBLE_INTEGRATOR_MODEL_HPP
#define SBMPO_DOUBLE_INTEGRATOR_MODEL_HPP

#include <sbmpo_models/models/benchmarking_model.hpp>

#define M_2PI 6.283185307179586

namespace sbmpo_models {

    using namespace sbmpo;

    const float GOAL_THRESHOLD_X = 0.05f;
    const float GOAL_THRESHOLD_V = 0.05f;

    class DoubleIntegratorModel : public BenchmarkingModel {

        public:

        DoubleIntegratorModel() {}

        State initial_state() {
            return START_STATE;
        }

        // Evaluate a node with a control
        State next_state(const State &state, const Control& control, const float time_span) {
            
            // Update state
            State next = state;
            next[0] += state[1] * time_span;
            next[1] += control[0] * time_span;
            return next;

        }

        // Get the cost of a control
        float cost(const State& state, const Control& control, const float time_span) {
            return time_span;
        }

        // Get the heuristic of a state
        float heuristic(const State& state) {
            return 0;
        }

        // Determine if node is valid
        bool is_valid(const State& state) {
            return true;
        }

        // Determine if state is goal
        bool is_goal(const State& state) {
            return abs(GOAL_STATE[0] - state[0]) <= GOAL_THRESHOLD_X
                && abs(GOAL_STATE[1] - state[1]) <= GOAL_THRESHOLD_V;
        }

    };

}

#endif