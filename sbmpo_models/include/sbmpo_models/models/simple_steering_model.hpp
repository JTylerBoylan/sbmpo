#ifndef SBMPO_SIMPLE_STEERING_MODEL_HPP
#define SBMPO_SIMPLE_STEERING_MODEL_HPP

#include <sbmpo_models/models/benchmarking_model.hpp>

#define M_2PI 6.283185307179586

namespace sbmpo_models {

    using namespace sbmpo;

    const int INTEGRATION_SIZE = 5;

    class SimpleSteeringModel : public BenchmarkingModel {

        public:

        SimpleSteeringModel() {}

        State initial_state() {
            return START_STATE;
        }

        // Evaluate a node with a control
        State next_state(const State &state, const Control& control, const float time_span) {
            
            // Update state
            State next = state;
            float time_increment = time_span / INTEGRATION_SIZE;
            for (int i = 0; i < INTEGRATION_SIZE; i++) {
                next[0] += cosf(next[2]) * control[0] * time_increment;
                next[1] += sinf(next[2]) * control[0] * time_increment;
                next[2] += control[1] * time_increment;
            }

            // Angle wrap
            if (next[2] >= M_2PI || next[2] < 0)
                next[2] += next[2] >= M_2PI ? -M_2PI : M_2PI;

            return next;
        }

        // Get the cost of a control
        float cost(const State &state, const Control& control, const float time_span) {
            return control[0]*time_span;
        }

        // Get the heuristic of a state
        float heuristic(const State& state) {
            const float dx = GOAL_STATE[0] - state[0];
            const float dy = GOAL_STATE[1] - state[1];
            return sqrtf(dx*dx + dy*dy);
        }

        // Determine if node is valid
        bool is_valid(const State& state) {
            
            // Bound check
            if (state[0] - BOUNDS[0][0] < BODY_RADIUS ||
                state[1] - BOUNDS[0][1] < BODY_RADIUS ||
                BOUNDS[1][0] - state[0] < BODY_RADIUS ||
                BOUNDS[1][1] - state[1] < BODY_RADIUS)
                return false;

            // Obstacle collision check
            for (int o = 0; o < OBSTACLES.size(); o++) {
                const float dx = state[0] - OBSTACLES[o][0];
                const float dy = state[1] - OBSTACLES[o][1];
                const float threshold = OBSTACLES[o][2] + BODY_RADIUS;
                if (dx*dx + dy*dy < threshold*threshold)
                    return false;
            }

            return true;
        }

        // Determine if state is goal
        bool is_goal(const State& state) {
            return heuristic(state) <= GOAL_THRESHOLD;
        }

    };

}

#endif