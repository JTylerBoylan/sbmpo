#ifndef SBMPO_BOOK_MODEL_HPP
#define SBMPO_BOOK_MODEL_HPP

#include <sbmpo/model.hpp>
#include <sbmpo_models/csv_util.hpp>
#include <math.h>

#define M_2PI 6.283185307179586

namespace sbmpo_models {

    using namespace sbmpo;

    const float bounds[2][2] = {
        {-1.0, -1.0},
        {6.0, 6.0}  
    };

    #define BODY_RADIUS 0.0f

    #define NUM_OBSTACLES 3
    const float obstacles[3][NUM_OBSTACLES] = {
        {3.1, 1.2, 0.5},
        {3.5, 3.7, 0.5},
        {1.0, 0.5, 0.5}
    };

    class SBMPOBookModel : public sbmpo::Model {

        // Evaluate a node with a control
        bool next_state(State &state2, const State &state1, const Control& control, const float time_span) {
            state2[0] = state1[0] + cosf(state1[2]) * control[0] * time_span;
            state2[1] = state1[1] + sinf(state1[2]) * control[0] * time_span;
            state2[2] = state1[2] + control[1] * time_span;

            // Angle wrap
            if (state2[2] >= M_2PI || state2[2] < 0)
                state2[2] += state2[2] >= M_2PI ? -M_2PI : M_2PI;

            return true;
        }

        // Get the cost of a control
        float cost(const State& state2, const State &state1, const Control& control, const float time_span) {
            return abs(control[0]) * time_span;
        }

        // Get the heuristic of a state
        float heuristic(const State& state, const State &goal) {
            const float dx = goal[0] - state[0];
            const float dy = goal[1] - state[1];
            return sqrtf(dx*dx + dy*dy);
        }

        // Determine if node is valid
        bool is_valid(const State& state) {
            
            // Bound check
            if (state[0] - bounds[0][0] < BODY_RADIUS ||
                state[1] - bounds[0][1] < BODY_RADIUS ||
                bounds[1][0] - state[0] < BODY_RADIUS ||
                bounds[1][1] - state[1] < BODY_RADIUS)
                return false;

            // Obstacle collision check
            for (int o = 0; o < NUM_OBSTACLES; o++) {
                const float dx = state[0] - obstacles[o][0];
                const float dy = state[1] - obstacles[o][1];
                const float threshold = obstacles[o][2] + BODY_RADIUS;
                if (dx*dx + dy*dy < threshold*threshold)
                    return false;
            }

            return true;
        }

        // Determine if state is goal
        bool is_goal(const State& state, const State& goal, const float goal_threshold) {
            return heuristic(state, goal) <= goal_threshold;
        }

    };

}

#endif