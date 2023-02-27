#ifndef SBMPO_GRID_2D_MODEL_HPP
#define SBMPO_GRID_2D_MODEL_HPP

#include <sbmpo_models/benchmark_model.hpp>

#define M_2PI 6.283185307179586

namespace sbmpo_models {

using namespace sbmpo;

class Grid2DModel : public BenchmarkModel {

    const float BODY_RADIUS = 0.20f;
    const float GOAL_THRESHOLD = 0.25f;

    const std::vector<std::vector<float>> BOUNDS = {
        {-5.0, -5.0},
        {5.0, 5.0}  
    };

    public:

    State start_state = {-3.0f, -3.0f};
    State goal_state = {3.0f, 3.0f};

    Grid2DModel() : BenchmarkModel() {}

    State initial_state() {
        return start_state;
    }

    // Evaluate a node with a control
    State next_state(const State &state, const Control& control, const float time_span) {
        
        // Update state
        State next = state;
        next[0] += control[0] * time_span;
        next[1] += control[1] * time_span;
        return next;
        
    }

    // Get the cost of a control
    float cost(const State& state, const Control& control, const float time_span) {
        return sqrtf(control[0]*control[0] + control[1]*control[1]) * time_span;
    }

    // Get the heuristic of a state
    float heuristic(const State& state) {
        const float dx = goal_state[0] - state[0];
        const float dy = goal_state[1] - state[1];
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
        for (int o = 0; o < obstacles.size(); o++) {
            const float dx = state[0] - obstacles[o][0];
            const float dy = state[1] - obstacles[o][1];
            const float threshold = obstacles[o][2] + BODY_RADIUS;
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