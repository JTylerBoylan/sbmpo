#ifndef SBMPO_DOUBLE_INTEGRATOR_MODEL_HPP
#define SBMPO_DOUBLE_INTEGRATOR_MODEL_HPP

#include <sbmpo_models/benchmark_model.hpp>

#define M_2PI 6.283185307179586

namespace sbmpo_models {

using namespace sbmpo;

class DoubleIntegratorModel : public BenchmarkModel {

    const float GOAL_THRESHOLD_X = 0.05f;
    const float GOAL_THRESHOLD_V = 0.05f;
    const float ACC_MIN = -1.0f;
    const float ACC_MAX = 1.0f;

    public:

    State start_state = {0.0f, 0.0f};
    State goal_state = {10.0f, 0.0f};

    DoubleIntegratorModel() {}

    State initial_state() {
        return start_state;
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

        float r = state[0] - goal_state[0];
        float vi = state[1];

        float b,c;
        if (r + 0.5f* vi*std::abs(vi) / ACC_MAX < 0) {
            b = 2.0f * vi / ACC_MIN;
            c = (vi*vi + 2.0f*(ACC_MAX - ACC_MIN) * r) / (ACC_MAX * ACC_MIN);
        } else if (r - 0.5f*vi*std::abs(vi) / ACC_MIN > 0) {
            b = 2.0f * vi / ACC_MAX;
            c = (vi*vi - 2.0f*(ACC_MAX - ACC_MIN) * r) / (ACC_MAX * ACC_MIN);
        }

        return 0.5f * (-b + sqrtf(b*b - 4*c));
    }

    // Determine if node is valid
    bool is_valid(const State& state) {
        return true;
    }

    // Determine if state is goal
    bool is_goal(const State& state) {
        return abs(goal_state[0] - state[0]) <= GOAL_THRESHOLD_X
            && abs(goal_state[1] - state[1]) <= GOAL_THRESHOLD_V;
    }

};

}

#endif