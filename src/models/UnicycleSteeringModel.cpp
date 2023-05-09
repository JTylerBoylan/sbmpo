#include <sbmpo/models/UnicycleSteeringModel.hpp>
#include <math.h>

namespace sbmpo_models {

using namespace sbmpo;

State UnicycleSteeringModel::next_state(const State& state, const Control& control, const float time_span) {
    // Update state
    State next_state = state;
    float time_increment = time_span / integration_steps_;
    for (int i = 0; i < integration_steps_; i++) {
        next_state[X] += cosf(next_state[Q]) * control[V] * time_increment;
        next_state[Y] += sinf(next_state[Q]) * control[V] * time_increment;
        next_state[Q] += control[U] * time_increment;
    }
    // Angle wrap
    if (next_state[Q] > M_PI)         next_state[Q] -= M_2PI;
    else if (next_state[Q] <= -M_PI)  next_state[Q] += M_2PI;

    return next_state;
}

float UnicycleSteeringModel::cost(const State& state, const Control& control, const float time_span) {
    return std::abs(control[V])*time_span;
}

float UnicycleSteeringModel::heuristic(const State& state, const State& goal) {
    const float dx = goal[X] - state[X];
    const float dy = goal[Y] - state[Y];
    const float dq = abs(atan2f(dy,dx) - state[Q]);
    return sqrtf(dx*dx + dy*dy) + std::abs(dq < M_PI ? dq : M_2PI - dq);
}

bool UnicycleSteeringModel::is_goal(const State& state, const State& goal) {
    return heuristic(state, goal) <= goal_threshold_;
}

bool UnicycleSteeringModel::is_valid(const State& state) {
    return true;
}

}