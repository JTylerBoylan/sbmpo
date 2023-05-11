#include <sbmpo/models/AckermannSteeringModel.hpp>
#include <math.h>

namespace sbmpo_models {

using namespace sbmpo;

State AckermannSteeringModel::next_state(const State& state, const Control& control, const float time_span) {
    // Integrate control into state (Euler)
    State next_state = state;
    const float time_increment = time_span / integration_steps_;
    for (int i = 0; i < integration_steps_; i++) {
        next_state[X] += cosf(next_state[Q]) * next_state[V] * time_increment;
        next_state[Y] += sinf(next_state[Q]) * next_state[V] * time_increment;
        next_state[Q] += tanf(next_state[G]) * next_state[V] * time_increment * inv_wheel_base_length_;
        next_state[V] += control[dVdt] * time_increment;
        next_state[G] += control[dGdt] * time_increment;
    }

    // Angle wrap
    if (state[Q] > M_PI)         next_state[Q] -= M_2PI;
    else if (state[Q] <= -M_PI)  next_state[Q] += M_2PI;

    return next_state;
}

float AckermannSteeringModel::cost(const State& state, const Control& control, const float time_span) {
    return time_span;
}

float AckermannSteeringModel::heuristic(const State& state, const State& goal) {
    const float dx = goal[X] - state[X];
    const float dy = goal[Y] - state[Y];
    const float dq = abs(atan2f(dy,dx) - state[Q]);
    return sqrtf(dx*dx + dy*dy)/max_velocity_ + std::abs(dq < M_PI ? dq : M_2PI - dq)/(max_velocity_*max_turn_angle_*inv_wheel_base_length_);
}

bool AckermannSteeringModel::is_goal(const State& state, const State& goal) {
    return heuristic(state, goal) <= goal_threshold_;
}

bool AckermannSteeringModel::is_valid(const State& state) {
    const bool valid =   
        state[V] <= max_velocity_ && 
        state[V] >= min_velocity_ &&
        state[G] <= max_turn_angle_ && 
        state[G] >= min_turn_angle_ &&
        std::abs(state[V]*state[V]*inv_wheel_base_length_*state[G]) <= max_centrifugal_;
    return valid;
}

}