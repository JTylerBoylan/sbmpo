#include <sbmpo/models/DoubleIntegratorModel.hpp>
#include <math.h>

namespace sbmpo_models {

using namespace sbmpo;

State DoubleIntegratorModel::next_state(const State& state, const Control& control, const float time_span) {

    // Update state
    State next_state = state;
    float time_step = time_span / integration_steps_;
    for (int i = 0; i < integration_steps_; i++) {
        next_state[X] += state[V] * time_step;
        next_state[V] += control[A] * time_step;
    }
    return next_state;

}

float DoubleIntegratorModel::cost(const State& state, const Control& control, const float time_span) {
    return time_span;
}

float DoubleIntegratorModel::heuristic(const State& state, const State& goal) {

    /*
    *   Time Optimal Heuristic
    */

    float q_10 = state[X] - goal[X];
    float q_20 = state[V] - goal[V];

    float b = 0, c = 0;
    if (q_10 + 0.5f*q_20*std::abs(q_20) / max_acc_ >= 0) {
        b = 2.0f * q_20 / min_acc_;
        c = (q_20*q_20 + 2.0f*(max_acc_ - min_acc_)*q_10) / (max_acc_ * min_acc_);
    } else if (q_10 - 0.5f*q_20*std::abs(q_20) / min_acc_ < 0) {
        b = 2.0f * q_20 / max_acc_;
        c = (q_20*q_20 - 2.0f*(max_acc_ - min_acc_)*q_10) / (max_acc_ * min_acc_);
    }

    return 0.5f * (-b + sqrtf(b*b - 4*c));
}

bool DoubleIntegratorModel::is_goal(const State& state, const State& goal) {
    return std::abs(goal[X] - state[X]) <= threshold_x_
        && std::abs(goal[V] - state[V]) <= threshold_v_;
}

bool DoubleIntegratorModel::is_valid(const State& state) {
    return true;
}

}