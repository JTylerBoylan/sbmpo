#ifndef SBMPO_DOUBLE_INTEGRATOR_MODEL_HPP
#define SBMPO_DOUBLE_INTEGRATOR_MODEL_HPP

#include <sbmpo/model.hpp>

namespace sbmpo_models {

using namespace sbmpo;

class DoubleIntegratorModel : public Model {

    public:

    enum States {X, V};
    enum Controls {A};

    DoubleIntegratorModel() {
        threshold_x_ = 0.01f;
        threshold_v_ = 0.01f;
        min_acc_ = -1.0f;
        max_acc_ = 1.0f;
    }

    // Evaluate a node with a control
    virtual State next_state(const State &state, const Control& control, const float time_span) {
        
        // Update state
        State next_state = state;
        next_state[X] += state[V] * time_span;
        next_state[V] += control[A] * time_span;
        return next_state;

    }

    // Get the cost of a control
    virtual float cost(const State& state, const Control& control, const float time_span) {
        return time_span;
    }

    // Get the heuristic of a state
    virtual float heuristic(const State& state, const State& goal) {

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

    // Determine if node is valid
    virtual bool is_valid(const State& state) {
        return true;
    }

    // Determine if state is goal
    virtual bool is_goal(const State& state, const State& goal) {
        return std::abs(goal[X] - state[X]) <= threshold_x_
            && std::abs(goal[V] - state[V]) <= threshold_v_;
    }

    virtual ~DoubleIntegratorModel() {}

    /// @brief Set the goal threshold X value
    /// @param goal_threshold_x Value to set as goal threshold X
    void set_goal_threshold_x(float goal_threshold_x) {
        threshold_x_ = goal_threshold_x;
    }

    /// @brief Set the goal threshold V value
    /// @param goal_threshold_v Value to set as goal threshold V
    void set_goal_threshold_v(float goal_threshold_v) {
        threshold_v_ = goal_threshold_v;
    }

    /// @brief Set the minimum acceleration value
    /// @param min_acceleration Value to set as the minimum acceleration
    void set_min_acceleration(float min_acceleration) {
        min_acc_ = min_acceleration;
    }

    /// @brief Set the maximum acceleration value
    /// @param max_acceleration Value to set as the maximum acceleration
    void set_max_acceleration(float max_acceleration) {
        max_acc_ = max_acceleration;
    }

    protected:

    float threshold_x_;
    float threshold_v_;
    float min_acc_;
    float max_acc_;


};

}

#endif