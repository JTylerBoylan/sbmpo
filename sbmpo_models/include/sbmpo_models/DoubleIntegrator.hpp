#ifndef SBMPO_DOUBLE_INTEGRATOR_MODEL_HPP
#define SBMPO_DOUBLE_INTEGRATOR_MODEL_HPP

#include <sbmpo/model.hpp>

namespace sbmpo_models {

using namespace sbmpo;

class DoubleIntegratorModel : public Model {

    public:

    DoubleIntegratorModel() {
        this->start_state_ = {0.0f, 0.0f};
        this->goal_state_ = {10.0f, 0.0f};
        this->threshold_x_ = 0.01f;
        this->threshold_v_ = 0.01f;
        this->min_acc_ = -1.0f;
       this-> max_acc_ = 1.0f;
    }

    // Get the initial state
    virtual State initial_state() {
        return start_state_;
    }

    // Evaluate a node with a control
    virtual State next_state(const State &state, const Control& control, const float time_span) {
        
        // Update state
        State next = state;
        next[0] += state[1] * time_span;
        next[1] += control[0] * time_span;
        return next;

    }

    // Get the cost of a control
    virtual float cost(const State& state, const Control& control, const float time_span) {
        return time_span;
    }

    // Get the heuristic of a state
    virtual float heuristic(const State& state) {

        /*
        *   Time Optimal Heuristic
        */

        float q_10 = state[0] - goal_state_[0];
        float q_20 = state[1] - goal_state_[1];

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
    virtual bool is_goal(const State& state) {
        return std::abs(goal_state_[0] - state[0]) <= threshold_x_
            && std::abs(goal_state_[1] - state[1]) <= threshold_v_;
    }

    virtual ~DoubleIntegratorModel() {}
    
    /// @brief Set the start state of the model
    /// @param start_state State to set as start
    void set_start_state(State start_state) {
        this->start_state_ = start_state;
    }

    /// @brief Set the goal state of the model
    /// @param goal_state State to set as goal
    void set_goal_state(State goal_state) {
        this->goal_state_ = goal_state;
    }

    /// @brief Set the goal threshold X value
    /// @param goal_threshold_x Value to set as goal threshold X
    void set_goal_threshold_x(float goal_threshold_x) {
        this->threshold_x_ = goal_threshold_x;
    }

    /// @brief Set the goal threshold V value
    /// @param goal_threshold_v Value to set as goal threshold V
    void set_goal_threshold_v(float goal_threshold_v) {
        this->threshold_v_ = goal_threshold_v;
    }

    /// @brief Set the minimum acceleration value
    /// @param min_acceleration Value to set as the minimum acceleration
    void set_min_acceleration(float min_acceleration) {
        this->min_acc_ = min_acceleration;
    }

    /// @brief Set the maximum acceleration value
    /// @param max_acceleration Value to set as the maximum acceleration
    void set_max_acceleration(float max_acceleration) {
        this->max_acc_ = max_acceleration;
    }

    protected:

    State start_state_;
    State goal_state_;

    float threshold_x_;
    float threshold_v_;
    float min_acc_;
    float max_acc_;


};

}

#endif