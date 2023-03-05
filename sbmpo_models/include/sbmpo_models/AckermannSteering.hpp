#ifndef SBMPO_ACKERMANN_STEERING_MODEL_HPP
#define SBMPO_ACKERMANN_STEERING_MODEL_HPP

#include <sbmpo/model.hpp>

#define M_2PI 6.283185307179586

namespace sbmpo_models {

using namespace sbmpo;

class AckermannSteeringModel : public Model {

    public:

    enum States {X, Y, Q, V, G};
    enum Controls {dVdt, dGdt};

    AckermannSteeringModel() {

        start_state_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        goal_state_ = {5.0f, 5.0f, 0.0f, 0.0f, 0.0f};

        goal_threshold_ = 0.5f;
        integration_steps_ = 5;

        inv_wheel_base_length_ = 1.0f;

        max_velocity_ = 10.0f; // m/s
        min_velocity_ = -2.0f; // m/s
        max_turn_angle_ = M_PIf / 6.0f; // rad
        min_turn_angle_ = -M_PIf / 6.0f; // rad
        max_centrifugal_ = 10.0f; // m/s^2
    }

    // Return initial state
    virtual State initial_state() { return start_state_; }

    // Evaluate a node with a control
    virtual void next_state(State& state, const Control& control, const float time_span) {

        // Integrate control into state (Euler)
        float time_increment = time_span / integration_steps_;
        for (int i = 0; i < integration_steps_; i++) {
            state[X] += cosf(state[Q]) * state[V] * time_increment;
            state[Y] += sinf(state[Q]) * state[V] * time_increment;
            state[Q] += state[G] * state[V] * time_increment * inv_wheel_base_length_;
            state[V] += control[dVdt] * time_increment;
            state[G] += control[dGdt] * time_increment;
            if (!is_valid(state))
                return;
        }

        // Angle wrap
        while (state[Q] >= M_2PI)  state[Q] -= M_2PI;
        while (state[Q] < 0)       state[Q] += M_2PI;

    }

    // Get the cost of a control
    virtual float cost(const State& state, const Control& control, const float time_span) {
        float cost_time = time_span;
        return cost_time;
    }

    // Get the heuristic of a state
    virtual float heuristic(const State& state) {
        float dx = (goal_state_[X] - state[X]);
        float dy = (goal_state_[Y] - state[Y]);
        float dq = (goal_state_[Q] - state[Q]);
        float dv = (goal_state_[V] - state[V]);
        float dg = (goal_state_[G] - state[G]);
        return sqrt(dx*dx + dy*dy + dq*dq + dv*dv + dg*dg);
    }

    // Determine if state is goal
    virtual bool is_goal(const State& state) {
        return heuristic(state) <= goal_threshold_;
    }

    // Determine if state is valid
    virtual bool is_valid(const State& state) {
        return  std::abs(state[V]*state[V]*inv_wheel_base_length_*state[G]) <= max_centrifugal_;
    }

    virtual ~AckermannSteeringModel() {}

    /// @brief Set the start state of the model
    /// @param start_state State to set as start
    void set_start_state(State start_state) {
        start_state_ = start_state;
    }

    /// @brief Set the goal state of the model
    /// @param goal_state State to set as goal
    void set_goal_state(State goal_state) {
        goal_state_ = goal_state;
    }

    /// @brief Set the goal threshold value
    /// @param goal_threshold Value to set as goal threshold
    void set_goal_threshold(float goal_threshold) {
        goal_threshold_ = goal_threshold;
    }

    /// @brief Set the number of integration steps (Euler)
    /// @param integration_steps Number of integration steps
    void set_integration_steps(int integration_steps) {
        integration_steps_ = integration_steps;
    }

    /// @brief Set the wheel base length of the vehicle
    /// @param wheel_base_length Value to set as wheel base length
    void set_wheel_base_length(float wheel_base_length) {
        inv_wheel_base_length_ = 1.0f / wheel_base_length;
    }

    /// @brief Set the velocity limits
    /// @param min_velocity Value of minimum velocity
    /// @param max_velocity Value of maximim velocity
    void set_velocity_bounds(float min_velocity, float max_velocity) {
        min_velocity_ = min_velocity;
        max_velocity_ = max_velocity;
    }

    /// @brief Set the turn angle limits
    /// @param min_turn_angle Value of minimum turn angle
    /// @param max_turn_angle Value of maximim turn angle
    void set_velocity_bounds(float min_turn_angle, float max_turn_angle) {
        min_turn_angle_ = min_turn_angle;
        max_turn_angle_ = max_turn_angle;
    }

    /// @brief Set the maximum centrifugal acceleration of the car
    /// @param max_centrifugal Value to set as maximum centrigual acceleration
    void set_max_centrifugal(float max_centrifugal) {
        max_centrifugal_ = max_centrifugal;
    }

    private:

        State start_state_;
        State goal_state_;

        float goal_threshold_;
        int integration_steps_;

        float inv_wheel_base_length_;

        float max_velocity_, min_velocity_;
        float max_turn_angle_, min_turn_angle_;
        float max_centrifugal_;


};

}

#endif