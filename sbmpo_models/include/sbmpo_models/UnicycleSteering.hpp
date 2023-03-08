#ifndef SBMPO_SIMPLE_STEERING_MODEL_HPP
#define SBMPO_SIMPLE_STEERING_MODEL_HPP

#include <sbmpo/model.hpp>

#define M_2PI 6.283185307179586

namespace sbmpo_models {

using namespace sbmpo;

class UnicycleSteeringModel : public Model {

    public:

    enum States {X, Y, Q};
    enum Controls {V, U};

    UnicycleSteeringModel() {
        start_state_ = {0.0f, 0.0f, 0.0f};
        goal_state_ = {5.0f, 5.0f};
        goal_threshold_ = 0.25f;
        integration_steps_ = 5;
    }

    // Get the initial state
    virtual State initial_state() {
        return start_state_;
    }

    // Evaluate a node with a control
    virtual State next_state(const State &state, const Control& control, const float time_span) {
        
        // Update state
        State next_state = state;
        float time_increment = time_span / integration_steps_;
        for (int i = 0; i < integration_steps_; i++) {
            next_state[X] += cosf(next_state[Q]) * control[V] * time_increment;
            next_state[Y] += sinf(next_state[Q]) * control[V] * time_increment;
            next_state[Q] += control[U] * time_increment;
        }

        // Angle wrap
        while (next_state[Q] >= M_2PI)  next_state[Q] -= M_2PI;
        while (next_state[Q] < 0)       next_state[Q] += M_2PI;

        return next_state;
    }

    // Get the cost of a control
    virtual float cost(const State &state, const Control& control, const float time_span) {
        return control[V]*time_span;
    }

    // Get the heuristic of a state
    virtual float heuristic(const State& state) {
        const float dx = goal_state_[X] - state[X];
        const float dy = goal_state_[Y] - state[Y];
        const float dq = atan2f(dy,dx);
        return sqrtf(dx*dx + dy*dy) + std::abs(dq);
    }

    // Determine if node is valid
    virtual bool is_valid(const State& state) {
        return true;
    }

    // Determine if state is goal
    virtual bool is_goal(const State& state) {
        return heuristic(state) <= goal_threshold_;
    }

    virtual ~UnicycleSteeringModel() {}

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

    protected:

    State start_state_;
    State goal_state_;

    float goal_threshold_;
    int integration_steps_;

};

}

#endif