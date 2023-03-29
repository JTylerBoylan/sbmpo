#ifndef SBMPO_UNICYCLE_STEERING_MODEL_HPP
#define SBMPO_UNICYCLE_STEERING_MODEL_HPP

#include <sbmpo/model.hpp>

#define M_2PI 6.283185307179586

namespace sbmpo_models {

using namespace sbmpo;

class UnicycleSteeringModel : public Model {

    public:

    enum States {X, Y, Q};
    enum Controls {V, U};

    UnicycleSteeringModel() {
        goal_threshold_ = 0.25f;
        integration_steps_ = 5;
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
        if (next_state[Q] > M_PI)         next_state[Q] -= M_2PI;
        else if (next_state[Q] <= -M_PI)  next_state[Q] += M_2PI;

        return next_state;
    }

    // Get the cost of a control
    virtual float cost(const State &state, const Control& control, const float time_span) {
        return std::abs(control[V])*time_span;
    }

    // Get the heuristic of a state
    virtual float heuristic(const State& state, const State& goal) {
        const float dx = goal[X] - state[X];
        const float dy = goal[Y] - state[Y];
        const float dq = abs(atan2f(dy,dx) - state[Q]);
        return sqrtf(dx*dx + dy*dy) + std::abs(dq < M_PI ? dq : M_2PI - dq);
    }

    // Determine if node is valid
    virtual bool is_valid(const State& state) {
        return true;
    }

    // Determine if state is goal
    virtual bool is_goal(const State& state, const State& goal) {
        return heuristic(state, goal) <= goal_threshold_;
    }

    virtual ~UnicycleSteeringModel() {}

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

    float goal_threshold_;
    int integration_steps_;

};

}

#endif