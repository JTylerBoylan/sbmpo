#ifndef SBMPO_SIMPLE_STEERING_MODEL_HPP
#define SBMPO_SIMPLE_STEERING_MODEL_HPP

#include <sbmpo/model.hpp>

#define M_2PI 6.283185307179586

namespace sbmpo_models {

using namespace sbmpo;

class SimpleRobotModel : public Model {

    public:

    SimpleRobotModel() {
        this->start_state_ = {0.0f, 0.0f, 0.0f};
        this->goal_state_ = {5.0f, 5.0f};
        this->body_radius_ = 0.25f;
        this->goal_threshold_ = 0.25f;
        this->integration_steps_ = 5;
    }

    virtual State initial_state() {
        return start_state_;
    }

    // Evaluate a node with a control
    virtual State next_state(const State &state, const Control& control, const float time_span) {
        
        // Update state
        State next = state;
        float time_increment = time_span / integration_steps_;
        for (int i = 0; i < integration_steps_; i++) {
            next[0] += cosf(next[2]) * control[0] * time_increment;
            next[1] += sinf(next[2]) * control[0] * time_increment;
            next[2] += control[1] * time_increment;
        }

        // Angle wrap
        if (next[2] >= M_2PI || next[2] < 0)
            next[2] += next[2] >= M_2PI ? -M_2PI : M_2PI;

        return next;
    }

    // Get the cost of a control
    virtual float cost(const State &state, const Control& control, const float time_span) {
        return control[0]*time_span;
    }

    // Get the heuristic of a state
    virtual float heuristic(const State& state) {
        const float dx = goal_state_[0] - state[0];
        const float dy = goal_state_[1] - state[1];
        return sqrtf(dx*dx + dy*dy);
    }

    // Determine if node is valid
    virtual bool is_valid(const State& state) {
        return true;
    }

    // Determine if state is goal
    virtual bool is_goal(const State& state) {
        return heuristic(state) <= goal_threshold_;
    }

    virtual ~SimpleRobotModel() {}

    void set_start_state(State start_state) {
        this->start_state_ = start_state;
    }

    void set_goal_state(State goal_state) {
        this->goal_state_ = goal_state;
    }

    void set_body_radius(float body_radius) {
        this->body_radius_ = body_radius;
    }

    void set_goal_threshold(float goal_threshold) {
        this->goal_threshold_ = goal_threshold;
    }

    void set_integration_steps(int integration_steps) {
        this->integration_steps_ = integration_steps;
    }

    protected:

    State start_state_;
    State goal_state_;

    float body_radius_ = 0.25f;
    float goal_threshold_ = 0.25f;
    int integration_steps_ = 5;

};

}

#endif