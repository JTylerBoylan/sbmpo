#ifndef SBMPO_GRID_2D_MODEL_HPP
#define SBMPO_GRID_2D_MODEL_HPP

#include <sbmpo/model.hpp>

#define M_2PI 6.283185307179586

namespace sbmpo_models {

using namespace sbmpo;

class Grid2DModel : public Model {

    public:

    Grid2DModel() {
        this->start_state_ = {0.0f, 0.0f};
        this->goal_state_ = {5.0f, 5.0f};
        this->goal_threshold_ = 0.25f;
    }

    virtual State initial_state() {
        return start_state_;
    }

    // Evaluate a node with a control
    virtual State next_state(const State &state, const Control& control, const float time_span) {
        
        // Update state
        State next = state;
        next[0] += control[0] * time_span;
        next[1] += control[1] * time_span;
        return next;
        
    }

    // Get the cost of a control
    virtual float cost(const State& state, const Control& control, const float time_span) {
        return sqrtf(control[0]*control[0] + control[1]*control[1]) * time_span;
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

    virtual ~Grid2DModel() {}

    void set_start_state(State start_state) {
        this->start_state_ = start_state;
    }

    void set_goal_state(State goal_state) {
        this->goal_state_ = goal_state;
    }

    void set_goal_threshold(float goal_threshold) {
        this->goal_threshold_ = goal_threshold;
    }

    protected:

    State start_state_;
    State goal_state_;

    float goal_threshold_;

};

}

#endif