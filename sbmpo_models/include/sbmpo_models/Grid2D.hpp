#ifndef SBMPO_GRID_2D_MODEL_HPP
#define SBMPO_GRID_2D_MODEL_HPP

#include <sbmpo/model.hpp>

namespace sbmpo_models {

using namespace sbmpo;

class Grid2DModel : public Model {

    public:

    enum States {X, Y};
    enum Controls {Vx, Vy};

    Grid2DModel() {
        goal_threshold_ = 0.25f;
    }

    // Evaluate a node with a control
    virtual State next_state(const State &state, const Control& control, const float time_span) {
        
        // Update state
        State next_state = state;
        next_state[X] += control[Vx] * time_span;
        next_state[Y] += control[Vy] * time_span;
        return next_state;
        
    }

    // Get the cost of a control
    virtual float cost(const State& state, const Control& control, const float time_span) {
        return sqrtf(control[Vx]*control[Vx] + control[Vy]*control[Vy]) * time_span;
    }

    // Get the heuristic of a state
    virtual float heuristic(const State& state, const State& goal) {
        const float dx = goal[X] - state[X];
        const float dy = goal[Y] - state[Y];
        return sqrtf(dx*dx + dy*dy);
    }

    // Determine if node is valid
    virtual bool is_valid(const State& state) {
        return true;
    }

    // Determine if state is goal
    virtual bool is_goal(const State& state, const State& goal) {
        return heuristic(state, goal) <= goal_threshold_;
    }

    virtual ~Grid2DModel() {}

    /// @brief Set the goal threshold value
    /// @param goal_threshold Value to set as goal threshold
    void set_goal_threshold(float goal_threshold) {
        goal_threshold_ = goal_threshold;
    }

    protected:

    float goal_threshold_;

};

}

#endif