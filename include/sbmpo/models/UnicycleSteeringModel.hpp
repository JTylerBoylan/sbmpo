#ifndef SBMPO_MODEL_UNICYCLE_STEERING_HPP_
#define SBMPO_MODEL_UNICYCLE_STEERING_HPP_

#include <sbmpo/types/Model.hpp>

#define M_2PI 6.283185307179586F

namespace sbmpo_models {

using namespace sbmpo;

class UnicycleSteeringModel : public Model {

public:

    enum States {X, Y, Q};
    enum Controls {V, U};

    UnicycleSteeringModel() {
        goal_threshold_ = 0.25;
        integration_steps_ = 5;
    }

    State next_state(const State& state, const Control& control, const float time_span) override;

    float cost(const State& state, const Control& control, const float time_span) override;

    float heuristic(const State& state, const State& goal) override;

    bool is_goal(const State& state, const State& goal) override;

    bool is_valid(const State& state) override;

    void set_goal_threshold(float goal_threshold) {
        goal_threshold_ = goal_threshold;
    }

    void set_integration_steps(int integration_steps) {
        integration_steps_ = integration_steps;
    }

protected:

    float goal_threshold_;
    int integration_steps_;

};

}

#endif