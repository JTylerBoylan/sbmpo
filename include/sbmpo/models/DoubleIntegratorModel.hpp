#ifndef SBMPO_MODEL_DOUBLE_INTEGRATOR_HPP_
#define SBMPO_MODEL_DOUBLE_INTEGRATOR_HPP_

#include <sbmpo/types/types.hpp>
#include <sbmpo/types/Model.hpp>

namespace sbmpo_models {

using namespace sbmpo;

class DoubleIntegratorModel : public Model {

    public:

    enum States {X, V};
    enum Controls {A};

    DoubleIntegratorModel() {
        threshold_x_ = 0.05f;
        threshold_v_ = 0.05f;
        min_acc_ = -1.0f;
        max_acc_ = 1.0f;
        integration_steps_ = 10;
    }

    virtual State next_state(const State &state, const Control& control, const float time_span);

    virtual float cost(const State& state, const Control& control, const float time_span);

    virtual float heuristic(const State& state, const State& goal);

    virtual bool is_valid(const State& state);

    virtual bool is_goal(const State& state, const State& goal);

    void set_integration_steps(size_t integration_steps) {
        integration_steps_ = integration_steps;
    }

    void set_goal_threshold_x(float goal_threshold_x) {
        threshold_x_ = goal_threshold_x;
    }

    void set_goal_threshold_v(float goal_threshold_v) {
        threshold_v_ = goal_threshold_v;
    }

    void set_min_acceleration(float min_acceleration) {
        min_acc_ = min_acceleration;
    }

    void set_max_acceleration(float max_acceleration) {
        max_acc_ = max_acceleration;
    }

    protected:

    int integration_steps_;
    float threshold_x_;
    float threshold_v_;
    float min_acc_;
    float max_acc_;


};

}

#endif