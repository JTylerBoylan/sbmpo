#ifndef SBMPO_MODEL_ACKERMANN_STEERING_HPP_
#define SBMPO_MODEL_ACKERMANN_STEERING_HPP_

#include <sbmpo/types/Model.hpp>

#define M_2PI 6.283185307179586F

namespace sbmpo_models {

using namespace sbmpo;

class AckermannSteeringModel : public Model {

public:

    enum States {X, Y, Q, V, G};
    enum Controls {dVdt, dGdt};

    AckermannSteeringModel() {
        goal_threshold_ = 0.25f;
        integration_steps_ = 5;

        inv_wheel_base_length_ = 1.0f;

        max_velocity_ = 5.0f; // m/s
        min_velocity_ = 0.0f; // m/s
        max_turn_angle_ = M_2PI / 12.0f; // rad
        min_turn_angle_ = -M_2PI / 12.0f; // rad
        max_centrifugal_ = 10.0f; // m/s^2
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

    void set_wheel_base_length(float wheel_base_length) {
        inv_wheel_base_length_ = 1.0f / wheel_base_length;
    }

    void set_velocity_bounds(float min_velocity, float max_velocity) {
        min_velocity_ = min_velocity;
        max_velocity_ = max_velocity;
    }

    void set_turn_angle_bounds(float min_turn_angle, float max_turn_angle) {
        min_turn_angle_ = min_turn_angle;
        max_turn_angle_ = max_turn_angle;
    }

    void set_max_centrifugal(float max_centrifugal) {
        max_centrifugal_ = max_centrifugal;
    }

protected:

    float goal_threshold_;
    int integration_steps_;
    float inv_wheel_base_length_;
    float max_velocity_, min_velocity_;
    float max_turn_angle_, min_turn_angle_;
    float max_centrifugal_;

};

}

#endif