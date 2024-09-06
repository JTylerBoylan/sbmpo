#ifndef SBMPO_MODEL_ACKERMANN_STEERING_HPP_
#define SBMPO_MODEL_ACKERMANN_STEERING_HPP_

#include <sbmpo/types/Model.hpp>

#define M_2PI 6.283185307179586F

namespace sbmpo_models
{
    using namespace sbmpo;

    class AckermannSteeringModel : public Model
    {

    public:
        enum States
        {
            X,
            Y,
            Q,
            V,
            G
        };
        enum Controls
        {
            dVdt,
            dGdt
        };

        AckermannSteeringModel()
        {
            goal_threshold_ = 0.25f;
            integration_steps_ = 5;
            inv_wheel_base_length_ = 1.0f;
            max_velocity_ = 5.0f;             // m/s
            min_velocity_ = 0.0f;             // m/s
            max_turn_angle_ = M_2PI / 12.0f;  // rad
            min_turn_angle_ = -M_2PI / 12.0f; // rad
            max_centrifugal_ = 10.0f;         // m/s^2
        }

        State next_state(const State &state, const Control &control, const float time_span) override
        {
            // Integrate control into state (Euler)
            State next_state = state;
            const float time_increment = time_span / integration_steps_;
            for (int i = 0; i < integration_steps_; i++)
            {
                next_state[X] += cosf(next_state[Q]) * next_state[V] * time_increment;
                next_state[Y] += sinf(next_state[Q]) * next_state[V] * time_increment;
                next_state[Q] += tanf(next_state[G]) * next_state[V] * time_increment * inv_wheel_base_length_;
                next_state[V] += control[dVdt] * time_increment;
                next_state[G] += control[dGdt] * time_increment;
            }

            // Angle wrap
            if (state[Q] > M_PI)
                next_state[Q] -= M_2PI;
            else if (state[Q] <= -M_PI)
                next_state[Q] += M_2PI;

            return next_state;
        }

        float cost(const State &state1, const State &state2, const Control &control, const float time_span) override
        {
            return time_span;
        }

        float heuristic(const State &state, const State &goal) override
        {
            const float dx = goal[X] - state[X];
            const float dy = goal[Y] - state[Y];
            const float dq = abs(atan2f(dy, dx) - state[Q]);
            return sqrtf(dx * dx + dy * dy) / max_velocity_ + std::abs(dq < M_PI ? dq : M_2PI - dq) / (max_velocity_ * max_turn_angle_ * inv_wheel_base_length_);
        }

        bool is_goal(const State &state, const State &goal) override
        {
            return heuristic(state, goal) <= goal_threshold_;
        }

        bool is_valid(const State &state) override
        {
            const bool valid =
                state[V] <= max_velocity_ &&
                state[V] >= min_velocity_ &&
                state[G] <= max_turn_angle_ &&
                state[G] >= min_turn_angle_ &&
                std::abs(state[V] * state[V] * inv_wheel_base_length_ * state[G]) <= max_centrifugal_;
            return valid;
        }

        void set_goal_threshold(float goal_threshold)
        {
            goal_threshold_ = goal_threshold;
        }

        void set_integration_steps(int integration_steps)
        {
            integration_steps_ = integration_steps;
        }

        void set_wheel_base_length(float wheel_base_length)
        {
            inv_wheel_base_length_ = 1.0f / wheel_base_length;
        }

        void set_velocity_bounds(float min_velocity, float max_velocity)
        {
            min_velocity_ = min_velocity;
            max_velocity_ = max_velocity;
        }

        void set_turn_angle_bounds(float min_turn_angle, float max_turn_angle)
        {
            min_turn_angle_ = min_turn_angle;
            max_turn_angle_ = max_turn_angle;
        }

        void set_max_centrifugal(float max_centrifugal)
        {
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