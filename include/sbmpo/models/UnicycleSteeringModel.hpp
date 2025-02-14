#ifndef SBMPO_MODEL_UNICYCLE_STEERING_HPP_
#define SBMPO_MODEL_UNICYCLE_STEERING_HPP_

#include <sbmpo/types/Model.hpp>

#define M_2PI 6.283185307179586F

namespace sbmpo_models
{
    using namespace sbmpo;

    class UnicycleSteeringModel : public Model
    {

    public:
        enum States
        {
            X,
            Y,
            Q
        };
        enum Controls
        {
            V,
            U
        };

        UnicycleSteeringModel()
        {
            horizon_time_ = 1.0f;
            goal_threshold_ = 0.25;
            integration_steps_ = 5;
        }

        State next_state(const State &state, const Control &control) override
        {
            // Update state
            State next_state = state;
            float time_increment = horizon_time_ / integration_steps_;
            for (int i = 0; i < integration_steps_; i++)
            {
                next_state[X] += std::cos(next_state[Q]) * control[V] * time_increment;
                next_state[Y] += std::sin(next_state[Q]) * control[V] * time_increment;
                next_state[Q] += control[U] * time_increment;
            }
            // Angle wrap
            if (next_state[Q] > M_PI)
                next_state[Q] -= M_2PI;
            else if (next_state[Q] <= -M_PI)
                next_state[Q] += M_2PI;

            return next_state;
        }

        float cost(const State &state1, const State &state2, const Control &control) override
        {
            const float dx = state2[X] - state1[X];
            const float dy = state2[Y] - state1[Y];
            return std::sqrt(dx * dx + dy * dy);
        }

        float heuristic(const State &state, const State &goal) override
        {
            const float dx = goal[X] - state[X];
            const float dy = goal[Y] - state[Y];
            const float dq = std::abs(atan2f(dy, dx) - state[Q]);
            return sqrtf(dx * dx + dy * dy) + std::abs(dq < M_PI ? dq : M_2PI - dq);
        }

        bool is_goal(const State &state, const State &goal) override
        {
            return heuristic(state, goal) <= goal_threshold_;
        }

        bool is_valid(const State &state) override
        {
            return true;
        }

        void set_horizon_time(float time_span)
        {
            horizon_time_ = time_span;
        }

        void set_goal_threshold(float goal_threshold)
        {
            goal_threshold_ = goal_threshold;
        }

        void set_integration_steps(int integration_steps)
        {
            integration_steps_ = integration_steps;
        }

    protected:
        float horizon_time_;
        float goal_threshold_;
        int integration_steps_;
    };

}

#endif