#ifndef SBMPO_MODEL_HPP_
#define SBMPO_MODEL_HPP_

#include <math.h>
#include <sbmpo/types/types.hpp>

namespace sbmpo
{
    class Model
    {
    public:
        /*
            Dynamics of the system
            How does each state change with respect to the controls?
        */
        virtual State next_state(const State &state, const Control &control) = 0;

        /*
            Cost of a state and control
            What am I trying to minimize?
            i.e Distance, Time, Energy
        */
        virtual float cost(const State &state1, const State &state2, const Control &control) = 0;

        /*
            Heuristic of a state with respect to the goal
            Leads the planner to the goal
            What is the lowest cost possible from this state to the goal?
        */
        virtual float heuristic(const State &state, const State &goal) = 0;

        /*
            Is this state close enough to the goal to end the plan?
        */
        virtual bool is_goal(const State &state, const State &goal) = 0;

        /*
            Does this state meet the model constraints?
            i.e Boundary constraints, Obstacles, State limits
        */
        virtual bool is_valid(const State &state) = 0;

        /*
            Get control samples based on the current state
            Enabled using SearchParameters.sample_type = DYNAMIC
        */
        virtual std::vector<Control> get_dynamic_samples(const State &state)
        {
            return {};
        };
    };

}

#endif