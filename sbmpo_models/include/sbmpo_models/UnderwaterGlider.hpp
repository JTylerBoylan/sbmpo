#ifndef SBMPO_UNDERWATER_GLIDER_MODEL_HPP
#define SBMPO_UNDERWATER_GLIDER_MODEL_HPP

#include <sbmpo/model.hpp>

namespace sbmpo_models {

using namespace sbmpo;

class UnderwaterGliderModel : public Model {

    public:

    enum States {X, Y, Q, dXdt, dYdt, dQdt, M};
    enum Controls {dMdt};

    UnderwaterGliderModel() {
        max_depth_ = 10.0f;
    }

    // Evaluate a node with a control
    virtual State next_state(const State &state, const Control& control, const float time_span) {
        
        State next_state = state;

        /*
            Dynamics of the system
            How does each state change with respect to the controls?
        */

       return next_state;

    }

    // Get the cost of a control
    virtual float cost(const State& state, const Control& control, const float time_span) {

        /*
            Cost of a state and control
            What am I trying to minimize?
            i.e Distance, Time, Energy
        */

        return 0.0f;
    }

    // Get the heuristic of a state
    virtual float heuristic(const State& state, const State& goal) {

        /*
            Heuristic of a state with respect to the goal
            Leads the planner to the goal
            What is the lowest cost possible from this state to the goal?
        */

        return 0.0f;
    }

    // Determine if node is valid
    virtual bool is_valid(const State& state) {

        /*
            Does this state meet the model constraints?
            i.e Boundary constraints, Obstacles, State limits
        */

        return true;
    }

    // Determine if state is goal
    virtual bool is_goal(const State& state, const State& goal) {

        /*
            Is this state close enough to the goal to end the plan?
        */
       
        return true;
    }

    virtual ~UnderwaterGliderModel() {}

    /// @brief Set the max depth value
    /// @param max_depth Max depth value
    void set_max_depth(float max_depth) {
        max_depth_ = max_depth;
    }

    protected:

    float max_depth_;


};

}

#endif