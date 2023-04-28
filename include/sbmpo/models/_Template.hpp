#ifndef SBMPO_TEMPLATE_MODEL_HPP
#define SBMPO_TEMPLATE_MODEL_HPP

#include <sbmpo/model.hpp>

namespace sbmpo_models {

using namespace sbmpo;

class MyCustomModel : public Model {

    public:

    // States of the Model
    enum States {X, Y, Z, R, P, Y};

    // Controls of the Model
    enum Controls {U1, U2, U3};

    // Constructor
    MyCustomModel() {
        parameter1_ = 0.0f;
        parameter2_ = std::numeric_limits<float>::infinity();
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

    // Deconstructor
    virtual ~MyCustomModel() {}

    /// @brief Set the parameter value
    /// @param parameter1 Parameter value
    void set_parameter1(float parameter1) {
        parameter1_ = parameter1;
    }

    /// @brief Set the parameter value
    /// @param parameter2 Parameter value
    void set_parameter2(float parameter2) {
        parameter2_ = parameter2;
    }

    protected:

    float parameter1_;
    float parameter2_;


};

}

#endif