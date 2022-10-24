#ifndef SBMPO_HPP
#define SBMPO_HPP

#include <sbmpo/model.hpp>
#include <ctime>

namespace sbmpo {

    // Run the planner on a model
    void run(Model& model, const PlannerParameters &parameters, PlannerResults& results);

}

#endif