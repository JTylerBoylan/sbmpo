#include <sbmpo/SBMPO.hpp>
#include <sbmpo/algorithms/Astar.hpp>
#include <sbmpo/models/UnicycleSteeringModel.hpp>
#include <sbmpo/tools/PrintTool.hpp>

using namespace sbmpo;
using namespace sbmpo_algorithms;
using namespace sbmpo_models;

int main(int argc, char **argv)
{
    SearchParameters parameters;
    parameters.grid_resolution = {0.01, 0.01, 0};
    parameters.max_iterations = 5000;
    parameters.max_generations = 250;
    parameters.time_limit_us = 10000;
    parameters.sample_type = ControlSampleType::FIXED;
    parameters.fixed_samples = {
        {0.25, -M_2PI / 8.0},
        {0.25, -M_2PI / 16.0},
        {0.25, 0.0},
        {0.25, M_2PI / 16.0},
        {0.25, M_2PI / 8.0}};

    /*
        Drive straight forward
    */
    parameters.start_state = {0, 0, 0};
    parameters.goal_state = {20, 20, 0};

    sbmpo_io::print_parameters(parameters);

    auto model = std::make_shared<UnicycleSteeringModel>();
    model->set_horizon_time(0.5);
    SBMPO sbmpo(model);
    sbmpo.run(parameters);

    sbmpo_io::print_results(sbmpo.results());
    sbmpo_io::print_stats(sbmpo.results());

    return 0;
}