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
    parameters.grid_resolution = {0.05, 0.05, -1};
    parameters.max_iterations = 5000;
    parameters.max_generations = 50;
    parameters.time_limit_us = 10000;
    parameters.sample_time = 1.0;
    parameters.samples = {
        {0.25, -M_2PI / 8.0},
        {0.25, 0.0},
        {0.25, M_2PI / 8.0}};

    /*
        Drive straight forward
    */
    parameters.start_state = {0, 0, 0};
    parameters.goal_state = {10, 0, 0};

    sbmpo_io::print_parameters(parameters);

    SBMPO<UnicycleSteeringModel> sbmpo;
    sbmpo.run(parameters);

    sbmpo_io::print_results(sbmpo.results());
    sbmpo_io::print_stats(sbmpo.results());

    return 0;
}