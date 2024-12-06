#include <sbmpo/models/UnicycleSteeringModel.hpp>
#include <sbmpo/benchmarks/Obstacles2DBenchmark.hpp>

using namespace sbmpo;
using namespace sbmpo_models;
using namespace sbmpo_benchmarks;

int main(int argc, char **argv)
{
    // Path to csv workspace
    std::string csv_folder;

    // Check arguments
    if (argc > 1)
    {
        csv_folder = argv[1];
    }
    else
    {
        printf("\nMissing CSV folder path.\n");
        return 0;
    }

    // Create Obstacle2D model
    auto model = std::make_shared<Obstacle2DModel<UnicycleSteeringModel>>();
    model->set_horizon_time(0.5f);
    model->set_body_radius(0.125f);
    model->set_goal_threshold(0.5f);
    model->set_integration_steps(10);
    model->set_map_bounds({-10, -10, 10, 10});

    // Create new benchmark
    Obstacles2DBenchmark benchmarker(csv_folder, model);
    benchmarker.set_runs_per_param(1);
    benchmarker.set_verbose(true);
    benchmarker.set_print_path(true);
    benchmarker.set_print_nodes(true);

    // Run benchmark (saves to csv folder)
    benchmarker.benchmark();

    return 0;
}