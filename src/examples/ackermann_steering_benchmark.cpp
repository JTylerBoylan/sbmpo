#include <sbmpo/models/AckermannSteeringModel.hpp>
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

    // Create Obstacles2D model
    auto model = std::make_shared<Obstacle2DModel<AckermannSteeringModel>>();
    model->set_horizon_time(0.5f);
    model->set_body_radius(0.5f);
    model->set_map_bounds({-20.0f, -20.0f, 20.0f, 20.0f});

    // Create new UnicycleSteering benchmark
    Obstacles2DBenchmark benchmarker(csv_folder, model);
    benchmarker.set_runs_per_param(1);

    // Run benchmark (saves to csv folder)
    benchmarker.benchmark();

    return 0;
}