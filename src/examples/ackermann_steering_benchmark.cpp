#include <sbmpo/models/AckermannSteering.hpp>
#include <sbmpo/tools/benchmarks/Obstacles2D.hpp>

using namespace sbmpo;
using namespace sbmpo_models;
using namespace sbmpo_benchmarking;

int main (int argc, char ** argv) {

    // Path to csv workspace
    std::string csv_folder;

    // Check arguments
    if (argc > 1) {
        csv_folder = argv[1];
    } else {
        printf("\nMissing CSV folder path.\n");
        return 0;
    }

    // Create new UnicycleSteering benchmark
    Obstacles2DBenchmark<AckermannSteeringModel> benchmarker(csv_folder);

    // Change benchmark parameters
    benchmarker.model()->set_body_radius(0.5f);
    benchmarker.model()->set_map_bounds({-20.0f, -20.0f, 20.0f, 20.0f});
    benchmarker.set_runs_per_param(1);

    // Run benchmark (saves to csv folder)
    benchmarker.benchmark();

    return 0;

}