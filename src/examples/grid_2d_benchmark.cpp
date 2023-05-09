#include <sbmpo/models/Grid2D.hpp>
#include <sbmpo/tools/benchmarks/Obstacles2D.hpp>

using namespace sbmpo;
using namespace sbmpo_models;

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

    // Create new benchmarker
    Obstacles2DBenchmark<Grid2DModel> benchmarker(csv_folder);

    // Change benchmark parameters
    benchmarker.model()->set_body_radius(0.125f);

    // Run benchmark (saves to csv folder)
    benchmarker.benchmark();

    return 0;

}