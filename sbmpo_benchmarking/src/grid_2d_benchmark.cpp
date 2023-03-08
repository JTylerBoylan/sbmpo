#include <sbmpo_benchmarking/benchmark_models/Grid2DBenchmark.hpp>

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

    // Create new Grid2D benchmark
    sbmpo_benchmarking::Grid2DBenchmark grid2dModel(csv_folder);

    // Change benchmark parameters
    grid2dModel.set_body_radius(0.125f);

    // Run benchmark (saves to csv folder)
    grid2dModel.benchmark(grid2dModel);

    return 0;

}