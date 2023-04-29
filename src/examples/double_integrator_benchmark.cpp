#include <sbmpo/models/DoubleIntegrator.hpp>
#include <sbmpo/tools/benchmark.hpp>

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

    // Create new benchmark
    Benchmark<DoubleIntegratorModel> benchmarker(csv_folder);

    // Run benchmark on the model (saves to csv folder)
    benchmarker.benchmark();

    return 0;

}