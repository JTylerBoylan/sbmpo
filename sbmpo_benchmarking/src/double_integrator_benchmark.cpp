#include <sbmpo_models/DoubleIntegrator.hpp>
#include <sbmpo_benchmarking/benchmark.hpp>

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

    // Create new Double Integrator model
    sbmpo_models::DoubleIntegratorModel doubleIntegratorModel;

    // Create new benchmark
    sbmpo_benchmarking::Benchmark benchmarker(csv_folder);

    // Run benchmark on the model (saves to csv folder)
    benchmarker.benchmark(doubleIntegratorModel);

    return 0;

}