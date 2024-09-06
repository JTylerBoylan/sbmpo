#include <sbmpo/models/DoubleIntegratorModel.hpp>
#include <sbmpo/benchmarks/Benchmark.hpp>

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

    // Create new benchmark
    Benchmark<DoubleIntegratorModel> benchmarker(csv_folder);
    benchmarker.set_runs_per_param(10000);

    // Run benchmark on the model (saves to csv folder)
    benchmarker.benchmark();

    return 0;
}
