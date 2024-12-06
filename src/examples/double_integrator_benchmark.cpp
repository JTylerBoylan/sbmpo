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

    // Create model
    auto model = std::make_shared<DoubleIntegratorModel>();
    model->set_horizon_time(0.25f);

    // Create new benchmark
    Benchmark benchmarker(csv_folder, model);
    benchmarker.set_runs_per_param(10000);

    // Run benchmark on the model (saves to csv folder)
    benchmarker.benchmark();

    return 0;
}
