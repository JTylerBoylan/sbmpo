#include <sbmpo_models/DoubleIntegrator.hpp>
#include <sbmpo_benchmarking/benchmark.hpp>

int main (int argc, char ** argv) {

    std::string csv_folder;

    if (argc > 1) {
        csv_folder = argv[1];
    } else {
        printf("\nMissing CSV folder path.\n");
        return 0;
    }

    sbmpo_models::DoubleIntegratorModel doubleIntegratorModel;
    sbmpo_benchmarking::Benchmark benchmarker(csv_folder);
    benchmarker.benchmark(doubleIntegratorModel);

    return 0;

}