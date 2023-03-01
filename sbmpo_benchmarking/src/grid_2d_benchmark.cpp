#include <sbmpo_benchmarking/benchmark_models/Grid2DBenchmark.hpp>

int main (int argc, char ** argv) {

    std::string csv_folder;

    if (argc > 1) {
        csv_folder = argv[1];
    } else {
        printf("\nMissing CSV folder path.\n");
        return 0;
    }

    sbmpo_benchmarking::Grid2DBenchmark grid2dModel(csv_folder);
    grid2dModel.benchmark(&grid2dModel);

    return 0;

}