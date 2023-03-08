#include <sbmpo_benchmarking/benchmark_models/UnicycleSteeringBenchmark.hpp>

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
    sbmpo_benchmarking::UnicycleSteeringBenchmark unicycleSteeringModel(csv_folder);

    // Change benchmark parameters
    unicycleSteeringModel.set_start_state({-3.0f, -3.0f, 0.0f});
    unicycleSteeringModel.set_goal_state({3.0f, 3.0f});
    unicycleSteeringModel.set_body_radius(0.125f);

    // Run benchmark (saves to csv folder)
    unicycleSteeringModel.benchmark(unicycleSteeringModel);

    return 0;

}