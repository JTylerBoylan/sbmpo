#include <sbmpo_benchmarking/benchmark_models/SimpleRobotBenchmark.hpp>

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

    // Create new SimpleRobot benchmark
    sbmpo_benchmarking::SimpleRobotBenchmark simpleRobotModel(csv_folder);

    // Change benchmark parameters
    simpleRobotModel.set_start_state({-3.0f, -3.0f, 0.0f});
    simpleRobotModel.set_goal_state({3.0f, 3.0f});
    simpleRobotModel.set_body_radius(0.125f);

    // Run benchmark (saves to csv folder)
    simpleRobotModel.benchmark(simpleRobotModel);

    return 0;

}