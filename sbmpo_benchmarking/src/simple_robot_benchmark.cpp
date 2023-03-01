#include <sbmpo_benchmarking/benchmark_models/SimpleRobotBenchmark.hpp>

int main (int argc, char ** argv) {

    std::string csv_folder;

    if (argc > 1) {
        csv_folder = argv[1];
    } else {
        printf("\nMissing CSV folder path.\n");
        return 0;
    }

    sbmpo_benchmarking::SimpleRobotBenchmark simpleRobotModel(csv_folder);
    simpleRobotModel.set_start_state({-3.0f, -3.0f, 0.0f});
    simpleRobotModel.set_goal_state({3.0f, 3.0f});
    simpleRobotModel.set_body_radius(0.125f);
    simpleRobotModel.benchmark(&simpleRobotModel);

    return 0;

}