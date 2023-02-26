#ifndef SBMPO_MODELS_BENCHMARKING_HPP
#define SBMPO_MODELS_BENCHMARKING_HPP

#include <ros/ros.h>
#include <sbmpo/sbmpo.hpp>

#include <sbmpo_models/csv_util.hpp>
#include <sbmpo_models/models/benchmarking_model.hpp>
#include <sbmpo_models/models/grid_2D_model.hpp>
#include <sbmpo_models/models/simple_steering_model.hpp>
#include <sbmpo_models/models/double_integrator_model.hpp>

namespace sbmpo_models {

int seq = 0;
void print_parameters(const sbmpo::SBMPOParameters &params) {
    ROS_INFO("---- Planner Parameters [%d] ----", seq);
    ROS_INFO("Max iterations: %d", params.max_iterations);
    ROS_INFO("Max generations: %d", params.max_generations);
    ROS_INFO("Sample Time: %.2f", params.sample_time);;

    std::string st;

    for (float f : params.grid_resolution)
        st += std::to_string(f) + " ";
    ROS_INFO("Grid Resolution: %s", st.c_str());
    st.clear();

    ROS_INFO("Samples:");
    for (sbmpo::Control control : params.samples) {
        for (float f : control)
            st += std::to_string(f) + " ";
        ROS_INFO("  - %s", st.c_str());
        st.clear();
    }
}

void print_results(sbmpo::SBMPO &results) {
    ROS_INFO("---- Planner Path [%d] ----", seq++);
    int c = 0;
    for (std::shared_ptr<sbmpo::Node> node : results.node_path()) {
        ROS_INFO("  (%d) x: %.3f, y: %.3f, w: %.3f, g: %.3f, rhs: %.3f, f: %.3f",
            ++c,
            node->state()[0], node->state()[1], node->state()[2],
            node->g(), node->rhs(), node->f());
    }
    ROS_INFO("--------");
}

void print_stats(const float timeMs, const int exitCode, const int iterations, const float cost, const int bufferSize, const float successRate) {
    ROS_INFO("---- Planner Stats ----");
    ROS_INFO("  Time: %.2f ms", timeMs);
    ROS_INFO("  Exit Code: %d", exitCode);
    ROS_INFO("  Iterations: %d", iterations);
    ROS_INFO("  Cost: %.2f", cost);
    ROS_INFO("  Buffer Size: %d", bufferSize);
    ROS_INFO("  Success Rate: %.1f%%", successRate * 100);
    ROS_INFO("--------");
}

void print_obstacles(const std::vector<std::array<float, 3>> obstacles) {
    ROS_INFO("Obstacles:");
    for (std::array<float, 3> obs : obstacles) {
        std::string st;
        for (float f : obs)
            st += std::to_string(f) + " ";
        ROS_INFO(" - %s", st.c_str());
    }
}

}


#endif