#include <ros/ros.h>
#include <ros/package.h>
#include <sbmpo_models/book_model.hpp>
#include <sbmpo/sbmpo.hpp>
#include <ctime>

#define VERBOSE true
#define RUNS 10

void print_parameters(const sbmpo::Parameters &params);

void print_obstacles(std::vector<std::array<float, 3>> obstacles);

void print_results(sbmpo::SBMPO &results, const float time_ms, const int exit_code);

int main (int argc, char ** argv) {

    ros::init(argc, argv, "planner_node");
    ros::NodeHandle node("~");

    std::string param_config = ros::package::getPath("sbmpo_models") + "/config/book_model.csv";
    std::string result_datafile = ros::package::getPath("sbmpo_models") + "/results/book_model_results.csv";
    std::string obstacles_file = ros::package::getPath("sbmpo_models") + "/results/obstacles.csv";

    sbmpo_models::clearFile(result_datafile);
    sbmpo_models::clearFile(obstacles_file);

    std::vector<sbmpo::Parameters> parameter_list;
    sbmpo_models::fromConfig(param_config, parameter_list);

    sbmpo_models::SBMPOBookModel book_model;

    for (auto param = parameter_list.begin(); param != parameter_list.end(); ++param) {

        if (VERBOSE) print_parameters(*param);

        sbmpo::SBMPO planner;
        std::vector<std::array<float, 3>> obstacles;
        int exit_code;
        double time_ms = 0.0f;
        for (int r = 0; r < RUNS; r++) {

            obstacles = book_model.randomize_obstacles(3, 1.0, 4.0);

            clock_t cstart = std::clock();

            exit_code = planner.run(book_model, *param);

            clock_t cend = std::clock();

            time_ms += double(cend - cstart) / double(CLOCKS_PER_SEC * RUNS) * 1000.0;

        }

        sbmpo_models::addToData(obstacles_file, obstacles);
        if (VERBOSE) print_obstacles(obstacles);

        if (VERBOSE) print_results(planner, time_ms, exit_code);

        if (VERBOSE) ROS_INFO("Writing results to file %s ...", result_datafile.c_str());

        sbmpo_models::addToData(result_datafile, planner, time_ms, exit_code, 
            sbmpo_models::SaveOptions::STATS |
            sbmpo_models::SaveOptions::PATH
        );

    }

    if (VERBOSE) ROS_INFO("Finished.");

    return 0;

}

int seq = 0;
void print_parameters(const sbmpo::Parameters &params) {
    ROS_INFO("---- Planner Parameters [%d] ----", seq);
    ROS_INFO("Max iterations: %d", params.max_iterations);
    ROS_INFO("Max generations: %d", params.max_generations);
    ROS_INFO("Sample Time: %.2f", params.sample_time);
    ROS_INFO("Sample Time Increment: %.2f", params.sample_time_increment);
    ROS_INFO("Goal Threshold: %.2f", params.goal_threshold);

    std::string st;
    for (float f : params.initial_state)
        st += std::to_string(f) + " ";
    ROS_INFO("Initial State: %s", st.c_str());
    st.clear();

    for (float f : params.initial_control)
        st += std::to_string(f) + " ";
    ROS_INFO("Initial Control: %s", st.c_str());
    st.clear();

    for (float f : params.goal_state)
        st += std::to_string(f) + " ";
    ROS_INFO("Goal State: %s", st.c_str());
    st.clear();

    for (float f : params.grid_states)
        st += std::to_string(f) + " ";
    ROS_INFO("Grid Active: %s", st.c_str());
    st.clear();

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

void print_obstacles(std::vector<std::array<float, 3>> obstacles) {
    ROS_INFO("Obstacles:");
    for (std::array<float, 3> obs : obstacles) {
        std::string st;
        for (float f : obs)
            st += std::to_string(f) + " ";
        ROS_INFO(" - %s", st.c_str());
    }
}

void print_results(sbmpo::SBMPO &results, const float time_ms, const int exit_code) {
    ROS_INFO("---- Planner Results [%d] ----", seq++);
    ROS_INFO("Computing Time: %.3f ms", time_ms);
    ROS_INFO("Number of Nodes: %d", results.size());
    ROS_INFO("Exit code: %d", exit_code);
    ROS_INFO("Path:");
    for (int idx : results.path()) {
        sbmpo::Vertex v = results.graph[idx];
        ROS_INFO("  (%d) [@%d] x: %.3f, y: %.3f, w: %.3f, v: %.3f, u: %.3f, g: %.3f, f: %.3f",
            v.gen, v.idx,
            v.state[0], v.state[1], v.state[2],
            v.control[0], v.control[1],
            v.g, v.f);
    }
    ROS_INFO("--------");
}

