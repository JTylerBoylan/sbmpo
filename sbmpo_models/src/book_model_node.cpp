#include <ros/ros.h>
#include <ros/package.h>
#include <sbmpo_models/book_model.hpp>
#include <sbmpo/sbmpo.hpp>

#define VERBOSE true

void print_parameters(const sbmpo::PlannerParameters &params);

void print_results(const sbmpo::PlannerResults &results);

int main (int argc, char ** argv) {

    ros::init(argc, argv, "planner_node");
    ros::NodeHandle node("~");

    std::string param_config = ros::package::getPath("sbmpo_models") + "/config/book_model.csv";
    std::string result_datafile = ros::package::getPath("sbmpo_models") + "/results/book_model_results.csv";

    sbmpo_models::clearFile(result_datafile);

    std::vector<sbmpo::PlannerParameters> parameter_list;
    sbmpo_models::fromConfig(param_config, parameter_list);

    sbmpo_models::SBMPOBookModel book_model;

    for (auto param = parameter_list.begin(); param != parameter_list.end(); ++param) {

        if (VERBOSE) print_parameters(*param);

        sbmpo::PlannerResults results;
        sbmpo::run(book_model, *param, results);
        print_results(results);

        if (VERBOSE) ROS_INFO("Writing results to file %s ...", result_datafile.c_str());
        sbmpo_models::addToData(result_datafile, results);

        // Call this to avoid memory leak
        sbmpo::deconstruct(results);
    }

    if (VERBOSE) ROS_INFO("Finished.");

    return 0;

}

int seq = 0;
void print_parameters(const sbmpo::PlannerParameters &params) {
    ROS_INFO("---- Planner Parameters [%d] ----", seq);
    ROS_INFO("Max iterations: %d", params.max_iterations);
    ROS_INFO("Max generations: %d", params.max_generations);
    ROS_INFO("Sample Time: %.2f", params.sample_time);
    ROS_INFO("Sample Time Increment: %.2f", params.sample_time_increment);
    ROS_INFO("Goal Threshold: %.2f", params.conditions.goal_threshold);

    std::string st;
    for (float f : params.conditions.initial_state)
        st += std::to_string(f) + " ";
    ROS_INFO("Initial State: %s", st.c_str());
    st.clear();

    for (float f : params.conditions.initial_control)
        st += std::to_string(f) + " ";
    ROS_INFO("Initial Control: %s", st.c_str());
    st.clear();

    for (float f : params.conditions.goal_state)
        st += std::to_string(f) + " ";
    ROS_INFO("Goal State: %s", st.c_str());
    st.clear();

    for (float f : params.grid_parameters.active)
        st += std::to_string(f) + " ";
    ROS_INFO("Grid Active: %s", st.c_str());
    st.clear();

    for (float f : params.grid_parameters.resolution)
        st += std::to_string(f) + " ";
    ROS_INFO("Grid Resolution: %s", st.c_str());
    st.clear();

    for (float f : params.grid_parameters.size)
        st += std::to_string(f) + " ";
    ROS_INFO("Grid Size: %s", st.c_str());
    st.clear();

    ROS_INFO("Branchout:");
    for (sbmpo::Control control : params.branchout) {
        for (float f : control)
            st += std::to_string(f) + " ";
        ROS_INFO("  - %s", st.c_str());
        st.clear();
    }
}

void print_results(const sbmpo::PlannerResults &results) {
    ROS_INFO("---- Planner Results [%d] ----", seq++);
    ROS_INFO("Computing Time: %.3f ms", results.time_ms);
    ROS_INFO("Number of Nodes: %d", results.high);
    ROS_INFO("Exit code: %s", sbmpo::code2string(results.exit_code).c_str());
    ROS_INFO("Path:");
    for (sbmpo::Index idx : results.path) {
        sbmpo::Node n = results.buffer[idx];
        ROS_INFO("  (%d) [@%d] x: %.3f, y: %.3f, w: %.3f, v: %.3f, u: %.3f, g: %.3f, f: %.3f",
            n.lineage.generation, n.lineage.id,
            n.state[0], n.state[1], n.state[2],
            n.control[0], n.control[1],
            n.heuristic.g, n.heuristic.f);
    }
    ROS_INFO("--------");
}

