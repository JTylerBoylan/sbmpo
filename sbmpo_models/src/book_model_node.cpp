#include <ros/ros.h>
#include <ros/package.h>
#include <sbmpo_models/book_model.hpp>
#include <sbmpo/sbmpo.hpp>

#define VERBOSE true

void print_csv(const sbmpo_models::CSVMap &csv_map);

void print_parameters(const sbmpo::PlannerParameters &params);

void print_results(const sbmpo::PlannerResults &results);

int main (int argc, char ** argv) {

    ros::init(argc, argv, "planner_node");
    ros::NodeHandle node("~");

    std::string param_csv = ros::package::getPath("sbmpo_models") + "/config/book_model.csv";
    std::string result_csv = ros::package::getPath("sbmpo_models") + "/results/book_model_results.csv";

    sbmpo_models::CSVMap csv_map = sbmpo_models::read_csv(param_csv);
    sbmpo_models::CSVMap csv_results;

    std::vector<sbmpo::PlannerParameters> parameter_list = sbmpo_models::fromCSVMap(csv_map);

    sbmpo_models::SBMPOBookModel book_model;

    for (auto param = parameter_list.begin(); param != parameter_list.end(); ++param) {

        if (VERBOSE) print_parameters(*param);

        sbmpo::PlannerResults results;
        sbmpo::run(book_model, *param, results);
        print_results(results);

        csv_results = sbmpo_models::addToCSVMap(results, csv_results);

        // Call this to avoid memory leak
        sbmpo::deconstruct(results);
    }

    if (VERBOSE) ROS_INFO("Writing to CSV... (%s)", result_csv.c_str());

    sbmpo_models::write_csv(result_csv, csv_results);

    if (VERBOSE) ROS_INFO("Finished.");

    return 0;

}

void print_csv(const sbmpo_models::CSVMap &csv_map) {
    ROS_INFO("---- Loaded CSV ----");
    for (auto pair = csv_map.begin(); pair != csv_map.end(); ++pair) {
        ROS_INFO("%s:", pair->first.c_str());
        for (float f : pair->second) {
            ROS_INFO("  - %.2f", f);
        }
    }
}

int seq = 0;
void print_parameters(const sbmpo::PlannerParameters &params) {
    ROS_INFO("---- Planner Parameters [%d] ----", seq);
    ROS_INFO("Max iterations: %d", params.max_iterations);
    ROS_INFO("Max generations: %d", params.max_generations);
    ROS_INFO("Sample Time: %.2f", params.sample_time);
    ROS_INFO("Sample Time Increment: %.2f", params.sample_time_increment);
    ROS_INFO("Branchout:");
    for (sbmpo::Control ctl : params.branchout) {
        std::string cstr;
        for (int c = 0; c < ctl.size(); c++) {
            cstr += std::to_string(ctl[c]);
            if (c != ctl.size() - 1)
                cstr += ", ";
        }
        ROS_INFO("  - (%s)", cstr.c_str());
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

