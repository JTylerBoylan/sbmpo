#include <ros/ros.h>
#include <ros/package.h>
#include <sbmpo_models/book_model.hpp>
#include <sbmpo/sbmpo.hpp>
#include <ctime>

bool verbose = true;
int runsPerParam = 1;
int numberOfObstacles = 0;

std::string paramsConfigFile = ros::package::getPath("sbmpo_models") + "/config/book_model_config.csv";
std::string resultsSaveFile = ros::package::getPath("sbmpo_models") + "/results/book_model/results.csv";
std::string statsSaveFile = ros::package::getPath("sbmpo_models") + "/results/book_model/stats.csv";
std::string obstaclesSaveFile = ros::package::getPath("sbmpo_models") + "/results/book_model/obstacles.csv";

float obstacleMinXY = 1.0f;
float obstacleMaxXY = 4.0f;

void print_parameters(const sbmpo::Parameters &params);
void print_results(sbmpo::SBMPO &results);
void print_stats(const float timeMs, const int exitCode, const float cost, const int bufferSize, const float successRate);
void print_obstacles(const std::vector<std::array<float, 3>> obstacles);

int main (int argc, char ** argv) {

    ros::init(argc, argv, "planner_node");
    ros::NodeHandle node("~");

    node.getParam("verbose", verbose);
    node.getParam("runs", runsPerParam);
    node.getParam("num_obstacles", numberOfObstacles);
    node.getParam("config_file_path", paramsConfigFile);
    node.getParam("results_file_path", resultsSaveFile);
    node.getParam("stats_file_path", statsSaveFile);
    node.getParam("obstacles_file_path", obstaclesSaveFile);
    node.getParam("obstacle_min_bound", obstacleMinXY);
    node.getParam("obstacle_max_bound", obstacleMaxXY);

    sbmpo_models::clearFile(resultsSaveFile);
    sbmpo_models::clearFile(statsSaveFile);
    sbmpo_models::clearFile(obstaclesSaveFile);

    std::vector<sbmpo::Parameters> parameterList;
    sbmpo_models::readParametersFromFile(paramsConfigFile, parameterList);

    sbmpo::SBMPO sbmpoPlanner;
    sbmpo_models::SBMPOBookModel bookModel;

    for (auto param = parameterList.begin(); param != parameterList.end(); ++param) {

        if (verbose) print_parameters(*param);
        
        std::vector<std::array<float, 3>> obstacles;

        int exitCode;
        double timeMs = 0.0;
        double cost = 0.0;
        int bufferSize = 0;
        int successCount = 0;

        for (int r = 0; r < runsPerParam; r++) {

            obstacles = bookModel.randomize_obstacles(numberOfObstacles, obstacleMinXY, obstacleMaxXY);

            clock_t clockStart = std::clock();

            exitCode = sbmpoPlanner.run(bookModel, *param);

            clock_t clockEnd = std::clock();

            timeMs += double(clockEnd - clockStart) / double(CLOCKS_PER_SEC) * 1000.0;
            cost += exitCode ? 0 : sbmpoPlanner.cost();
            bufferSize += sbmpoPlanner.size();
            if (!exitCode) successCount++;

        }

        float timeMsAvg = timeMs / runsPerParam;
        float costAvg = cost / runsPerParam;
        float bufferSizeAvg = double(bufferSize) / runsPerParam;
        float successRate = double(successCount) / runsPerParam;

        if (verbose) print_stats(timeMsAvg, exitCode, costAvg, bufferSizeAvg, successRate);
        if (verbose) print_results(sbmpoPlanner);
        if (verbose) ROS_INFO("Writing results to file %s ...", resultsSaveFile.c_str());


        sbmpo_models::appendStatsToFile(statsSaveFile, timeMsAvg, exitCode, costAvg, bufferSizeAvg, successRate);
        sbmpo_models::appendResultsToFile(resultsSaveFile, sbmpoPlanner);
        sbmpo_models::appendObstaclesToFile(obstaclesSaveFile, obstacles);

    }

    if (verbose) ROS_INFO("Finished.");

    return 0;

}

int seq = 0;
void print_parameters(const sbmpo::Parameters &params) {
    ROS_INFO("---- Planner Parameters [%d] ----", seq);
    ROS_INFO("Max iterations: %d", params.max_iterations);
    ROS_INFO("Max generations: %d", params.max_generations);
    ROS_INFO("Sample Time: %.2f", params.sample_time);

    std::string st;

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

void print_results(sbmpo::SBMPO &results) {
    ROS_INFO("---- Planner Path [%d] ----", seq++);
    int c = 0;
    for (int idx : results.vertex_path()) {
        sbmpo::Vertex v = results.graph.vertex(idx);
        ROS_INFO("  (%d) [@%d] x: %.3f, y: %.3f, w: %.3f, g: %.3f, rhs: %.3f, f: %.3f",
            ++c, v.idx,
            v.state[0], v.state[1], v.state[2],
            v.g, v.rhs, v.f);
    }
    ROS_INFO("--------");
}

void print_stats(const float timeMs, const int exitCode, const float cost, const int bufferSize, const float successRate) {
    ROS_INFO("---- Planner Stats ----");
    ROS_INFO("  Time: %.2f ms", timeMs);
    ROS_INFO("  Exit Code: %d", exitCode);
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