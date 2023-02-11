#include <ros/ros.h>
#include <ros/package.h>
#include <sbmpo_models/basic_model.hpp>
#include <sbmpo/sbmpo.hpp>
#include <ctime>

bool verbose = true;
int runsPerParam = 1;
int obstacleMinN = 15, obstacleMaxN = 25;
float obstacleMinX = -2.5, obstacleMaxX = 2.5;
float obstacleMinY = -2.5, obstacleMaxY = 2.5;
float obstacleMinR = 0.25, obstacleMaxR = 0.5;

std::string paramsConfigFile = ros::package::getPath("sbmpo_models") + "/config/basic_model_config.csv";
std::string resultsSaveFile = ros::package::getPath("sbmpo_models") + "/results/basic_model/results.csv";
std::string statsSaveFile = ros::package::getPath("sbmpo_models") + "/results/basic_model/stats.csv";
std::string obstaclesSaveFile = ros::package::getPath("sbmpo_models") + "/results/basic_model/obstacles.csv";

void print_parameters(const sbmpo::Parameters &params);
void print_results(sbmpo::SBMPO &results);
void print_stats(const float timeMs, const int exitCode, const float cost, const int bufferSize, const float successRate);
void print_obstacles(const std::vector<std::array<float, 3>> obstacles);

int main (int argc, char ** argv) {

    ros::init(argc, argv, "planner_node");
    ros::NodeHandle node("~");

    srand(time(NULL));

    node.getParam("verbose", verbose);
    node.getParam("runs", runsPerParam);

    node.getParam("obstacles_min_num", obstacleMinN);
    node.getParam("obstacles_max_num", obstacleMaxN);
    node.getParam("obstacles_min_x", obstacleMinX);
    node.getParam("obstacles_max_x", obstacleMaxX);
    node.getParam("obstacles_min_y", obstacleMinY);
    node.getParam("obstacles_max_y", obstacleMaxY);
    node.getParam("obstacles_min_r", obstacleMinR);
    node.getParam("obstacles_max_r", obstacleMaxR);

    node.getParam("config_file_path", paramsConfigFile);
    node.getParam("results_file_path", resultsSaveFile);
    node.getParam("stats_file_path", statsSaveFile);
    node.getParam("obstacles_file_path", obstaclesSaveFile);

    sbmpo_models::clearFile(resultsSaveFile);
    sbmpo_models::clearFile(statsSaveFile);
    //sbmpo_models::clearFile(obstaclesSaveFile);

    std::vector<sbmpo::Parameters> parameterList;
    sbmpo_models::readParametersFromFile(paramsConfigFile, parameterList);

    sbmpo::SBMPO sbmpoPlanner;
    sbmpo_models::SBMPOBasicModel basicModel;

    /*
    std::vector<std::array<float, 3>> obstacles = {
        {0,0,0.5},
        {2,2,0.5},
        {-4,4,0.5},
        {-6,2,0.5},
        {0,7.5,0.5},
        {-2.5,0,0.5},
        {0,4,0.5},
        {5,-2,0.5},
        {7.5,4,0.5},
        {-2,1.4,0.5},
        {0,-5,0.5}
    };
    */
    
    //std::vector<std::array<float, 3>> obstacles = { {0,0,5} };

    //basicModel.set_obstacles(obstacles);
    
    std::vector<std::vector<std::array<float,3>>> obstaclesList;
    sbmpo_models::readObstaclesFromFile(obstaclesSaveFile, obstaclesList);

    int par = 0;
    for (auto param = parameterList.begin(); param != parameterList.end(); ++param) {

        if (verbose) print_parameters(*param);

        int exitCode;
        double timeMs = 0.0;
        double cost = 0.0;
        int bufferSize = 0;
        int successCount = 0;

        /*
        std::vector<std::array<float,3>>  obstacles = basicModel.randomize_obstacles(
                                                        obstacleMinN, obstacleMaxN, 
                                                        obstacleMinX, obstacleMaxX, 
                                                        obstacleMinY, obstacleMaxY, 
                                                        obstacleMinR ,obstacleMaxR);
        */
        
        std::vector<std::array<float,3>> obstacles = obstaclesList[par++];
        basicModel.set_obstacles(obstacles);
        
        for (int r = 0; r < runsPerParam; r++) {

            clock_t clockStart = std::clock();

            exitCode = sbmpoPlanner.run(basicModel, *param);

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
        if (verbose) print_obstacles(obstacles);
        if (verbose) ROS_INFO("Writing results to file %s ...", resultsSaveFile.c_str());


        sbmpo_models::appendStatsToFile(statsSaveFile, timeMsAvg, exitCode, costAvg, bufferSizeAvg, successRate);
        sbmpo_models::appendResultsToFile(resultsSaveFile, sbmpoPlanner);
        //sbmpo_models::appendObstaclesToFile(obstaclesSaveFile, obstacles);

    }

    if (verbose) ROS_INFO("Finished.");

    return 0;

}

int seq = 0;
void print_parameters(const sbmpo::Parameters &params) {
    ROS_INFO("---- Planner Parameters [%d] ----", seq);
    ROS_INFO("Max iterations: %d", params.max_iterations);
    ROS_INFO("Max generations: %d", params.max_generations);
    ROS_INFO("Sample Time: %.2f", params.sample_time);;

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
