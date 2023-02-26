#include <ros/ros.h>
#include <ros/package.h>

#include <sbmpo_models/benchmarking_util.hpp>
#include <ctime>

bool verbose = true;
int runsPerParam = 1;
int obstacleMinN = 15, obstacleMaxN = 25;
float obstacleMinX = -2.5, obstacleMaxX = 2.5;
float obstacleMinY = -2.5, obstacleMaxY = 2.5;
float obstacleMinR = 0.25, obstacleMaxR = 0.5;

std::string paramsConfigFile = ros::package::getPath("sbmpo_models") + "/benchmarking/config.csv";
std::string resultsSaveFile = ros::package::getPath("sbmpo_models") + "/benchmarking/nodes.csv";
std::string statsSaveFile = ros::package::getPath("sbmpo_models") + "/benchmarking/stats.csv";
std::string obstaclesSaveFile = ros::package::getPath("sbmpo_models") + "/benchmarking/obstacles.csv";

std::string modelType = "simple_steering";

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

    node.getParam("model_type", modelType);

    std::shared_ptr<sbmpo_models::BenchmarkingModel> model;
    if (modelType == "simple_steering")
        model = std::make_shared<sbmpo_models::SimpleSteeringModel>();
    else if (modelType == "grid_2d")
        model = std::make_shared<sbmpo_models::Grid2DModel>();
    else if (modelType == "double_integrator")
        model = std::make_shared<sbmpo_models::DoubleIntegratorModel>();
    
    node.getParam("body_radius", model->BODY_RADIUS);
    node.getParam("min_bounds", model->BOUNDS[0]);
    node.getParam("max_bounds", model->BOUNDS[1]);
    node.getParam("start_state", model->START_STATE);
    node.getParam("goal_state", model->GOAL_STATE);
    node.getParam("goal_threshold", model->GOAL_THRESHOLD);

    sbmpo_models::clearFile(resultsSaveFile);
    sbmpo_models::clearFile(statsSaveFile);
    //sbmpo_models::clearFile(obstaclesSaveFile);

    std::vector<sbmpo::SBMPOParameters> parameterList;
    sbmpo_models::readParametersFromFile(paramsConfigFile, parameterList);

    std::vector<std::vector<std::array<float,3>>> obstaclesList;
    sbmpo_models::readObstaclesFromFile(obstaclesSaveFile, obstaclesList);

    int par = 0;
    for (auto param = parameterList.begin(); param != parameterList.end(); ++param) {

        if (verbose) sbmpo_models::print_parameters(*param);

        int exitCode;
        double timeMs = 0.0;
        double cost = 0.0;
        int bufferSize = 0;
        int successCount = 0;
        int iterations = 0;

        /*
        std::vector<std::array<float,3>>  obstacles = basicModel.randomize_obstacles(
                                                        obstacleMinN, obstacleMaxN, 
                                                        obstacleMinX, obstacleMaxX, 
                                                        obstacleMinY, obstacleMaxY, 
                                                        obstacleMinR ,obstacleMaxR);
        */
        
        std::vector<std::array<float,3>> obstacles = obstaclesList[par++];
        model->OBSTACLES = obstacles;
        
        sbmpo::SBMPO sbmpo(*model, *param);
        for (int r = 0; r < runsPerParam; r++) {

            sbmpo.run();

            exitCode = sbmpo.exit_code();
            timeMs += sbmpo.time_us() / 1E3;
            cost += exitCode ? 0 : sbmpo.cost();
            iterations += sbmpo.iterations();
            bufferSize += sbmpo.size();
            if (!exitCode) successCount++;

        }

        float timeMsAvg = timeMs / runsPerParam;
        float iterationsAvg = double(iterations) / runsPerParam;
        float costAvg = cost / successCount;
        float bufferSizeAvg = double(bufferSize) / runsPerParam;
        float successRate = double(successCount) / runsPerParam;

        if (verbose) sbmpo_models::print_stats(timeMsAvg, exitCode, iterationsAvg, costAvg, bufferSizeAvg, successRate);
        if (verbose) sbmpo_models::print_results(sbmpo);
        if (verbose) sbmpo_models::print_obstacles(obstacles);
        if (verbose) ROS_INFO("Writing results to file %s ...", resultsSaveFile.c_str());

        sbmpo_models::appendStatsToFile(statsSaveFile, timeMsAvg, exitCode, iterationsAvg, costAvg, bufferSizeAvg, successRate);
        sbmpo_models::appendNodesToFile(resultsSaveFile, sbmpo.node_path());
        sbmpo_models::appendNodesToFile(resultsSaveFile, sbmpo.all_nodes());
        //sbmpo_models::appendObstaclesToFile(obstaclesSaveFile, obstacles);

    }

    if (verbose) ROS_INFO("Finished.");

    return 0;

}


