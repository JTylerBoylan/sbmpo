#include <ros/ros.h>
#include <ros/package.h>

#include <sbmpo_models/benchmarking_util.hpp>
#include <sbmpo_models/models/grid_2d_model.hpp>
#include <sbmpo_models/models/simple_steering_model.hpp>
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

int main (int argc, char ** argv) {

    ros::init(argc, argv, "planner_node");
    ros::NodeHandle node("~");

    srand(time(NULL));

    node.getParam("verbose", verbose);
    node.getParam("runs", runsPerParam);

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
        unsigned long timeUs = 0.0;
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
        
        sbmpo_models::SimpleSteeringModel model;

        std::vector<std::array<float,3>> obstacles = obstaclesList[par++];
        model.obstacles = obstacles;
        
        sbmpo::SBMPO sbmpo(model, *param);

        for (int r = 0; r < runsPerParam; r++) {

            sbmpo.reset();
            sbmpo.run();

            exitCode = sbmpo.exit_code();
            timeUs += sbmpo.time_us();
            cost += exitCode ? 0 : sbmpo.cost();
            iterations += sbmpo.iterations();
            bufferSize += sbmpo.size();
            if (!exitCode) successCount++;

        }

        unsigned long timeUsAvg = timeUs / runsPerParam;
        float iterationsAvg = double(iterations) / runsPerParam;
        float costAvg = cost / successCount;
        float bufferSizeAvg = double(bufferSize) / runsPerParam;
        float successRate = double(successCount) / runsPerParam;

        if (verbose) sbmpo_models::print_stats(timeUsAvg, exitCode, iterationsAvg, costAvg, bufferSizeAvg, successRate);
        if (verbose) sbmpo_models::print_results(sbmpo);
        if (verbose) sbmpo_models::print_obstacles(obstacles);
        if (verbose) ROS_INFO("Writing results to file %s ...", resultsSaveFile.c_str());

        sbmpo_models::appendStatsToFile(statsSaveFile, timeUsAvg, exitCode, iterationsAvg, costAvg, bufferSizeAvg, successRate);
        sbmpo_models::appendNodesToFile(resultsSaveFile, sbmpo.node_path());
        sbmpo_models::appendNodesToFile(resultsSaveFile, sbmpo.all_nodes());
        //sbmpo_models::appendObstaclesToFile(obstaclesSaveFile, obstacles);

    }

    if (verbose) ROS_INFO("Finished.");

    return 0;

}


