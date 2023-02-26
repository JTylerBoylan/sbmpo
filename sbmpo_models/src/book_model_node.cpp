#include <ros/ros.h>
#include <ros/package.h>
#include <sbmpo_models/csv_util.hpp>
#include <sbmpo_models/benchmarking_util.hpp>
#include <sbmpo_models/models/book_model.hpp>
#include <sbmpo/sbmpo.hpp>
#include <ctime>

bool verbose = true;
int runsPerParam = 1;

int obstacleMinN = 2, obstacleMaxN = 5;
float obstacleMinX = 1.0f, obstacleMinY = 1.0f;
float obstacleMaxX = 4.0f, obstacleMaxY = 4.0f;
float obstacleMinR = 0.5, obstacleMaxR = 1.5;

std::string paramsConfigFile = ros::package::getPath("sbmpo_models") + "/config/book_model_config.csv";
std::string resultsSaveFile = ros::package::getPath("sbmpo_models") + "/results/book_model/results.csv";
std::string statsSaveFile = ros::package::getPath("sbmpo_models") + "/results/book_model/stats.csv";
std::string obstaclesSaveFile = ros::package::getPath("sbmpo_models") + "/results/book_model/obstacles.csv";

int main (int argc, char ** argv) {

    ros::init(argc, argv, "planner_node");
    ros::NodeHandle node("~");

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
    sbmpo_models::clearFile(obstaclesSaveFile);

    std::vector<sbmpo::SBMPOParameters> parameterList;
    sbmpo_models::readParametersFromFile(paramsConfigFile, parameterList);

    sbmpo_models::SBMPOBookModel bookModel;

    for (auto param = parameterList.begin(); param != parameterList.end(); ++param) {

        if (verbose) sbmpo_models::print_parameters(*param);
        
        std::vector<std::array<float, 3>> obstacles;

        int exitCode;
        double timeMs = 0.0;
        double cost = 0.0;
        int bufferSize = 0;
        int successCount = 0;
        int iterations = 0;

        sbmpo::SBMPO sbmpo(bookModel, *param);
        for (int r = 0; r < runsPerParam; r++) {

            obstacles = bookModel.randomize_obstacles(obstacleMinN, obstacleMaxN,
                                                      obstacleMinX, obstacleMaxX,
                                                      obstacleMinY, obstacleMaxY,
                                                      obstacleMinR, obstacleMaxR);

            sbmpo.run();

            exitCode = sbmpo.exit_code();
            timeMs +=  sbmpo.time_us();
            iterations += sbmpo.iterations();
            cost += exitCode ? 0 : sbmpo.cost();
            bufferSize += sbmpo.size();
            if (!exitCode) successCount++;

        }

        float timeMsAvg = timeMs / runsPerParam;
        float iterationsAvg = double(iterations) / runsPerParam;
        float costAvg = cost / successCount;
        float bufferSizeAvg = double(bufferSize) / runsPerParam;
        float successRate = double(successCount) / runsPerParam;

        if (verbose) sbmpo_models::print_stats(timeMsAvg, exitCode, iterations, costAvg, bufferSizeAvg, successRate);
        if (verbose) sbmpo_models::print_results(sbmpo);
        if (verbose) ROS_INFO("Writing results to file %s ...", resultsSaveFile.c_str());


        sbmpo_models::appendStatsToFile(statsSaveFile, timeMsAvg, iterationsAvg, exitCode, costAvg, bufferSizeAvg, successRate);
        sbmpo_models::appendNodesToFile(resultsSaveFile, sbmpo.node_path());
        sbmpo_models::appendNodesToFile(resultsSaveFile, sbmpo.all_nodes());
        sbmpo_models::appendObstaclesToFile(obstaclesSaveFile, obstacles);

    }

    if (verbose) ROS_INFO("Finished.");

    return 0;

}