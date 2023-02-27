#include <ros/ros.h>

#include <sbmpo_models/models/simple_steering_model.hpp>

bool verbose = true;
int runsPerParam = 1;

int main (int argc, char ** argv) {

    ros::init(argc, argv, "steering_model_benchmark");
    ros::NodeHandle node("~");

    srand(time(NULL));

    sbmpo_models::BenchmarkModel * model = new sbmpo_models::SimpleSteeringModel();

    model->set_obstacle_type(sbmpo_models::ObstacleType::FROM_FILE);
    model->benchmark();

    delete model;
    return 0;

}