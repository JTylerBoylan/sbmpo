#include <sbmpo_models/models/grid_2d_model.hpp>

bool verbose = true;
int runsPerParam = 1;

int main (int argc, char ** argv) {

    srand(time(NULL));

    sbmpo_models::BenchmarkModel * model = new sbmpo_models::Grid2DModel();

    model->set_obstacle_type(sbmpo_models::ObstacleType::FROM_FILE);
    model->benchmark();

    delete model;
    return 0;

}