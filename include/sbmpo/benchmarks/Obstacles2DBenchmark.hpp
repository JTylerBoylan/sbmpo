#ifndef SBMPO_BENCHMARK_OBSTACLES2D_HPP_
#define SBMPO_BENCHMARK_OBSTACLES2D_HPP_

#include <sbmpo/types/types.hpp>
#include <sbmpo/benchmarks/Benchmark.hpp>
#include <sbmpo/benchmarks/models/Obstacles2DModel.hpp>

namespace sbmpo_io {

// Print obstacles
void print_obstacles(const sbmpo_benchmarks::Obstacles& obstacles) {
    printf("Obstacles:\n");
    for (std::array<float, 3> obs : obstacles) {
        std::string st;
        for (float f : obs)
            st += std::to_string(f) + " ";
        printf(" - %s\n", st.c_str());
    }
}

}

namespace sbmpo_csv {

// Read obstacles from file
std::vector<sbmpo_benchmarks::Obstacles> get_obstacles(const std::string& obstacles_file) {

    std::vector<sbmpo_benchmarks::Obstacles> obstaclesList;

    std::ifstream myFile(obstacles_file);
    if(!myFile.is_open()) 
        throw std::runtime_error("Could not open file");

    std::string line, value;
    while (std::getline(myFile, line)) {

        sbmpo_benchmarks::Obstacles obstacles;
        std::stringstream ss(line);

        std::getline(ss, value, ',');
        int num_obstacles = std::stoi(value);

        for (int obs = 0; obs < num_obstacles; obs++) {

            float x,y,r;

            std::getline(ss, value, ',');
            x = std::stof(value);
            std::getline(ss, value, ',');
            y = std::stof(value);
            std::getline(ss, value, ',');
            r = std::stof(value);

            obstacles.push_back({x, y, r});
        }

        obstaclesList.push_back(obstacles);
    }

    return obstaclesList;
}

}

namespace sbmpo_benchmarks {

using namespace sbmpo;

const std::string OBSTACLES_FILE = "obstacles.csv";

template <typename ModelType, typename SearchType = DEFAULT_SEARCH_ALGORITHM>
class Obstacles2DBenchmark :  public Benchmark<Obstacle2DModel<ModelType>, SearchType> {
static_assert(std::is_base_of<sbmpo::Model, ModelType>::value, "ModelType must derive from sbmpo::Model");
static_assert(std::is_base_of<sbmpo::SearchAlgorithm, SearchType>::value, "SearchType must derive from sbmpo::SearchAlgorithm");

    public:

    Obstacles2DBenchmark(std::string csv_folder) 
    : Benchmark<Obstacle2DModel<ModelType>, SearchType>(csv_folder) {}

    void benchmark() override {
        // Clear results
        sbmpo_csv::clear_file(this->csv_folder_ + STATS_FILE);
        sbmpo_csv::clear_file(this->csv_folder_ + NODES_FILE);
        this->index_ = 0;

        size_t par = 0;
        auto param_list = sbmpo_csv::get_params(this->csv_folder_ + PARAMS_FILE);
        auto obstacles_list = sbmpo_csv::get_obstacles(this->csv_folder_ + OBSTACLES_FILE);
        for (auto param = param_list.cbegin(); param != param_list.cend(); ++param) {
            this->model_->set_obstacles(par < obstacles_list.size() ? obstacles_list[par++] : Obstacles());
            this->run(*param);
        }

        if (this->verbose_) printf("Finished benchmarking.\n");
    }



};

}

#endif