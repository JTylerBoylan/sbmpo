#ifndef SBMPO_BENCHMARK_OBSTACLES2D_HPP
#define SBMPO_BENCHMARK_OBSTACLES2D_HPP

#include <sbmpo/tools/benchmark.hpp>
#include <sbmpo/tools/benchmarks/models/Obstacles2D.hpp>

namespace sbmpo_benchmarking {

using namespace sbmpo;

template <typename ModelType>
class Obstacles2DBenchmark :  public Benchmark<ModelType> {
static_assert(std::is_base_of<sbmpo::Model, ModelType>::value, "ModelType must derive from sbmpo::Model");

    public:

    /// @brief Create a new 2D obstacles benchmark
    /// @param csv_folder Path to csv workspace folder
    Obstacles2DBenchmark(std::string csv_folder) : Benchmark<ModelType>(csv_folder)
    {
        this->model_ = std::make_shared<Obstacle2DModel<ModelType>>();
        obstacles_file_ = csv_folder + "obstacles.csv";
    }

    /// @brief Get an instance of the model
    /// @return Pointer to model object
    std::shared_ptr<Obstacle2DModel<ModelType>> model() {
        return std::dynamic_pointer_cast<Obstacle2DModel<ModelType>>(this->model_);
    }

    /// @brief Benchmark a model with 2D obstacles
    /// @param model Model to be benchmarked
    void benchmark() override {

        this->csv_tool_.clear_results_file();

        size_t par = 0;
        std::vector<SBMPOParameters> parameters_list = this->csv_tool_.get_params();
        std::vector<Obstacles> obstaclesList = get_obstacles();
        for (auto param = parameters_list.begin(); param != parameters_list.end(); ++param) {
            model()->set_obstacles(par < obstaclesList.size() ? obstaclesList[par++] : Obstacles());
            this->run_param(*param);
        }

        if (this->verbose_) printf("Finished benchmarking.\n");
    }

    protected:

    std::string obstacles_file_;

    // Print obstacles
    void print_obstacles(const Obstacles obstacles) {
        printf("Obstacles:\n");
        for (std::array<float, 3> obs : obstacles) {
            std::string st;
            for (float f : obs)
                st += std::to_string(f) + " ";
            printf(" - %s\n", st.c_str());
        }
    }

    // Read obstacles from file
    std::vector<std::vector<std::array<float,3>>> get_obstacles() {

        std::vector<std::vector<std::array<float,3>>> obstaclesList;

        std::ifstream myFile(obstacles_file_);
        if(!myFile.is_open()) 
            throw std::runtime_error("Could not open file");

        std::string line, value;
        while (std::getline(myFile, line)) {

            std::vector<std::array<float,3>> obstacles;
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

};

}

#endif