#ifndef SBMPO_BENCHMARK_OBSTACLES2D_HPP_
#define SBMPO_BENCHMARK_OBSTACLES2D_HPP_

#include <sbmpo/types/types.hpp>
#include <sbmpo/benchmarks/Benchmark.hpp>
#include <sbmpo/benchmarks/models/Obstacles2DModel.hpp>

#include <cassert>
#define assertm(exp, msg) assert(((void)msg, exp))

namespace sbmpo_io
{
    // Print obstacles
    void print_obstacles(const sbmpo_benchmarks::Obstacles &obstacles)
    {
        printf("Obstacles:\n");
        for (std::array<float, 3> obs : obstacles)
        {
            std::string st;
            for (float f : obs)
                st += std::to_string(f) + " ";
            printf(" - %s\n", st.c_str());
        }
    }
}

namespace sbmpo_csv
{
    // Read obstacles from file
    std::vector<sbmpo_benchmarks::Obstacles> get_obstacles(const std::string &obstacles_file)
    {
        std::ifstream myFile(obstacles_file);
        if (!myFile.is_open())
            throw std::runtime_error("Could not open file");

        std::vector<sbmpo_benchmarks::Obstacles> obstaclesList;
        std::string line, value;
        while (std::getline(myFile, line))
        {
            std::stringstream ss(line);

            std::getline(ss, value, ',');
            int num_obstacles = std::stoi(value);

            sbmpo_benchmarks::Obstacles obstacles(num_obstacles);
            for (int obs = 0; obs < num_obstacles; obs++)
            {
                float x, y, r;
                std::getline(ss, value, ',');
                x = std::stof(value);
                std::getline(ss, value, ',');
                y = std::stof(value);
                std::getline(ss, value, ',');
                r = std::stof(value);
                obstacles[obs] = {x, y, r};
            }
            obstaclesList.push_back(obstacles);
        }
        return obstaclesList;
    }
}

namespace sbmpo_benchmarks
{
    using namespace sbmpo;

    const std::string OBSTACLES_FILE = "obstacles.csv";

    template <typename ModelType>
    class Obstacles2DBenchmark : public Benchmark
    {
    public:
        Obstacles2DBenchmark(std::string csv_folder, std::shared_ptr<Obstacle2DModel<ModelType>> model, std::unique_ptr<SearchAlgorithm> search = nullptr)
            : Benchmark(csv_folder, model, std::move(search))
        {
            obstacle2d_model_ = model;
        }

        void benchmark() override
        {
            obstacles_list_ = sbmpo_csv::get_obstacles(this->csv_folder_ + OBSTACLES_FILE);
            num_obstacles_ = obstacles_list_.size();
            if (num_obstacles_ != this->runs_per_param_)
            {
                printf("Warning: Number of obstacles in 'obstacles.csv' (%lu) does not match 'runs_per_param' (%d).\n",
                       num_obstacles_, this->runs_per_param_);
            }
            Benchmark::benchmark();
        }

        void run(const SearchParameters &params) override
        {
            if (this->verbose_)
                sbmpo_io::print_parameters(params, this->index_);

            SearchResults avg_results;
            avg_results.exit_code = SOLUTION_FOUND;
            for (size_t r = 0; r < this->runs_per_param_; r++)
            {
                obstacle2d_model_->set_obstacles(r < num_obstacles_ ? obstacles_list_[r] : Obstacles());
                this->search_->solve(params);
                avg_results.time_us += this->results_->time_us;
                avg_results.cost += this->results_->cost;
                avg_results.iteration += this->results_->iteration;
                avg_results.success_rate += this->results_->success_rate;
                if (this->results_->exit_code != SOLUTION_FOUND)
                {
                    avg_results.exit_code = this->results_->exit_code;
                }
            }
            avg_results.time_us /= this->runs_per_param_;
            avg_results.cost /= this->runs_per_param_;
            avg_results.iteration /= this->runs_per_param_;
            avg_results.success_rate /= this->runs_per_param_;

            if (this->verbose_)
                sbmpo_io::print_results(this->results_.get(), this->index_);
            if (this->verbose_)
                sbmpo_io::print_stats(&avg_results, this->index_);

            if (this->verbose_)
                printf("Writing results in folder %s ...\n", this->csv_folder_.c_str());
            sbmpo_csv::append_stats(this->csv_folder_ + STATS_FILE, avg_results);
            if (this->print_path_)
                sbmpo_csv::append_node_path(this->csv_folder_ + NODES_FILE, this->results_->node_path);
            if (this->print_nodes_)
                sbmpo_csv::append_nodes(this->csv_folder_ + NODES_FILE, this->results_->nodes);
            if (this->verbose_)
                printf("\n");
            this->index_++;
        }

    protected:
        std::shared_ptr<Obstacle2DModel<ModelType>> obstacle2d_model_;
        size_t num_obstacles_;
        std::vector<sbmpo_benchmarks::Obstacles> obstacles_list_;
    };

}

#endif