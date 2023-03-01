#ifndef SBMPO_BENCHMARK_OBSTACLES2D_HPP
#define SBMPO_BENCHMARK_OBSTACLES2D_HPP

#include <sbmpo_benchmarking/benchmark.hpp>

namespace sbmpo_benchmarking {

using namespace sbmpo;

typedef std::vector<std::array<float,3>> Obstacles;

class Obstacles2DBenchmark : public Benchmark {

    public:

    Obstacles2DBenchmark(std::string csv_folder) : Benchmark(csv_folder)
    {}

    void set_obstacles(Obstacles obstacles) {
        this->obstacles_ = obstacles;
    }

    void benchmark(sbmpo::Model * model) override {

        csv_tool_.clear_results_file();

        size_t par = 0;
        std::vector<SBMPOParameters> parameters_list = csv_tool_.get_params();
        std::vector<Obstacles> obstaclesList = csv_tool_.get_obstacles();
        for (auto param = parameters_list.begin(); param != parameters_list.end(); ++param) {

            if (verbose_) print_parameters(*param);

            int exit_code = UNKNOWN_ERROR;
            unsigned long time_us = 0.0;
            double cost = 0.0;
            int node_count = 0;
            int success_count = 0;
            int iterations = 0;

            obstacles_ = par < obstaclesList.size() ? obstaclesList[par++] : Obstacles();
            
            sbmpo::SBMPO sbmpo(*model, *param);

            for (int r = 0; r < runs_per_param_; r++) {

                sbmpo.reset();
                sbmpo.run();

                exit_code = sbmpo.exit_code();
                time_us += sbmpo.time_us();
                cost += exit_code ? 0 : sbmpo.cost();
                iterations += sbmpo.iterations();
                node_count += sbmpo.size();
                if (!exit_code) success_count++;

            }

            unsigned long time_us_avg = time_us / runs_per_param_;
            float iterations_avg = double(iterations) / runs_per_param_;
            float cost_avg = cost / success_count;
            float node_count_avg = double(node_count) / runs_per_param_;
            float success_rate = double(success_count) / runs_per_param_;

            if (verbose_) print_stats(time_us_avg, exit_code, iterations_avg, cost_avg, node_count_avg, success_rate);
            if (verbose_) print_results(sbmpo);
            if (verbose_) print_obstacles(obstacles_);

            if (verbose_) printf("Writing results in folder %s ...\n\n", csv_tool_.get_save_folder().c_str());
            csv_tool_.append_stats(time_us_avg, exit_code, iterations_avg, cost_avg, node_count_avg, success_rate);
            csv_tool_.append_nodes(sbmpo.node_path());
            csv_tool_.append_nodes(sbmpo.all_nodes());

        }

        if (verbose_) printf("Finished benchmarking.\n");
    }

    protected:

    Obstacles obstacles_;

    void print_obstacles(const Obstacles obstacles) {
        printf("Obstacles:\n");
        for (std::array<float, 3> obs : obstacles) {
            std::string st;
            for (float f : obs)
                st += std::to_string(f) + " ";
            printf(" - %s\n", st.c_str());
        }
    }

};

}

#endif