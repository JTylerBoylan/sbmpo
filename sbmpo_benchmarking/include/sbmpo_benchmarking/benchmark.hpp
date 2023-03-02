#ifndef SBMPO_BENCHMARK_BENCHMARK_HPP
#define SBMPO_BENCHMARK_BENCHMARK_HPP

#include <sbmpo/sbmpo.hpp>
#include <sbmpo_benchmarking/csv_tool.hpp>

namespace sbmpo_benchmarking {

using namespace sbmpo;

class Benchmark {

    public:

    /// @brief Create a new benchmark
    /// @param csv_folder Path to csv workspace folder
    Benchmark(std::string csv_folder) 
        : csv_tool_(csv_folder) {
        this->verbose_ = true;
        this->runs_per_param_ = 1;
    }

    /// @brief Change path to csv workspace folder
    /// @param folder_path Path to workspace folder
    void set_folder(std::string folder_path) {
        csv_tool_.set_save_folder(folder_path);
    }

    /// @brief Change verbose option (default true)
    /// @param tf True if verbose
    void set_verbose(bool tf) {
        this->verbose_ = tf;
    }

    /// @brief Change number of runs per parameter set
    /// @param runs_per_param Number of runs
    void set_runs_per_param(int runs_per_param) {
        this->runs_per_param_ = runs_per_param;
    }

    /// @brief Benchmark a model
    /// @param model Model to be benchmarked
    virtual void benchmark(sbmpo::Model &model) {

        csv_tool_.clear_results_file();

        std::vector<SBMPOParameters> parameters_list = csv_tool_.get_params();
        for (auto param = parameters_list.begin(); param != parameters_list.end(); ++param) {

            if (verbose_) print_parameters(*param);

            int exit_code = UNKNOWN_ERROR;
            unsigned long time_us = 0.0;
            double cost = 0.0;
            int node_count = 0;
            int success_count = 0;
            int iterations = 0;
            
            sbmpo::SBMPO sbmpo(model, *param);

            for (int r = 0; r < runs_per_param_; r++) {

                sbmpo.reset();
                sbmpo.run();

                exit_code = sbmpo.exit_code();
                time_us += sbmpo.time_us();
                cost += sbmpo.cost();
                iterations += sbmpo.iterations();
                node_count += sbmpo.size();
                if (!exit_code) success_count++;

            }

            unsigned long time_us_avg = time_us / runs_per_param_;
            float iterations_avg = double(iterations) / runs_per_param_;
            float cost_avg = cost / runs_per_param_;
            float node_count_avg = double(node_count) / runs_per_param_;
            float success_rate = double(success_count) / runs_per_param_;

            if (verbose_) print_stats(time_us_avg, exit_code, iterations_avg, cost_avg, node_count_avg, success_rate);
            if (verbose_) print_results(sbmpo);

            if (verbose_) printf("Writing results in folder %s ...\n", csv_tool_.get_save_folder().c_str());
            csv_tool_.append_stats(time_us_avg, exit_code, iterations_avg, cost_avg, node_count_avg, success_rate);
            csv_tool_.append_node_path(sbmpo.node_path(), sbmpo.control_path());
            csv_tool_.append_nodes(sbmpo.all_nodes());
            printf("\n");

        }

        if (verbose_) printf("Finished benchmarking.\n");
    }

    protected:

    bool verbose_;
    int runs_per_param_;

    CSVTool csv_tool_;

    int print_count = 0;
    // Print parameters
    void print_parameters(const sbmpo::SBMPOParameters &params) {
        printf("---- Planner Parameters [%d] ----\n", print_count);
        printf("Max iterations: %d\n", params.max_iterations);
        printf("Max generations: %d\n", params.max_generations);
        printf("Sample Time: %.2f\n", params.sample_time);;

        std::string st;

        for (float f : params.grid_resolution)
            st += std::to_string(f) + " ";
        printf("Grid Resolution: %s\n", st.c_str());
        st.clear();

        printf("Samples:\n");
        for (sbmpo::Control control : params.samples) {
            for (float f : control)
                st += std::to_string(f) + " ";
            printf("  - %s\n", st.c_str());
            st.clear();
        }
    }

    // Print results
    void print_results(sbmpo::SBMPO &results) {
        printf("---- Planner Path [%d] ----\n", print_count++);
        std::vector<sbmpo::Node::Ptr> node_path = results.node_path();
        std::vector<sbmpo::State> state_path = results.state_path();
        std::vector<sbmpo::Control> control_path = results.control_path();
        for (size_t n = 0; n < node_path.size(); n++) {
            sbmpo::Node::Ptr node = node_path[n];
            printf(" (%d) ", node->generation());
            printf("state: [");
            for (float s : state_path[n])
                printf(" %.3f", s);
            printf(" ], control: [");
            if (n != node_path.size() - 1)
                for (float c : control_path[n])
                    printf(" %.3f", c);
            printf("], g: %.3f, rhs: %.2f, f: %.3f\n", node->g(), node->rhs(), node->f());
        }
        printf("--------\n");
    }

    // Print statistics
    void print_stats(const unsigned long timeUs, const int exitCode, const int iterations, const float cost, const int bufferSize, const float successRate) {
        printf("---- Planner Stats ----\n");
        printf("  Time: %lu us\n", timeUs);
        printf("  Exit Code: %d\n", exitCode);
        printf("  Iterations: %d\n", iterations);
        printf("  Cost: %.2f\n", cost);
        printf("  Buffer Size: %d\n", bufferSize);
        printf("  Success Rate: %.1f\n", successRate * 100);
        printf("--------\n");
    }

};

}

#endif