#ifndef SBMPO_BENCHMARK_HPP_
#define SBMPO_BENCHMARK_HPP_

#include <sbmpo/sbmpo.hpp>
#include <sbmpo/tools/csv_tool.hpp>
#include <sbmpo/tools/print_tool.hpp>

namespace sbmpo {

using namespace sbmpo;

template <typename ModelType>
class Benchmark {
static_assert(std::is_base_of<sbmpo::Model, ModelType>::value, "ModelType must derive from sbmpo::Model");

    public:

    /// @brief Create a new benchmark
    /// @param csv_folder Path to csv workspace folder
    Benchmark(std::string csv_folder) 
        : csv_tool_(csv_folder) {
        model_ = std::make_shared<ModelType>();
        verbose_ = true;
        runs_per_param_ = 1;
    }

    /// @brief Get an instance of the model
    /// @return Pointer to model object
    std::shared_ptr<Model> model() {
        return model_;
    }

    /// @brief Change path to csv workspace folder
    /// @param folder_path Path to workspace folder
    void set_folder(std::string folder_path) {
        csv_tool_.set_save_folder(folder_path);
    }

    /// @brief Change verbose option (default true)
    /// @param tf True if verbose
    void set_verbose(bool tf) {
        verbose_ = tf;
    }

    /// @brief Change number of runs per parameter set
    /// @param runs_per_param Number of runs
    void set_runs_per_param(int runs_per_param) {
        runs_per_param_ = runs_per_param;
    }

    /// @brief Benchmark a model
    virtual void benchmark() {

        csv_tool_.clear_results_file();

        std::vector<SBMPOParameters> parameters_list = csv_tool_.get_params();
        for (auto param = parameters_list.begin(); param != parameters_list.end(); ++param)
            run_param(*param);

        if (verbose_) printf("Finished benchmarking.\n");
    }

    protected:

    std::shared_ptr<Model> model_;

    bool verbose_;
    int runs_per_param_;

    CSVTool csv_tool_;
    PrintTool print_tool_;

    void run_param(SBMPOParameters &param) {

        if (verbose_) print_tool_.print_parameters(param);

        int exit_code = UNKNOWN_ERROR;
        unsigned long time_us = 0.0;
        float cost = 0.0;
        int node_count = 0;
        int success_count = 0;
        int iterations = 0;
        
        sbmpo::SBMPO sbmpo(*model_, param);

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
        float iterations_avg = float(iterations) / runs_per_param_;
        float cost_avg = cost / runs_per_param_;
        float node_count_avg = float(node_count) / runs_per_param_;
        float success_rate = float(success_count) / runs_per_param_;

        if (verbose_) print_tool_.print_stats(time_us_avg, exit_code, iterations_avg, cost_avg, node_count_avg, success_rate);
        if (verbose_) print_tool_.print_results(sbmpo);

        if (verbose_) printf("Writing results in folder %s ...\n", csv_tool_.get_save_folder().c_str());
        csv_tool_.append_stats(time_us_avg, exit_code, iterations_avg, cost_avg, node_count_avg, success_rate);
        csv_tool_.append_node_path(sbmpo.node_path(), sbmpo.control_path());
        csv_tool_.append_nodes(sbmpo.all_nodes());
        printf("\n");
    }

};

}

#endif