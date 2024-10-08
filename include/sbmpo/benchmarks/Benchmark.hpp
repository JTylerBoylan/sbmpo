#ifndef SBMPO_BENCHMARK_HPP_
#define SBMPO_BENCHMARK_HPP_

#include <sbmpo/types/types.hpp>
#include <sbmpo/SBMPO.hpp>
#include <sbmpo/tools/PrintTool.hpp>
#include <sbmpo/tools/CSVTool.hpp>

#define DEFAULT_CSV_FOLDER "/sbmpo_ws/sbmpo/csv/"

namespace sbmpo_benchmarks
{
    using namespace sbmpo;

    const std::string PARAMS_FILE = "config.csv";
    const std::string NODES_FILE = "nodes.csv";
    const std::string STATS_FILE = "stats.csv";

    template <typename ModelType, typename SearchType = DEFAULT_SEARCH_ALGORITHM>
    class Benchmark : public SBMPO<ModelType, SearchType>
    {
        static_assert(std::is_base_of<sbmpo::Model, ModelType>::value, "ModelType must derive from sbmpo::Model");
        static_assert(std::is_base_of<sbmpo::SearchAlgorithm, SearchType>::value, "SearchType must derive from sbmpo::SearchAlgorithm");

    public:
        Benchmark(std::string csv_folder = DEFAULT_CSV_FOLDER)
            : SBMPO<ModelType, SearchType>(),
              csv_folder_(csv_folder),
              verbose_(true),
              runs_per_param_(1),
              index_(1),
              dynamic_sampling_(false)
        {
        }

        void set_folder(std::string folder_path)
        {
            csv_folder_ = folder_path;
        }

        void set_verbose(bool tf)
        {
            verbose_ = tf;
        }

        void set_print_path(bool tf)
        {
            print_path_ = tf;
        }

        void set_print_nodes(bool tf)
        {
            print_nodes_ = tf;
        }

        void set_runs_per_param(int runs_per_param)
        {
            runs_per_param_ = runs_per_param;
        }

        void set_dynamic_sampling(std::function<std::vector<Control>(const State &)> dynamicSamplingFcn)
        {
            dynamic_sampling_ = true;
            dynamicSamplingFcn_ = dynamicSamplingFcn;
        }

        virtual void benchmark()
        {
            // Clear results
            sbmpo_csv::clear_file(csv_folder_ + STATS_FILE);
            sbmpo_csv::clear_file(csv_folder_ + NODES_FILE);

            // Loop through parameter set
            auto param_list = sbmpo_csv::get_params(csv_folder_ + PARAMS_FILE);

            if (dynamic_sampling_)
            {
                for (auto &param : param_list)
                {
                    param.sample_type = sbmpo::ControlSampleType::DYNAMIC;
                    param.getDynamicSamples = dynamicSamplingFcn_;
                }
            }

            printf("Starting benchmarking.\n");
            for (auto param = param_list.cbegin(); param != param_list.cend(); ++param)
            {
                if (!verbose_)
                    printf("Running Parameters [%d]\n", index_);
                this->run(*param);
            }

            printf("Finished benchmarking.\n");
        }

        virtual void run(const SearchParameters &params) override
        {
            if (verbose_)
                sbmpo_io::print_parameters(params, index_);

            SearchResults avg_results;
            for (int r = 0; r < runs_per_param_; r++)
            {
                this->search_->solve(params);
                if (r == 0)
                {
                    avg_results = *this->results();
                }
                else
                {
                    avg_results.time_us += this->results_->time_us;
                    avg_results.success_rate += this->results_->success_rate;
                }
            }
            avg_results.time_us /= runs_per_param_;
            avg_results.success_rate /= runs_per_param_;

            if (verbose_)
                sbmpo_io::print_results(&avg_results, index_);
            if (verbose_)
                sbmpo_io::print_stats(&avg_results, index_);

            if (verbose_)
                printf("Writing results in folder %s ...\n", csv_folder_.c_str());
            sbmpo_csv::append_stats(csv_folder_ + STATS_FILE, avg_results);
            if (print_path_)
                sbmpo_csv::append_node_path(csv_folder_ + NODES_FILE, avg_results.node_path);
            if (print_nodes_)
                sbmpo_csv::append_nodes(csv_folder_ + NODES_FILE, avg_results.nodes);
            if (verbose_)
                printf("\n");
            index_++;
        }

    protected:
        std::string csv_folder_;
        bool verbose_;
        bool print_path_;
        bool print_nodes_;
        uint16_t runs_per_param_;
        uint16_t index_;
        bool dynamic_sampling_;
        std::function<std::vector<Control>(const State &)> dynamicSamplingFcn_;
    };

}

#endif