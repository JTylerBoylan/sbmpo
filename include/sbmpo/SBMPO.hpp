#ifndef SBMPO_HPP_
#define SBMPO_HPP_

#include <sbmpo/types/types.hpp>
#include <sbmpo/types/Model.hpp>
#include <sbmpo/types/SearchAlgorithm.hpp>
#include <sbmpo/algorithms/Astar.hpp>
#include <chrono>

#define DEFAULT_SEARCH_ALGORITHM sbmpo_algorithms::Astar

namespace sbmpo
{
    template <typename ModelType, typename SearchType = DEFAULT_SEARCH_ALGORITHM>
    class SBMPO
    {
        static_assert(std::is_base_of<sbmpo::Model, ModelType>::value, "ModelType must derive from sbmpo::Model");
        static_assert(std::is_base_of<sbmpo::SearchAlgorithm, SearchType>::value, "SearchType must derive from sbmpo::SearchAlgorithm");

    public:
        SBMPO()
            : results_(std::make_unique<SearchResults>()),
              model_(std::make_unique<ModelType>()),
              search_(std::make_unique<SearchType>(model_.get(), results_.get())) {}

        virtual void run(const SearchParameters &parameters)
        {
            search_->solve(parameters);
        }

        SearchResults *results() { return results_.get(); }

        std::vector<State> state_path() { return results_->state_path; };

        std::vector<Control> control_path() { return results_->control_path; }

        std::vector<Node *> node_path() { return results_->node_path; }

        std::vector<Node *> nodes() { return results_->nodes; }

        ModelType *model() { return model_.get(); }

        SearchType *algorithm() { return search_.get(); };

        time_t time_us() { return results_->time_us; }

        uint16_t iterations() { return results_->iteration; }

        ExitCode exit_code() { return results_->exit_code; }

        float cost() { return results_->cost; }

        size_t size() { return results_->node_count; }

        void quit() { results_->exit_code = sbmpo::QUIT_SEARCH; }

    protected:
        std::unique_ptr<SearchResults> results_;
        std::unique_ptr<ModelType> model_;
        std::unique_ptr<SearchType> search_;
    };

}

#endif