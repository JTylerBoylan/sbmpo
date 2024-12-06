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
    class SBMPO
    {
    public:
        SBMPO(std::shared_ptr<Model> model, std::unique_ptr<SearchAlgorithm> search = nullptr)
            : model_(model),
              search_(std::move(search)),
              results_(std::make_unique<SearchResults>())
        {
            if (!search_)
                search_ = std::make_unique<DEFAULT_SEARCH_ALGORITHM>(model_.get(), results_.get());
        }

        virtual void run(const SearchParameters &parameters)
        {
            search_->solve(parameters);
        }

        SearchResults *results() { return results_.get(); }

        std::vector<State> state_path() { return results_->state_path; };

        std::vector<Control> control_path() { return results_->control_path; }

        std::vector<Node *> node_path() { return results_->node_path; }

        std::vector<Node *> nodes() { return results_->nodes; }

        Model *model() { return model_.get(); }

        SearchAlgorithm *algorithm() { return search_.get(); };

        time_t time_us() { return results_->time_us; }

        uint16_t iterations() { return results_->iteration; }

        ExitCode exit_code() { return results_->exit_code; }

        float cost() { return results_->cost; }

        size_t size() { return results_->node_count; }

        void quit() { results_->exit_code = sbmpo::QUIT_SEARCH; }

    protected:
        std::shared_ptr<Model> model_;
        std::unique_ptr<SearchAlgorithm> search_;
        std::unique_ptr<SearchResults> results_;
    };

}

#endif