#ifndef SBMPO_SEARCH_ALGORITHM_HPP_
#define SBMPO_SEARCH_ALGORITHM_HPP_

#include <sbmpo/types/types.hpp>
#include <sbmpo/types/Model.hpp>

namespace sbmpo
{
    class SearchAlgorithm
    {
    public:
        SearchAlgorithm(Model *model, SearchResults *results)
            : model_(model), results_(results) {}

        virtual void solve(const SearchParameters &parameters) = 0;

    protected:
        Model *model_;
        SearchResults *results_;
    };
}

#endif