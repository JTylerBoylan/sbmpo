#ifndef SBMPO_SEARCH_ALGORITHM_HPP_
#define SBMPO_SEARCH_ALGORITHM_HPP_

#include <sbmpo/types/types.hpp>
#include <sbmpo/types/Model.hpp>

namespace sbmpo {

class SearchAlgorithm {

public:

    SearchAlgorithm(const std::shared_ptr<Model> model) : model_(model) {}

    virtual SearchResults solve(const SearchParameters parameters) = 0;

protected:

    const std::shared_ptr<Model> model_;

};

}

#endif