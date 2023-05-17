#ifndef SBMPO_ASTAR_HPP_
#define SBMPO_ASTAR_HPP_

#include <sbmpo/types/types.hpp>
#include <sbmpo/types/SearchAlgorithm.hpp>
#include <sbmpo/grids/ImplicitGrid.hpp>
#include <sbmpo/queues/PriorityQueue.hpp>

namespace sbmpo_algorithms {

using namespace sbmpo;

class Astar : public SearchAlgorithm {

public:

    Astar(const std::shared_ptr<Model> model, std::shared_ptr<SearchResults> results) 
    : SearchAlgorithm(model, results) {}

    void solve(const SearchParameters parameters) override;

    std::shared_ptr<SearchResults> latest() { return results_; }

private:

    SearchParameters params_;
    std::shared_ptr<ImplicitGrid> grid_;
    std::shared_ptr<PriorityQueue> queue_;
    NodePtr start_node_, goal_node_, best_node_;

    void initialize_();

    NodePtr getNeighbor_(const NodePtr node, const Control& control, const float sample_time);

    void updateLineage_(NodePtr child, const NodePtr parent, const Control& control);

    void generatePath_();

};

}

#endif