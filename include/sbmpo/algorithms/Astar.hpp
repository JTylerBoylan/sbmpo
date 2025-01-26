#ifndef SBMPO_ASTAR_HPP_
#define SBMPO_ASTAR_HPP_

#include <sbmpo/types/types.hpp>
#include <sbmpo/types/SearchAlgorithm.hpp>
#include <sbmpo/grids/ImplicitGrid.hpp>
#include <sbmpo/queues/PriorityQueue.hpp>
#include <unordered_set>

namespace sbmpo_algorithms
{
    using namespace sbmpo;

    class Astar : public SearchAlgorithm
    {
    public:
        Astar(Model *model, SearchResults *results)
            : SearchAlgorithm(model, results) {}

        void solve(const SearchParameters &parameters) override;

        SearchResults *latest() { return results_; }

    private:
        SearchParameters params_;
        std::shared_ptr<ImplicitGrid> grid_;
        std::shared_ptr<PriorityQueue> queue_;
        std::unordered_set<Node *> closed_set_;
        Node *start_node_, *goal_node_, *best_node_;

        void initialize_();

        Node *getNeighbor_(const Node *node, const Control &control);

        void updateLineage_(Node *child, Node *parent, const Control &control);

        void generatePath_();
    };

}

#endif