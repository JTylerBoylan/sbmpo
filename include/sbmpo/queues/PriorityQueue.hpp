#ifndef SBMPO_PRIORITY_QUEUE_HPP_
#define SBMPO_PRIORITY_QUEUE_HPP_

#include <sbmpo/types/types.hpp>
#include <sbmpo/types/Node.hpp>
#include <queue>

namespace sbmpo
{
    class PriorityQueue
    {
    public:
        PriorityQueue() {}

        Node *pop()
        {
            Node *best_node = node_priority_queue_.top();
            node_priority_queue_.pop();
            return best_node;
        }

        void push(Node *node)
        {
            node_priority_queue_.push(node);
        }

        bool empty()
        {
            return node_priority_queue_.empty();
        }

    private:
        struct CompareNodePtr
        {
            bool operator()(const Node *a, const Node *b) const
            {
                return a->f > b->f;
            }
        };

        std::priority_queue<Node *, std::vector<Node *>, CompareNodePtr> node_priority_queue_;
    };
}

#endif