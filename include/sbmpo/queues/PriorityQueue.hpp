#ifndef SBMPO_PRIORITY_QUEUE_HPP_
#define SBMPO_PRIORITY_QUEUE_HPP_

#include <sbmpo/types/types.hpp>
#include <sbmpo/types/Node.hpp>
#include <queue>
#include <mutex>

namespace sbmpo {

class PriorityQueue {

public:

    PriorityQueue() {}

    NodePtr pop() {
        std::lock_guard<std::mutex> lock(mutex_);
        NodePtr best_node = node_priority_queue_.top();
        node_priority_queue_.pop();
        return best_node;
    }

    void push(NodePtr node) {
        std::lock_guard<std::mutex> lock(mutex_);
        node_priority_queue_.push(node);
    }

    bool empty() { 
        std::lock_guard<std::mutex> lock(mutex_);
        return node_priority_queue_.empty();
    }

private:

    struct CompareNodePtr {
        bool operator()(const NodePtr& a, const NodePtr& b) const {
            return a->f() > b->f();
        }
    };

    std::priority_queue<NodePtr, std::vector<NodePtr>, CompareNodePtr> node_priority_queue_;
    std::mutex mutex_;


};

}

#endif