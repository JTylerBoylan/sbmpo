#ifndef SBMPO_PRIORITY_HEAP_HPP_
#define SBMPO_PRIORITY_HEAP_HPP_

#include <sbmpo/types/node_queue.hpp>
#include <queue>
#include <memory>

namespace sbmpo {

class PriorityHeap : public NodeQueue {

public:

    PriorityHeap() {}

    /// @brief Get the best node from the queue
    /// @return Pointer to Node with the best f score
    Node::Ptr pop() noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        Node::Ptr best_node = node_priority_queue_.top();
        node_priority_queue_.pop();
        return best_node;
    }

    /// @brief Insert a node into the queue
    /// @param node Node to be inserted
    void insert(const Node::Ptr& node) noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        node_priority_queue_.push(node);
    }

    /// @brief Check if the queue is empty
    /// @return True if the queue is empty
    bool empty() noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        return node_priority_queue_.empty();
    }

    /// @brief Clear the queue
    void clear() noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        while (!node_priority_queue_.empty()) {
            node_priority_queue_.pop();
        }
    }

private:

    struct CompareNodePtr {
        bool operator()(const Node::Ptr& a, const Node::Ptr& b) const {
            return a->f() > b->f();
        }
    };

    std::priority_queue<Node::Ptr, std::vector<Node::Ptr>, CompareNodePtr> node_priority_queue_;
    std::mutex mutex_;

};

}

#endif
