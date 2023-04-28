#ifndef SBMPO_TYPE_QUEUE_HPP
#define SBMPO_TYPE_QUEUE_HPP

#include <sbmpo/types/node.hpp>

#include <algorithm>

namespace sbmpo {

class NodeQueue {

    public:

    /// @brief Create a new Node Queue
    /// @param size Maximum nodes in the queue
    NodeQueue(int size) {
        heap_.reserve(size);
    }

    /// @brief Insert a node into the queue
    /// @param node Node to be inserted
    void insert(const Node::Ptr node) {
        heap_.push_back(node);
        std::push_heap(heap_.begin(), heap_.end(), NodeComparator());
    }

    /// @brief Remove a node from queue
    /// @param node Node to be removed
    void remove(const Node::Ptr node) {
        // Find the node in the queue
        auto it = std::find(heap_.begin(), heap_.end(), node);
        if (it == heap_.end())
            return;
        // Remove the node from the queue
        std::swap(*it, heap_.back());
        heap_.pop_back();
        // Rebuild the heap
        std::make_heap(heap_.begin(), heap_.end(), NodeComparator());
    }

    /// @brief Get best node from queue
    /// @return Pointer to Node with best f score
    Node::Ptr pop() {
        Node::Ptr node = heap_.front();
        std::pop_heap(heap_.begin(), heap_.end(), NodeComparator());
        heap_.pop_back();
        return node;
    }

    /// @brief Check if queue is empty
    /// @return True/False if queue is empty
    bool empty() {
        return heap_.empty();
    }

    /// @brief Clear the heap
    void clear() {
        heap_.clear();
    }

    private:

    std::vector<Node::Ptr> heap_;

    // Queue ordering comparator
    struct NodeComparator {
        bool operator()(const Node::Ptr& nodeA, const Node::Ptr& nodeB) {
            return nodeA->f() > nodeB->f();
        }
    };

};

}

#endif