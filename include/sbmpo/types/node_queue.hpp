#ifndef SBMPO_TYPE_QUEUE_HPP_
#define SBMPO_TYPE_QUEUE_HPP_

#include <sbmpo/types/node.hpp>

#include <algorithm>

namespace sbmpo {

class NodeQueue {
public:

    /// @brief Create a new Node Queue
    /// @param size Maximum nodes in the queue
    NodeQueue(int size) : heap_(size), index_map_() {}

    /// @brief Insert a node into the queue
    /// @param node Node to be inserted
    void insert(const Node::Ptr& node) {
        heap_.push_back(node);
        size_t index = heap_.size() - 1;
        index_map_[node] = index;
        push_heap(heap_.begin(), heap_.end(), NodeComparator());
    }

    /// @brief Remove a node from queue
    /// @param node Node to be removed
    void remove(const Node::Ptr& node) {
        auto it = index_map_.find(node);
        if (it == index_map_.end())
            return;
        size_t index = it->second;
        index_map_.erase(it);
        if (index == heap_.size() - 1) {
            heap_.pop_back();
            return;
        }
        Node::Ptr last_node = heap_.back();
        heap_[index] = last_node;
        index_map_[last_node] = index;
        heap_.pop_back();
        int parent = (index - 1) / 2;
        if (index > 0 && heap_[index]->f() < heap_[parent]->f())
            heapify_up(index);
        else
            heapify_down(index);
    }

    /// @brief Get best node from queue
    /// @return Pointer to Node with best f score
    Node::Ptr pop() {
        Node::Ptr node = heap_.front();
        index_map_.erase(node);
        pop_heap(heap_.begin(), heap_.end(), NodeComparator());
        heap_.pop_back();
        return node;
    }

    /// @brief Check if queue is empty
    /// @return True/False if queue is empty
    bool empty() const { return heap_.empty(); }

    /// @brief Clear the heap
    void clear() {
        heap_.clear();
        index_map_.clear();
    }

private:
    std::vector<Node::Ptr> heap_;
    std::unordered_map<Node::Ptr, size_t> index_map_;

    struct NodeComparator {
        bool operator()(const Node::Ptr& nodeA, const Node::Ptr& nodeB) const {
            return nodeA->f() > nodeB->f();
        }
    };

    void heapify_up(size_t index) {
        Node::Ptr node = heap_[index];
        size_t parent = (index - 1) / 2;
        while (index > 0 && node->f() < heap_[parent]->f()) {
            heap_[index] = heap_[parent];
            index_map_[heap_[index]] = index;
            index = parent;
            parent = (index - 1) / 2;
        }
        heap_[index] = node;
        index_map_[node] = index;
    }

    void heapify_down(size_t index) {
        Node::Ptr node = heap_[index];
        size_t child = 2 * index + 1;
        while (child < heap_.size()) {
            if (child + 1 < heap_.size() && heap_[child + 1]->f() < heap_[child]->f())
                child++;
            if (heap_[child]->f() >= node->f())
                break;
            heap_[index] = heap_[child];
            index_map_[heap_[index]] = index;
            index = child;
            child = 2 * index + 1;
        }
        heap_[index] = node;
        index_map_[node] = index;
    }
};


}

#endif