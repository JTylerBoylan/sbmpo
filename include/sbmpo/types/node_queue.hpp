#ifndef SBMPO_TYPE_QUEUE_HPP_
#define SBMPO_TYPE_QUEUE_HPP_

#include <set>
#include <memory>
#include <unordered_map>

#include <sbmpo/types/node.hpp>

namespace sbmpo {

class NodeQueue {
public:

    /// @brief Insert a node into the queue
    /// @param node Node to be inserted
    void insert(const Node::Ptr& node) {
        auto it = nodes_.insert(node);
        index_map_[node] = it;
    }

    /// @brief Remove a node from the queue
    /// @param node Node to be removed
    void remove(const Node::Ptr& node) {
        auto it = index_map_.find(node);
        if (it != index_map_.end()) {
            nodes_.erase(it->second);
            index_map_.erase(it);
        }
    }

    /// @brief Get the best node from the queue
    /// @return Pointer to Node with the best f score
    Node::Ptr pop() {
        if (nodes_.empty()) {
            return nullptr;
        }

        Node::Ptr node = *nodes_.begin();
        nodes_.erase(nodes_.begin());
        index_map_.erase(node);
        return node;
    }

    /// @brief Check if the queue is empty
    /// @return True if the queue is empty
    bool empty() const {
        return nodes_.empty();
    }

    /// @brief Clear the queue
    void clear() {
        nodes_.clear();
        index_map_.clear();
    }

private:
    struct NodeComparator {
        bool operator()(const Node::Ptr& nodeA, const Node::Ptr& nodeB) const {
            if (nodeA->f() == nodeB->f()) {
                return nodeA.get() < nodeB.get(); // Ensure unique ordering
            }
            return nodeA->f() < nodeB->f();
        }
    };

    std::multiset<Node::Ptr, NodeComparator> nodes_;
    std::unordered_map<Node::Ptr, std::multiset<Node::Ptr, NodeComparator>::iterator> index_map_;
};

} // namespace sbmpo


#endif