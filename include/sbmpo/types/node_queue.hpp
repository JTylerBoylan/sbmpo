#ifndef SBMPO_TYPE_QUEUE_HPP_
#define SBMPO_TYPE_QUEUE_HPP_

#include <sbmpo/types/node.hpp>

namespace sbmpo {

class NodeQueue {

public:

    NodeQueue() {}

    /// @brief Get the best node from the queue
    /// @return Pointer to Node with the best f score
    virtual Node::Ptr pop() noexcept = 0;

    /// @brief Insert a node into the queue
    /// @param node Node to be inserted
    virtual void insert(const Node::Ptr& node) noexcept = 0;

    /// @brief Check if the queue is empty
    /// @return True if the queue is empty
    virtual bool empty() noexcept = 0;

    /// @brief Clear the queue
    virtual void clear() noexcept = 0;

};

}

#endif