#include <sbmpo/queues/PriorityQueue.hpp>

namespace sbmpo {

NodePtr PriorityQueue::pop() {
    std::lock_guard<std::mutex> lock(mutex_);
    NodePtr best_node = node_priority_queue_.top();
    node_priority_queue_.pop();
    return best_node;
}

void PriorityQueue::push(NodePtr node) {
    std::lock_guard<std::mutex> lock(mutex_);
    node_priority_queue_.push(node);
}

bool PriorityQueue::empty() { 
    std::lock_guard<std::mutex> lock(mutex_);
    return node_priority_queue_.empty();
}

}