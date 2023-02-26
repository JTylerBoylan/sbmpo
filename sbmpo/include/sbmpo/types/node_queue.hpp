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
        heap.reserve(size);
    }

    /// @brief Insert a node into the queue
    /// @param node Node to be inserted
    void insert(std::shared_ptr<Node> node) {
        int size = heap.size();
        if (size == 0) {
            heap.push_back(node);
        } else {
            heap.push_back(node);
            for (int i = size / 2 - 1; i >= 0; i--) {
                heapify(i);
            }
        }
    }

    /// @brief Remove a node from queue
    /// @param node Node to be removed
    void remove(std::shared_ptr<Node> node) {

        int size = heap.size();

        int i;
        bool found = false;
        for (i = 0; i < size; i++) {
            if (node == heap[i]) {
                found = true;
                break;
            }
        }

        if (!found)
            return;
        
        std::swap(heap[i], heap[size - 1]);

        heap.pop_back();
        for (int i = size / 2 - 1; i >= 0; i--) {
            heapify(i);
        }
    }

    /// @brief Get best node from queue
    /// @return Pointer to Node with best f score
    std::shared_ptr<Node> pop() {
        std::shared_ptr<Node> top = heap[0];
        heap[0] = heap[heap.size() - 1];
        heap.pop_back();
        heapify(0);
        return top;
    }

    /// @brief Check if queue is empty
    /// @return True/False if queue is empty
    bool empty() {
        return heap.size() == 0;
    }

    /// @brief Clear the heap
    void clear() {
        heap.clear();
    }

    private:

    std::vector<std::shared_ptr<Node>> heap;

    bool compare (std::shared_ptr<Node> nodeA, std::shared_ptr<Node> nodeB) {
        return nodeA->f() < nodeB->f();
    }

    void heapify(int i) {

        int size = heap.size();
        
        int min = i;
        int l = 2 * i + 1;
        int r = 2 * i + 2;
        if (l < size && compare(heap[l], heap[min]))
            min = l;
        if (r < size && compare(heap[r], heap[min]))
            min = r;

        if (min != i) {
            std::swap(heap[i], heap[min]);
            heapify(min);
        }
    }

};

}

#endif