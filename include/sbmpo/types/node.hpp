#ifndef SBMPO_TYPE_NODE_HPP
#define SBMPO_TYPE_NODE_HPP

#include <vector>
#include <memory>
#include <limits>
#include <mutex>

namespace sbmpo {

typedef std::vector<float> State;
typedef std::vector<float> Control;

class Node {

    public:

    /// @brief Pointer to a Node
    typedef std::shared_ptr<Node> Ptr;

    /// @brief Link a parent node to a child node by a control
    /// @param parent_node Parent node 
    /// @param child_node Child node
    /// @param control Control of linkage
    static void link_nodes(Node::Ptr parent_node, Node::Ptr child_node, const Control& control, const float cost) noexcept {
        const float new_g = parent_node->gval_ + cost;
        if (new_g < child_node->gval_) {
            child_node->gval_ = new_g;
            child_node->parent_ = {parent_node, control};
        }
        parent_node->add_child_(child_node);
    }

    /// @brief Node constructor
    /// @param state State of the Node
    Node(const State& state) {
        this->state_ = state;
        this->gval_ = std::numeric_limits<float>::infinity();
        this->fval_ = std::numeric_limits<float>::infinity();
        this->hval_ = std::numeric_limits<float>::infinity();
        this->gen_ = 0;
    }

    /// @brief Get Node state
    /// @return Reference to the state of the Node
    State& state() { return state_; }

    /// @brief Get the parent of this Node
    /// @return Reference pointer to parent Nodes and Control pair
    std::pair<Node::Ptr, Control> &parent() { return parent_; }

    /// @brief Get the children of this Node
    /// @return Reference vector of pointers to children Nodes
    std::vector<Node::Ptr> &children() { return children_; }

    /// @brief Get the f value of the Node
    /// @return Reference to F value float
    float &f() { return fval_; }

    /// @brief Get the g value of the Node
    /// @return Reference to G value float
    float &g() { return gval_; }

    /// @brief Get the h value of the Node
    /// @return Reference to h value float
    float &h() { return hval_; }

    /// @brief Get the generation of the Node
    /// @return Reference to the generation
    int &generation() { return gen_; }

    private:

    // Node state
    State state_;
    
    // Node linkages
    std::pair<Node::Ptr, Control> parent_;
    std::vector<Node::Ptr> children_;

    // Node properties
    float gval_, fval_, hval_;
    int gen_;

    std::mutex mutex_;

    void add_child_(Node::Ptr child) {
        std::lock_guard<std::mutex> lock(mutex_);
        children_.push_back(child);
    }

};

}


#endif