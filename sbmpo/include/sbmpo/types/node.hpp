#ifndef SBMPO_TYPE_NODE_HPP
#define SBMPO_TYPE_NODE_HPP

#include <sbmpo/types/types.hpp>

#include <memory>
#include <limits>

namespace sbmpo {

class Node : std::enable_shared_from_this<Node> {

    public:

    /// @brief Create a new Node
    /// @param state State of the Node
    Node(const State state) {
        this->state_ = state;
        this->rhs_ = std::numeric_limits<float>::infinity();
        this->gval_ = std::numeric_limits<float>::infinity();
        this->fval_ = std::numeric_limits<float>::infinity();
        this->gen_ = 0;
    }

    /// @brief Add a Node as a parent
    /// @param parent_node Pointer to Node to add as parent
    void link_to(std::shared_ptr<Node> parent_node, Control control) {
        this->parents_.push_back({parent_node, control});
        parent_node->children_.push_back(shared_from_this());
        if (parent_node->gen_ + 1 < this->gen_)
            this->gen_ = parent_node->gen_ + 1;
    }

    /// @brief Get Node state
    /// @return Reference to the state of the Node
    State& state() { return state_; }

    /// @brief Get the parents of this Node
    /// @return Reference vector of pointers to parent Nodes 
    std::vector<std::pair<std::shared_ptr<Node>, Control>> &parents() { return parents_; }

    /// @brief Get the children of this Node
    /// @return Reference vector of pointers to children Nodes
    std::vector<std::shared_ptr<Node>> &children() { return children_; }

    /// @brief Get the f value of the Node
    /// @return Reference to F value float
    float &f() { return fval_; }

    /// @brief Get the f value of the Node
    /// @return Reference to F value float
    float &g() { return gval_; }

    /// @brief Get the f value of the Node
    /// @return Reference to F value float
    float &rhs() { return rhs_; }

    /// @brief Get the generation of the Node
    /// @return Reference to the generation
    unsigned long &generation() { return gen_; }

    private:

    // Node state
    State state_;
    
    // Node linkages
    std::vector<std::pair<std::shared_ptr<Node>, Control>> parents_;
    std::vector<std::shared_ptr<Node>> children_;

    // Node properties
    float fval_, gval_, rhs_;
    unsigned long gen_;

};

}


#endif