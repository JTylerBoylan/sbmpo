#ifndef SBMPO_TYPE_NODE_HPP
#define SBMPO_TYPE_NODE_HPP

#include <vector>
#include <memory>
#include <limits>

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
    static void link_nodes(Node::Ptr parent_node, Node::Ptr child_node, Control control) {
        child_node->parents_.push_back({parent_node, control});
        parent_node->children_.push_back(child_node);
        if (parent_node->gen_ + 1 < child_node->gen_ || child_node->gen_ == 0)
            child_node->gen_ = parent_node->gen_ + 1;
    }

    /// @brief Node constructor
    /// @param state State of the Node
    Node(const State state) {
        this->state_ = state;
        this->rhs_ = std::numeric_limits<float>::infinity();
        this->gval_ = std::numeric_limits<float>::infinity();
        this->fval_ = std::numeric_limits<float>::infinity();
        this->gen_ = 0;
    }

    /// @brief Get Node state
    /// @return Reference to the state of the Node
    State& state() { return state_; }

    /// @brief Get the parents of this Node
    /// @return Reference vector of pointers to parent Nodes 
    std::vector<std::pair<Node::Ptr, Control>> &parents() { return parents_; }

    /// @brief Get the children of this Node
    /// @return Reference vector of pointers to children Nodes
    std::vector<Node::Ptr> &children() { return children_; }

    /// @brief Get the f value of the Node
    /// @return Reference to F value float
    float &f() { return fval_; }

    /// @brief Get the g value of the Node
    /// @return Reference to G value float
    float &g() { return gval_; }

    /// @brief Get the rhs value of the Node
    /// @return Reference to rhs value float
    float &rhs() { return rhs_; }

    /// @brief Get the generation of the Node
    /// @return Reference to the generation
    int &generation() { return gen_; }

    private:

    // Node state
    State state_;
    
    // Node linkages
    std::vector<std::pair<Node::Ptr, Control>> parents_;
    std::vector<Node::Ptr> children_;

    // Node properties
    float fval_, gval_, rhs_;
    int gen_;

};

}


#endif