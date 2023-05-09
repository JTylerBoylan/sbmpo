#ifndef SBMPO_NODE_HPP_
#define SBMPO_NODE_HPP_

#include <sbmpo/types/types.hpp>
#include <limits>

namespace sbmpo {

class Node {

public:

    Node(const State& state) 
    : state_(state) {
        this->gval_ = std::numeric_limits<float>::infinity();
        this->fval_ = std::numeric_limits<float>::infinity();
        this->hval_ = std::numeric_limits<float>::infinity();
        this->gen_ = 0;
        this->parent_ = nullptr;
    }

    const State& state() const { return state_; }

    Control& control() { return control_; }

    float& f() { return fval_; }

    float& g() { return gval_; }

    float& h() { return hval_; }

    uint16_t& generation() { return gen_; }

    NodePtr& parent() { return parent_; }

protected:

    const State state_;

    float gval_, hval_, fval_;

    uint16_t gen_;
    NodePtr parent_;
    Control control_;

};

}

#endif