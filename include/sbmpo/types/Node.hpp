#ifndef SBMPO_NODE_HPP_
#define SBMPO_NODE_HPP_

#include <sbmpo/types/types.hpp>
#include <limits>

namespace sbmpo
{
    struct Node
    {
        const State state;
        float g = std::numeric_limits<float>::infinity(),
              h = std::numeric_limits<float>::infinity(),
              f = std::numeric_limits<float>::infinity();
        uint16_t generation = 0;
        Node *parent = nullptr;
        Control control;

        Node(const State &state)
            : state(state) {}
    };

}

#endif