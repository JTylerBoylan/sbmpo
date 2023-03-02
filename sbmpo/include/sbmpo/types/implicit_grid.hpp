#ifndef SBMPO_TYPE_IMPLICITGRID_HPP
#define SBMPO_TYPE_IMPLICITGRID_HPP

#include <sbmpo/types/node.hpp>

#include <cmath>
#include <functional>
#include <unordered_map>

namespace sbmpo {

class ImplicitGrid {

    typedef std::vector<int> GridKey;

    struct GridKeyHash {
        std::size_t operator()(const std::vector<int>& v) const {
            std::size_t seed = v.size();
            for(auto& i : v) {
                seed ^= i + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
            return seed;
        }
    };

    public:

    /// @brief Create a new Implicit Grid
    /// @param grid_resolutions Grid resolutions of gridded states
    ImplicitGrid(std::vector<float> grid_resolutions) {
        this->grid_resolutions_ = grid_resolutions;
    }

    /// @brief Get Node pointer from State position
    /// @param state State of the Node
    /// @return Existing Node on Implicit Grid or a new Node
    Node::Ptr get(const State &state) {
        GridKey key = to_key_(state);
        if (node_map_.count(key))
            return node_map_[key];
        Node::Ptr new_node = std::make_shared<Node>(state);
        node_map_[key] = new_node;
        return new_node;
    }

    /// @brief Get the number of nodes on the grid
    /// @return Size of node map
    size_t size() { return node_map_.size(); }

    /// @brief Get all the nodes on the grid
    /// @return Vector with all node pointers
    std::vector<Node::Ptr> nodes() {
        std::vector<Node::Ptr> node_vec;
        for (auto it = node_map_.begin(); it != node_map_.end(); ++it)
            node_vec.push_back(it->second);
        return node_vec;
    }

    /// @brief Clear the Implicit Grid
    void clear() {
        node_map_.clear();
    }

    private:

    std::vector<float> grid_resolutions_;

    std::unordered_map<GridKey, Node::Ptr, GridKeyHash> node_map_;

    /// Convert State to a GridKey
    GridKey to_key_(const State &state) {
        GridKey key;
        for (size_t s = 0; s < state.size(); s++)
            if (grid_resolutions_[s])
                key.push_back( roundf(state[s] / grid_resolutions_[s]) );
        return key;
    }

};

}

#endif