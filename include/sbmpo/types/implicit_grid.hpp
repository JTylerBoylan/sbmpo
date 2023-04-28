#ifndef SBMPO_TYPE_IMPLICITGRID_HPP
#define SBMPO_TYPE_IMPLICITGRID_HPP

#include <sbmpo/types/node.hpp>

#include <cmath>
#include <functional>
#include <unordered_map>

namespace sbmpo {

class ImplicitGrid {

    using GridKey = std::vector<int>;

    struct GridKeyHash {
        std::size_t operator()(const std::vector<int>& v) const {
            std::size_t seed = v.size();
            for (const auto& i : v) {
                seed ^= std::hash<int>()(i) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
            return seed;
        }
    };

public:

    /// @brief Create a new Implicit Grid
    /// @param grid_resolutions Grid resolutions of gridded states
    explicit ImplicitGrid(std::vector<float> grid_resolutions) noexcept
        : grid_resolutions_(std::move(grid_resolutions)) {}

    /// @brief Get Node pointer from State position
    /// @param state State of the Node
    /// @return Existing Node on Implicit Grid or a new Node
    Node::Ptr get(const State& state) noexcept {
        GridKey key;
        key.reserve(state.size());
        for (std::size_t s = 0; s < state.size(); s++) {
            if (grid_resolutions_[s] != 0.0f) {
                key.push_back(static_cast<int>(std::roundf(state[s] / grid_resolutions_[s])));
            }
        }
        const auto it = node_map_.find(key);
        if (it != node_map_.end()) {
            return it->second;
        }
        const auto node = std::make_shared<Node>(state);
        node_map_.emplace(std::move(key), node);
        return node;
    }

    /// @brief Get the number of nodes on the grid
    /// @return Size of node map
    std::size_t size() const noexcept {
        return node_map_.size();
    }

    /// @brief Get all the nodes on the grid
    /// @return Vector with all node pointers
    std::vector<Node::Ptr> nodes() const noexcept {
        std::vector<Node::Ptr> node_vec;
        node_vec.reserve(node_map_.size());
        for (const auto& pair : node_map_) {
            node_vec.push_back(pair.second);
        }
        return node_vec;
    }

    /// @brief Clear the Implicit Grid
    void clear() noexcept {
        node_map_.clear();
    }

private:

    std::vector<float> grid_resolutions_;
    std::unordered_map<GridKey, Node::Ptr, GridKeyHash> node_map_;

};

}

#endif