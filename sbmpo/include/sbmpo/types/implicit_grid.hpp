#ifndef SBMPO_TYPE_IMPLICITGRID_HPP
#define SBMPO_TYPE_IMPLICITGRID_HPP

#include <sbmpo/types/node.hpp>

#include <map>
#include <math.h>

namespace sbmpo {

class ImplicitGrid {

    public:

    /// @brief Create a new Implicit Grid
    /// @param grid_resolutions Grid resolutions of gridded states
    ImplicitGrid(std::vector<float> grid_resolutions) {
        this->grid_resolutions_ = grid_resolutions;
    }

    /// @brief Get Node pointer from State position
    /// @param state State of the Node
    /// @return Existing Node on Implicit Grid or a new Node
    std::shared_ptr<Node> get(const State &state) {
        GridKey key = to_key_(state);
        if (node_map_.count(key))
            return node_map_[key];
        std::shared_ptr<Node> new_node = std::make_shared<Node>(state);
        node_map_[key] = new_node;
        return new_node;
    }

    private:

    std::vector<float> grid_resolutions_;

    std::map<GridKey, std::shared_ptr<Node>> node_map_;

    // Convert State to a GridKey
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