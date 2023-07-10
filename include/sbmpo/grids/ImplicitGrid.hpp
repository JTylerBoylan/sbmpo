#ifndef SBMPO_IMPLICIT_GRID_HPP_
#define SBMPO_IMPLICIT_GRID_HPP_

#include <sbmpo/types/types.hpp>
#include <sbmpo/types/Node.hpp>
#include <unordered_map>
#include <mutex>

namespace sbmpo {

using GridKey = std::vector<int>;

class ImplicitGrid {

public:

    ImplicitGrid(const std::vector<float> grid_resolution) 
    : resolution_(grid_resolution) {}

    NodePtr get(const State& state);

    std::vector<NodePtr> nodes() const;

    std::size_t size() const { return node_map_.size(); }

    void clear() { 
        std::lock_guard<std::mutex> lock(mutex_);
        node_map_.clear(); 
    }

protected:

    struct GridKeyHash {
        std::size_t operator()(const std::vector<int>& v) const {
            std::size_t seed = v.size();
            for (const auto& i : v) {
                seed ^= std::hash<int>()(i) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
            return seed;
        }
    };

    const std::vector<float> resolution_;
    std::unordered_map<GridKey, NodePtr, GridKeyHash> node_map_;
    std::mutex mutex_;

    GridKey state_to_key_(const State &state);

    NodePtr key_to_node_(const GridKey &key);

    State key_to_state_(const GridKey &key, const State &ref_state);

    NodePtr create_node_on_map_(const GridKey &key, const State& state);

};

}

#endif