#include <sbmpo/grids/ImplicitGrid.hpp>
#include <math.h>

namespace sbmpo {

NodePtr ImplicitGrid::get(const State& state) {
    GridKey key = state_to_key_(state);
    NodePtr node = key_to_node_(key);
    if (!node) {
        node = create_node_on_map_(key, state);
    }
    return node;
}

std::vector<NodePtr> ImplicitGrid::nodes() const {
        std::vector<NodePtr> node_vec;
        node_vec.reserve(node_map_.size());
        for (const auto& pair : node_map_) {
            node_vec.emplace_back(pair.second);
        }
        return node_vec;
    }

GridKey ImplicitGrid::state_to_key_(const State &state) {
    GridKey key(state.size());
    for (std::size_t s = 0; s < state.size(); s++) {
        if (resolution_[s] > 0.0f) {
            key[s] = static_cast<int>(std::round(state[s] / resolution_[s]));
        }
    }
    return key;
}

NodePtr ImplicitGrid::key_to_node_(const GridKey &key) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = node_map_.find(key);
    if (it != node_map_.end()) {
        return it->second;
    }
    return nullptr;
}

State ImplicitGrid::key_to_state_(const GridKey &key, const State &ref_state) {
    State new_state(ref_state.size());
    for (std::size_t k  = 0; k < key.size(); k++) {
        if (resolution_[k] > 0.0f) {
            new_state[k] = static_cast<float>(key[k]) * resolution_[k];
        } else {
            new_state[k] = ref_state[k];
        }
    }
    return new_state;
}

NodePtr ImplicitGrid::create_node_on_map_(const GridKey &key, const State& state) {
    State key_state = key_to_state_(key, state);
    NodePtr node = std::make_shared<Node>(std::move(key_state));
    std::lock_guard<std::mutex> lock(mutex_);
    node_map_.emplace(std::move(key), node);
    return node;
}

}