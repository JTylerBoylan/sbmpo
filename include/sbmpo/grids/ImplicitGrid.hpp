#ifndef SBMPO_IMPLICIT_GRID_HPP_
#define SBMPO_IMPLICIT_GRID_HPP_

#include <sbmpo/types/types.hpp>
#include <sbmpo/types/Node.hpp>
#include <unordered_map>
#include <mutex>

namespace sbmpo
{
    using GridKey = std::vector<int>;

    class ImplicitGrid
    {
    public:
        ImplicitGrid(const std::vector<float> grid_resolution)
            : resolution_(grid_resolution) {}

        Node *get(const State &state)
        {
            GridKey key = state_to_key_(state);
            Node *node = key_to_node_(key);
            if (!node)
            {
                node = create_node_on_map_(key, state);
            }
            return node;
        }

        std::vector<Node *> nodes() const
        {
            std::vector<Node *> node_vec;
            node_vec.reserve(node_map_.size());
            for (const auto &pair : node_map_)
            {
                node_vec.emplace_back(pair.second.get());
            }
            return node_vec;
        }

        std::size_t size() const { return node_map_.size(); }

        void clear()
        {
            std::lock_guard<std::mutex> lock(mutex_);
            node_map_.clear();
        }

    protected:
        struct GridKeyHash
        {
            std::size_t operator()(const std::vector<int> &v) const
            {
                std::size_t seed = v.size();
                for (const auto &i : v)
                {
                    seed ^= std::hash<int>()(i) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
                }
                return seed;
            }
        };

        const std::vector<float> resolution_;
        std::unordered_map<GridKey, std::unique_ptr<Node>, GridKeyHash> node_map_;
        std::mutex mutex_;

        GridKey state_to_key_(const State &state)
        {
            GridKey key(state.size());
            for (std::size_t s = 0; s < state.size(); s++)
            {
                if (resolution_[s] > 0.0f)
                {
                    key[s] = static_cast<int>(std::round(state[s] / resolution_[s]));
                }
            }
            return key;
        }

        Node *key_to_node_(const GridKey &key)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto it = node_map_.find(key);
            if (it != node_map_.end())
            {
                return it->second.get();
            }
            return nullptr;
        }

        State key_to_state_(const GridKey &key, const State &ref_state)
        {
            State new_state(ref_state.size());
            for (std::size_t k = 0; k < key.size(); k++)
            {
                if (resolution_[k] > 0.0f)
                {
                    new_state[k] = static_cast<float>(key[k]) * resolution_[k];
                }
                else
                {
                    new_state[k] = ref_state[k];
                }
            }
            return new_state;
        }

        Node *create_node_on_map_(const GridKey &key, const State &state)
        {
            State key_state = key_to_state_(key, state);
            std::unique_ptr<Node> node = std::make_unique<Node>(key_state);
            std::lock_guard<std::mutex> lock(mutex_);
            node_map_.emplace(key, std::move(node));
            return node_map_[key].get();
        }
    };

}

#endif