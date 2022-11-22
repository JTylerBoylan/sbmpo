#ifndef SBMPO_TYPES_HPP
#define SBMPO_TYPES_HPP

#include <vector>
#include <set>
#include <map>
#include <algorithm>
#include <limits>
#include <cmath>

namespace sbmpo {

    #define INVALID_INDEX -1

    typedef std::vector<float> State;
    typedef std::vector<float> Control;
    typedef std::vector<int> GridKey;

    struct Vertex {
        int idx, gen;
        float f, g, rhs;
        State state;
        Control control;
    };

    struct Graph {

        std::vector<Vertex> buffer;
        std::map<int, std::set<int>> link_back;
        std::map<int, std::set<int>> link_forward;

        Graph() {}

        Graph(int size) {
            buffer.reserve(size);
        }

        Vertex &operator[](int i) {
            return buffer[i];
        }

        const size_t size(){
            return buffer.size();
        }

        const void insert(Vertex &vertex) {
            buffer.push_back(vertex);
        }

        const void add_edge(const Vertex &v1, const Vertex &v2) {
                link_forward[v1.idx].insert(v2.idx);
                link_back[v2.idx].insert(v1.idx);
        };

        const std::set<int> getPredecessors(const Vertex &v) {
            return link_back[v.idx];
        }

        const std::set<int> getSuccessors(const Vertex &v) {
            return link_forward[v.idx];
        }
    };
    
    struct ImplicitGrid {

        std::map<GridKey, int> grid;
        std::vector<bool> states;
        std::vector<float> resolution;

        ImplicitGrid() {}

        ImplicitGrid(std::vector<bool> _states, std::vector<float> _res) {
            states = _states;
            resolution = _res;
        }

        const GridKey to_key(const State &state) {
            GridKey key;
            for (int i = 0; i < state.size(); i++)
                if (states[i])
                    key.push_back(floor(state[i]/resolution[i]));
            return key;
        }

        const void insert(const State &state, const int idx) {
            GridKey key = to_key(state);
            grid[key] = idx;
        };

        const int find(const State &state) {
            GridKey key = to_key(state);
            if (!grid.count(key))
                grid[key] = INVALID_INDEX;
            return grid[key];
        }

    };

    enum ExitCode {GOAL_REACHED, ITERATION_LIMIT, GENERATION_LIMIT, NO_NODES_LEFT, INVALID_PATH};

    struct Parameters {
        int max_iterations, max_generations;
        float sample_time, sample_time_increment;
        float goal_threshold;
        State initial_state, goal_state;
        Control initial_control;
        std::vector<bool> grid_states;
        std::vector<float> grid_resolution;
        std::vector<Control> samples;
    };

}

#endif