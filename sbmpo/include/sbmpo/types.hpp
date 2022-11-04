#ifndef SBMPO_TYPES_HPP
#define SBMPO_TYPES_HPP

#include <vector>
#include <set>
#include <queue>
#include <functional>
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
        std::vector<int> predecessors;
        std::vector<int> successors;
        State state;
        Control control;
    };

    struct Graph {

        std::vector<Vertex> buffer;

        Vertex &operator[](int i) {
            return buffer[i];
        }

        const size_t size(){
            return buffer.size();
        }

        const void insert(Vertex &vertex) {
            vertex.idx = buffer.size();
            buffer.push_back(vertex);
        }

        const void add_edge(Vertex &v1, Vertex &v2) {
                v1.successors.push_back(v2.idx);
                v2.predecessors.push_back(v1.idx);
        };
    };

    typedef std::priority_queue<int, std::set<int>, std::function<bool(int,int)>> Queue;
    
    struct ImplicitGrid {

        std::map<GridKey, int> grid;
        std::vector<bool> states;
        std::vector<float> resolution;

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

    enum ExitCode {GOAL_REACHED, ITERATION_LIMIT, GENERATION_LIMIT, NO_NODES_LEFT};

    struct Parameters {
        int max_iterations, max_generations;
        float sample_time, sample_time_increment;
        State initial_state, goal_state;
        Control initial_control;
        float goal_threshold;
        std::vector<Control> samples;
        std::vector<bool> grid_states;
        std::vector<float> grid_resolution;
    };

}

#endif