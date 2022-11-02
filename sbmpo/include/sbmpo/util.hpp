#ifndef SBMPO_UTIL_HPP
#define SBMPO_UTIL_HPP

#include <sbmpo/types.hpp>
#include <math.h>

namespace sbmpo {


    // Initialize starting node
    void startingNode(Node &init, const PlannerParameters &parameters) {
        init.lineage.id = 0;
        init.lineage.parent = -1;
        init.lineage.generation = 0;
        init.heuristic.f = std::numeric_limits<float>::infinity();
        init.heuristic.g = 0.0f;
        init.state = parameters.conditions.initial_state;
        init.control = parameters.conditions.initial_control;
    }

    // Initializes planner
    void initialize(PlannerResults &results, const PlannerParameters &parameters) {
        const int size = parameters.max_iterations*parameters.branchout.size() + 1;
        results.buffer = NodeBuffer(size);
        results.best = 0;
        results.high = 0;
        results.exit_code = NONE;
        startingNode(results.buffer[0], parameters);
    }

    // Deconstruct planner buffer
    void deconstruct(PlannerResults &results) {}

    // Convert state position to implicit grid key
    GridKey toGridKey(const State &state, const ImplicitGridParameters grid_parameters) {
        GridKey key;
        const GridResolution &resolution = grid_parameters.resolution;
        for (int i = 0; i < state.size(); i++)
            if (grid_parameters.active[i])
                key.push_back(floor(state[i]/resolution[i]));
        return key;
    }

    // Convert node to grid index directly
    Index& toNodeIndex(const Node &node, const ImplicitGridParameters grid_parameters, IndexKeyMap &map) {
        const GridKey key = toGridKey(node.state, grid_parameters);
        if (map.count(key))
            return map[key];
        map[key] = INVALID_INDEX;
        return map[key];
    }

    // Update successors
    void updateSuccessors(Node &node, NodeBuffer &buffer, const int sample_size, const float diff, const Index start) {
        Index child_start = node.lineage.child;
        if (child_start != INVALID_INDEX && child_start != start)
            for (int c = 0; c < sample_size; c++) {
                Node& child = buffer[child_start + c];
                child.heuristic.f -= diff;
                updateSuccessors(child, buffer, sample_size, diff, start);
            }
    }

    std::string code2string(PlannerExitCode exit_code) {
        switch (exit_code) {
            case GOAL_REACHED: return "GOAL_REACHED";
            case ITERATION_LIMIT: return "ITERATION_LIMIT";
            case GENERATION_LIMIT: return "GENERATION_LIMIT";
            case NO_NODES_LEFT: return "NO_NODES_LEFT";
            default: return "NONE";
        }
    }

}

#endif