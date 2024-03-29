#ifndef SBMPO_TYPES_HPP_
#define SBMPO_TYPES_HPP_

#include <vector>
#include <memory>

namespace sbmpo {

using State = std::vector<float>;
using Control = std::vector<float>;

class Node;
using NodePtr = std::shared_ptr<Node>;

enum ExitCode {SOLUTION_FOUND, ITERATION_LIMIT, NO_NODES_IN_QUEUE, GENERATION_LIMIT,
                RUNNING, QUIT_SEARCH, TIME_LIMIT, INVALID_START_STATE};

struct SearchParameters {

    uint32_t max_iterations = 100000U;
    uint32_t max_generations = 1000U;
    time_t time_limit_us = 10000000UL; // 10s
    float sample_time = 1.0F;

    std::vector<float> grid_resolution;
    State start_state, goal_state;
    std::vector<Control> samples;

};

struct SearchResults {

    time_t time_us = 0;
    ExitCode exit_code = RUNNING;
    uint32_t iteration = 0;
    size_t node_count = 0;
    float cost = 0.0f;
    float success_rate = 0.0f;
    
    std::vector<NodePtr> node_path;
    std::vector<State> state_path;
    std::vector<Control> control_path;
    std::vector<NodePtr> nodes;

};

}

#endif