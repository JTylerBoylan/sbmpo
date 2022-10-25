#include <sbmpo/sbmpo.hpp>

namespace sbmpo {

    void run(Model& model, const PlannerParameters &parameters, PlannerResults& results) {

        std::clock_t cstart = std::clock();

        // Initialize planner
        initialize(results, parameters);

        // Set comparator function
        const std::function<bool(Index,Index)> comp = [&](Index a, Index b) {
            return results.buffer[a].heuristic.f > results.buffer[b].heuristic.f;
        };

        // Initialize queue
        NodeQueue queue(comp);

        // Initialize implicit grid
        IndexKeyMap implicit_grid;

        // Iteration
        Index next = 0;
        for (int iter = 0; iter < parameters.max_iterations; iter++) {

            // Get best node
            Node& node = results.buffer[next];

            // Goal check
            results.exit_code = GOAL_REACHED;
            if (model.is_goal(node.state, parameters.conditions.goal_state, parameters.conditions.goal_threshold))
                break;

            // Generation check
            results.exit_code = GENERATION_LIMIT;
            if (node.lineage.generation >= parameters.max_generations)
                break;

            // Sampling
            for (int n = 0; n < parameters.branchout.size(); n++) {

                // Initialize new node
                const Index idx = results.high + n + 1;
                Node& child = results.buffer[idx];
                child.lineage.id = idx;
                child.lineage.parent = node.lineage.id;
                child.lineage.child = INVALID_INDEX;
                child.lineage.generation = node.lineage.generation + 1;
                child.heuristic.g = node.heuristic.g;
                child.state = node.state;
                child.control = parameters.branchout[n];

                // Evaluate using model
                bool valid = true;
                for (float t = 0; t < parameters.sample_time; t+= parameters.sample_time_increment) {
                    model.next_state(child.state, child.state, child.control, parameters.sample_time_increment);
                    if (model.is_valid(child.state))
                        continue;
                    valid = false;
                    break;
                }

                if (!valid)
                    continue;

                // Calculate heuristics
                child.heuristic.g += model.cost(child.state, node.state, child.control, parameters.sample_time_increment);
                child.heuristic.f = child.heuristic.g + model.heuristic(child.state, parameters.conditions.goal_state);

                // Get location on implicit grid
                Index& grid_node_index = toNodeIndex(child, parameters.grid_parameters, implicit_grid);

                if (grid_node_index != INVALID_INDEX) {

                    // If there is a node on implicit grid, compare with the child
                    Node &grid_node = results.buffer[grid_node_index];

                    // If the child node has a lower g score than the existing one, swap with the existing node
                    const float diff = child.heuristic.g - grid_node.heuristic.g;
                    if (diff < 0) {
                        child.lineage.child = grid_node.lineage.child;
                        Node temp = grid_node;
                        grid_node = child;
                        child = temp;
                        // Propogate difference in g to child nodes
                        updateSuccessors(grid_node, results.buffer, parameters.branchout.size(), diff, node.lineage.id);
                    }

                    // No need to add node to priority queue because the existing node was already added and
                    // the samples would be identical given they are the same

                } else {

                    // If there is no node on implicit grid, add child node
                    grid_node_index = child.lineage.id;

                    // Add child to queue
                    queue.push(child.lineage.id);

                }
            }

            // Update child
            node.lineage.child = results.high + 1;

            // Update high
            results.high += parameters.branchout.size();

            // Check if queue is empty
            results.exit_code = NO_NODES_LEFT;
            if (queue.empty())
                break;

            // Get new best
            next = queue.top();

            // Remove from queue
            queue.pop();

            // Update best result
            if (results.buffer[next].heuristic.f < results.buffer[results.best].heuristic.f)
                results.best = next;

            results.exit_code = ITERATION_LIMIT;
        }

        // Generate best path
        PlannerPath &path = results.path;
        for (int i = results.best; i != -1; i = results.buffer[i].lineage.parent)
            path.push_back(i);
        std::reverse(path.begin(), path.end());

        results.time_ms = (std::clock() - cstart) / double(CLOCKS_PER_SEC) * 1000.0;
    }

}