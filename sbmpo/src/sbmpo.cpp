#include <sbmpo/sbmpo.hpp>

namespace sbmpo {

    SBMPO::SBMPO() {}

    void SBMPO::initialize(const Parameters &params) {

        // Copy parameters
        parameters = params;

        // Create new graph, queue, and grid
        graph = Graph();
        queue = Queue(&graph);
        grid = ImplicitGrid();

        // Copy starting vertex values
        Vertex start;
        start.idx = 0;
        start.state = params.initial_state;
        start.control = params.initial_control;
        start.g = INFINITY;
        start.rhs = 0;
        start.gen = 0;
        graph.insert(start);

        // Copy goal vertex values
        Vertex goal;
        goal.idx = 1;
        goal.state = params.goal_state;
        goal.g = INFINITY;
        goal.rhs = INFINITY;
        graph.insert(goal);

        // Initialize grid
        grid.states = params.grid_states;
        grid.resolution = params.grid_resolution;

        // Insert start node in implicit grid
        grid.insert(params.initial_state, 0);

        // Add start node to priority queue
        queue.insert(0);
    }

    int SBMPO::run(Model &model, const Parameters &params) {

        // Initialize
        initialize(params);

        // Calculate heuristic for start node
        graph[0].f = model.heuristic(graph[0].state, graph[1].state);

        // Begin iterations
        for (int i = 0; i < parameters.max_iterations; i++) {

            // Check if queue is empty
            if (queue.empty())
                return NO_NODES_LEFT;

            // Get next best vertex
            best = queue.pop();
            const Vertex v = graph[best];

            // Check if we are at the goal
            if (model.is_goal(v.state, graph[1].state, parameters.goal_threshold))
                return GOAL_REACHED;

            // Check if max generations is reached
            if (v.gen > parameters.max_generations)
                return GENERATION_LIMIT;

            // Generate children
            generate_children(v, model);

            if (v.g > v.rhs) {
                graph[best].g = v.rhs;
                for (int suc : graph.getSuccessors(v))
                    update_vertex(graph[suc], model);
            } else {
                graph[best].g = INFINITY;
                update_vertex(graph[best], model);
                for (int suc : graph.getSuccessors(v))
                    update_vertex(graph[suc], model);
            }

            // Next iteration
        }

        return ITERATION_LIMIT;
    } 

    std::vector<int> SBMPO::path() {
        std::vector<int> path;
        // TODO
        return path;
    }

    const void SBMPO::generate_children(const Vertex vertex, Model &model) {

        for (Control control : parameters.samples) {
            
            // Create new state
            State new_state = vertex.state;

            // Evaluate state in model
            bool invalid = false;
            for (float t = 0.0f; t < parameters.sample_time; t += parameters.sample_time_increment) {
                model.next_state(new_state, new_state, control, parameters.sample_time_increment);
                if (!model.is_valid(new_state)) {
                    invalid = true;
                    break;
                }
            }

            // Skip if not valid state
            if (invalid)
                continue;

            // Get vertex from grid
            int u = grid.find(new_state);

            if (u == INVALID_INDEX) {
            // If grid space is empty, insert child state
                Vertex child;
                child.idx = graph.size();
                child.gen = vertex.gen + 1;
                child.state = new_state;
                child.control = control;
                child.rhs = INFINITY;
                child.g = INFINITY;
                graph.insert(child);
                graph.add_edge(vertex, child);
                grid.insert(child.state, child.idx);
            } else {
            // Else add edge from vertex to graph
                if (u != vertex.idx)
                    graph.add_edge(vertex, graph[u]);
            }
        }
    }

    const void SBMPO::update_vertex(Vertex &vertex, Model &model) {
        if (vertex.idx != 0) {
            vertex.rhs = INFINITY;
            for (int pred : graph.getPredecessors(vertex))
                vertex.rhs = std::min(vertex.rhs, graph[pred].g + 
                    model.cost(vertex.state, graph[pred].state, vertex.control, parameters.sample_time));
        }
        queue.remove(vertex.idx);
        if (vertex.g != vertex.rhs) {
            vertex.f = std::min(vertex.g, vertex.rhs) + model.heuristic(vertex.state, graph[1].state);
            queue.insert(vertex.idx);
        }
    }

}