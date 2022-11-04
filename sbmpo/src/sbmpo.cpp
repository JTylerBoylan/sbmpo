#include <sbmpo/sbmpo.hpp>

namespace sbmpo {

    void SBMPO::initialize(const Parameters &params) {

        // Copy parameters
        parameters = params;

        // Create new graph, queue, and grid
        graph = Graph();
        queue = Queue(queue_compare);
        grid = ImplicitGrid();

        // Copy starting vertex values
        graph[0].state = params.initial_state;
        graph[0].control = params.initial_control;
        graph[0].g = INFINITY;
        graph[0].rhs = 0;

        // Copy goal vertex values
        graph[1].state = params.goal_state;
        graph[1].g = INFINITY;
        graph[1].rhs = INFINITY;

        // Initialize grid
        grid.states = params.grid_states;
        grid.resolution = params.grid_resolution;

        // Insert start node in implicit grid
        grid.insert(params.initial_state, 0);

        // Add start node to priority queue
        queue.push(0);
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
            best = queue.top();
            Vertex& v = graph[best];

            // Check if we are at the goal
            if (model.is_goal(v.state, graph[1].state, parameters.goal_threshold))
                return GOAL_REACHED;

            // Check if max generations is reached
            if (v.gen > parameters.max_generations)
                return GENERATION_LIMIT;

            // Generate children
            generate_children(v, model);

            if (v.g > v.rhs) {
                v.g = v.rhs;
                for (int suc : v.successors)
                    update_vertex(graph[suc], model);
            } else {
                v.g = INFINITY;
                update_vertex(v, model);
                for (int suc : v.successors)
                    update_vertex(graph[suc], model);
            }

            // Next iteration
        }

        return ITERATION_LIMIT;
    } 

    std::vector<int> SBMPO::path() {
        std::vector<int> path;
        return path;
    }

    const void SBMPO::generate_children(Vertex &vertex, Model &model) {

        for (Control control : parameters.samples) {
            
            // Create new vertex
            Vertex child = vertex;
            child.control = control;

            // Evaluate vertex in model
            bool invalid = false;
            for (int t = 0; t < parameters.sample_time; t+= parameters.sample_time_increment) {
                model.next_state(child.state, child.state, child.control, parameters.sample_time_increment);
                if (!model.is_valid(child.state)) {
                    invalid = true;
                    break;
                }
            }

            // Skip if not valid state
            if (invalid)
                continue;

            // Get vertex from grid
            int u = grid.find(child.state);

            if (u == INVALID_INDEX) {
            // If grid space is empty, insert child state
                child.idx = graph.size();
                child.rhs = INFINITY;
                child.g = INFINITY;
                graph.insert(child);
                graph.add_edge(vertex, child);
                grid.insert(child.state, child.idx);
            } else {
            // Else add edge from vertex to graph
                graph.add_edge(vertex, graph[u]);
            }
        }
    }

    const void SBMPO::update_vertex(Vertex &vertex, Model &model) {
        if (vertex.idx != 0) {
            vertex.rhs = INFINITY;
            for (int pred : vertex.predecessors)
                vertex.rhs = std::min(vertex.rhs, graph[pred].g + 
                    model.cost(vertex.state, graph[pred].state, vertex.control, parameters.sample_time));
        }
        queue.remove(vertex.idx);
        if (vertex.g != vertex.rhs) {
            vertex.f = std::min(vertex.g, vertex.rhs) + model.heuristic(vertex.state, graph[1].state);
            queue.push(vertex.idx);
        }
    }

}