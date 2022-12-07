#include <sbmpo/sbmpo.hpp>

namespace sbmpo {

    SBMPO::SBMPO() {}

    void SBMPO::initialize(const Parameters &params) {

        // Copy parameters
        parameters = params;

        int max_size = params.max_iterations*params.samples.size();

        // Create new graph, queue, and grid
        graph = Graph(max_size);
        queue = Queue(&graph, max_size);
        grid = ImplicitGrid(params.grid_states, params.grid_resolution);

        // Reset path
        best = 0;
        path_.clear();
    }

    int SBMPO::run(Model &model, const Parameters &params) {

        // Start clock
        std::clock_t cstart = std::clock();

        // Initialize
        initialize(params);

        // Insert start into graph
        graph.add_vertex(params.initial_state);
        graph[0].rhs = 0;
        graph[0].gen = 0;
        // Insert start into implicit grid
        grid.insert(graph[0].state, 0);
        // Insert start into priority queue
        queue.insert(0);

        // Calculate heuristic for start node
        graph[0].f = model.heuristic(graph[0].state, graph[0].state);

        // Begin iterations
        for (int i = 0; i < parameters.max_iterations; i++) {

            // Check if queue is empty
            if (queue.empty())
                return NO_NODES_LEFT;

            // Get next best vertex
            best = queue.pop();
            const Vertex v = graph[best];

            // Check if we are at the goal
            if (model.is_goal(v.state, parameters.goal_state, parameters.goal_threshold))
                return generate_path() ? GOAL_REACHED : INVALID_PATH;

            // Check if max generations is reached
            if (v.gen > parameters.max_generations)
                return GENERATION_LIMIT;

            if (v.g > v.rhs) {
                graph[best].g = v.rhs;
            } else {
                graph[best].g = INFINITY;
                update_vertex(graph[best], model);
            }

            // Generate children
            generate_children(v, model);

            // Update successors
            for (int suc : graph.getSuccessors(v)) {
                Edge edge = graph.edges[suc];
                update_vertex(graph[edge.vertex2], model);
            }
            
            // Update current
            update_vertex(graph[best], model);

            // Next iteration
        }

        return ITERATION_LIMIT;
    } 

    std::vector<int> SBMPO::path() {
        return path_;
    }

    const void SBMPO::generate_children(const Vertex vertex, Model &model) {

        for (Control control : parameters.samples) {
            
            // Evaluate new state
            int n = parameters.sample_time / parameters.sample_time_increment;
            bool invalid = false;
            State new_state;
            for (int i = 1; i <= n; i++) {
                if (!model.next_state(new_state, vertex.state, control, parameters.sample_time_increment * i)) {
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
            // If grid space is empty, create new vertex
                int vertex2 = graph.add_vertex(new_state);
                graph[vertex2].gen = vertex.gen + 1;
                int edge = graph.add_edge(vertex.idx, vertex2, control);
                grid.insert(new_state, vertex2);
            } else {
            // Else add edge from vertex to graph
                if (u != vertex.idx && u != 0)
                    graph.add_edge(vertex.idx, u, control);
            }
        }
    }

    const void SBMPO::update_vertex(Vertex &vertex, Model &model) {
        if (vertex.idx == 0)
            return;
        vertex.rhs = INFINITY;
        for (int pred : graph.getPredecessors(vertex)) {
            Edge edge = graph.edges[pred];
            Vertex vertex_back = graph[edge.vertex1];
            vertex.rhs = std::min(vertex.rhs, vertex_back.g + 
                model.cost(vertex.state, vertex_back.state, edge.control, parameters.sample_time));
        }
        queue.remove(vertex.idx);
        if (vertex.g != vertex.rhs) {
            vertex.f = std::min(vertex.g, vertex.rhs) + model.heuristic(vertex.state, parameters.goal_state);
            queue.insert(vertex.idx);
        }
    }

    const bool SBMPO::generate_path() {
        Vertex v;
        int i = best;
        do {
            if (std::count(path_.begin(), path_.end(), i))
                return false;
            path_.push_back(i);
            v = graph[i];
            std::set<int> predecessors = graph.getPredecessors(v);
            i = graph.edges[*predecessors.begin()].vertex1;
            for (int pred : predecessors) {
                Edge edge = graph.edges[pred];
                if (graph[edge.vertex1].g < graph[i].g)
                    i = edge.vertex1;
            }
        } while (graph.getPredecessors(v).size() > 0);
        std::reverse(path_.begin(), path_.end());
        return true;
    }

}
