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

        // Reset best
        best = 0;
    }

    int SBMPO::run(Model &model, const Parameters &params) {

        // Initialize
        initialize(params);

        // Insert start into graph
        int stidx = graph.add_vertex(params.initial_state);
        Vertex &start_vertex = graph.vertex(stidx);
        start_vertex.rhs = 0;
        start_vertex.gen = 0;
        // Insert start into implicit grid
        grid.insert(start_vertex.state, 0);
        // Insert start into priority queue
        queue.insert(0);

        // Calculate heuristic for start node
        start_vertex.f = model.heuristic(start_vertex.state, start_vertex.state);

        // Begin iterations
        for (int i = 0; i < parameters.max_iterations; i++) {

            // Check if queue is empty
            if (queue.empty())
                return NO_NODES_LEFT;

            // Get next best vertex
            best = queue.pop();
            Vertex &v = graph.vertex(best);

            // Check if we are at the goal
            if (model.is_goal(v.state, parameters.goal_state, parameters.goal_threshold))
                return generate_path() ? GOAL_REACHED : INVALID_PATH;

            // Check if max generations is reached
            if (v.gen > parameters.max_generations)
                return GENERATION_LIMIT;

            if (v.g > v.rhs) {
                v.g = v.rhs;
            } else {
                v.g = std::numeric_limits<float>::infinity();
                update_vertex(v, model);
            }

            // Generate children
            generate_children(v, model);

            // Update successors
            for (int suc : graph.getSuccessors(v)) {
                Edge edge = graph.edges[suc];
                update_vertex(graph.vertex(edge.vertex2), model);
            }
            
            // Update current
            update_vertex(v, model);

            // Next iteration
        }

        return ITERATION_LIMIT;
    }

    void SBMPO::generate_children(const Vertex vertex, Model &model) {

        for (Control control : parameters.samples) {
            
            // Evaluate new state
            State new_state = vertex.state;
            model.next_state(new_state, control, parameters.sample_time);

            // Skip if not valid state
            if (!model.is_valid(new_state))
                continue;

            // Get vertex from grid
            int u = grid.find(new_state);

            if (u == INVALID_INDEX) {
            // If grid space is empty, create new vertex
                int vertex2 = graph.add_vertex(new_state);
                Vertex &new_vertex = graph.vertex(vertex2);
                new_vertex.gen = vertex.gen + 1;
                float cost = model.cost(new_vertex.state, vertex.state, control, parameters.sample_time);
                int edge = graph.add_edge(vertex.idx, vertex2, control, cost);
                grid.insert(new_state, vertex2);
            } else {
            // Else add edge from vertex to graph
                if (u == vertex.idx || u == 0) 
                    continue;
                float cost = model.cost(graph.vertex(u).state, vertex.state, control, parameters.sample_time);
                graph.add_edge(vertex.idx, u, control, cost);
            }
        }
    }

    void SBMPO::update_vertex(Vertex &vertex, Model &model) {
        if (vertex.idx == 0)
            return;
        vertex.rhs = std::numeric_limits<float>::infinity();
        for (int pred : graph.getPredecessors(vertex)) {
            Edge edge = graph.edges[pred];
            Vertex vertex_back = graph.vertex(edge.vertex1);
            vertex.rhs = std::min(vertex.rhs, vertex_back.g + edge.cost);
        }
        queue.remove(vertex.idx);
        if (vertex.g != vertex.rhs) {
            vertex.f = std::min(vertex.g, vertex.rhs) + model.heuristic(vertex.state, parameters.goal_state);
            queue.insert(vertex.idx);
        }
    }

    bool SBMPO::generate_path() {
      vertex_path_.clear();
      edge_path_.clear();
      cost_ = 0.0f;

      int current_vertex = best;
      while (true) {
        if (std::count(vertex_path_.begin(), vertex_path_.end(), current_vertex)) {
          return false;
        }

        vertex_path_.push_back(current_vertex);
        Vertex v = graph.vertex(current_vertex);
        std::set<int> predecessors = graph.getPredecessors(v);

        if (predecessors.empty()) {
          break;
        }

        int min_vertex = INT_MAX;
        int min_edge = INT_MAX;
        for (int pred : predecessors) {
          Edge edge = graph.edges[pred];
          if (graph.vertex(edge.vertex1).g < graph.vertex(min_vertex).g) {
            min_vertex = edge.vertex1;
            min_edge = edge.idx;
          }
        }

        edge_path_.push_back(min_edge);
        cost_ += graph.edges[min_edge].cost;
        current_vertex = min_vertex;
      }

      std::reverse(vertex_path_.begin(), vertex_path_.end());
      std::reverse(edge_path_.begin(), edge_path_.end());
      return true;
    }


}
