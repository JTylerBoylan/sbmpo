#include <sbmpo/sbmpo.hpp>

namespace sbmpo {

    SBMPORun SBMPO::run(Model &model, const Parameters &params) {

        // Start timer
        using namespace std::chrono;
        high_resolution_clock::time_point clock_start = high_resolution_clock::now();

        // Initialize run
        SBMPORun sbmpo_run(params);
        int &best = sbmpo_run.best;
        Graph &graph = sbmpo_run.graph;
        Queue &queue = sbmpo_run.queue;
        ImplicitGrid &grid = sbmpo_run.grid;
        ExitCode &exit_code = sbmpo_run.results.exit_code;

        // Insert start into graph
        Vertex &start_vertex  = graph.add_vertex(model.initial_state());
        start_vertex.rhs = 0;
        start_vertex.gen = 0;
        // Insert start into implicit grid
        grid.insert(start_vertex.state, 0);
        // Insert start into priority queue
        queue.insert(0);

        // Calculate heuristic for start node
        start_vertex.f = model.heuristic(start_vertex.state);

        // Begin iterations
        int &i = sbmpo_run.results.iterations;
        while (true) {

            // Check if iteration limit is reached
            if (i++ >= params.max_iterations) {
                exit_code = ITERATION_LIMIT;
                break;
            }

            // Check if queue is empty
            if (queue.empty()) {
                exit_code = NO_NODES_LEFT;
                break;
            }

            // Get next best vertex
            best = queue.pop();
            Vertex &current_vertex = graph.vertex(best);

            // Check if we are at the goal
            if (model.is_goal(current_vertex.state)) {
                exit_code = GOAL_REACHED;
                break;
            }

            // Check if max generations is reached
            if (current_vertex.gen > params.max_generations) {
                exit_code = GENERATION_LIMIT;
                break;
            }

            if (current_vertex.g > current_vertex.rhs) {
                current_vertex.g = current_vertex.rhs;
            } else {
                current_vertex.g = std::numeric_limits<float>::infinity();
                update_vertex(model, current_vertex, graph, queue);
            }

            // Generate children
            generate_children(model, current_vertex, params.samples, params.sample_time, graph, grid);

            // Update successors
            for (int suc : graph.getSuccessors(current_vertex)) {
                Edge edge = graph.edge(suc);
                update_vertex(model, graph.vertex(edge.vertex2), graph, queue);
            }
            
            // Update current
            update_vertex(model, current_vertex, graph, queue);

            // Next iteration
        }

        // Valid path check
        if (!generate_path(best, sbmpo_run.results, graph) && exit_code == GOAL_REACHED)
            exit_code = INVALID_PATH;

        // End timer
        high_resolution_clock::time_point clock_end = high_resolution_clock::now();
        duration<double> time_span = duration_cast<duration<double>>(clock_end - clock_start);
        sbmpo_run.results.time_us = long(time_span.count() * 1E6);

        // Finish
        return sbmpo_run;
    }

    void SBMPO::generate_children(Model &model, const Vertex &vertex, const std::vector<Control> samples, const float sample_time, Graph &graph, ImplicitGrid &grid) {

        for (Control control : samples) {
            
            // Evaluate new state
            State new_state = vertex.state;
            model.next_state(new_state, control, sample_time);

            // Skip if not valid state
            if (!model.is_valid(new_state))
                continue;

            // Get vertex from grid
            int u = grid.find(new_state);

            if (u == INVALID_INDEX) {
            // If grid space is empty, create new vertex
                Vertex &new_vertex = graph.add_vertex(new_state);
                new_vertex.gen = vertex.gen + 1;
                float cost = model.cost(new_vertex.state, control, sample_time);
                graph.add_edge(vertex.idx, new_vertex.idx, control, cost);
                grid.insert(new_state, new_vertex.idx);
            } else {
            // Else add edge from vertex to graph
                if (u == vertex.idx || u == 0) 
                    continue;
                float cost = model.cost(graph.vertex(u).state, control, sample_time);
                graph.add_edge(vertex.idx, u, control, cost);
            }
        }
    }

    void SBMPO::update_vertex(Model &model, Vertex &vertex, Graph &graph, Queue &queue) {
        if (vertex.idx == 0)
            return;
        vertex.rhs = std::numeric_limits<float>::infinity();
        for (int pred : graph.getPredecessors(vertex)) {
            Edge edge = graph.edge(pred);
            Vertex vertex_back = graph.vertex(edge.vertex1);
            vertex.rhs = std::min(vertex.rhs, vertex_back.g + edge.cost);
        }
        queue.remove(vertex.idx);
        if (vertex.g != vertex.rhs) {
            vertex.f = std::min(vertex.g, vertex.rhs) + model.heuristic(vertex.state);
            queue.insert(vertex.idx);
        }
    }

    bool SBMPO::generate_path(int best, SBMPOResults &results, Graph &graph) {

        results.cost = 0.0;
        int current_vertex = best;
        while (true) {

            if (std::count(results.vertex_index_path.begin(), results.vertex_index_path.end(), current_vertex))
                return false;

            results.vertex_index_path.push_back(current_vertex);
            Vertex v = graph.vertex(current_vertex);
            results.state_path.push_back(v.state);
            std::set<int> predecessors = graph.getPredecessors(v);

            if (predecessors.empty())
                break;

            Edge edge = graph.edge(*(predecessors.begin()));
            int min_edge = edge.idx;
            int min_vertex = edge.vertex1;
            for (int pred : predecessors) {
                edge = graph.edge(pred);
                if (graph.vertex(edge.vertex1).g < graph.vertex(min_vertex).g) {
                    min_vertex = edge.vertex1;
                    min_edge = edge.idx;
                }
            }

            results.edge_index_path.push_back(min_edge);
            Edge e = graph.edge(min_edge);
            results.control_path.push_back(e.control);
            results.cost += e.cost;
            current_vertex = min_vertex;
        }

        std::reverse(results.state_path.begin(), results.state_path.end());
        std::reverse(results.control_path.begin(), results.control_path.end());
        std::reverse(results.vertex_index_path.begin(), results.vertex_index_path.end());
        std::reverse(results.edge_index_path.begin(), results.edge_index_path.end());
        return true;
    }


}
