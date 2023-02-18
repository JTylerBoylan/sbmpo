#ifndef SBMPO_TYPES_HPP
#define SBMPO_TYPES_HPP

#include <vector>
#include <set>
#include <map>
#include <algorithm>
#include <limits>
#include <cmath>
#include <chrono>

namespace sbmpo {

    const int INVALID_INDEX = -1;

    typedef std::vector<float> State;
    typedef std::vector<float> Control;
    typedef std::vector<int> GridKey;

    struct Vertex {
        int idx, gen;
        float f, g, rhs;
        State state;
    };

    struct Edge {
        int idx;
        float cost;
        int vertex1, vertex2;
        Control control;
    };

    struct Graph {

        std::vector<Vertex> vertices;
        std::vector<Edge> edges;
        std::map<int, std::set<int>> link_back;
        std::map<int, std::set<int>> link_forward;

        Graph() {}

        Graph(int size) {
            vertices.reserve(size);
        }

        Vertex &vertex(int i) {
            return vertices[i];
        }

        Edge &edge(int i) {
            return edges[i];
        }

        size_t size(){
            return vertices.size();
        }

        Vertex &add_vertex(const State &state) {
            Vertex vertex;
            vertex.idx = vertices.size();
            vertex.state = state;
            vertex.rhs = std::numeric_limits<float>::infinity();
            vertex.g = std::numeric_limits<float>::infinity();
            vertices.push_back(vertex);
            return vertices.back();
        }

        Edge &add_edge(const int vertex1, const int vertex2, const Control &control, const float cost) {
            Edge edge;
            edge.idx = edges.size();
            edge.cost = cost;
            edge.vertex1 = vertex1;
            edge.vertex2 = vertex2;
            edge.control = control;
            edges.push_back(edge);
            link_forward[vertex1].insert(edge.idx);
            link_back[vertex2].insert(edge.idx);
            return edges.back();
        };

        std::set<int> getPredecessors(const Vertex &v) {
            return link_back[v.idx];
        }

        std::set<int> getSuccessors(const Vertex &v) {
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

        GridKey to_key(const State &state) {
            GridKey key;
            for (size_t i = 0; i < state.size(); i++)
                if (states[i])
                    key.push_back(floor(state[i]/resolution[i]));
            return key;
        }

        void insert(const State &state, const int idx) {
            GridKey key = to_key(state);
            grid[key] = idx;
        };

        int find(const State &state) {
            GridKey key = to_key(state);
            if (!grid.count(key))
                grid[key] = INVALID_INDEX;
            return grid[key];
        }

    };

    struct Queue {

        std::vector<int> heap;
        Graph * graph;

        Queue() : graph(NULL) {}

        Queue(Graph * grph, int size) : graph(grph) {
            heap.reserve(size);
        }

        bool compare (int a, int b) {
            return graph->vertices[a].f < graph->vertices[b].f;
        }

        void swap(int *a, int *b) {
            int temp = *b;
            *b = *a;
            *a = temp;
        }

        void heapify(int i) {

            int size = heap.size();
            
            int min = i;
            int l = 2 * i + 1;
            int r = 2 * i + 2;
            if (l < size && compare(heap[l], heap[min]))
                min = l;
            if (r < size && compare(heap[r], heap[min]))
                min = r;

            if (min != i) {
                swap(&heap[i], &heap[min]);
                heapify(min);
            }
        }

        void insert(int idx) {
            int size = heap.size();
            if (size == 0) {
                heap.push_back(idx);
            } else {
                heap.push_back(idx);
                for (int i = size / 2 - 1; i >= 0; i--) {
                    heapify(i);
                }
            }
        }

        void remove(int idx) {

            int size = heap.size();

            int i;
            bool found = false;
            for (i = 0; i < size; i++) {
                if (idx == heap[i]) {
                    found = true;
                    break;
                }
            }

            if (!found)
                return;
            
            swap(&heap[i], &heap[size - 1]);

            heap.pop_back();
            for (int i = size / 2 - 1; i >= 0; i--) {
                heapify(i);
            }
        }

        int pop() {
            int top = heap[0];
            heap[0] = heap[heap.size() - 1];
            heap.pop_back();
            heapify(0);
            return top;
        }

        bool empty() {
            return heap.size() == 0;
        }

    };

    enum ExitCode {GOAL_REACHED, ITERATION_LIMIT, GENERATION_LIMIT, NO_NODES_LEFT, INVALID_PATH};

    struct Parameters {
        int max_iterations, max_generations;
        float sample_time;
        std::vector<bool> grid_states;
        std::vector<float> grid_resolution;
        std::vector<Control> samples;
    };

    struct SBMPOResults {
        ExitCode exit_code;
        time_t time_us;
        double cost;
        int iterations;
        std::vector<State> state_path;
        std::vector<Control> control_path;
        std::vector<int> vertex_index_path;
        std::vector<int> edge_index_path;
        SBMPOResults() {
            exit_code = INVALID_PATH;
            time_us = 0;
            cost = 0.0;
            iterations = 0;
        }
    };

    struct SBMPORun {

        Graph graph;
        Queue queue;
        ImplicitGrid grid;
        int best;
        SBMPOResults results;

        SBMPORun() {}

        SBMPORun(Parameters params) {
            int max_size = params.max_iterations*params.samples.size()+1;
            graph = Graph(max_size);
            queue = Queue(&graph, max_size);
            grid = ImplicitGrid(params.grid_states, params.grid_resolution);
            best = 0;
            results.iterations = 0;
        }

        time_t time_us() { return results.time_us; }

        double cost() { return results.cost; }

        size_t size() { return graph.size(); }

        ExitCode exit_code() { return results.exit_code; }

        std::vector<State> state_path() { return results.state_path; }

        std::vector<Control> control_path() { return results.control_path; }


    };

}

#endif
