#ifndef SBMPO_HPP
#define SBMPO_HPP

#include <sbmpo/model.hpp>
#include <sbmpo/queue.hpp>
#include <ctime>

namespace sbmpo {

    class SBMPO {

        public:

            Parameters parameters;

            Graph graph;

            Queue queue;

            ImplicitGrid grid;

            int best;

            SBMPO();

            void initialize(const Parameters &parameters);

            int run(Model &model, const Parameters &parameters);

            int size() { return graph.size(); }

            float cost() { return cost_; }

            std::vector<State> state_path() { return state_path_; }

            std::vector<Control> control_path() { return control_path_; }

            std::vector<int> vertex_path() { return vertex_path_; }

            std::vector<int> edge_path() {return edge_path_; }
            
            Vertex &vertex(int i) { return graph.vertex(i); }

            Edge &edge(int i) { return graph.edge(i); }

        private:

            std::vector<State> state_path_;
            std::vector<Control> control_path_;
            std::vector<int> vertex_path_;
            std::vector<int> edge_path_;
            float cost_;

            void generate_children(const Vertex vertex, Model &model);

            void update_vertex(Vertex &vertex, Model &model);

            bool generate_path();
        
    };

}

#endif