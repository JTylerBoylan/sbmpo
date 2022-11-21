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

            Vertex start, goal;

            int best;

            SBMPO();

            void initialize(const Parameters &parameters);

            int run(Model &model, const Parameters &parameters);

            std::vector<int> path();
            
            inline Vertex vertex(int i) { return graph[i]; }

            inline int size() { return graph.size(); }

            inline float cost() { return graph[best].rhs; }

        private:

            const void generate_children(const Vertex vertex, Model &model);

            const void update_vertex(Vertex &vertex, Model &model);
        
    };

}

#endif