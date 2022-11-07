#ifndef SBMPO_HPP
#define SBMPO_HPP

#include <sbmpo/model.hpp>
#include <sbmpo/queue.hpp>

namespace sbmpo {

    class SBMPO {

        public:

            Parameters parameters;

            Graph graph;

            Queue queue;

            ImplicitGrid grid;

            SBMPO();

            void initialize(const Parameters &parameters);

            int run(Model &model, const Parameters &parameters);

            std::vector<int> path();

            inline int size() { return graph.size(); }

            inline float cost() { return graph[best].g; }

        private:

            int best;

            const void generate_children(const Vertex vertex, Model &model);

            const void update_vertex(Vertex &vertex, Model &model);
        
    };

}

#endif