#ifndef SBMPO_HPP
#define SBMPO_HPP

#include <sbmpo/model.hpp>

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

            inline int size() { return high; }

            inline float cost() { return graph[best].g; }

        private:

            int best, high;

            std::function<bool(int,int)> queue_compare = [&](int a, int b) {
                return graph[a].f > graph[b].f; 
            };

            const void generate_children(Vertex &vertex, Model &model);

            const void update_vertex(Vertex &vertex, Model &model);
        
    };

}

#endif