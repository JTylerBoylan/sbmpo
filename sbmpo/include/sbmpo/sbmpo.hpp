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

            void initialize(const Parameters &parameters);

            int run(Model &model, const Parameters &parameters);

            std::vector<int> path();

        private:

            int best, high;

            const std::function<bool(int,int)> queue_compare = [&](int a, int b) {
                return graph[a].f > graph[b].f; 
            };

            const void generate_children(Vertex &vertex, Model &model);

            const void update_vertex(Vertex &vertex, Model &model);
        
    };

}

#endif