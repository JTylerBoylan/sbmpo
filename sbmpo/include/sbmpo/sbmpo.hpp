#ifndef SBMPO_HPP
#define SBMPO_HPP

#include <sbmpo/model.hpp>
#include <ctime>

namespace sbmpo {

    class SBMPO {

        public:

            static SBMPORun run(Model &model, const Parameters &parameters);

        private:

            SBMPO() {}

            static void generate_children(Model &model, const Vertex &vertex, const std::vector<Control> samples, const float sample_time, Graph &graph, ImplicitGrid &grid);

            static void update_vertex(Model &model, Vertex &vertex, Graph &graph, Queue &queue);

            static bool generate_path(int best, SBMPOResults &results, Graph &graph);
        
    };

}

#endif