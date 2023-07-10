#ifndef SBMPO_PRINT_TOOL_HPP_
#define SBMPO_PRINT_TOOL_HPP_

#include <sbmpo/types/types.hpp>
#include <sbmpo/types/Node.hpp>
#include <string>

namespace sbmpo_io {

// Print parameters
void print_parameters(const sbmpo::SearchParameters params, int index = 0) {
    printf("---- Planner Parameters [%d] ----\n", index);
    printf("Max iterations: %d\n", params.max_iterations);
    printf("Max generations: %d\n", params.max_generations);
    printf("Sample Time: %.2f\n", params.sample_time);;

    std::string st;

    for (float f : params.grid_resolution)
        st += std::to_string(f) + " ";
    printf("Grid Resolution: %s\n", st.c_str());
    st.clear();

    for (float f : params.start_state)
        st += std::to_string(f) + " ";
    printf("Start State: %s\n", st.c_str());
    st.clear();

    for (float f : params.goal_state)
        st += std::to_string(f) + " ";
    printf("Goal State: %s\n", st.c_str());
    st.clear();

    printf("Samples:\n");
    for (sbmpo::Control control : params.samples) {
        for (float f : control)
            st += std::to_string(f) + " ";
        printf("  - %s\n", st.c_str());
        st.clear();
    }
}

// Print statistics
void print_stats(const sbmpo::SearchResults results, int index = 0) {
    printf("---- Planner Stats [%d] ----\n", index);
    printf("  Time: %lu us\n", results.time_us);
    printf("  Exit Code: %d\n", results.exit_code);
    printf("  Iterations: %d\n", results.iteration);
    printf("  Cost: %.2f\n", results.cost);
    printf("  Buffer Size: %lu\n", results.node_count);
    printf("  Success Rate: %.1f\n", results.success_rate * 100);
    printf("--------\n");
}

// Print results
void print_results(sbmpo::SearchResults results, int index = 0) {
    printf("---- Planner Path [%d] ----\n", index);
    std::vector<sbmpo::NodePtr> node_path = results.node_path;
    for (size_t n = 0; n < node_path.size(); n++) {
        sbmpo::NodePtr node = node_path[n];
        printf(" (%d) ", node->generation());
        printf("state: [");
        for (float s : node->state())
            printf(" %.3f", s);
        printf(" ], control: [");
        for (float c : node->control())
            printf(" %.3f", c);
        printf("], g: %.3f, h: %.3f, f: %.3f\n", node->g(), node->h(), node->f());
    }
    printf("--------\n");
}

}

#endif