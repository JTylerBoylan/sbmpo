#ifndef SBMPO_PRINT_TOOL_HPP_
#define SBMPO_PRINT_TOOL_HPP_

#include <sbmpo/sbmpo.hpp>
#include <string>

namespace sbmpo {

class PrintTool {

    public:

    PrintTool() :print_count_(0) {}

    // Print parameters
    void print_parameters(const sbmpo::SBMPOParameters &params) {
        printf("---- Planner Parameters [%d] ----\n", print_count_);
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
    void print_stats(const unsigned long timeUs, const int exitCode, const int iterations, const float cost, const int bufferSize, const float successRate) {
        printf("---- Planner Stats ----\n");
        printf("  Time: %lu us\n", timeUs);
        printf("  Exit Code: %d\n", exitCode);
        printf("  Iterations: %d\n", iterations);
        printf("  Cost: %.2f\n", cost);
        printf("  Buffer Size: %d\n", bufferSize);
        printf("  Success Rate: %.1f\n", successRate * 100);
        printf("--------\n");
    }

    // Print results
    void print_results(sbmpo::SBMPO &results) {
        printf("---- Planner Path [%d] ----\n", print_count_++);
        std::vector<sbmpo::Node::Ptr> node_path = results.node_path();
        std::vector<sbmpo::State> state_path = results.state_path();
        std::vector<sbmpo::Control> control_path = results.control_path();
        for (size_t n = 0; n < node_path.size(); n++) {
            sbmpo::Node::Ptr node = node_path[n];
            printf(" (%d) ", node->generation());
            printf("state: [");
            for (float s : state_path[n])
                printf(" %.3f", s);
            printf(" ], control: [");
            if (n != node_path.size() - 1)
                for (float c : control_path[n])
                    printf(" %.3f", c);
            printf("], g: %.3f, rhs: %.3f, h: %.3f, f: %.3f\n", node->g(), node->rhs(), node->h(), node->f());
        }
        printf("--------\n");
    }

    private:

    int print_count_;

};

}

#endif