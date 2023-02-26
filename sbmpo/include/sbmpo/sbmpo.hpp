#ifndef SBMPO_SBMPO_HPP
#define SBMPO_SBMPO_HPP

#include <sbmpo/types/implicit_grid.hpp>
#include <sbmpo/types/node_queue.hpp>
#include <sbmpo/model.hpp>

namespace sbmpo {

enum ExitCode {GOAL_REACHED, ITERATION_LIMIT, GENERATION_LIMIT, NO_NODES_LEFT, INVALID_PATH, UNKNOWN_ERROR};

struct SBMPOParameters {
    int max_iterations, max_generations;
    float sample_time;
    std::vector<float> grid_resolution;
    Branchouts samples;
};

class SBMPO {

    public:

    /// @brief Create a new SBMPO object
    /// @param model Model to run the planner on
    /// @param params SBMPO Parameters for planning
    SBMPO(Model &model, const SBMPOParameters &params);

    /// @brief Run the SBMPO planner
    void run();

    /// @brief Iteration count of planner
    /// @return Iteration value
    unsigned long iterations(){ return iterations_; }

    /// @brief Exit code of the plan run
    /// @return Value of the exit code
    ExitCode exit_code() { return exit_code_; }

    /// @brief Computation time of the plan run
    /// @return Time in microseconds
    time_t time_us() { return time_us_; }

    /// @brief Cost of the latest planner run
    /// @return Cost of the plan
    float cost() { return cost_; }

    /// @brief Get the plan path in terms of Nodes
    /// @return List of Node pointers
    std::vector<std::shared_ptr<Node>> node_path() { return node_path_; }

    /// @brief Get the plan path in terms of States
    /// @return List of States
    std::vector<State> state_path() { return state_path_; }

    /// @brief Get the plan path in terms of Controls
    /// @return List of Controls
    std::vector<Control> control_path() { return control_path_; }

    private:

    Model * model_;
    SBMPOParameters parameters_;

    std::shared_ptr<ImplicitGrid> implicit_grid_;
    std::shared_ptr<NodeQueue> node_queue_;

    std::shared_ptr<Node> start_node_;
    std::shared_ptr<Node> best_node_;

    ExitCode exit_code_;
    unsigned long iterations_;
    time_t time_us_;
    float cost_;

    std::vector<std::shared_ptr<Node>> node_path_;
    std::vector<State> state_path_;
    std::vector<Control> control_path_;

    // Initialize SBMPO run
    void initialize();

    // Generate children from node
    void generate_children(const std::shared_ptr<Node> node);

    // Update node
    void update_node(const std::shared_ptr<Node> node);

    // Generate path
    bool generate_path();

    ~SBMPO() {
        delete model_;
    }

};

}

#endif