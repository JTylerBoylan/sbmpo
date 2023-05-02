#include <sbmpo/sbmpo.hpp>

#include <chrono>
#include <execution>
#include <sbmpo/types/heaps/priority_heap.hpp>

namespace sbmpo {

SBMPO::SBMPO(Model& model, const SBMPOParameters& params) {

    model_ = &model;
    parameters_ = params;

    implicit_grid_ = std::make_shared<ImplicitGrid>(std::move(params.grid_resolution));

    node_queue_ = std::make_shared<PriorityHeap>();

}

void SBMPO::run() noexcept {

    // Start clock
    using namespace std::chrono;
    high_resolution_clock::time_point clock_start = high_resolution_clock::now();

    // Initialize run
    initialize();

    // Iteration Loop
    while (true) {

        // Max iterations check
        if (iterations_ >= parameters_.max_iterations) {
            exit_code_ = ITERATION_LIMIT;
            break;
        }

        // Empty queue check
        if (node_queue_->empty()) {
            exit_code_ = NO_NODES_LEFT;
            break;
        }

        // Get best node
        next_node_ = node_queue_->pop();

        // Goal check
        if (next_node_ == goal_node_ || model_->is_goal(next_node_->state(), parameters_.goal_state)) {
            exit_code_ = GOAL_REACHED;
            best_node_ = next_node_;
            break;
        }

        // Generation check
        if (next_node_->generation() > parameters_.max_generations) {
            exit_code_ = MAX_GENERATIONS;
            best_node_ = next_node_;
            break;
        }

        // Best H score check
        if (next_node_->h() < best_node_->h())
            best_node_ = next_node_;

        // Generate children from best node
        generate_children(next_node_);

        // Next iteration
        ++iterations_;
    }

    // Generate state and control paths
    if(!generate_path())
        exit_code_ = INVALID_PATH;

    // Set plan cost
    cost_ = best_node_->g();

    // End timer
    high_resolution_clock::time_point clock_end = high_resolution_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(clock_end - clock_start);
    time_us_ = long(time_span.count() * 1E6);

}

void SBMPO::reset() noexcept {
    exit_code_ = UNKNOWN_ERROR;
    iterations_ = 0;
    time_us_ = 0;
    cost_ = 0.0f;
    start_node_ = nullptr;
    goal_node_ = nullptr;
    next_node_ = nullptr;
    best_node_ = nullptr;
    implicit_grid_->clear();
    node_queue_->clear();
    node_path_.clear();
    state_path_.clear();
    control_path_.clear();
}

void SBMPO::initialize() noexcept {
    iterations_ = 0;
    exit_code_ = UNKNOWN_ERROR;
    cost_ = 0.0f;

    start_node_ = implicit_grid_->get(parameters_.start_state);
    start_node_->g() = 0;
    start_node_->f() = model_->heuristic(start_node_->state(), parameters_.goal_state);

    node_queue_->insert(start_node_);

    goal_node_ = implicit_grid_->get(parameters_.goal_state);

    best_node_ = start_node_;
}

void SBMPO::generate_children(const Node::Ptr parent_node) noexcept {

    auto generate_child = [&](const Control& control) {

        // Evaluate control on a state
        State new_state = model_->next_state(parent_node->state(), control, parameters_.sample_time);

        // Skip if invalid state
        if (!model_->is_valid(new_state)) {
            return;
        }

        // Pull node from implicit grid
        Node::Ptr child_node = implicit_grid_->get(new_state);

        // Skip if landed on same node or start node
        if (child_node == parent_node || child_node == start_node_) {
            return;
        }

        // Calculate cost between nodes
        const float cost = model_->cost(child_node->state(), control, parameters_.sample_time);

        // Create link between parent and child 
        Node::link_nodes(parent_node, child_node, control, cost);

        // Check if the node is new
        if (child_node->f() == std::numeric_limits<float>::infinity()) {

            // Calculate F score
            const float heuristic = model_->heuristic(child_node->state(), parameters_.goal_state);
            child_node->h() = heuristic;
            child_node->f() = child_node->g() + heuristic;
            child_node->generation() = parent_node->generation() + 1;

            // Add new node to queue
            node_queue_->insert(child_node);

        }

        // End of sample
    };

    std::for_each(std::execution::par_unseq, parameters_.samples.cbegin(), parameters_.samples.cend(), generate_child);
}

bool SBMPO::generate_path() noexcept {

    size_t path_size = best_node_->generation() + 1;
    node_path_ = std::vector<Node::Ptr>(path_size);
    state_path_ = std::vector<State>(path_size);
    control_path_ = std::vector<Control>(path_size - 1);

    Node::Ptr node = best_node_;
    for (int idx = path_size - 1; idx >= 0; idx--) {
        node_path_[idx] = node;
        state_path_[idx] = node->state();
        if (idx != 0)
            control_path_[idx-1] = node->parent().second;
        node = node->parent().first;
    }

    return true;
}

}