#include <sbmpo/sbmpo.hpp>

#include <chrono>

namespace sbmpo {

SBMPO::SBMPO(Model &model, const SBMPOParameters &params) {

    this->model_ = &model;
    this->parameters_ = params;

    this->implicit_grid_ = std::make_shared<ImplicitGrid>(params.grid_resolution);

    int max_size = params.max_iterations*params.samples.size() + 1;
    this->node_queue_ = std::make_shared<NodeQueue>(max_size);

}

void SBMPO::run() {

    // Start clock
    using namespace std::chrono;
    high_resolution_clock::time_point clock_start = high_resolution_clock::now();

    // Initialize run
    this->initialize();

    // Iteration Loop
    while (true) {

        // Max iterations check
        if (iterations_++ > parameters_.max_iterations) {
            exit_code_ = ITERATION_LIMIT;
            break;
        }

        // Empty queue check
        if (node_queue_->empty()) {
            exit_code_ = NO_NODES_LEFT;
            break;
        }

        // Get best node
        std::shared_ptr<Node> best_node = node_queue_->pop();

        // Goal check
        if (model_->is_goal(best_node->state())) {
            exit_code_ = GOAL_REACHED;
            break;
        }

        // Generation check
        if (best_node->generation() > parameters_.max_generations) {
            exit_code_ = GENERATION_LIMIT;
            break;
        }

        // Update vertex if changed
        if (best_node->g() > best_node->rhs()) {
            best_node->g() = float(best_node->rhs());
        } else {
            best_node->g() = std::numeric_limits<float>::infinity();
            this->update_node(best_node);
        }

        // Generate children from best node
        this->generate_children(best_node);

        // Update children
        for (std::shared_ptr<Node> chld : best_node->children())
            this->update_node(chld);

        // Next iteration
    }

    // Generate state and control paths
    if(!this->generate_path())
        exit_code_ = INVALID_PATH;

    // End timer
    high_resolution_clock::time_point clock_end = high_resolution_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(clock_end - clock_start);
    time_us_ = long(time_span.count() * 1E6);

}

void SBMPO::initialize() {
    this->iterations_ = 0;
    this->exit_code_ = UNKNOWN_ERROR;

    start_node_ = implicit_grid_->get(model_->initial_state());
    start_node_->rhs() = 0;
    start_node_->g() = 0;

    node_queue_->insert(start_node_);
}

void SBMPO::generate_children(const std::shared_ptr<Node> parent_node) {

    for (Control control : parameters_.samples) {

        // Evaluate control on a state
        State new_state = model_->next_state(parent_node->state(), control, parameters_.sample_time);

        // Skip if invalid state
        if (model_->is_valid(new_state))
            continue;

        // Pull node from implicit grid
        std::shared_ptr<Node> child_node = implicit_grid_->get(new_state);

        // Skip if landed on same node or start node
        if (child_node == parent_node || child_node == start_node_)
            continue;

        // Add link to parent node
        child_node->link_to(parent_node, control);

    }

}

void SBMPO::update_node(const std::shared_ptr<Node> node) {
    if (node == start_node_)
        return;
    node->rhs() = std::numeric_limits<float>::infinity();
    for (std::pair<std::shared_ptr<Node>, Control> prnt : node->parents())
        node->rhs() = std::min(node->rhs(), prnt.first->g() + model_->cost(prnt.first->state(), prnt.second, parameters_.sample_time));
    node_queue_->remove(node);
    if (node->g() != node->rhs()) {
        node->f() = std::min(node->g(), node->rhs()) + model_->heuristic(node->state());
        node_queue_->insert(node);
    }
}

bool SBMPO::generate_path() {

    this->cost_ = 0.0;
    std::shared_ptr<Node> node = best_node_;
    while (true) {

        if (std::count(node_path_.begin(), node_path_.end(), node))
            return false;

        node_path_.push_back(node);
        state_path_.push_back(node->state());

        std::vector<std::pair<std::shared_ptr<Node>, Control>> parents = node->parents();
        if (parents.empty())
            break;

        std::pair<std::shared_ptr<Node>, Control> min_parent;
        for (std::pair<std::shared_ptr<Node>, Control> prnt : parents)
            if (prnt.first->g() < min_parent.first->g())
                min_parent = prnt;

        control_path_.push_back(min_parent.second);
        cost_ += model_->cost(node->state(), min_parent.second, parameters_.sample_time);
        node = min_parent.first;
    }

    std::reverse(node_path_.begin(), node_path_.end());
    std::reverse(state_path_.begin(), state_path_.end());
    std::reverse(control_path_.begin(), control_path_.end());
    return true;
}

}