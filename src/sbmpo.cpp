#include <sbmpo/sbmpo.hpp>

#include <chrono>

namespace sbmpo {

SBMPO::SBMPO(Model &model, const SBMPOParameters &params) {

    model_ = &model;
    parameters_ = params;

    implicit_grid_ = std::make_shared<ImplicitGrid>(params.grid_resolution);

    int max_size = params.max_iterations*params.samples.size() + 1;
    node_queue_ = std::make_shared<NodeQueue>(max_size);

}

void SBMPO::run() {

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
        if (model_->is_goal(next_node_->state(), parameters_.goal_state)) {
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

        // Update vertex if changed
        if (next_node_->g() > next_node_->rhs()) {
            next_node_->g() = float(next_node_->rhs());
        } else {
            next_node_->g() = std::numeric_limits<float>::infinity();
            update_node(next_node_);
        }

        // Generate children from best node
        generate_children(next_node_);

        // Update children
        for (Node::Ptr chld : next_node_->children())
            update_node(chld);

        // Next iteration
        iterations_++;
    }

    // Generate state and control paths
    if(!generate_path())
        exit_code_ = INVALID_PATH;

    // Set plan cost
    cost_ = best_node_->rhs();

    // End timer
    high_resolution_clock::time_point clock_end = high_resolution_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(clock_end - clock_start);
    time_us_ = long(time_span.count() * 1E6);

}

void SBMPO::reset() {
    exit_code_ = UNKNOWN_ERROR;
    iterations_ = 0;
    time_us_ = 0;
    cost_ = 0.0f;
    start_node_ = nullptr;
    next_node_ = nullptr;
    best_node_ = nullptr;
    implicit_grid_->clear();
    node_queue_->clear();
    node_path_.clear();
    state_path_.clear();
    control_path_.clear();
}

void SBMPO::initialize() {
    iterations_ = 0;
    exit_code_ = UNKNOWN_ERROR;
    cost_ = 0.0f;

    start_node_ = implicit_grid_->get(parameters_.start_state);
    start_node_->rhs() = 0;
    start_node_->f() = model_->heuristic(start_node_->state(), parameters_.goal_state);

    node_queue_->insert(start_node_);

    best_node_ = start_node_;
}

void SBMPO::generate_children(const Node::Ptr parent_node) {

    for (Control control : parameters_.samples) {

        // Evaluate control on a state
        State new_state = model_->next_state(parent_node->state(), control, parameters_.sample_time);

        // Skip if invalid state
        if (!model_->is_valid(new_state))
            continue;

        // Pull node from implicit grid
        Node::Ptr child_node = implicit_grid_->get(new_state);

        // Skip if landed on same node or start node
        if (child_node == parent_node || child_node == start_node_)
            continue;

        // Link the nodes
        Node::link_nodes(parent_node, child_node, control);

    }

}

void SBMPO::update_node(const Node::Ptr node) {
    if (node == start_node_)
        return;
    node->rhs() = std::numeric_limits<float>::infinity();
    for (std::pair<Node::Ptr, Control> prnt : node->parents())
        node->rhs() = std::min(node->rhs(), prnt.first->g() + model_->cost(prnt.first->state(), prnt.second, parameters_.sample_time));
    node_queue_->remove(node);
    if (node->g() != node->rhs()) {
        node->h() = node->h() == std::numeric_limits<float>::infinity() ? model_->heuristic(node->state(), parameters_.goal_state) : node->h();
        node->f() = std::min(node->g(), node->rhs()) + node->h();
        node_queue_->insert(node);
    }
    if (node->h() < best_node_->h())
        best_node_ = node;
}

bool SBMPO::generate_path() {

    Node::Ptr node = best_node_;
    while (true) {

        if (std::count(node_path_.begin(), node_path_.end(), node))
            return false;

        node_path_.push_back(node);
        state_path_.push_back(node->state());

        std::vector<std::pair<Node::Ptr, Control>> parents = node->parents();
        if (parents.empty())
            break;

        std::pair<Node::Ptr, Control> min_parent = parents[0];
        for (auto prnt = parents.begin()+1; prnt != parents.end(); ++prnt)
            if (prnt->first->g() < min_parent.first->g())
                min_parent = *prnt;

        control_path_.push_back(min_parent.second);
        node = min_parent.first;
    }

    std::reverse(node_path_.begin(), node_path_.end());
    std::reverse(state_path_.begin(), state_path_.end());
    std::reverse(control_path_.begin(), control_path_.end());
    return true;
}

}