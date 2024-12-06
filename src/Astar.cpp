#include <sbmpo/algorithms/Astar.hpp>
#include <chrono>

namespace sbmpo_algorithms
{
    using namespace sbmpo;

    void Astar::solve(const SearchParameters parameters)
    {
        // Start clock
        using namespace std::chrono;
        auto clock_start = high_resolution_clock::now();

        params_ = parameters;

        // Initialize run
        initialize_();

        // Check if start state is valid
        if (!model_->is_valid(params_.start_state))
        {
            results_->exit_code = INVALID_START_STATE;
            return;
        }

        // Iteration loop
        while (true)
        {
            // Check for quit command
            if (results_->exit_code == QUIT_SEARCH)
            {
                break;
            }

            // Check if time limit reached
            const float curr_time = time_t(duration_cast<duration<double>>(high_resolution_clock::now() - clock_start).count() * 1E6);
            if (curr_time > parameters.time_limit_us)
            {
                results_->exit_code = TIME_LIMIT;
                break;
            }

            // Check if iteration limit reached
            if (results_->iteration >= params_.max_iterations)
            {
                results_->exit_code = ITERATION_LIMIT;
                break;
            }

            // Check if the queue is empty
            if (queue_->empty())
            {
                results_->exit_code = NO_NODES_IN_QUEUE;
                break;
            }

            // Next best node
            Node *current_node = queue_->pop();

            // Check if closed
            if (closed_set_.find(current_node) != closed_set_.end())
                continue;

            // Add to closed set
            closed_set_.insert(current_node);

            // Check for solution
            if (current_node == goal_node_ || model_->is_goal(current_node->state, params_.goal_state))
            {
                results_->exit_code = SOLUTION_FOUND;
                best_node_ = current_node;
                break;
            }

            // Check if generation limit reached
            if (current_node->generation > params_.max_generations)
            {
                results_->exit_code = GENERATION_LIMIT;
                best_node_ = current_node;
                break;
            }

            // Best H score check
            if (current_node->h < best_node_->h)
                best_node_ = current_node;

            // Get control samples
            std::vector<Control> controls;
            switch (params_.sample_type)
            {
            case FIXED:
                controls = params_.fixed_samples;
                break;
            case DYNAMIC:
                controls = model_->get_dynamic_samples(current_node->state);
                break;
            }

            // Loop through all neighbors of the node
            for (const Control &control : controls)
            {
                // Get neighbor based on control
                Node *neighbor = getNeighbor_(current_node, control);

                // Check if valid neighbor
                if (!neighbor)
                    return;

                // Update if better path found
                const float new_g = current_node->g + model_->cost(current_node->state, neighbor->state, control);
                if (new_g < neighbor->g)
                {
                    neighbor->g = new_g;
                    updateLineage_(neighbor, current_node, control);

                    // Add into queue
                    queue_->push(neighbor);
                }
            }

            // Next iteration
            ++results_->iteration;

            // Update clock
            auto clock_now = high_resolution_clock::now();
            results_->time_us = time_t(duration_cast<duration<double>>(clock_now - clock_start).count() * 1E6);
        }

        // Generate path to goal
        generatePath_();

        results_->success_rate = !results_->exit_code;
        results_->nodes = std::move(grid_->nodes());
        results_->node_count = results_->nodes.size();

        // End clock
        auto clock_end = high_resolution_clock::now();
        results_->time_us = time_t(duration_cast<duration<double>>(clock_end - clock_start).count() * 1E6);

        // End of solve
    }

    void Astar::initialize_()
    {
        // Create new grid, queue, and results
        grid_ = std::make_shared<ImplicitGrid>(params_.grid_resolution);
        queue_ = std::make_shared<PriorityQueue>();
        closed_set_.clear();
        results_->iteration = 0;
        results_->exit_code = RUNNING;
        results_->time_us = 0;
        results_->cost = 0.0f;
        results_->success_rate = 0.0f;
        results_->node_count = 0;
        results_->node_path.clear();
        results_->state_path.clear();
        results_->control_path.clear();
        results_->nodes.clear();
        // Initialize start and goal on the grid
        start_node_ = grid_->get(params_.start_state);
        goal_node_ = grid_->get(params_.goal_state);
        // Set start node values
        start_node_->g = 0;
        start_node_->f = model_->heuristic(start_node_->state, params_.goal_state);
        // Add start to queue
        queue_->push(start_node_);
        // Set start as best
        best_node_ = start_node_;
    }

    Node *Astar::getNeighbor_(const Node *node, const Control &control)
    {
        // Get new state from control
        State new_state = model_->next_state(node->state, control);
        // Check if the state is valid
        if (model_->is_valid(new_state))
        {
            // Get the corresponding node
            Node *child_node = grid_->get(new_state);
            // Invalid if it's the same node
            if (child_node != node)
            {
                return child_node;
            }
        }
        return nullptr;
    }

    void Astar::updateLineage_(Node *child, Node *parent, const Control &control)
    {
        // Check if node's heuristic was already evaluated
        if (child->h == std::numeric_limits<float>::infinity())
        {
            // If not, find the heuristic
            child->h = model_->heuristic(child->state, params_.goal_state);
        }
        // Update Node's properties
        child->f = child->g + child->h;
        child->generation = parent->generation + 1;
        child->parent = parent;
        child->control = control;
    }

    void Astar::generatePath_()
    {

        results_->cost = best_node_->g;

        const size_t max_gen = best_node_->generation;
        results_->node_path.reserve(max_gen + 1);
        results_->state_path.reserve(max_gen + 1);
        results_->control_path.reserve(max_gen + 1);

        Node *node = best_node_;
        while (node)
        {
            results_->node_path.emplace_back(node);
            results_->state_path.emplace_back(node->state);
            if (node != start_node_)
                results_->control_path.emplace_back(node->parent->control);
            node = node->parent;
        }

        std::reverse(results_->node_path.begin(), results_->node_path.end());
        std::reverse(results_->state_path.begin(), results_->state_path.end());
        std::reverse(results_->control_path.begin(), results_->control_path.end());
    }

}