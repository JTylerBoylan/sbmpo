#ifndef SBMPO_BOOK_MODEL_HPP
#define SBMPO_BOOK_MODEL_HPP

#include <sbmpo/model.hpp>
#include <sbmpo_models/csv_util.hpp>
#include <math.h>
#include <array>
#include <random>

#define M_2PI 6.283185307179586

namespace sbmpo_models {

    using namespace sbmpo;

    const float BODY_RADIUS = 0.20f;

    const float bounds[2][2] = {
        {-10.0, -10.0},
        {10.0, 10.0}  
    };

    class SBMPOBasicModel : public Model {

        public:

        std::vector<std::array<float,3>> obstacles;
        SBMPO &planner;

        SBMPOBasicModel(SBMPO &sbmpo)
        : planner(sbmpo) {}

        // Evaluate a node with a control
        void next_state(State &state, const Control& control, const float time_span) {
            
            // Update state
            state[0] += control[0] * time_span;
            state[1] += control[1] * time_span;

        }

        // Get the cost of a control
        float cost(const State& state2, const State &state1, const Control& control, const float time_span) {
            return sqrtf(control[0]*control[0] + control[1]*control[1]) * time_span;
        }

        // Get the heuristic of a state
        float heuristic(const State& state, const State &goal) {
            const float dx = goal[0] - state[0];
            const float dy = goal[1] - state[1];
            return sqrtf(dx*dx + dy*dy);
        }

        // Determine if node is valid
        bool is_valid(const State& state) {
            
            // Bound check
            if (state[0] - bounds[0][0] < BODY_RADIUS ||
                state[1] - bounds[0][1] < BODY_RADIUS ||
                bounds[1][0] - state[0] < BODY_RADIUS ||
                bounds[1][1] - state[1] < BODY_RADIUS)
                return false;

            // Obstacle collision check
            for (int o = 0; o < obstacles.size(); o++) {
                const float dx = state[0] - obstacles[o][0];
                const float dy = state[1] - obstacles[o][1];
                const float threshold = obstacles[o][2] + BODY_RADIUS;
                if (dx*dx + dy*dy < threshold*threshold)
                    return false;
            }

            return true;
        }

        // Determine if state is goal
        bool is_goal(const State& state, const State& goal, const float goal_threshold) {
            return heuristic(state, goal) <= goal_threshold;
        }

        void set_obstacles(std::vector<std::array<float,3>> &obstacle_vector) {
            obstacles = obstacle_vector;
        }

        // Generate random obstacles
        bool init = false;
        std::vector<std::array<float, 3>> randomize_obstacles(int n, float min, float max) {

            if (!init) {
                srand(time(NULL));
                init = true;
            }

            obstacles.clear();
            for (int i = 0; i < n;) {

                float x = ((rand() % int((max - min) * 10)) + min*10) / 10.0f;
                float y = ((rand() % int((max - min) * 10)) + min*10) / 10.0f;
                
                // Distance buffer around origin (start state)
                if (sqrtf(x*x + y*y) < 0.5)
                    continue;

                // Distance buffer around goal
                if (sqrtf((5.0f-x)*(5.0f-x) + (5.0f-y)*(5.0f-y)) < 0.5)
                    continue;

                // Check for overlap
                bool lap = false;
                for (std::array<float,3> ob : obstacles) {
                    float dox = ob[0] - x;
                    float doy = ob[1] - y;
                    if (sqrtf(dox*dox + doy*doy) < 0.5) {
                        lap = true;
                        break;
                    }
                }

                // Add obstacle
                if (!lap) {
                    obstacles.push_back({x, y, 0.5f});
                    i++;
                }

            }

            return obstacles;
            
        }

    };

}

#endif