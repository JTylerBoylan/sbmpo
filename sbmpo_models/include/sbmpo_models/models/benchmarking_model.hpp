#ifndef SBMPO_BENCHMARKING_MODEL_HPP
#define SBMPO_BENCHMARKING_MODEL_HPP

#include <sbmpo/model.hpp>
#include <math.h>
#include <array>
#include <random>

namespace sbmpo_models {

class BenchmarkingModel : public Model {

    public:

    float BODY_RADIUS = 0.20f;

    std::vector<std::vector<float>> BOUNDS = {
        {-5.0, -5.0},
        {5.0, 5.0}  
    };

    State START_STATE = {-3.0,-3.0};
    State GOAL_STATE = {3.0,3.0};

    float GOAL_THRESHOLD = 0.25f;

    std::vector<std::array<float,3>> OBSTACLES;

    // Generate random obstacles
    const int dec = 10;
    std::vector<std::array<float, 3>> randomize_obstacles(int min_n, int max_n, float min_x, float max_x, float min_y, float max_y, float min_r, float max_r) {

        OBSTACLES.clear();
        int n = (rand() % (max_n - min_n)) + min_n;
        for (int i = 0; i < n;) {

            float x = float(rand() % int((max_x - min_x) * dec)) / dec + min_x;
            float y = float(rand() % int((max_y - min_y) * dec)) / dec + min_y;
            float r = float(rand() % int((max_r - min_r) * dec)) / dec + min_r;
            
            // Distance buffer around origin (start state)
            if (sqrtf(x*x + y*y) < BODY_RADIUS + r)
                continue;

            // Distance buffer around goal
            if (sqrtf((GOAL_STATE[0]-x)*(GOAL_STATE[0]-x) + (GOAL_STATE[1]-y)*(GOAL_STATE[1]-y)) < GOAL_THRESHOLD)
                continue;

            // Check for overlap
            bool lap = false;
            for (std::array<float,3> ob : OBSTACLES) {
                float dox = ob[0] - x;
                float doy = ob[1] - y;
                if (sqrtf(dox*dox + doy*doy) < r + ob[2]) {
                    lap = true;
                    break;
                }
            }

            // Add obstacle
            if (!lap) {
                OBSTACLES.push_back({x, y, r});
                i++;
            }

        }

        return OBSTACLES;
    }

};

}

#endif