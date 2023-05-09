#ifndef SBMPO_BENCHMARK_MODEL_OBSTACLE2D_HPP_
#define SBMPO_BENCHMARK_MODEL_OBSTACLE2D_HPP_

#include <sbmpo/sbmpo.hpp>

namespace sbmpo {

using namespace sbmpo;

typedef std::vector<std::array<float,3>> Obstacles;

template<typename ModelType>
class Obstacle2DModel : public ModelType {

    public:

    Obstacle2DModel() {
        body_radius_ = 0.25f;
        map_bounds_ = {-10.0f, -10.0f, 10.0f, 10.0f};
    }

    /// @brief Set the obstacles of the benchmark
    /// @param obstacles Obstacles to be set
    void set_obstacles(Obstacles obstacles) {
        obstacles_ = obstacles;
    }

    /// @brief Change the body radius (default 0.25)
    /// @param body_radius New body radius value
    void set_body_radius(float body_radius) {
        body_radius_ = body_radius;
    }

    /// @brief Change the benchmark map boundaries
    /// @param map_bounds Array of 4 boundary values ([xmin ymin xmax ymax])
    void set_map_bounds(std::array<float, 4> map_bounds) {
        map_bounds_ = map_bounds;
    }

    // Determine if node is valid (with obstacles and map bounds)
    bool is_valid(const State& state) override {
        
        // Check model function
        if (!ModelType::is_valid(state))
            return false;

        // Bound check
        if (state[0] - map_bounds_[0] < body_radius_ ||
            state[1] - map_bounds_[1] < body_radius_ ||
            map_bounds_[2] - state[0] < body_radius_ ||
            map_bounds_[3] - state[1] < body_radius_)
            return false;

        // Obstacle collision check
        for (size_t o = 0; o < obstacles_.size(); o++) {
            const float dx = state[0] - obstacles_[o][0];
            const float dy = state[1] - obstacles_[o][1];
            const float threshold = obstacles_[o][2] + body_radius_;
            if (dx*dx + dy*dy < threshold*threshold)
                return false;
        }

        return true;
    }

    protected:

    float body_radius_;
    std::array<float, 4> map_bounds_;
    Obstacles obstacles_;

};

}

#endif