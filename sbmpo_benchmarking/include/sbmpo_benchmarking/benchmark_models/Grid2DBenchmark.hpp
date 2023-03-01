#ifndef SBMPO_BENCHMARKING_GRID2D_HPP
#define SBMPO_BENCHMARKING_GRID2D_HPP

#include <sbmpo_models/Grid2D.hpp>
#include <sbmpo_benchmarking/benchmarks/Obstacles2D.hpp>

namespace sbmpo_benchmarking {

using namespace sbmpo_models;

class Grid2DBenchmark : public Grid2DModel, public Obstacles2DBenchmark
{

    public:

    /// @brief Create a new Grid2D benchmark
    /// @param csv_folder Path to csv workspace folder
    Grid2DBenchmark(std::string csv_folder) : Grid2DModel(), Obstacles2DBenchmark(csv_folder) 
    {
        this->body_radius_ = 0.25f;
        this->map_bounds_ = {-10.0f, -10.0f, 10.0f, 10.0f};
    }

    /// @brief Change the body radius (default 0.25)
    /// @param body_radius New body radius value
    void set_body_radius(float body_radius) {
        this->body_radius_ = body_radius;
    }

    /// @brief Change the benchmark map boundaries
    /// @param map_bounds Array of 4 boundary values ([xmin ymin xmax ymax])
    void set_map_bounds(std::array<float, 4> map_bounds) {
        this->map_bounds_ = map_bounds;
    }

    // Determine if node is valid (with obstacles and map bounds)
    bool is_valid(const State& state) override {
        
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

};

}

#endif