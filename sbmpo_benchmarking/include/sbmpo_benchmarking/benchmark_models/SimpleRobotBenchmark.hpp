#ifndef SBMPO_BENCHMARKING_SIMPLE_ROBOT_HPP
#define SBMPO_BENCHMARKING_SIMPLE_ROBOT_HPP

#include <sbmpo_models/SimpleRobot.hpp>
#include <sbmpo_benchmarking/benchmarks/Obstacles2D.hpp>

namespace sbmpo_benchmarking {

using namespace sbmpo_models;

class SimpleRobotBenchmark : public SimpleRobotModel, public Obstacles2DBenchmark
{

    public:

    SimpleRobotBenchmark(std::string csv_folder) : SimpleRobotModel(), Obstacles2DBenchmark(csv_folder) 
    {
        this->body_radius_ = 0.25;
        this->map_bounds_ = {-10.0f, -10.0f, 10.0f, 10.0f};
    }

    void set_body_radius(float body_radius) {
        this->body_radius_ = body_radius;
    }

    void set_map_bounds(std::array<float, 4> map_bounds) {
        this->map_bounds_ = map_bounds;
    }

    // Determine if node is valid
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