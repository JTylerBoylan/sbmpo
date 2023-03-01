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
        this->map_bounds_ = {
            {-5.0, -5.0},
            {5.0, 5.0}  
        };

    }

    // Determine if node is valid
    bool is_valid(const State& state) override {
        
        // Bound check
        if (state[0] - map_bounds_[0][0] < body_radius_ ||
            state[1] - map_bounds_[0][1] < body_radius_ ||
            map_bounds_[1][0] - state[0] < body_radius_ ||
            map_bounds_[1][1] - state[1] < body_radius_)
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

        std::vector<std::vector<float>> map_bounds_;

};

}

#endif