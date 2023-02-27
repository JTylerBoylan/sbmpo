#ifndef SBMPO_MODELS_BENCHMARK_MODEL_HPP
#define SBMPO_MODELS_BENCHMARK_MODEL_HPP

#include <string>

#include <sbmpo/sbmpo.hpp>
#include <sbmpo_models/csv_tool.hpp>

namespace sbmpo_models {

using namespace sbmpo;

enum ObstacleType {NONE, FROM_FILE, RANDOM};

struct RandomObstacleParameters {
    int minN, maxN;
    float minX, maxX;
    float minY, maxY;
    float minR, maxR;
    std::vector<std::array<float, 2>> buf;
    float bufR;
};

class BenchmarkModel : public Model {

    public:

    std::vector<std::array<float,3>> obstacles;

    BenchmarkModel() {
        this->verbose_ = true;
        this->runs_per_param_ = 1;
        this->obstacle_type_ = NONE;
    }

    void set_verbose(bool tf) {
        this->verbose_ = tf;
    }

    void set_runs_per_param(uint runs_per_param) {
        this->runs_per_param_ = runs_per_param;
    }

    void set_random_obstacle_parameters(RandomObstacleParameters rand_obs_params) {
        this->obs_params_ = rand_obs_params;
    }

    void benchmark() {

        csv_tool_.clear_results_file();
        if (obstacle_type_ == RANDOM)
            csv_tool_.clear_obstacles_file();

        std::vector<std::vector<std::array<float,3>>> obstaclesList;
        if (obstacle_type_ == FROM_FILE)
            obstaclesList = csv_tool_.get_obstacles();

        int par = 0;
        std::vector<SBMPOParameters> parameters_list = csv_tool_.get_params();
        for (auto param = parameters_list.begin(); param != parameters_list.end(); ++param) {

            if (verbose_) print_parameters(*param);

                int exit_code;
                unsigned long time_us = 0.0;
                double cost = 0.0;
                int node_count = 0;
                int success_count = 0;
                int iterations = 0;

                if (obstacle_type_ == RANDOM)
                    this->randomize_obstacles();

                if (obstacle_type_ == FROM_FILE)
                    obstacles = obstaclesList[par++];
                
                sbmpo::SBMPO sbmpo(*this, *param);

                for (int r = 0; r < runs_per_param_; r++) {

                    sbmpo.reset();
                    sbmpo.run();

                    exit_code = sbmpo.exit_code();
                    time_us += sbmpo.time_us();
                    cost += exit_code ? 0 : sbmpo.cost();
                    iterations += sbmpo.iterations();
                    node_count += sbmpo.size();
                    if (!exit_code) success_count++;

                }

                unsigned long time_us_avg = time_us / runs_per_param_;
                float iterations_avg = double(iterations) / runs_per_param_;
                float cost_avg = cost / success_count;
                float node_count_avg = double(node_count) / runs_per_param_;
                float success_rate = double(success_count) / runs_per_param_;

                if (verbose_) print_stats(time_us_avg, exit_code, iterations_avg, cost_avg, node_count_avg, success_rate);
                if (verbose_) print_results(sbmpo);
                if (verbose_) print_obstacles(obstacles);

                if (verbose_) printf("Writing results in folder %s ...\n\n", csv_tool_.get_save_folder().c_str());
                csv_tool_.append_stats(time_us_avg, exit_code, iterations_avg, cost_avg, node_count_avg, success_rate);
                csv_tool_.append_nodes(sbmpo.node_path());
                csv_tool_.append_nodes(sbmpo.all_nodes());
                if (obstacle_type_ == RANDOM)
                    csv_tool_.append_obstacles(obstacles);

        }

        if (verbose_) printf("Finished benchmarking.\n");
    }

    void set_folder(std::string folder_path) {
        csv_tool_.set_save_folder(folder_path);
    }

    void set_obstacle_type(ObstacleType obstacle_type) {
        obstacle_type_ = obstacle_type;
    }

    private:

    bool verbose_;
    uint runs_per_param_;

    CSVTool csv_tool_;

    ObstacleType obstacle_type_;
    RandomObstacleParameters obs_params_;

    // Generate random obstacles
    const int dec = 10;
    void randomize_obstacles() {

        obstacles.clear();
        int n = (rand() % (obs_params_.maxN - obs_params_.minN)) + obs_params_.minN;
        for (int i = 0; i < n;) {

            // Generate obstacle positions
            float x = float(rand() % int((obs_params_.maxX - obs_params_.minX) * dec)) / dec + obs_params_.minX;
            float y = float(rand() % int((obs_params_.maxY - obs_params_.minY) * dec)) / dec + obs_params_.minY;
            float r = float(rand() % int((obs_params_.maxR - obs_params_.minR) * dec)) / dec + obs_params_.minR;

            // Check if overlaps buffer positions
            bool lap_buf = false;
            for (std::array<float,2> buf_pos : obs_params_.buf) {
                if (sqrtf((buf_pos[0]-x)*(buf_pos[0]-x) + (buf_pos[1]-y)*(buf_pos[1]-y)) < obs_params_.bufR){
                    lap_buf = true;
                    break;
                }
            }
            
            if (lap_buf)
                continue;

            // Check for overlap
            bool lap_obs = false;
            for (std::array<float,3> ob : obstacles) {
                float dox = ob[0] - x;
                float doy = ob[1] - y;
                if (sqrtf(dox*dox + doy*doy) < r + ob[2]) {
                    lap_obs = true;
                    break;
                }
            }

            if (lap_obs)
                continue;

            // Add obstacle
            obstacles.push_back({x, y, r});
            i++;
        }
    }

    int print_count = 0;
    void print_parameters(const sbmpo::SBMPOParameters &params) {
        printf("---- Planner Parameters [%d] ----\n", print_count);
        printf("Max iterations: %d\n", params.max_iterations);
        printf("Max generations: %d\n", params.max_generations);
        printf("Sample Time: %.2f\n", params.sample_time);;

        std::string st;

        for (float f : params.grid_resolution)
            st += std::to_string(f) + " ";
        printf("Grid Resolution: %s\n", st.c_str());
        st.clear();

        printf("Samples:\n");
        for (sbmpo::Control control : params.samples) {
            for (float f : control)
                st += std::to_string(f) + " ";
            printf("  - %s\n", st.c_str());
            st.clear();
        }
    }

    void print_results(sbmpo::SBMPO &results) {
        printf("---- Planner Path [%d] ----\n", print_count++);
        int c = 0;
        for (std::shared_ptr<sbmpo::Node> node : results.node_path()) {
            printf("  (%d) x: %.3f, y: %.3f, g: %.3f, rhs: %.3f, f: %.3f\n",
                ++c,
                node->state()[0], node->state()[1],
                node->g(), node->rhs(), node->f());
        }
        printf("--------\n");
    }

    void print_stats(const unsigned long timeUs, const int exitCode, const int iterations, const float cost, const int bufferSize, const float successRate) {
        printf("---- Planner Stats ----\n");
        printf("  Time: %lu us\n", timeUs);
        printf("  Exit Code: %d\n", exitCode);
        printf("  Iterations: %d\n", iterations);
        printf("  Cost: %.2f\n", cost);
        printf("  Buffer Size: %d\n", bufferSize);
        printf("  Success Rate: %.1f\n", successRate * 100);
        printf("--------\n");
    }

    void print_obstacles(const std::vector<std::array<float, 3>> obstacles) {
        printf("Obstacles:\n");
        for (std::array<float, 3> obs : obstacles) {
            std::string st;
            for (float f : obs)
                st += std::to_string(f) + " ";
            printf(" - %s\n", st.c_str());
        }
    }

};

}

#endif