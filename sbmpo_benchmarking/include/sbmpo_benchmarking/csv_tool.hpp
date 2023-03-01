#ifndef SBMPO_BECNHMARKING_CSV_TOOL_HPP
#define SBMPO_BECNHMARKING_CSV_TOOL_HPP

#include <string>
#include <iostream>
#include <fstream>
#include <stdexcept> // std::runtime_error
#include <sstream> // std::stringstream

#include <sbmpo/sbmpo.hpp>

namespace sbmpo_benchmarking {

using namespace sbmpo;

class CSVTool {

    const std::string params_file = "config.csv";
    const std::string nodes_file = "nodes.csv";
    const std::string stats_file = "stats.csv";
    const std::string obstacles_file = "obstacles.csv";

    public:

    CSVTool(std::string csv_folder) {
        this->csv_folder_ = csv_folder;
    }

    void set_save_folder(std::string folder) {
        this->csv_folder_ = folder;
    }

    std::string get_save_folder() { return this->csv_folder_; }

    void clear_results_file() {
        this->clear_file(csv_folder_ + stats_file);
        this->clear_file(csv_folder_ + nodes_file);
    }

    void clear_obstacles_file() {
        this->clear_file(csv_folder_ + obstacles_file);
    }

    // Read from config
    std::vector<SBMPOParameters> get_params() {

        std::vector<SBMPOParameters> parameters;

        std::ifstream myFile(csv_folder_ + params_file);
        if(!myFile.is_open()) 
            throw std::runtime_error("Could not open file");

        std::string line, value;
        while (std::getline(myFile, line)) {

            SBMPOParameters param;
            std::stringstream ss(line);

            std::getline(ss, value, ',');
            param.max_iterations = std::stof(value);
            std::getline(ss, value, ',');
            param.max_generations = std::stof(value);
            std::getline(ss, value, ',');
            param.sample_time = std::stof(value);

            std::getline(ss, value, ',');
            int num_states = std::stof(value);
            std::getline(ss, value, ',');
            int num_controls = std::stof(value);

            for (int a = 0; a < num_states; a++) {
                std::getline(ss, value, ',');
                param.grid_resolution.push_back(std::stof(value));
            }

            std::getline(ss, value, ',');
            int num_branchouts = std::stof(value);

            for (int b = 0; b < num_branchouts; b++) {
                Control control(num_controls);
                for (int c = 0; c < num_controls; c++) {
                    std::getline(ss, value, ',');
                    control[c] = std::stof(value);
                }
                param.samples.push_back(control);
            }

            parameters.push_back(param);

        }

        myFile.close();

        return parameters;
    }

    std::vector<float> get_timings() {

        std::vector<float> timeList;

        std::ifstream myFile(csv_folder_ + stats_file);
        if(!myFile.is_open()) 
            throw std::runtime_error("Could not open file");

        std::string line, value;
        while (std::getline(myFile, line)) {
            std::stringstream ss(line);
            std::getline(ss, value, ',');
            float time = std::stof(value);
            timeList.push_back(time);
        }

        return timeList;
    }

    std::vector<std::vector<std::array<float,3>>> get_obstacles() {

        std::vector<std::vector<std::array<float,3>>> obstaclesList;

        std::ifstream myFile(csv_folder_ + obstacles_file);
        if(!myFile.is_open()) 
            throw std::runtime_error("Could not open file");

        std::string line, value;
        while (std::getline(myFile, line)) {

            std::vector<std::array<float,3>> obstacles;
            std::stringstream ss(line);

            std::getline(ss, value, ',');
            int num_obstacles = std::stoi(value);

            for (int obs = 0; obs < num_obstacles; obs++) {

                float x,y,r;

                std::getline(ss, value, ',');
                x = std::stof(value);
                std::getline(ss, value, ',');
                y = std::stof(value);
                std::getline(ss, value, ',');
                r = std::stof(value);

                obstacles.push_back({x, y, r});
            }

            obstaclesList.push_back(obstacles);
        }

        return obstaclesList;
    }

        // Add stats to data
    void append_stats(const unsigned long time_us, const int exit_code, const float cost, const float iterations, const int buffer_size, const float success_rate) {

        std::ofstream myFile(csv_folder_ + stats_file, std::ofstream::out | std::fstream::app);

        myFile << time_us;
        myFile << ",";
        myFile << exit_code;
        myFile << ",";
        myFile << cost;
        myFile << ",";
        myFile << iterations;
        myFile << ",";
        myFile << buffer_size;
        myFile << ",";
        myFile << success_rate;
        myFile << "\n";

    }

        // Add to results file
    void append_nodes(std::vector<Node::Ptr> nodes) {

        std::ofstream myFile(csv_folder_ + nodes_file, std::ofstream::out | std::fstream::app);      

        myFile << nodes.size();
        myFile << ",";
        myFile << nodes[0]->state().size();

        for (size_t n = 0; n < nodes.size(); n++) {

            const Node::Ptr node = nodes[n];

            myFile << ",";
            myFile << node->generation();

            myFile << ",";
            myFile << node->f();
            myFile << ",";
            myFile << node->g();
            myFile << ",";
            myFile << node->rhs();
            
            for (float s : node->state()) {
                myFile << ",";
                myFile << s;
            }

        }
        
        myFile << '\n';

        myFile.close();
    }

    void append_obstacles(std::vector<std::array<float, 3>> obstacles) {
        std::ofstream myFile(csv_folder_ + obstacles_file, std::ofstream::out | std::fstream::app);
        myFile << obstacles.size();
        for (std::array<float, 3> ob : obstacles)
            for (int o = 0; o < 3; o++)
                myFile << "," << ob[o];
        myFile << '\n';
        myFile.close();
    }

    private:

    std::string csv_folder_;

    void clear_file(const std::string &filename) {
        std::ofstream myFile(filename, std::ofstream::out | std::ofstream::trunc);
        myFile.close();
    }

};

}

#endif