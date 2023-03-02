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

    /// @brief Create a new CSV Tool
    /// @param csv_folder Path to csv workspace folder
    CSVTool(std::string csv_folder) {
        this->csv_folder_ = csv_folder;
    }

    /// @brief Change the csv workspace folder
    /// @param folder Path to new workspace folder
    void set_save_folder(std::string folder) {
        this->csv_folder_ = folder;
    }

    /// @brief Get current workspace folder
    /// @return Path to workspace folder
    std::string get_save_folder() { return this->csv_folder_; }

    /// @brief Clear results files (stats.csv and nodes.csv)
    void clear_results_file() {
        this->clear_file(csv_folder_ + stats_file);
        this->clear_file(csv_folder_ + nodes_file);
    }

    /// @brief Get parameters from config.csv file
    /// @return List of SBMPOParameters
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

    /// @brief Get timings from latest SBMPO result
    /// @return List of timings (in microseconds)
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

    /// @brief Append SMPO run results to stats.csv
    /// @param time_us Computation time (in microseconds)
    /// @param exit_code Exit code
    /// @param cost Cost of plan
    /// @param iterations Number of iterations
    /// @param node_count Number of nodes on grid
    /// @param success_rate Success rate of planner
    void append_stats(const unsigned long time_us, const int exit_code, const float cost, const float iterations, const int node_count, const float success_rate) {

        std::ofstream myFile(csv_folder_ + stats_file, std::ofstream::out | std::fstream::app);

        myFile << time_us;
        myFile << ",";
        myFile << exit_code;
        myFile << ",";
        myFile << cost;
        myFile << ",";
        myFile << iterations;
        myFile << ",";
        myFile << node_count;
        myFile << ",";
        myFile << success_rate;
        myFile << "\n";

    }

    /// @brief Append path of nodes to nodes.csv
    /// @param nodes List of nodes to append
    /// @param control_path Control path of the nodes
    void append_node_path(std::vector<Node::Ptr> nodes, std::vector<Control> control_path) {

        // Bad parameter check
        if (nodes.empty() || control_path.size() != nodes.size() - 1)
            return;

        std::ofstream myFile(csv_folder_ + nodes_file, std::ofstream::out | std::fstream::app);

        myFile << nodes.size();
        myFile << ",";
        myFile << nodes[0]->state().size();
        myFile << ",";
        myFile << control_path[0].size();

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

            if (n < nodes.size() - 1)
                for (float c : control_path[n]) {
                    myFile << ",";
                    myFile << c;
                }
        }
        
        myFile << '\n';

        myFile.close();

    }

    /// @brief Append list of nodes to nodes.csv
    /// @param nodes List of nodes to append
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

    private:

    std::string csv_folder_;

    // Clear a file
    void clear_file(const std::string &filename) {
        std::ofstream myFile(filename, std::ofstream::out | std::ofstream::trunc);
        myFile.close();
    }

};

}

#endif