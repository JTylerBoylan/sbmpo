#ifndef CSV_PARSER_HPP
#define CSV_PARSER_HPP

#include <sbmpo/types.hpp>
#include <sbmpo/sbmpo.hpp>
#include <iostream>
#include <fstream>
#include <stdexcept> // std::runtime_error
#include <sstream> // std::stringstream

#include <ros/ros.h>

namespace sbmpo_models {

    using namespace sbmpo;

    // Read from config
    void readParametersFromFile(const std::string& filename, std::vector<Parameters>& parameters) {

        std::ifstream myFile(filename);
        if(!myFile.is_open()) 
            throw std::runtime_error("Could not open file");

        std::string line, value;
        while (std::getline(myFile, line)) {

            Parameters param;
            std::stringstream ss(line);

            std::getline(ss, value, ',');
            param.max_iterations = std::stof(value);
            std::getline(ss, value, ',');
            param.max_generations = std::stof(value);
            std::getline(ss, value, ',');
            param.sample_time = std::stof(value);
            std::getline(ss, value, ',');
            param.sample_time_increment = std::stof(value);
            std::getline(ss, value, ',');
            param.goal_threshold = std::stof(value);

            std::getline(ss, value, ',');
            int num_states = std::stof(value);
            std::getline(ss, value, ',');
            int num_controls = std::stof(value);
            std::getline(ss, value, ',');
            int num_active = std::stof(value);
            
            for (int s = 0; s < num_states; s++) {
                std::getline(ss, value, ',');
                param.initial_state.push_back(std::stof(value));
            }

            for (int s = 0; s < num_states; s++) {
                std::getline(ss, value, ',');
                param.goal_state.push_back(std::stof(value));
            }

            for (int c = 0; c < num_controls; c++) {
                std::getline(ss, value, ',');
                param.initial_control.push_back(std::stof(value));
            }

            for (int s = 0; s < num_states; s++) {
                std::getline(ss, value, ',');
                param.grid_states.push_back(std::stof(value));
            }

            for (int a = 0; a < num_active; a++) {
                std::getline(ss, value, ',');
                param.grid_resolution.push_back(std::stof(value));
            }

            std::getline(ss, value, ',');
            int num_branchouts = std::stof(value);

            for (int b = 0; b < num_branchouts; b++) {
                Control control;
                for (int c = 0; c < num_controls; c++) {
                    std::getline(ss, value, ',');
                    control.push_back(std::stof(value));
                }
                param.samples.push_back(control);
            }

            parameters.push_back(param);

        }

        myFile.close();
    }

    void clearFile(const std::string &filename) {
        std::ofstream myFile(filename, std::ofstream::out | std::ofstream::trunc);
        myFile.close();
    }

    // Add stats to data
    void appendStatsToFile(const std::string& filename, 
            const float time_ms, const int exit_code, const float cost, const int buffer_size, const float success_rate) {

        std::ofstream myFile(filename, std::ofstream::out | std::fstream::app);

        myFile << time_ms;
        myFile << ",";
        myFile << exit_code;
        myFile << ",";
        myFile << cost;
        myFile << ",";
        myFile << buffer_size;
        myFile << ",";
        myFile << success_rate;
        myFile << "\n";

    }

    // Add to results file
    void appendResultsToFile(const std::string& filename, SBMPO &results) {

        std::ofstream myFile(filename, std::ofstream::out | std::fstream::app);      

        myFile << results.path().size();
        for (int idx : results.path()) {
            myFile << ",";
            myFile << idx;
        }

        myFile << ",";
        myFile << results.size();
        myFile << ",";
        myFile << results.graph[0].state.size();

        for (int b = 0; b < results.size(); b++) {

            const sbmpo::Vertex& vertex = results.graph[b];

            myFile << ",";
            myFile << vertex.idx;
            myFile << ",";
            myFile << vertex.gen;

            myFile << ",";
            myFile << vertex.f;
            myFile << ",";
            myFile << vertex.g;
            myFile << ",";
            myFile << vertex.rhs;
            
            for (float s : vertex.state) {
                myFile << ",";
                myFile << s;
            }

        }
        
        myFile << '\n';

        myFile.close();
    }
    

    void appendObstaclesToFile(const std::string& filename, std::vector<std::array<float, 3>> obstacles) {
        std::ofstream myFile(filename, std::ofstream::out | std::fstream::app);
        myFile << obstacles.size();
        for (std::array<float, 3> ob : obstacles)
            for (int o = 0; o < 3; o++)
                myFile << "," << ob[o];
        myFile << '\n';
        myFile.close();
    }

    void readObstaclesFromFile(const std::string& filename, std::vector<std::vector<std::array<float,3>>> obstaclesList) {

        std::ifstream myFile(filename);
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
    }

    void readComputationTimeFromFile(const std::string& filename, std::vector<float> timeList) {

        std::ifstream myFile(filename);
        if(!myFile.is_open()) 
            throw std::runtime_error("Could not open file");

        std::string line, value;
        while (std::getline(myFile, line)) {
            std::stringstream ss(line);
            std::getline(ss, value, ',');
            float time = std::stof(value);
            timeList.push_back(time);
        }
    }

}


#endif