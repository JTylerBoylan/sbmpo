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
    void fromConfig(const std::string& filename, std::vector<Parameters>& parameters) {

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

    // Add to results file
    enum SaveOptions {STATS = 0b1, PATH = 0b10, BUFFER = 0b100, STATE_ONLY = 0b1000};
    void addToData(const std::string& filename, SBMPO &results, 
            const float time_ms, const int exit_code, const int options = (STATS | PATH | BUFFER)) {

        std::ofstream myFile(filename, std::ofstream::out | std::fstream::app);

        myFile << options;

        if (options & STATS) {
            myFile << ",";
            myFile << time_ms;
            myFile << ",";
            myFile << exit_code;
        }
        

        if (options & PATH) {
            myFile << ",";
            myFile << results.path().size();
            for (int idx : results.path()) {
                myFile << ",";
                myFile << idx;
            }
        }

        if (options & (PATH | BUFFER)) {
            myFile << ",";
            myFile << results.graph[0].state.size();
            myFile << ",";
            myFile << results.graph[0].control.size();
            myFile << ",";
            myFile << results.size();
        }

        for (int b = 0; b < results.size(); b++) {

            if (!(options & BUFFER) && (options & PATH) && !std::count(results.path().begin(), results.path().end(), b))
                continue;

            const sbmpo::Vertex& vertex = results.graph[b];

            if (!(options & STATE_ONLY)) {
                myFile << ",";
                myFile << vertex.idx;
                myFile << ",";
                myFile << vertex.gen;
                myFile << ",";

                myFile << vertex.f;
                myFile << ",";
                myFile << vertex.g;
            }

            for (float s : vertex.state) {
                myFile << ",";
                myFile << s;
            }

            if (!(options & STATE_ONLY)) {
                for (int c = 0; c < results.graph[0].control.size(); c++) {
                    myFile << ",";
                    myFile << (c < vertex.control.size()) ? vertex.control[c] : 0;
                }
            }

        }
        
        myFile << '\n';

        myFile.close();
    }  

}


#endif