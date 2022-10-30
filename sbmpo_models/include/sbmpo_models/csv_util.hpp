#ifndef CSV_PARSER_HPP
#define CSV_PARSER_HPP

#include <sbmpo/types.hpp>
#include <iostream>
#include <fstream>
#include <stdexcept> // std::runtime_error
#include <sstream> // std::stringstream
#include <vector>
#include <algorithm>

#include <ros/ros.h>

namespace sbmpo_models {

    using namespace sbmpo;

    // Read from config
    void fromConfig(const std::string& filename, std::vector<PlannerParameters>& parameters) {

        std::ifstream myFile(filename);
        if(!myFile.is_open()) 
            throw std::runtime_error("Could not open file");

        std::string line, value;
        while (std::getline(myFile, line)) {
            PlannerParameters param;
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
            param.conditions.goal_threshold = std::stof(value);

            std::getline(ss, value, ',');
            int num_states = std::stof(value);
            
            for (int s = 0; s < num_states; s++) {
                std::getline(ss, value, ',');
                param.conditions.initial_state.push_back(std::stof(value));
            }

            for (int s = 0; s < num_states; s++) {
                std::getline(ss, value, ',');
                param.conditions.goal_state.push_back(std::stof(value));
            }

            for (int s = 0; s < num_states; s++) {
                std::getline(ss, value, ',');
                param.grid_parameters.active.push_back(std::stof(value));
            }

            std::getline(ss, value, ',');
            int num_controls = std::stof(value);

            for (int c = 0; c < num_controls; c++) {
                std::getline(ss, value, ',');
                param.conditions.initial_control.push_back(std::stof(value));
            }

            std::getline(ss, value, ',');
            int num_active = std::stof(value);

            for (int a = 0; a < num_active; a++) {
                std::getline(ss, value, ',');
                param.grid_parameters.resolution.push_back(std::stof(value));
            }

            for (int a = 0; a < num_active; a++) {
                std::getline(ss, value, ',');
                param.grid_parameters.size.push_back(std::stof(value));
            }

            std::getline(ss, value, ',');
            int num_branchouts = std::stof(value);

            for (int b = 0; b < num_branchouts; b++) {
                Control control;
                for (int c = 0; c < num_controls; c++) {
                    std::getline(ss, value, ',');
                    control.push_back(std::stof(value));
                }
                param.branchout.push_back(control);
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
    void addToData(const std::string& filename, const PlannerResults &results, const int options = (STATS | PATH | BUFFER)) {

        std::ofstream myFile(filename, std::ofstream::out | std::fstream::app);

        myFile << options;

        if (options & STATS) {
            myFile << ",";
            myFile << results.time_ms;
            myFile << ",";
            myFile << results.exit_code;
        }
        

        if (options & PATH) {
            myFile << ",";
            myFile << results.path.size();
            for (Index idx : results.path) {
                myFile << ",";
                myFile << idx;
            }
        }

        if (options & (PATH | BUFFER)) {
            myFile << ",";
            myFile << results.buffer[0].state.size();
            myFile << ",";
            myFile << results.buffer[0].control.size();
            myFile << ",";
            myFile << results.high;
        }

        for (int b = 0; b < results.high; b++) {

            if (!(options & BUFFER) && (options & PATH) &&!std::count(results.path.begin(), results.path.end(), b))
                continue;

            const sbmpo::Node& node = results.buffer[b];

            if (!(options & STATE_ONLY)) {
                myFile << ",";
                myFile << node.lineage.id;
                myFile << ",";
                myFile << node.lineage.parent;
                myFile << ",";
                myFile << node.lineage.child;
                myFile << ",";
                myFile << node.lineage.generation;
                myFile << ",";

                myFile << node.heuristic.f;
                myFile << ",";
                myFile << node.heuristic.g;
            }

            for (float s : node.state) {
                myFile << ",";
                myFile << s;
            }

            if (!(options & STATE_ONLY)) {
                for (int c = 0; c < node.control.size(); c++) {
                    myFile << ",";
                    myFile << node.control[c];
                }
            }

        }
        
        myFile << '\n';

        myFile.close();
    }  

}


#endif