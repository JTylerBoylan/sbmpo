#ifndef CSV_PARSER_HPP
#define CSV_PARSER_HPP

#include <sbmpo/types.hpp>
#include <iostream>
#include <fstream>
#include <stdexcept> // std::runtime_error
#include <sstream> // std::stringstream
#include <vector>

#include <ros/ros.h>

namespace sbmpo_models {

    using namespace sbmpo;

    // Read from config
    void fromConfig(const std::string& filename, std::vector<PlannerParameters>& parameters) {

        std::ifstream myFile(filename);
        if(!myFile.is_open()) 
            throw std::runtime_error("Could not open file");

        std::string line, value;
        while (std::getline(myFile, line), '\n') {
            PlannerParameters param;
            std::stringstream ss(line);

            std::getline(ss, value);
            param.max_iterations = std::stof(value);
            std::getline(ss, value);
            param.max_generations = std::stof(value);
            std::getline(ss, value);
            param.sample_time = std::stof(value);
            std::getline(ss, value);
            param.sample_time_increment = std::stof(value);
            std::getline(ss, value);
            param.conditions.goal_threshold = std::stof(value);

            std::getline(ss, value);
            int num_states = std::stof(value);
            for (int s = 0; s < num_states; s++) {
                std::getline(ss, value);
                param.conditions.initial_state.push_back(std::stof(value));
                std::getline(ss, value);
                param.conditions.goal_state.push_back(std::stof(value));
                std::getline(ss, value);
                param.grid_parameters.active.push_back(std::stof(value));
            }

            std::getline(ss, value);
            int num_controls = std::stof(value);
            for (int c = 0; c < num_controls; c++) {
                std::getline(ss, value);
                param.conditions.initial_control.push_back(std::stof(value));
            }

            std::getline(ss, value);
            int num_active = std::stof(value);
            for (int a = 0; a < num_active; a++) {
                std::getline(ss, value);
                param.grid_parameters.resolution.push_back(std::stof(value));
                std::getline(ss, value);
                param.grid_parameters.size.push_back(std::stof(value));
            }

            std::getline(ss, value);
            int num_branchouts = std::stof(value);
            for (int b = 0; b < num_branchouts; b++) {
                Control control;
                for (int c = 0; c < num_controls; c++) {
                    std::getline(ss, value);
                    control.push_back(std::stof(value));
                }
                param.branchout.push_back(control);
            }

            parameters.push_back(param);
        }

        myFile.close();
    }

    // Add to results file
    void addToData(const std::string& filename, const PlannerResults &results) {

        std::ofstream myFile(filename, std::ofstream::out | std::fstream::app);

        myFile << results.time_ms;
        myFile << ",";
        myFile << results.exit_code;
        myFile << ",";

        myFile << results.path.size();
        myFile << ",";
        for (Index idx : results.path) {
            myFile << idx;
            myFile << ",";
        }

        myFile << results.buffer[0].state.size();
        myFile << ",";
        myFile << results.buffer[0].control.size();
        myFile << ",";
        myFile << results.high;
        myFile << ",";

        for (Node *node = &(results.buffer[0]); node != &(results.buffer[results.high]); ++node) {

            myFile << node->lineage.id;
            myFile << ",";
            myFile << node->lineage.parent;
            myFile << ",";
            myFile << node->lineage.child;
            myFile << ",";
            myFile << node->lineage.generation;
            myFile << ",";

            myFile << node->heuristic.f;
            myFile << ",";
            myFile << node->heuristic.g;
            myFile << ",";

            for (float s : node->state) {
                myFile << s;
                myFile << ",";
            }

            for (float c : node->control) {
                myFile << c;
                myFile << ",";
            }
        }
        
        myFile.close();
    }  

}


#endif