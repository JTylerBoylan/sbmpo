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

    typedef std::vector<std::pair<std::string, std::vector<float>>> CSVMap;

    // CSV Reading
    CSVMap read_csv(const std::string& filename) {

        CSVMap result;

        std::ifstream myFile(filename);
        if(!myFile.is_open()) 
            throw std::runtime_error("Could not open file");

        std::string line, colname;
        float val;
        if(myFile.good()) {
            std::getline(myFile, line, char(13));
            std::stringstream ss(line);
            while(std::getline(ss, colname, ','))
                result.push_back({colname, std::vector<float>()});
        }

        int c = 0;
        while(std::getline(myFile, line)) {
            std::stringstream ss(line);
            std::string sval;
            auto colIter = result.begin();
            while (std::getline(ss, sval, ',')){
                if (!sval.empty() && sval[0] != 13) {
                    val = std::stof(sval);
                    colIter->second.push_back(val);
                }
                ++colIter;
            }
        }

        myFile.close();
        return result;
    }

    // CSV Writing
    void write_csv(const std::string& filename, const CSVMap& csv_map) {

        std::ofstream myFile(filename, std::ofstream::out | std::fstream::trunc);

        int max_col_length = 0;

        for (int i = 0; i < csv_map.size(); i++) {
            myFile << csv_map[i].first;
            if (csv_map[i].second.size() > max_col_length)
                max_col_length = csv_map[i].second.size();
            if (i != csv_map.size() - 1) 
                myFile << ",";
        }
        myFile << "\n";

        for (int j = 0; j < max_col_length; j++) {
            for (int i = 0; i < csv_map.size(); i++) {
                std::vector<float> list = csv_map[i].second;
                if (list.size() > j)
                    myFile << csv_map[i].second[j];
                if (i != csv_map.size() - 1)
                    myFile << ",";
            }
            myFile << "\n";
        }
        

        myFile.close();

    }

    // Lookup value in CSV
    std::vector<float> csv_lookup(std::string key, const CSVMap& csv_map) {
        for (auto pair = csv_map.begin(); pair != csv_map.end(); ++pair)
            if (pair->first == key)
                return pair->second;
        return std::vector<float>();
    }

    // Convert CSVMap to PlannerParameters list
    std::vector<PlannerParameters> fromCSVMap(const CSVMap& csv_map) {
        
        const int size = csv_lookup("max_iterations", csv_map).size();

        std::vector<PlannerParameters> result(size);

        int sidx = 0, cidx = 0, bidx = 0, aidx = 0;
        for (int i = 0; i < size; i++) {
            result[i].max_iterations = csv_lookup("max_iterations", csv_map)[i];
            result[i].max_generations = csv_lookup("max_generations", csv_map)[i];
            result[i].sample_time = csv_lookup("sample_time", csv_map)[i];
            result[i].sample_time_increment = csv_lookup("sample_time_increment", csv_map)[i];
            result[i].conditions.goal_threshold = csv_lookup("goal_threshold", csv_map)[i];

            const int num_states = csv_lookup("num_states", csv_map)[i];
            for (int s = 0; s < num_states; s++) {
                result[i].conditions.initial_state.push_back(csv_lookup("initial_state", csv_map)[sidx + s]);
                result[i].conditions.goal_state.push_back(csv_lookup("goal_state", csv_map)[sidx + s]);
                result[i].grid_parameters.active.push_back(csv_lookup("grid_active", csv_map)[sidx + s]);
            }
            sidx += num_states;

            const int num_controls = csv_lookup("num_controls", csv_map)[i];
            for (int c = 0; c < num_controls; c++) {
                result[i].conditions.initial_control.push_back(csv_lookup("initial_control", csv_map)[cidx + c]);
            }
            cidx += num_controls;

            const int num_branchouts = csv_lookup("num_branchouts", csv_map)[i];
            for (int b = 0; b < num_branchouts; b++) {
                Control control;
                for (int c = 0; c < num_controls; c++)
                    control.push_back(csv_lookup("branchouts", csv_map)[bidx + num_controls * b + c]);
                result[i].branchout.push_back(control);
            }
            bidx += num_branchouts * num_controls;

            const int num_active = csv_lookup("num_active", csv_map)[i];
            for (int a = 0; a < num_active; a++) {
                result[i].grid_parameters.resolution.push_back(csv_lookup("grid_resolution", csv_map)[aidx + a]);
                result[i].grid_parameters.size.push_back(csv_lookup("grid_size", csv_map)[aidx + a]);
            }
            aidx += num_active;

        }

        return result;
    }

    // Add PlannerResults to CSVMap
    CSVMap addToCSVMap(const PlannerResults& results, const CSVMap& csv_map) {

        bool csv_init = csv_map.size() > 0;

        std::vector<float> time_ms = !csv_init ? std::vector<float>() : csv_lookup("time_ms", csv_map), 
                           exit_code = !csv_init ? std::vector<float>() : csv_lookup("exit_code", csv_map),
                           path_size = !csv_init ? std::vector<float>() : csv_lookup("path_size", csv_map), 
                           path = !csv_init ? std::vector<float>() : csv_lookup("path", csv_map), 
                           buffer_size = !csv_init ? std::vector<float>() : csv_lookup("buffer_size", csv_map), 
                           buffer = !csv_init ? std::vector<float>() : csv_lookup("buffer", csv_map),
                           num_states = !csv_init ? std::vector<float>() : csv_lookup("num_states", csv_map),
                           num_controls = !csv_init ? std::vector<float>() : csv_lookup("num_controls", csv_map);

        time_ms.push_back(results.time_ms);
        exit_code.push_back(results.exit_code);
        path_size.push_back(results.path.size());
        buffer_size.push_back(results.high);
        num_states.push_back(results.buffer[0].state.size());
        num_controls.push_back(results.buffer[0].control.size());
        for (sbmpo::Index idx : results.path)
            path.push_back(idx);
        for (int idx = 0; idx < results.high; idx++) {
            sbmpo::Node node = results.buffer[idx];
            buffer.push_back(node.lineage.id);
            buffer.push_back(node.lineage.parent);
            buffer.push_back(node.lineage.child);
            buffer.push_back(node.lineage.generation);
            buffer.push_back(node.heuristic.f);
            buffer.push_back(node.heuristic.g);
            for (float s : node.state)
                buffer.push_back(s);
            for (float c: node.control)
                buffer.push_back(c);
        }

        CSVMap csv_out;
        csv_out.push_back({"time_ms", time_ms});
        csv_out.push_back({"exit_code", exit_code});
        csv_out.push_back({"path_size", path_size});
        csv_out.push_back({"path", path});
        csv_out.push_back({"buffer_size", buffer_size});
        csv_out.push_back({"buffer", buffer});
        csv_out.push_back({"num_states", num_states});
        csv_out.push_back({"num_controls", num_controls});
        return csv_out;
    }   

}


#endif