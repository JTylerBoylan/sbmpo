#ifndef SBMPO_CSV_TOOL_HPP_
#define SBMPO_CSV_TOOL_HPP_

#include <string>
#include <iostream>
#include <fstream>
#include <stdexcept> // std::runtime_error
#include <sstream>   // std::stringstream

#include <sbmpo/types/types.hpp>
#include <sbmpo/types/Node.hpp>

namespace sbmpo_csv
{
    using namespace sbmpo;

    // Get parameters from config.csv file
    std::vector<SearchParameters> get_params(const std::string &params_file)
    {
        std::ifstream myFile(params_file);
        if (!myFile.is_open())
            throw std::runtime_error("Could not open file");

        std::vector<SearchParameters> parameters;
        std::string line, value;
        while (std::getline(myFile, line))
        {
            SearchParameters param;

            std::stringstream ss(line);
            std::getline(ss, value, ',');
            param.max_iterations = std::stoi(value);
            std::getline(ss, value, ',');
            param.max_generations = std::stoi(value);
            std::getline(ss, value, ',');
            param.sample_time = std::stof(value);
            std::getline(ss, value, ',');
            int num_states = std::stoi(value);
            std::getline(ss, value, ',');
            int num_controls = std::stoi(value);
            for (int a = 0; a < num_states; a++)
            {
                std::getline(ss, value, ',');
                param.grid_resolution.push_back(std::stof(value));
            }
            for (int a = 0; a < num_states; a++)
            {
                std::getline(ss, value, ',');
                param.start_state.push_back(std::stof(value));
            }
            for (int a = 0; a < num_states; a++)
            {
                std::getline(ss, value, ',');
                param.goal_state.push_back(std::stof(value));
            }
            std::getline(ss, value, ',');
            int num_branchouts = std::stoi(value);
            for (int b = 0; b < num_branchouts; b++)
            {
                Control control(num_controls);
                for (int c = 0; c < num_controls; c++)
                {
                    std::getline(ss, value, ',');
                    control[c] = std::stof(value);
                }
                param.fixed_samples.push_back(control);
            }
            parameters.push_back(param);
        }
        myFile.close();
        return parameters;
    }

    // Get timings from latest SBMPO result
    std::vector<float> get_timings(const std::string &stats_file)
    {
        std::ifstream myFile(stats_file);
        if (!myFile.is_open())
            throw std::runtime_error("Could not open file");

        std::vector<float> timeList;
        std::string line, value;
        while (std::getline(myFile, line))
        {
            std::stringstream ss(line);
            std::getline(ss, value, ',');
            float time = std::stof(value);
            timeList.push_back(time);
        }
        return timeList;
    }

    // Append SMPO run results to stats.csv
    void append_stats(const std::string &stats_file, const SearchResults &results)
    {
        std::ofstream myFile(stats_file, std::ofstream::out | std::fstream::app);

        myFile << results.time_us;
        myFile << ",";
        myFile << results.exit_code;
        myFile << ",";
        myFile << results.cost;
        myFile << ",";
        myFile << results.iteration;
        myFile << ",";
        myFile << results.node_count;
        myFile << ",";
        myFile << results.success_rate;
        myFile << "\n";
    }

    // Append path of nodes to nodes.csv
    void append_node_path(const std::string &nodes_file, const std::vector<Node *> &nodes)
    {
        std::ofstream myFile(nodes_file, std::ofstream::out | std::fstream::app);

        myFile << nodes.size();
        myFile << ",";
        myFile << nodes[0]->state.size();
        myFile << ",";
        myFile << (nodes.size() < 2 ? 0 : nodes[1]->control.size());
        for (size_t n = 0; n < nodes.size(); n++)
        {
            const Node *node = nodes[n];
            myFile << ",";
            myFile << node->generation;
            myFile << ",";
            myFile << node->f;
            myFile << ",";
            myFile << node->g;
            for (float s : node->state)
            {
                myFile << ",";
                myFile << s;
            }
            for (float c : node->control)
            {
                myFile << ",";
                myFile << c;
            }
        }
        myFile << '\n';
        myFile.close();
    }

    // Append list of nodes to nodes.csv
    void append_nodes(const std::string &nodes_file, const std::vector<Node *> &nodes)
    {
        std::ofstream myFile(nodes_file, std::ofstream::out | std::fstream::app);

        myFile << nodes.size();
        myFile << ",";
        myFile << nodes[0]->state.size();
        for (size_t n = 0; n < nodes.size(); n++)
        {
            const Node *node = nodes[n];
            myFile << ",";
            myFile << node->generation;
            myFile << ",";
            myFile << node->f;
            myFile << ",";
            myFile << node->g;
            for (float s : node->state)
            {
                myFile << ",";
                myFile << s;
            }
        }
        myFile << '\n';
        myFile.close();
    }

    // Clear a file
    void clear_file(const std::string &filename)
    {
        std::ofstream myFile(filename, std::ofstream::out | std::ofstream::trunc);
        myFile.close();
    }

}

#endif