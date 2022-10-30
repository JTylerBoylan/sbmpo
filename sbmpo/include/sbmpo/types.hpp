#ifndef SBMPO_TYPES_HPP
#define SBMPO_TYPES_HPP

#include <vector>
#include <queue>
#include <functional>
#include <algorithm>
#include <map>

namespace sbmpo {

    /*
        Node Types
    */

    typedef std::vector<float> State;
    typedef std::vector<float> Control;

    typedef int Index;
    #define INVALID_INDEX -1

    /*
        Node Structs
    */

    struct Lineage {
        Index id, parent, child;
        int generation;
    };

    struct Heuristic {
        float f, g;
    };

    struct Node {
        Lineage lineage;
        Heuristic heuristic;
        State state;
        Control control;
    };

    typedef Node* NodeBuffer;
    typedef std::priority_queue<Index, std::vector<Index>, const std::function<bool (Index,Index)>> NodeQueue;

    /*
        Implicit Grid Types
    */

    typedef unsigned long Key;

    typedef std::vector<bool> GridActive;
    typedef std::vector<float> GridResolution;
    typedef std::vector<int> GridSize;

    typedef std::vector<int> GridKey;
    typedef std::map<GridKey, Index> IndexKeyMap;

    /*
        Implicit Grid Structs
    */

    struct ImplicitGridParameters {
        GridActive active;
        GridResolution resolution;
        GridSize size;
    };

    /*
        Planner Types
    */

    typedef std::vector<Control> Branchout;
    typedef std::vector<Index> PlannerPath;

    /*
        Planner Structs
    */

    struct PlannerConditions {
        State initial_state;
        Control initial_control;
        State goal_state;
        float goal_threshold;
    };

    struct PlannerParameters {
        int max_iterations, max_generations;
        float sample_time, sample_time_increment;
        PlannerConditions conditions;
        Branchout branchout;
        ImplicitGridParameters grid_parameters;
    };

    enum PlannerExitCode {GOAL_REACHED, ITERATION_LIMIT, GENERATION_LIMIT, NO_NODES_LEFT, NONE};

    struct PlannerResults {
        NodeBuffer buffer;
        PlannerPath path;
        Index high, best;
        PlannerExitCode exit_code;
        double time_ms;
    };

}

#endif