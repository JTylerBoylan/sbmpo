# Sample Based Model Predictive Optimization (SBMPO) for robot trajectory planning

Sampling-Based Model Predictive Control (SBMPO) is a novel nonlinear MPC (NMPC) approach that enables
motion planning with dynamic models. This tool is also well suited to solve traditional MPC problems and has
been tested in various situations ranging from robotics, task scheduling, resource management, combustion
processes, and general optimization.

## Publication

```
@article{HARPER2021100159,
title = {SBMPO: Sampling Based Model Predictive Optimization for robot trajectory planning},
journal = {Software Impacts},
volume = {10},
pages = {100159},
year = {2021},
issn = {2665-9638},
doi = {https://doi.org/10.1016/j.simpa.2021.100159},
url = {https://www.sciencedirect.com/science/article/pii/S2665963821000671},
author = {Mario Harper and Camilo Ordonez and Emmanuel Collins}
}
```

## Dependencies
### CMake 3.15

## Installation
To install, clone this package into your workspace and make inside the build folder.

```
git clone https://github.com/JTylerBoylan/sbmpo.git
mkdir -p ./sbmpo/build
cd ./sbmpo/build && cmake ..
make
```

## Creating your own model
### Template
**Abstract model class is provided in [`sbmpo/include/sbmpo/types/Model.hpp`](https://github.com/JTylerBoylan/sbmpo/blob/main/sbmpo/include/sbmpo/types/Model.hpp)**
```
#include <sbmpo/types/types.hpp>
#include <sbmpo/types/Model.hpp>

namespace my_namespace {

  using namespace sbmpo;

  class MyCustomModel : public Model {
 
    // Constructor
    MyCustomModel() {}
    
    /*
        Dynamics of the system
        How does each state change with respect to the controls?
    */
    State next_state(const State& state, const Control& control, const float time_span) {}


    /*
        Cost of a state and control
        What am I trying to minimize?
        i.e Distance, Time, Energy
    */
    float cost(const State& state, const Control& control, const float time_span) {}


    /*
        Heuristic of a state with respect to the goal
        Leads the planner to the goal
        What is the lowest cost possible from this state to the goal?
    */
    float heuristic(const State& state, const State& goal) {}

    /*
        Is this state close enough to the goal to end the plan?
    */
    bool is_goal(const State& state, const State& goal) {}

    /*
        Does this state meet the model constraints?
        i.e Boundary constraints, Obstacles, State limits
    */
    bool is_valid(const State& state) {}
  
  };

}
```
### Running your model
#### Parameters
Before you can run your model, you must set parameters for the system.  
These parameters include:
| Name | Description | Type |
| ---- | ----------- | ---- |
| `max_iterations` | Maximum branchout iterations | `int` |
| `max_generations` | Maximum branchout generations | `int` |
| `sample_time` | Time period per branchout | `float` |
| `grid_resolution` | Grid resolutions | `std::vector<float>` |
| `start_state` | Initial state of plan | `sbmpo::State` |
| `goal_state` | Goal state of plan | `sbmpo::State` |
| `samples` | List of controls to be sampled in a branchout | `std::vector<sbmpo::Control>` |

#### Run the model
To run the model, simply create a SBMPO planner object with your custom model class and parameters, then execute it's `run()` function.
```
sbmpo::SBMPO<MyCustomModel> planner;
planner.run(params);
```

#### Evaluate the results
The results of the run is stored in the `sbmpo::SMBPO` class.  
Here are some of the functions you can use:
| Type | Function | Description |
| ---- | -------- | ----------- |
| `unsigned long` | `iterations()` | Get the number of iterations during the run |
| `int` | `exit_code()` | Get the exit code of the run *(see below)* |
| `time_t` | `time_us()` | Get the computation time of the run in microseconds |
| `float` | `cost()` | Get the cost of the best path |
| `size_t` | `size()` | Get the number of nodes on the grid |
| `std::vector<sbmpo::NodePtr>` | `node_path()` | Returns the best path found as a list of Node pointers |
| `std::vector<State>` | `state_path()` | Returns the best path found as a list of states |
| `std::vector<Control>` | `control_path()` | Returns the best path found as a list of controls |
| `std::vector<sbmpo::NodePtr>` | `nodes()` | Returns all nodes on the grid as a list of Node pointers |

##### Exit Codes:
| Exit Code | Description |
| --------- | ----------- |
|     0     | Solution found |
|     1     | Iteration limit reached |
|     2     | No nodes left in queue |
|     3     | Max generations reached |
|     5     | Unknown Error / Running |

#### Example code

```
#include <sbmpo/SBMPO.hpp>
#include <my_package/my_custom_model.hpp>
#include <sbmpo/tools/PrintTool.hpp>

using namespace my_namespace;

int main(int argc, char ** argv) {

  sbmpo::SBMPOParameters params;
  /* Add in parameters here */
  
  sbmpo::SBMPO planner(model, params);
  planner.run();
  
  sbmpo_io::print_parameters(params);
  sbmpo_io::print_results(planner.results());
  sbmpo_io::print_stats(planner.results());

  return 0;
}
```


