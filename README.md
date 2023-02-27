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
### CMake 3.5

## Installation
To install, clone this package into your workspace and use the provided build script

```
git clone https://github.com/JTylerBoylan/SBMPO.git
cd SBMPO && ./build.sh
```

## Creating your own model
### Template
**Abstract model class is provided in [`sbmpo/include/sbmpo/model.hpp`](https://github.com/JTylerBoylan/SBMPO/blob/main/sbmpo/include/sbmpo/model.hpp)**
```
#include <sbmpo/model.hpp>

namespace my_namespace {

  class my_custom_model : public sbmpo::Model {
 
    // Constructor
    my_custom_model() {}

    // Return the initial state of the model
    State initial_state() {}
    
    // Evaluate state and control in the model
    State next_state(const State& state, const Control& control, const float time_span) {}
    
    // Determine the cost between two states with a given control
    float cost(const State& state2, const State& state1, const Control& control, const float time_span) {}
    
    // Determine the heuristic of a state
    float heuristic(const State& state) {}
    
    // Determine if a state is the goal
    bool is_goal(const State& state) {}

    // Determine if a state statifies all constraints
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
| `samples` | List of controls to be sampled in a branchout | `std::vector<sbmpo::Control>` |

#### Run the model
To run the model, simply create a SBMPO planner object with your custom model class and parameters, then execute it's `run()` function.
```
sbmpo::SBMPO planner(my_custom_model, params);
planner.run();
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
| `std::vector<std::shared_ptr<Node>>` | `node_path()` | Returns the best path found as a list of Node pointers |
| `std::vector<State>` | `state_path()` | Returns the best path found as a list of states |
| `std::vector<Control>` | `control_path()` | Returns the best path found as a list of controls |
| `std::vector<std::shared_ptr<Node>>` | `all_nodes()` | Returns all nodes on the grid as a list of Node pointers |

##### Exit Codes:
| Exit Code | Description |
| --------- | ----------- |
|     0     | Path found |
|     1     | Iteration limit reached |
|     2     | Generation limit reached |
|     3     | No nodes left in queue |
|     4     | Invalid path generated |
|     5     | Unknown Error |

#### Example code

```
#include <sbmpo/sbmpo.hpp>
#include <my_package/my_custom_model.hpp>
#include <iostream>

using namespace my_namespace;

int main(int argc, char ** argv) {

  my_custom_model model;

  sbmpo::SBMPOParameters params;
  /* Add in parameters here */
  
  sbmpo::SBMPO planner(model, params);
  planner.run();
  
  std::cout << "---- Planner Results ----" << std::endl;
  std::cout << "Iterations: " << planner.iterations() << std::endl;
  std::cout << "Exit code: " << planner.exit_code() << std::endl;
  std::cout << "Computation Time: " << planner.time_us() << "us" << std::endl;
  std::cout << "Path cost: " << planner.cost() << std::endl;
  std::cout << "Number of nodes: " << planner.size() << std::endl;
  
  std::cout << "-- State Path --" << std::endl;
  for (sbmpo::State state : planner.state_path()) {
    std::cout << "  - [ ";
    for (float s : state) {
      std::cout << s << " ";
    }
    std::cout << "]" << std::endl;
  }
  
  std::cout << "-- Control Path --" << std::endl;
  for (sbmpo::Control control : planner.control_path()) {
    std::cout << "  - [ ";
    for (float c : control) {
      std::cout << c << " ";
    }
    std::cout << "]" << std::endl;
  }
  
  return 0;
}
```


