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
### ROS
This package was developed on [ROS Noetic Ninjemys](https://wiki.ros.org/noetic) 
> [Installation (Ubuntu 20.04)](https://wiki.ros.org/noetic/Installation/Ubuntu)   

## Installation
To install, clone this package into your workspace and build.  
*Build with `DCMAKE_BUILD_TYPE` flag set to `Release` for increased performance.*

```
cd ~/catkin_ws/src
git clone https://github.com/JTylerBoylan/SBMPO.git
cd ../
catkin_make -DCMAKE_BUILD_TYPE=Release
```


## Running a sample model
```
rosrun sbmpo_models <model_name>
```
#### Sample Models
- `book_model`


## Creating your own model
### Template
**Abstract model class is provided in [`sbmpo/include/sbmpo/model.hpp`](https://github.com/JTylerBoylan/SBMPO/blob/main/sbmpo/include/sbmpo/model.hpp)**
```
#include <sbmpo/model.hpp>

namespace my_namespace {

  class my_custom_model : public sbmpo::Model {
 
    // Constructor
    my_custom_model() {}
    
    // Evaluate state and control in the model (return true if the state is valid)
    bool next_state(State& state2, const State& state1, const Control& control, const float time_span) {}
    
    // Determine the cost between two states with a given control
    float cost(const State& state2, const State& state1, const Control& control, const float time_span) {}
    
    // Determine the heuristic of a state with respect to the goal
    float heuristic(const State& state, const State& goal) {}
    
    // Determine if a state is within the goal threshold
    bool is_goal(const State& state, const State& goal, const float goal_threshold) {}
  
  };

}
```
### Running your model
#### Parameters
Before you can run your model, you must set parameters for the system.  
These parameters include:  
- `max_iterations`: Maximum branchout iterations (`int`)
- `max_generations`: Maximum branchout generations (`int`)
- `sample_time`: Time period per branchout (`float`)
- `sample_time_increment`: Time resolution for state validation (`float`)
- `goal_threshold`: Goal threshold value (`float`)
- `initial_state`: Initial state (`std::vector<float>`)
- `goal_state`: Goal state (`std::vector<float>`)
- `initial_control`: Initial control (`std::vector<float>`)
- `grid_states`: Boolean vector corresponding to whether the respective state is gridded (`std::vector<bool>`)
- `grid_resolution`: Grid resolutions for gridded states only (`std::vector<float>`)
- `samples`: List of controls to be sampled in a branchout (`std::vector<std::vector<float>>`)

#### Run the model
To run the model, simply pass your custom model class and parameters into the SBMPO run function. The run function returns an exit code corresponding to the end result of the plan.
##### Exit Codes:
| Exit Code | Description |
| --------- | ----------- |
|     0     | Path found |
|     1     | Iteration limit reached |
|     2     | Generation limit reached |
|     3     | No nodes left in queue |

#### Evaluate the results
The results of the run is stored in the `sbmpo::SMBPO` class.  
Here are some of the functions you can use:
| Type | Function | Description |
| ---- | -------- | ----------- |
| `std::vector<int>` | `path()` | Returns the best path found as a list of indices |
| `sbmpo::Vertex` | `vertex(int i)` | Converts an index to its corresponding vertex |
| `int` | `size()` | Get the number of vertices generated |
| `float` | `cost()` | Get the cost of the best path |

#### Example code

```
#include <sbmpo/sbmpo.hpp>
#include <my_package/my_custom_model.hpp>
#include <iostream>

using namespace my_namespace;

int main(int argc, char ** argv) {

  my_custom_model model;

  sbmpo::Parameters params;
  /* Add in parameters here */
  
  sbmpo::SBMPO planner;
  int exit_code = planner.run(model, params);
  
  std::cout << "Planner finished with exit code: " << exit_code << endl;
  
  return 0;
}
```


