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


## Running a model
```
rosrun sbmpo_models <model_name>
```
#### Example Models
- `book_model`


## Creating your own model
### Template
**Abstract model class is provided in [`sbmpo/include/sbmpo/model.hpp`](https://github.com/JTylerBoylan/SBMPO/blob/main/sbmpo/include/sbmpo/model.hpp)**
```
#include <sbmpo/model.hpp>

namespace my_namespace {

  class my_custom_model : public sbmpo::Model {
    
    bool next_state(State& state2, const State& state1, const Control& control, const float time_span) {}
    
    float cost(const State& state2, const State& state1, const Control& control, const float time_span) {}
    
    float heuristic(const State& state, const State& goal) {}
    
    bool is_valid(const State& state) {}
    
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
- `goal_threshold`: 
- `initial_state`
- `goal_state`
- `initial_control`
- `grid_states`
- `grid_resolution`
- `samples`
```
int main(int argc, char ** argv) {

  sbmpo::Parameters params;
  params.max_iterations = <#>;
  params.max_generations = <#>;
  params.sample_time = <#>;
  params.sample_time_increment = <#>;
  params.goal_threshold = <#>;
  
  
  return 0;
}
```


