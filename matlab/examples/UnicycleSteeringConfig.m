%% Unicycle Steering Benchmark
% Jonathan Boylan
% 2023-03-01

clear
close all
clc

%% Parameters

runs = 1;

params = struct;
params.max_iterations = 5000;
params.max_generations = 100;
params.horizon_time = 0.5;
params.num_states = 3;
params.num_controls = 2;
params.grid_resolution = [0.10; 0.10; pi/36];
params.start_state = [-3; -3; 0];
params.goal_state = [3; 3; 0];
params.branchout_factor = 10;
params.branchouts = [
    [0.5; -0.3927], ...
    [0.5; -0.1963], ...
    [0.5; 0], ...
    [0.5; 0.1963], ...
    [0.5; 0.3927], ...
    [1.0; -0.3927], ...
    [1.0; -0.1963], ...
    [1.0; 0], ...
    [1.0; 0.1963], ...
    [1.0; 0.3927]
    ];


%% Write config file

sbmpo_config("../../csv/config.csv", params, runs);


%% Create random set of obstacles

obs = struct;
obs.min_n = 1;
obs.max_n = 1;
obs.min_x = 0;
obs.max_x = 0;
obs.min_y = 0;
obs.max_y = 0;
obs.min_r = 1;
obs.max_r = 1;

random_obstacles("../../csv/obstacles.csv", obs, runs);