%% Ackermann Steering Benchmark
% Jonathan Boylan
% 2023-03-27

clear
close all
clc

%% Parameters

runs = 1;

params = struct;
params.max_iterations = 50000;
params.max_generations = 50;
params.horizon_time = 0.25;
params.num_states = 5;
params.num_controls = 2;
params.grid_resolution = [0.1; 0.1; 0.015; -1; -1];
params.start_state = [-10; -10; pi/4; 0; 0];
params.goal_state = [10; 10; 0; 0; 0];
params.branchout_factor = 9;
params.branchouts = [
    [2.5; 0.523], ...
    [2.5; 0], ...
    [2.5; -0.523], ...
    [0; 0.523], ...
    [0; 0], ...
    [0; -0.523], ...
    [-1.25; 0.523], ...
    [-1.25; 0], ...
    [-1.25; -0.523] ...
    ];

%% Write config file

sbmpo_config("../../csv/config.csv", params, runs);

%% Generate some obstacles

obs = struct;
obs.min_n = 3;
obs.max_n = 5;
obs.min_x = -5;
obs.max_x = 5;
obs.min_y = -5;
obs.max_y = 5;
obs.min_r = 1;
obs.max_r = 2;
random_obstacles("../../csv/obstacles.csv", obs, runs)
