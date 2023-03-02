%% Grid2D Benchmark
% Jonathan Boylan
% 2023-03-01

clear
close all
clc

%% Parameters

runs = 10;

params = struct;
params.max_iterations = 5000;
params.max_generations = 100;
params.horizon_time = 0.25;
params.num_states = 2;
params.num_controls = 2;
params.grid_resolution = [0.125; 0.125];
params.branchout_factor = 9;
params.branchouts = [
    [-1; -1], ...
    [-1; 0], ...
    [-1; 1], ...
    [0; 1], ...
    [0; 0], ...
    [0; 1], ...
    [1; -1], ...
    [1; 0], ...
    [1; 1]
    ];


%% Write config file

sbmpo_config("../csv/config.csv", params, runs);
