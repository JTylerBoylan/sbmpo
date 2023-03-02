%% Simple Robot Benchmark
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
params.horizon_time = 0.5;
params.num_states = 3;
params.num_controls = 2;
params.grid_resolution = [0.3536; 0.3536; 0.0982];
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

sbmpo_config("../csv/config.csv", params, runs);

