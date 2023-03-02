%% Double Integrator Benchmark
% Jonathan Boylan
% 2023-03-01

clear
close all
clc

%% Parameters

runs = 1;

params = struct;
params.max_iterations = 1000;
params.max_generations = 100;
params.horizon_time = 0.1;
params.num_states = 2;
params.num_controls = 1;
params.grid_resolution = [0.01; 0.001];
params.branchout_factor = 7;
params.branchouts = [-1.0, -0.67, -0.33, 0, 0.33, 0.67, 1.0];


%% Write config file

sbmpo_config("../csv/config.csv", params, runs);
