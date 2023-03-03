%% Double Integrator Benchmark
% Jonathan Boylan
% 2023-03-01

clear
close all

%% Parameters

runs = 1;

params = struct;
params.max_iterations = 50000;
params.max_generations = 65;
params.horizon_time = 0.1;
params.num_states = 2;
params.num_controls = 1;
params.grid_resolution = [0.005; 0.0005];
params.branchout_factor = 15;
params.branchouts = linspace(-1,1,params.branchout_factor);


%% Write config file

sbmpo_config("../csv/config.csv", params, runs);
