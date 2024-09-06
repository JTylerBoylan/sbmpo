%% Double Integrator Benchmark
% Jonathan Boylan
% 2023-03-01

clear
close all

addpath('..')
%% Parameters

runs = 1;

params = struct;
params.max_iterations = 50000;
params.max_generations = 100;
params.horizon_time = 0.25;
params.num_states = 2;
params.num_controls = 1;
params.grid_resolution = [0.01; 0.025];
params.start_state = [-10; 0];
params.goal_state = [0; 0];
params.branchout_factor = 7;
params.branchouts = linspace(-1,1,params.branchout_factor);


%% Write config file

sbmpo_config("../../csv/config.csv", params, runs);
