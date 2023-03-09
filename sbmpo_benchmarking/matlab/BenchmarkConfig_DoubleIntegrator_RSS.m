%% Double Integrator Benchmark
% Jonathan Boylan
% 2023-03-01

clear
close all

%% Parameters

runs = 5;

params = struct;
params.max_iterations = 5000;
params.max_generations = 100;
params.horizon_time = 0.1;
params.num_states = 2;
params.num_controls = 1;
params.grid_resolution = [0.005; 0.0005];
%params.start_state = [-10; 0];
params.goal_state = [0; 0];
params.branchout_factor = 7;
params.branchouts = linspace(-1,1,params.branchout_factor);


%% Write config file

% generate random start states
start_states = (rand(2, 10)-0.5).*[15;7.5];

for i = 1:runs
    
    params.start_state = start_states(:,i);
    
    if (i == 1)
        sbmpo_config("../csv/config.csv", params, 1, true);
    else
        sbmpo_config("../csv/config.csv", params, 1, false);
    end
end
