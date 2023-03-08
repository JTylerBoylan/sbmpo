%% Simple Robot Benchmark - Horizon Time vs. Grid Resolution Plot
% Jonathan Boylan
% 2023-03-01

clear
close all

%% Parameters

params = struct;
params.max_iterations = 5000;
params.max_generations = 100;
% params.horizon_time = 0.5;
params.num_states = 3;
params.num_controls = 2;
% params.grid_resolution = [0.3536, 0.3536, 0.0982];
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

grid_resolution_xy = linspace(0.1, 0.25, 30);
horizon_time = linspace(0.25, 1.0, 30);
grid_resolution_q = 0.1963;

for gr = 1:length(grid_resolution_xy)
    for ht = 1:length(horizon_time)
        params.horizon_time = horizon_time(ht);
        grid_res_xy = grid_resolution_xy(gr);
        params.grid_resolution = [grid_res_xy; grid_res_xy; grid_resolution_q];

        % Write to config csv
        if (gr + ht == 2)
            sbmpo_config("../csv/config.csv", params, 1, true);
        else
            sbmpo_config("../csv/config.csv", params, 1, false);
        end  
   end
end


%% Other Params

goal_threshold = 0.25;
initial_state = [-3, -3];
goal_state = [3, 3];
