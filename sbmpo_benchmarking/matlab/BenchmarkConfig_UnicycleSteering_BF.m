%% Simple Steering Benchmark - Branchout Factor Comparison
% Jonathan Boylan
% 2023-03-01

clear
close all
clc

%% Parameters

omega_bar = pi / 5;

params = struct;
params.max_iterations = 10000;
params.max_generations = 100;
params.horizon_time = 2.5;
params.num_states = 3;
params.num_controls = 2;
params.grid_resolution = [0.125; 0.125; 0.1963];
params.start_state = [-3; -3; 0];
params.goal_state = [3; 3; 0];

branchout_factors = [9 15 21 27];
linear_controls = [0.1 0.3 0.5];
rotational_controls = {
        [0, +omega_bar, -omega_bar];
        [0, +omega_bar, -omega_bar, +omega_bar/2, -omega_bar/2];
        [0, +omega_bar, -omega_bar, +omega_bar/2, -omega_bar/2,...
            +omega_bar/4, -omega_bar/4];
        [0, +omega_bar, -omega_bar, +omega_bar/2, -omega_bar/2,...
            +omega_bar/4, -omega_bar/4, +omega_bar*3/4, -omega_bar*3/4];
      };

for run = 1:length(branchout_factors)
   
    params.branchout_factor = branchout_factors(run);
   
    rot_con = cell2mat(rotational_controls(run));
    branchouts = [];
    for lin = 1:length(linear_controls)
       for rot = 1:length(rot_con)
          branchouts = [
              branchouts, ...
              [linear_controls(lin); rot_con(rot)]
          ];
       end
    end
    
    params.branchouts = branchouts;
    
    % Write to config csv
    if (run == 1)
        sbmpo_config("../csv/config.csv", params, 1, true);
    else
        sbmpo_config("../csv/config.csv", params, 1, false);
    end
    
end
