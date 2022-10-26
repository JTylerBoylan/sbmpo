%% Book Model Configurer

clear
close all
clc

max_iter = [10000; 10000];
max_gen = [50; 50];
samp_time = [0.5; 0.5];
samp_del_time = [0.1; 0.1];
goal_thres = [0.5; 0.5];
num_states = [3; 3];
init_state = [0 0 0;
              0 0 0];
goal_state = [5 5 0;
              5 5 0];
grid_act = [1 1 1;
            1 1 1];
num_controls = [2; 2];
init_control = [0 0;
                0 0];
num_active = [3; 3];
grid_res = [0.05 0.05 0.62832;
            0.05 0.05 0.62832];
grid_size = [150 150 10;
             150 150 10];
num_branchout = [11; 11];
branchouts = [0.4 0 0.2 -0.3927 0.1333 0.3927 0.53 -0.589 0.3333 0.1963 ...
              0.4667 0.589 0.2667 -0.6872 0.0444 0.0982 0.4444 -0.2945 0.2444 0.4909;
              0.4 0 0.2 -0.3927 0.1333 0.3927 0.53 -0.589 0.3333 0.1963 ...
              0.4667 0.589 0.2667 -0.6872 0.0444 0.0982 0.4444 -0.2945 0.2444 0.4909];

conf = [max_iter max_gen samp_time samp_del_time goal_thres ...
        num_states init_state goal_state grid_act ...
        num_controls init_control ...
        num_active grid_res grid_size ...
        num_branchout branchouts];
    
 writematrix(conf, 'book_model.csv', 'Delimiter', ',')