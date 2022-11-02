%% Book Model Configurer

clear
close all
clc

max_iter = 3000;
max_gen = 100;
samp_time = 0.3;
samp_del_time = 0.1;
goal_thres = 0.3;
num_states = 3;
init_state = [0, 0, 1.5707];
goal_state = [5 5 0];
grid_act = [1 1 0];
num_controls = 2;
init_control = [0 0];
num_active = 2;
grid_res = [0.05 0.05];
grid_size = [100 100];

rot = [-0.785398 0 0.785398 -0.392699 0.392699 -0.196350 0.196350 -0.589049 0.589049];
vel = [0.1 0.3 0.5];

num_branchout = length(rot) * length(vel);
branchouts = zeros(1, num_branchout*num_controls);

for v = 1:length(vel)
    for u = 1:length(rot)
        branchouts(2*(v-1)*length(rot) + 2*(u-1) + 1) = vel(v);
        branchouts(2*(v-1)*length(rot) + 2*(u-1) + 2) = rot(u);
    end
end

conf = [max_iter max_gen samp_time samp_del_time goal_thres ...
        num_states init_state goal_state grid_act ...
        num_controls init_control ...
        num_active grid_res grid_size ...
        num_branchout branchouts];
    
 writematrix(conf, 'book_model_verify.csv', 'Delimiter', ',')