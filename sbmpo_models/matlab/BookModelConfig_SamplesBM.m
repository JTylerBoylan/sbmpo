%% Book Model Configurer

clear
close all
clc

runs = 12;

max_iter = repmat(10000,runs,1);
max_gen = repmat(100,runs,1);
samp_time = repmat(0.3,runs,1);
samp_del_time = repmat(0.1,runs,1);
goal_thres = repmat(0.3,runs,1);
num_states = repmat(3,runs,1);
num_controls = repmat(2,runs,1);
num_active = repmat(2,runs,1);
init_state = repmat([0, 0, 1.5707],runs,1);
goal_state = repmat([5, 5, 0],runs,1);
init_control = repmat([0, 0],runs,1);
grid_act = repmat([1, 1, 0],runs,1);
grid_res = repmat([0.01, 0.01],runs,1);

rot = {
        [0 -0.785398 0.785398 -0.589049 0.589049 -0.392699 0.392699 -0.196350 0.196350];
        [0 -0.589049 0.589049 -0.392699 0.392699 -0.196350 0.196350];
        [0 -0.392699 0.392699 -0.196350 0.196350];
        [0 -0.392699 0.392699];
        [0 -0.785398 0.785398 -0.589049 0.589049 -0.392699 0.392699 -0.196350 0.196350];
        [0 -0.589049 0.589049 -0.392699 0.392699 -0.196350 0.196350];
        [0 -0.392699 0.392699 -0.196350 0.196350];
        [0 -0.392699 0.392699];
        [0 -0.785398 0.785398 -0.589049 0.589049 -0.392699 0.392699 -0.196350 0.196350];
        [0 -0.589049 0.589049 -0.392699 0.392699 -0.196350 0.196350];
        [0 -0.392699 0.392699 -0.196350 0.196350];
        [0 -0.392699 0.392699];
      };
vel = {
        [0.1 0.3 0.5];
        [0.1 0.3 0.5];
        [0.1 0.3 0.5];
        [0.1 0.3 0.5];
        [0.3 0.5];
        [0.3 0.5];
        [0.3 0.5];
        [0.3 0.5];
        [0.5];
        [0.5];
        [0.5];
        [0.5];
      };
  
conf = cell(runs,1);
for r = 1:runs

    szv = length(cell2mat(vel(r)));
    szr = length(cell2mat(rot(r)));
    num_samples = szr * szv;
    samples = zeros(1, num_samples*num_controls(r));

    for v = 1:szv
        for u = 1:szr
            vmat = cell2mat(vel(r));
            rmat = cell2mat(rot(r));
            samples(2*(v-1)*szr + 2*(u-1) + 1) = vmat(v);
            samples(2*(v-1)*szr + 2*(u-1) + 2) = rmat(u);
        end
    end 

    conf(r) = {[max_iter(r) max_gen(r) samp_time(r) samp_del_time(r) goal_thres(r) ...
            num_states(r) num_controls(r) num_active(r)...
            init_state(r) goal_state(r) init_control(r) ...
            grid_act(r) grid_res(r)...
            num_samples samples]};

end
    
 writecell(conf, '../config/book_model_verify.csv', 'Delimiter', ',')