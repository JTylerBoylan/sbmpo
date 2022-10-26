function saveconfig(max_iter, max_gen, samp_time, samp_del_time, goal_thres, init_state, goal_state, grid_act, init_control, grid_res, grid_size, branchout)
%TOPLANNERCONFIG Convert params to config file

    conf = [max_iter' max_gen' samp_time' samp_del_time' goal_thres' init_state' goal_state' grid_act' init_control', grid_res', grid_size'];

end

