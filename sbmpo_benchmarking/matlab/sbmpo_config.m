function sbmpo_config(filename, params, runs, newfile)
    
    arguments
        filename string
        params struct
        runs int64
        newfile logical = true
    end

    config = [params.max_iterations, params.max_generations, params.horizon_time,...
        params.num_states, params.num_controls, params.grid_resolution.',...
        params.start_state.', params.goal_state.', ...
        params.branchout_factor, ...
        reshape(params.branchouts, [1 params.branchout_factor*params.num_controls])];
    
    config = repmat(config, [runs 1]);

    if (newfile)
        writematrix(config, filename, 'Delimiter', ',');
    else
        writematrix(config, filename, 'Delimiter', ',', 'WriteMode', 'append');
    end
end

